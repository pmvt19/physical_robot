#!/bin/bash

# --- Configuration ---
SERVER_CMD="redis-server"
CAMERA_CMD="python run_camera_elite.py"
CRASHY_CMD="python run_lidar.py"

# --- Function to handle script exit (e.g., when you hit Ctrl+C) ---
cleanup() {
    echo -e "\nCaught signal, attempting graceful shutdown of background processes..."
    
    # Kill the camera process. 2>/dev/null suppresses "No such process" errors.
    kill "$CAMERA_PID" 2>/dev/null
    
    # Kill the server process using its stored PID. 2>/dev/null suppresses errors.
    kill "$SERVER_PID" 2>/dev/null
    
    # Give processes a moment to shut down before exiting the script
    sleep 1 
    
    exit 0
}

# Trap the interrupt (Ctrl+C) and terminate (kill) signals
trap cleanup INT TERM

## --- 1. Start the Server (Non-Blocking) ---
echo "Starting Server: $SERVER_CMD"
# The '&' sends the command to the background
$SERVER_CMD & 
SERVER_PID=$! # Store the Process ID (PID)

# Verify the server process started
if [ -z "$SERVER_PID" ] || ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "ERROR: Server failed to start. Exiting."
    exit 1
fi
echo "Server started with PID: $SERVER_PID"
echo "---"

# Give the server a moment to fully initialize (crucial for client connections)
sleep 2

## --- 2. Start the Camera Process (Non-Blocking) ---
echo "Starting Camera Process: $CAMERA_CMD"
$CAMERA_CMD &
CAMERA_PID=$! # Store the Process ID (PID) of the camera command

if [ -z "$CAMERA_PID" ] || ! kill -0 "$CAMERA_PID" 2>/dev/null; then
    echo "WARNING: Camera process may have failed to start immediately."
fi
echo "Camera Process started with PID: $CAMERA_PID"
echo "---"


## --- 3. Loop to Run and Restart the Crash-Prone Process ---
echo "Starting crash-prone process: $CRASHY_CMD (will auto-restart if it crashes)"
while true; do
    
    # Run the crashy command (blocking until it finishes or crashes)
    $CRASHY_CMD
    
    EXIT_CODE=$? # Capture the exit code (0 = success, anything else = crash)
    
    # Check if the process exited cleanly
    if [ $EXIT_CODE -eq 0 ]; then
        echo "Crash-prone command finished cleanly (Exit Code 0). Exiting restart loop."
        break # Exit the while loop if success is reached
    else
        # Crash/Failure handling
        echo "WARNING: Crash-prone command failed (Exit Code $EXIT_CODE). Restarting in 5 seconds..."
        sleep 5
    fi

    # Check dependencies before restarting the crashy command:
    
    # A. Check Server status
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "FATAL: Server process (PID $SERVER_PID) is no longer running. Stopping all."
        break
    fi
    
    # B. Check Camera status
    if ! kill -0 "$CAMERA_PID" 2>/dev/null; then
        echo "FATAL: Camera process (PID $CAMERA_PID) is no longer running. Stopping all."
        # If the camera dies, you might choose to restart it here instead of breaking,
        # but breaking is safer if the lidar depends on the camera.
        break
    fi

done

## --- 4. Final Cleanup ---
# This part runs if the 'while' loop breaks cleanly or due to a fatal error.
cleanup