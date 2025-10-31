#!/bin/bash

# --- Configuration ---
SERVER_CMD="your_server_command --port 8080"  # REPLACE with your actual server command
CRASHY_CMD="your_crashing_command --config /etc/app.conf" # REPLACE with your actual crashy command

# --- Function to handle script exit (e.g., when you hit Ctrl+C) ---
cleanup() {
    echo -e "\nCaught signal, shutting down background processes..."
    # Kill the server process using its stored PID
    kill "$SERVER_PID" 2>/dev/null
    exit 0
}

# Trap the interrupt signal (Ctrl+C) to ensure the server is killed
trap cleanup INT TERM

# 1. Start the blocking server process in the background
echo "Starting server: $SERVER_CMD"
# The '&' sends the command to the background
$SERVER_CMD & 
SERVER_PID=$! # Store the Process ID (PID) of the last background command

if [ -z "$SERVER_PID" ] || ! kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "ERROR: Server failed to start. Exiting."
    exit 1
fi
echo "Server started with PID: $SERVER_PID"
echo "---"

# 2. Loop to run the crashy command and restart it on failure
echo "Starting crash-prone process: $CRASHY_CMD (will auto-restart if it crashes)"
while true; do
    
    # Run the crashy command
    $CRASHY_CMD
    
    EXIT_CODE=$? # Capture the exit code of the last command
    
    # Check if the process exited cleanly (exit code 0 is typically success)
    if [ $EXIT_CODE -eq 0 ]; then
        echo "Crash-prone command finished cleanly (Exit Code 0). Exiting restart loop."
        break # Exit the while loop if it finished successfully
    else
        # If the exit code is non-zero, it means a crash/failure
        echo "WARNING: Crash-prone command failed (Exit Code $EXIT_CODE). Restarting in 5 seconds..."
        sleep 5
    fi

    # Check if the server is still running before restarting the crashy command
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
        echo "FATAL: Server process (PID $SERVER_PID) is no longer running. Stopping all."
        break
    fi

done

# 3. Clean up and wait (this part only runs if the loop breaks)
cleanup

# Wait for the background process (the server) to finish.
# This is technically reached by the 'cleanup' function, but
# a 'wait' here would hold the script open until the server stops,
# though we handle server stop on failure/cleanup already.
# wait $SERVER_PID