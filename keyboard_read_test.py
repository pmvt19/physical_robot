from pynput import keyboard
from robot_interface import RobotInterface
from dxl_controller import *

motion = "none"
running = True


controller = DynamixelController(device_name='/dev/tty.usbserial-FTAKRMAJ', motor_ids=[1, 2])
ri = RobotInterface(controller=controller)




def on_press(key):
    global motion
    try:
        print('alphanumeric key {0} pressed'.format(key.char))
        # pass
    except AttributeError:
        # print('special key {0} pressed'.format(key))

        # print(type(key))
        # print(str(key))
        str_key = str(key)
        # print(str_key)
        if str_key == 'Key.up':
            motion = "forward"
        elif str_key == 'Key.down':
            motion = 'backward'
        elif str_key == 'Key.left':
            motion = 'rotate_left'
        elif str_key == 'Key.right':
            motion = 'rotate_right'
        

def on_release(key):
    global running, motion
    # print('{0} released'.format(key))
    if key == keyboard.Key.esc:
        # Stop listener
        running = False
        return False
    motion = "none"

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

while running:

    if motion == 'forward':
        ri.move_forward()
    elif motion == 'backward':
        ri.move_backward()
    elif motion == 'rotate_right':
        ri.rotate_right()
    elif motion == 'rotate_left':
        ri.rotate_left()
    elif motion == 'none':
        ri.stop_motion()
    else:
        ri.stop_motion()
