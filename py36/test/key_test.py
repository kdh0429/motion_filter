from pynput import keyboard
import time
import sys

flag = True

def on_press(key):
    try:
        print('alphanumeric key {0} pressed'.format(
            key.char))
    except AttributeError:
        print('special key {0} pressed'.format(
            key))

def on_release(key):
    global flag
    print('{0} released'.format(
        key))
    if key == keyboard.Key.esc:
        # Stop listener
        flag = False

# Collect events until released
# with keyboard.Listener(
#         on_press=on_press,
#         on_release=on_release) as listener:
#     listener.join()

# ...or, in a non-blocking fashion:
listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

while flag:
    # print("do_something")
    time.sleep(0.01)