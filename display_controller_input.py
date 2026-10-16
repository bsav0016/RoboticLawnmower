from evdev import InputDevice, categorize, ecodes, list_devices

# Find the controller
def find_controller():
    keywords = ["zikway", "gamesir", "gamepad"]
    for path in list_devices():
        try:
            dev = InputDevice(path)
            name = dev.name.lower()

            if "keyboard" in name:
                continue

            caps = dev.capabilities()
            abs_axes = caps.get(ecodes.EV_ABS,[])

            if any(k in name for k in keywords) and len(abs_axes) >= 4:
                return dev
        except:
            continue
    return None

# Use the correct device path
gamepad = find_controller()
if gamepad is None:
    print("Controller not found!")
    exit(1)

print(f"Listening to {gamepad.name} at {gamepad.path}...\n")

for event in gamepad.read_loop():
    if event.type == ecodes.EV_ABS:
        absevent = categorize(event)
	
        print(f"Analog: {ecodes.ABS[event.code]} = {event.value}")
    elif event.type == ecodes.EV_KEY:
        keyevent = categorize(event)
        print(f"Button: {keyevent.keycode} {'pressed' if keyevent.keystate == 1 else 'released'}")
