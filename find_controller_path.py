from evdev import InputDevice, list_devices

devices = [InputDevice(path) for path in list_devices()]
for dev in devices:
    print(dev.path, dev.name)
