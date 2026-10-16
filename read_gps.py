import serial
import pynmea2

port = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=1)

try:
    while True:
        try:
            line = port.readline().decode('ascii', errors='replace').strip()

            if not line:
                continue

            if line.startswith('$GPGGA'):
                msg = pynmea2.parse(line)
                if msg.latitude:
                    print(f"Latitude: {msg.latitude} {msg.lat_dir}, Longitude: {msg.longitude} {msg.lon_dir}")
                else:
                    print("No fix yet...")

        except pynmea2.ParseError:
            continue

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    port.close()
