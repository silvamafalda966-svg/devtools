"""HC-06 BT diagnostic: scan COM9/COM10 multi-baud"""
import serial
import struct
import time
import sys
import io

# Fix Windows GBK encoding
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

COM_PORTS = ["COM9", "COM10"]
BAUD_RATES = [9600, 19200, 38400, 57600, 115200]
TEST_TIME = 3

def try_port(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.3)
        ser.reset_input_buffer()

        start = time.time()
        raw = bytearray()
        while time.time() - start < TEST_TIME:
            n = ser.in_waiting
            if n:
                chunk = ser.read(n)
                raw.extend(chunk)
            time.sleep(0.05)

        ser.close()

        if len(raw) > 0:
            print(f"  [OK] {port} @ {baud}: received {len(raw)} bytes")

            # Show first 64 bytes as HEX
            for i in range(0, min(64, len(raw)), 32):
                print(f"     [{i:04x}]: {raw[i:i+32].hex(' ')}")

            # Search for 0xAA frame headers
            aa_positions = [i for i in range(len(raw)-1) if raw[i] == 0xAA]
            if aa_positions:
                print(f"     [AA] found {len(aa_positions)} x 0xAA, positions: {aa_positions[:10]}")

            # Try JustFloat parsing
            for offset in range(min(4, len(raw) - 31)):
                floats = []
                for i in range(8):
                    try:
                        f = struct.unpack('>f', raw[offset + i*4 : offset + i*4 + 4])[0]
                    except:
                        break
                    floats.append(f)
                if len(floats) == 8:
                    if 0 <= floats[5] <= 500 and 0 <= floats[4] <= 256:
                        print(f"     [JF] offset={offset} JustFloat: {['{:.2f}'.format(f) for f in floats]}")
                        break
            return True
        else:
            print(f"  [NO] {port} @ {baud}: no data")
            return False

    except serial.SerialException as e:
        msg = str(e)
        if "denied" in msg.lower() or "used" in msg.lower():
            print(f"  [BUSY] {port} @ {baud}: port busy")
        else:
            print(f"  [ERR] {port} @ {baud}: {e}")
        return False

if __name__ == "__main__":
    print("=" * 60)
    print("HC-06 BT Diagnostic")
    print("=" * 60)

    for port in COM_PORTS:
        print(f"\n--- Scanning {port} ---")
        baud_ok = None
        for baud in BAUD_RATES:
            ok = try_port(port, baud)
            if ok:
                baud_ok = baud

        if baud_ok is None:
            print(f"  >> {port}: NO DATA at any baud rate")
        else:
            print(f"  >> {port}: best baud = {baud_ok}")

    print("\n" + "=" * 60)
    print("Done")
    print("=" * 60)
