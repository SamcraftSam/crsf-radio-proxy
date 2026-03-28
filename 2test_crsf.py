import serial
import time
import struct
import random

PORT = "/dev/ttyACM0"   # change this
BAUD = 115200   # USB CDC, not 420k

ser = serial.Serial(PORT, BAUD, timeout=0.1)


# --- CRC8 (same as CRSF) ---
def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# --- Pack 16 channels (11-bit) ---
def pack_channels(ch):
    buf = bytearray()

    buf.append(0xC8)        # address
    buf.append(24)          # length
    buf.append(0x16)        # RC_CHANNELS_PACKED

    bitbuf = 0
    bits = 0

    for c in ch:
        bitbuf |= (c & 0x7FF) << bits
        bits += 11

        while bits >= 8:
            buf.append(bitbuf & 0xFF)
            bitbuf >>= 8
            bits -= 8

    # CRC
    crc = crc8(buf[2:])
    buf.append(crc)

    return buf


# --- Example channels ---
def make_channels():
    ch = [992] * 16  # mid everywhere
    
    # throttle = channel 2 in CRSF (0–1984 range typically)
    ch[2] = random.randint(172, 1811)  # real usable throttle range
    
    return ch

# --- Main loop ---
while True:
    # send RC frame
    frame = pack_channels(make_channels())
    ser.write(frame)

    # read telemetry
    data = ser.read(256)
    if data:
        print("RX:", data.hex(" "))

    time.sleep(0.005)  # 200 Hz
