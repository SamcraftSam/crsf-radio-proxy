import serial
import time
import random

# --- Configuration ---
PORT = "/dev/ttyACM0"
BAUD = 420000
TX_FREQ = 200  # Hz
TX_INTERVAL = 1.0 / TX_FREQ  # 0.005 seconds

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def pack_channels(ch):
    # CRSF Frame: [Addr] [Len] [Type] [Payload...] [CRC]
    # Payload for RC Channels is 22 bytes
    buf = bytearray([0xC8, 24, 0x16])
    bitbuf = 0
    bits = 0
    for c in ch:
        bitbuf |= (c & 0x7FF) << bits
        bits += 11
        while bits >= 8:
            buf.append(bitbuf & 0xFF)
            bitbuf >>= 8
            bits -= 8
    buf.append(crc8(buf[2:]))
    return buf

def make_channels():
    # Standard CRSF range is 172 to 1811 (1000-2000us)
    # Midpoint is 992
    ch = [992] * 16  
    ch[2] = random.randint(800, 1200) # Slight jitter on Throttle/Pitch for testing
    ch[13] = 1811
    ch[4] = 1811
    return ch

def run_tx_only():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0)
        # Flush buffers to start clean
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception as e:
        print(f"[ERROR] Could not open {PORT}: {e}")
        return

    print(f"[SYSTEM] Starting 200Hz TX-only test on {PORT}...")
    print("Press Ctrl+C to stop.")

    next_tx_time = time.perf_counter()
    packet_count = 0
    start_time = time.perf_counter()

    try:
        while True:
            current_time = time.perf_counter()

            if current_time >= next_tx_time:
                # 1. Generate Data
                channels = make_channels()
                frame = pack_channels(channels)

                # 2. Send over USB
                ser.write(frame)
                packet_count += 1

                # 3. Calculate next interval
                next_tx_time += TX_INTERVAL

                # Print stats every 2 seconds
                if packet_count % (TX_FREQ * 2) == 0:
                    elapsed = time.perf_counter() - start_time
                    print(f"[STATS] Sent {packet_count} packets. Avg Rate: {packet_count/elapsed:.2f} Hz")

            # 4. Precision Sleep
            # We sleep until we are very close to the target, then busy-wait
            time_to_wait = next_tx_time - time.perf_counter()
            if time_to_wait > 0.001:
                time.sleep(time_to_wait - 0.0005)

    except KeyboardInterrupt:
        print(f"\n[STOPPED] Sent {packet_count} total packets.")
    finally:
        ser.close()

if __name__ == "__main__":
    run_tx_only()
