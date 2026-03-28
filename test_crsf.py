import serial
import time

# --- Configuration ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 420000 
FREQ_HZ = 200      

# --- CRSF Helpers ---
def crsf_crc8(data):
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def pack_crsf_channels(channels_16):
    bits = 0
    bit_count = 0
    buffer = bytearray()
    for ch in channels_16:
        bits |= (ch & 0x7FF) << bit_count
        bit_count += 11
        while bit_count >= 8:
            buffer.append(bits & 0xFF)
            bits >>= 8
            bit_count -= 8
    return buffer

def create_crsf_packet(channels):
    header = bytearray([0xC8, 24, 0x16])
    payload = pack_crsf_channels(channels)
    packet = header + payload
    crc = crsf_crc8(packet[2:])
    packet.append(crc)
    return packet

# --- Main Test Loop ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
    print(f"Connected to Pico on {SERIAL_PORT}")
    
    start_time = time.time()
    last_print_time = 0  # Tracker for the 10s log

    while True:
        loop_start = time.time()
        
        # 1. Generate oscillating value for Roll (Channel 0)
        elapsed = time.time() - start_time
        val = int(992 + 819 * (0.5 * (1 + 0.5 * (elapsed % 2))))
        
        # 2. Build channels
        channels = [992] * 16
        channels[0] = val   
        channels[4] = 1811  # AUX1 ARM
        
        # 3. Create and Send packet
        packet = create_crsf_packet(channels)
        ser.write(packet)
        
        # 4. Periodic Logging (Every 10 seconds)
        if loop_start - last_print_time >= 10.0:
            # Format bytes as 0xEE 0x18 ...
            hex_formatted = " ".join(f"0x{b:02X}" for b in packet)
            print(f"\n[{time.strftime('%H:%M:%S')}] Current Packet:\n{hex_formatted}")
            last_print_time = loop_start

        # 5. Check for Telemetry coming back
        if ser.in_waiting:
            telemetry = ser.read(ser.in_waiting)
            # Just a quiet indicator that telemetry is flowing
            print(".", end="", flush=True) 

        # 6. Maintain Frequency (100Hz)
        sleep_time = (1.0 / FREQ_HZ) - (time.time() - loop_start)
        if sleep_time > 0:
            time.sleep(sleep_time)

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt:
    print("\nStopping test...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
