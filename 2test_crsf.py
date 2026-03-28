import serial
import time
import random

PORT = "/dev/ttyUSB0"   # Твой UART-адаптер
BAUD = 420000 

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc

def pack_channels(ch):
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
    ch = [1800] * 16  
    ch[2] = random.randint(172, 1811)
    ch[13] = 1811
    ch[4] = 1811
    return ch

def parse_crsf(raw_buffer):
    """ Разбирает буфер и возвращает список найденных валидных пакетов """
    packets = []
    while len(raw_buffer) >= 4:
        # Ищем заголовок (Address)
        if raw_buffer[0] in [0xC8, 0xEE, 0xEA, 0xEC]:
            length = raw_buffer[1]
            if length < 2 or length > 62: # CRSF max frame size check
                raw_buffer.pop(0)
                continue
            
            if len(raw_buffer) >= length + 2:
                packet = raw_buffer[:length + 2]
                payload = packet[2:-1]
                expected_crc = packet[-1]
                
                if crc8(payload) == expected_crc:
                    packets.append(packet)
                    del raw_buffer[:length + 2] # Удаляем обработанный пакет
                    continue
                else:
                    # CRC не совпал, возможно это не начало пакета
                    raw_buffer.pop(0)
            else:
                break # Ждем догрузки данных
        else:
            raw_buffer.pop(0) # Мусор в начале, удаляем по байту
            
    return packets

def run_test():
    # timeout=0 для неблокирующего чтения
    ser = serial.Serial(PORT, BAUD, timeout=0)
    
    tx_interval = 0.005 # 200 Hz
    next_tx_time = time.perf_counter()
    
    rx_raw_storage = bytearray()
    print(f"[SYSTEM] Starting Hardware UART Test on {PORT}...")

    while True:
        current_time = time.perf_counter()

        # 1. ЧТЕНИЕ И ПАРСИНГ
        if ser.in_waiting > 0:
            rx_raw_storage.extend(ser.read(ser.in_waiting))
            
            valid_packets = parse_crsf(rx_raw_storage)
            for p in valid_packets:
                # Выводим тип пакета (байт 2) и данные в HEX
                print(f"RX Packet [Type {p[2]:02x}]: {p.hex(' ')}")

        # 2. ПЕРЕДАЧА (200Hz)
        if current_time >= next_tx_time:
            frame = pack_channels(make_channels())
            try:
                ser.write(frame)
            except Exception as e:
                print(f"TX Error: {e}")
                
            next_tx_time += tx_interval

        # Чтобы не грузить ядро на 100%
        time.sleep(0.0001)

if __name__ == "__main__":
    try:
        run_test()
    except KeyboardInterrupt:
        print("\nStopped by user")
