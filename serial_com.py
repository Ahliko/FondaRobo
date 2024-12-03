#!/usr/bin/python
# -*- coding: utf-8 -*-

import time

import serial


def open_serial(port, baud, timeout):
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    if ser.isOpen():
        return ser
    else:
        print("SERIAL ERROR")


def getPingMotorTrame(id: bytes):
    # trame = b'\xff\xff\x03\x01\x53'
    # print(trame)
    # return trame + bytes(calculate_bitwise_checksum(trame))

    packet = bytearray()  # Utiliser bytearray pour manipuler les octets

    packet.append(0xFF)  # Start delimiter
    packet.append(0xFF)  # Start delimiter
    packet.append(id)  # ID
    packet.append(0x00)  # Len
    packet.append(0x01)  # Frame Type

    # Calculer la longueur du paquet et mettre à jour les champs de longueur
    length = len(packet) - 3
    packet[3] = length & 0xFF  # Length LSB4
    # Calculer la somme de contrôle (Checksum)
    checksum = 0
    for i in range(2, len(packet)):
        checksum += packet[i]
    checksum = 0xFF - (checksum & 0xFF)
    packet.append(checksum)

    return packet


def getWriteMotorTrame(id: int, params: list[hex]):
    trame = [0xff, 0xff, hex(len(params) + 3), 0x03, hex(id), params]
    return trame + [calculate_checksum(trame)]


def getLedTrame(id, on):
    packet = bytearray()  # Utiliser bytearray pour manipuler les octets

    packet.append(0xFF)  # Start delimiter
    packet.append(0xFF)  # Start delimiter
    packet.append(id)  # ID
    packet.append(0x00)  # Len
    packet.append(0x03)  # Frame Type
    packet.append(0x19)
    if on:
        packet.append(0x01)
    else:
        packet.append(0x00)

    # Calculer la longueur du paquet et mettre à jour les champs de longueur
    length = len(packet) - 3
    packet[3] = length & 0xFF  # Length LSB4
    # Calculer la somme de contrôle (Checksum)
    checksum = 0
    for i in range(2, len(packet)):
        checksum += packet[i]
    checksum = ~(checksum) & 0xFF
    packet.append(checksum)

    return packet

def close(ser):
    ser.close()


def write_data(ser, data):
    ser.write(data)


def read_data(ser, size=1):
    return ser.read(size)


def calculate_bitwise_checksum(byte_list):
    # Initialiser le checksum
    # checksum = 0
    # # Calculer le checksum avec décalage de bits
    # for byte in byte_list:
    #     checksum ^= byte  # Opération XOR
    #     checksum = (checksum << 1) & 0xFF  # Décalage à gauche et maintien dans les 8 bits
    # return checksum
    pass

def calculate_checksum(byte_list):
    # Initialiser le checksum
    # checksum = 0
    # # Calculer la somme des octets
    # for byte in byte_list:
    #     checksum = (checksum + byte) % 256  # S'assurer que le checksum reste dans les 8 bits
    # return checksum
    pass

def list_to_hex_string(int_list):
    # hex_list = [format(x, '02X') for x in int_list]
    # hex_string = ''.join(hex_list)
    return bytes(int_list)

# def bitwise_and_bytes(a, b):
#     result_int = int.from_bytes(a, byteorder="big") & int.from_bytes(b, byteorder="big")
#     return result_int.to_bytes(max(len(a), len(b)), byteorder="big")

if __name__ == "__main__":
    # we open the port. Check that this is correct by running "ls /dev/ttyACM*" in a terminal
    serial_port = open_serial("/dev/ttyACM0", 1000000, timeout=0.1)

    on_or_off = False
    # we create the packet for a LED ON/OFF command and alternate between them
    try:
        while True:

            data_id = 43

            # print(f"data_id raw = {data_id}")
            on_or_off = not on_or_off
            data = getLedTrame(data_id, on_or_off)

            print(f"to be send = {data}")
            # trame = b'\xFF\xFF\x43\x02\x01\xBC'
            write_data(serial_port, data)
            data = getPingMotorTrame(data_id)
            write_data(serial_port, data)

            # read the status packet (size 6)
            d = read_data(serial_port, 6)
            # in_hex = hex(int.from_bytes(in_bin,byteorder='little'))
            # or in_hex = ser.read().hex()
            print(d)
            time.sleep(0.5)
    except KeyboardInterrupt:
        serial_port.close()

