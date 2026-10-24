#!/usr/bin/env python3
"""Local QLET simulator for transport tests; never connects to a physical payload."""
import argparse
import socket
import struct
import time


def state_packet(yaw=5.0, pitch=-30.0, last_command=0):
    payload = bytearray(60)
    payload[0:2] = bytes([3, 255])
    payload[6] = 4
    struct.pack_into('<hhh', payload, 7, round(yaw * 100), round(pitch * 100), 0)
    struct.pack_into('<H', payload, 23, 250)  # Visible focal length: 25mm.
    payload[46] = last_command
    packet = bytearray([0xFC, 0x2C, 0x55, 62, 0x14, 0]) + payload
    checksum = 0
    for value in packet[3:]:
        checksum ^= value
    packet.append(checksum)
    return bytes(packet)


def decode_command(packet):
    if len(packet) < 7 or packet[:3] != b'\xfb\x2c\xaa' or len(packet) != packet[3] + 5:
        raise ValueError('bad command frame length/header')
    checksum = 0
    for value in packet[3:-1]:
        checksum ^= value
    if checksum != packet[-1]:
        raise ValueError('bad command checksum')
    return struct.unpack_from('<H', packet, 4)[0], packet[6:-1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bind-port', type=int, default=19001)
    parser.add_argument('--controller-port', type=int, default=19000)
    args = parser.parse_args()
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', args.bind_port))
        sock.settimeout(0.02)
        last_command = 0
        while True:
            sock.sendto(state_packet(last_command=last_command), ('127.0.0.1', args.controller_port))
            try:
                data, _ = sock.recvfrom(65536)
                command, payload = decode_command(data)
                last_command = command & 255
                print(f'{time.monotonic():.6f} cmd=0x{command:04x} payload={payload.hex(" ")}', flush=True)
            except socket.timeout:
                pass


if __name__ == '__main__':
    main()
