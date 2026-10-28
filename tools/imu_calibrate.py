#!/usr/bin/env python3
"""Legge la telemetria dell'ESP32 da seriale e stampa media e deviazione standard di IMU.

Solo lettura: non invia comandi (i motori restano fermi per il timeout del firmware).
Il nodo ROS deve essere fermo, perché la porta seriale è esclusiva.

Uso sul Pi:
    python3 tools/imu_calibrate.py --seconds 5 --label flat
"""
import argparse
import math
import os
import struct
import termios
import time

# Deve combaciare con TelemetryPacket in include/serial_protocol.h (27 byte, little-endian)
TELEMETRY_FORMAT = "<BBiihhhhhhBHBB"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)
SYNC = b"\xaa\x55"
STATUS_BIT_IMU_OK = 1 << 3


def open_raw(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                                 # iflag: raw
    attrs[1] = 0                                                 # oflag: raw
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL      # niente HUPCL
    attrs[3] = 0                                                 # lflag: raw
    attrs[4] = attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)
    return fd


def parse_packets(buffer):
    """Estrae i pacchetti validi dal buffer; restituisce (pacchetti, buffer residuo)."""
    packets = []
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            return packets, buffer[-1:]
        if len(buffer) - start < TELEMETRY_SIZE:
            return packets, buffer[start:]
        frame = buffer[start:start + TELEMETRY_SIZE]
        checksum = 0
        for byte in frame[:-2]:
            checksum ^= byte
        if frame[-1] == 0x0D and frame[-2] == checksum:
            packets.append(struct.unpack(TELEMETRY_FORMAT, frame))
            buffer = buffer[start + TELEMETRY_SIZE:]
        else:
            buffer = buffer[start + 1:]


def mean_std(values):
    mean = sum(values) / len(values)
    return mean, math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/lidarino")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--label", default="", help="nome della posizione, solo per l'output")
    args = parser.parse_args()

    fd = open_raw(args.port)
    buffer = b""
    packets = []
    end = time.time() + args.seconds
    try:
        while time.time() < end:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            if not chunk:
                raise SystemExit("Porta chiusa (hangup): l'ESP32 si è resettato?")
            new_packets, buffer = parse_packets(buffer + chunk)
            packets.extend(new_packets)
    finally:
        os.close(fd)

    if not packets:
        raise SystemExit("Nessun pacchetto valido ricevuto")

    imu_ok = sum(1 for p in packets if p[10] & STATUS_BIT_IMU_OK)
    print(f"[{args.label}] {len(packets)} pacchetti in {args.seconds:.0f} s, IMU_OK in {imu_ok}/{len(packets)}")
    for name, index, unit in (("accel", 4, "m/s^2"), ("gyro", 7, "rad/s")):
        for axis in range(3):
            mean, std = mean_std([p[index + axis] / 1000.0 for p in packets])
            print(f"  {name}_{'xyz'[axis]} = {mean:+8.4f} {unit}   std {std:.4f}")
    accel_norm = math.sqrt(sum(mean_std([p[4 + a] / 1000.0 for p in packets])[0] ** 2 for a in range(3)))
    print(f"  |accel| = {accel_norm:.4f} m/s^2")


if __name__ == "__main__":
    main()
