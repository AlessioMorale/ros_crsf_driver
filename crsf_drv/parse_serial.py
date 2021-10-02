#!/usr/bin/env python3
from operator import contains
import os
from typing import Container, final
from crsf_parser import CRSFParser
from serial import Serial


def print_frame(frame: Container) -> None:
    print(frame)


crsf_parser = CRSFParser(print_frame)

with Serial("/dev/ttyUSB0", 425000, timeout=2) as ser:
    input = bytearray()
    while True:
        values = ser.read(100)
        input.extend(values)
        crsf_parser.parse_stream(input)
