#!/usr/bin/python

import serial
import sys
import binascii

if __name__ == '__main__':
    tty_dev = sys.argv[1]
    bin_file = sys.argv[2]

    #parse bin file into hex lines
    h = binascii.hexlify(open(bin_file).read()) + '0'*7
    d = [h[i*8:(i+1)*8] for i in range(len(h)/8)]

    ser = serial.Serial(tty_dev, 115200, timeout=1.0)

    for i, data in enumerate(d):
        addr = i*4 + 0x4000
        command = '%.8x%s\r\n'%(addr, data)
        print command.strip()
        ser.write(command)
        result = ser.readline()
        assert command.strip() == result.strip()

    print ser.readline()
