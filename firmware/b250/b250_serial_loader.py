#!/usr/bin/python

import serial
import sys
import binascii

import select
def has_input(): return select.select([sys.stdin,],[],[],0.0)[0]

if __name__ == '__main__':
    tty_dev = sys.argv[1]
    bin_file = sys.argv[2]

    #parse bin file into hex lines
    h = binascii.hexlify(open(bin_file).read()) + '0'*7
    d = [h[i*8:(i+1)*8] for i in range(len(h)/8)]

    ser = serial.Serial(tty_dev, 115200, timeout=1.0)
    ser.write('\r\n\r\n') #for into known state
    ser.flush()

    for i, data in enumerate(d):
        addr = i*4 + 0x4000
        command = '%.8x%s\r\n'%(addr, data)
        if (addr & 0x1ff) == 0: print 'loading block -- 0x%.8x'%addr
        #print command.strip()
        ser.write(command)
        result = ser.readline()
        assert command.strip() == result.strip()

    print "Reading serial port will continue: Press ANYKEY to exit:\n"
    while not has_input():
        sys.stdout.write(ser.readline())
        sys.stdout.flush()
