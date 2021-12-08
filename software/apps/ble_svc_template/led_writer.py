#!/usr/bin/env python3

import struct
from bluepy.btle import Peripheral, DefaultDelegate
import argparse
import time

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
addr1 = 'C0:98:E5:49:01:44'
addr2 = 'C0:98:E5:49:01:45'

addresses = [addr1, addr2]

# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

LED_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
LED_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"


try:
    print("connecting")
    # bucklers = [Peripheral(a) for a in addresses]
    b45 = Peripheral(addr2)
    print("connected45")
    b44 = Peripheral(addr1)
    print("connected44")
    bucklers = [b44, b45]
    #characteristics = buckler.getCharacteristics(startHnd=0x0019, endHnd=0xFFFF, uuid=None)
    #for characteristic in characteristics:
    #    print("{}, hnd={}, supports {}".format(characteristic, hex(characteristic.handle),  characteristic.propertiesToString()))
    # Get service
    # sv = buckler.getServiceByUUID(LED_SERVICE_UUID)
    svs = [b.getServiceByUUID(LED_SERVICE_UUID) for b in bucklers]
    # Get characteristic
    # ch = sv.getCharacteristics(LED_CHAR_UUID)[0]
    chs = [s.getCharacteristics(LED_CHAR_UUID)[0] for s in svs] 

    while True:
        #input("Press any key to toggle the LED")
        time.sleep(5)
        for ch in chs:
            char = ord(ch.read())

            led_state = bool(char)
            print(led_state)
            ch.write(bytearray([not led_state]))
finally:
    for b in bucklers:
        b.disconnect()
