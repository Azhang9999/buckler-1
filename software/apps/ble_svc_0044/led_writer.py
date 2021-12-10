#!/usr/bin/env python3

import struct
from bluepy.btle import Peripheral, DefaultDelegate
import bluepy
import argparse
import time

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
addr1 = 'C0:98:E5:49:00:44'
addr2 = 'C0:98:E5:49:00:45'

addresses = [addr1, addr2]

# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

BUCKLER_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DATA_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"
INSTRUCTION_CHAR_UUID    = "32e6108b-2b22-4db5-a914-43ce41986c70"

def connect_to_peripheral(address):
    print("Connecting to " + address)
    periph = Peripheral(address)
    print("Connected to " + address)
    return periph


def run(bucklers):
    svs = [b.getServiceByUUID(BUCKLER_SERVICE_UUID) for b in bucklers]
    # Get characteristic
    # ch = sv.getCharacteristics(LED_CHAR_UUID)[0]
    data_chs = [s.getCharacteristics(DATA_CHAR_UUID)[0] for s in svs] 
    instruction_chs = [s.getCharacteristics(INSTRUCTION_CHAR_UUID)[0] for s in svs] 

    i = 0
    while True:
        #input("Press any key to toggle the LED")
        # time.sleep(0.5)
        for ch in data_chs:
            data = ch.read()
            print(data)
            char = sturct.unpack(data)
            #char = [int(d) for d in data]
            print(char)
        for ch in instruction_chs:
            ch.write(bytearray([i, i+1, i+2]))
        i+=1


try:
    print("connecting")
    bucklers = [connect_to_peripheral(a) for a in addresses]

    #characteristics = buckler.getCharacteristics(startHnd=0x0019, endHnd=0xFFFF, uuid=None)
    #for characteristic in characteristics:
    #    print("{}, hnd={}, supports {}".format(characteristic, hex(characteristic.handle),  characteristic.propertiesToString()))
    # Get service
    # sv = buckler.getServiceByUUID(LED_SERVICE_UUID)
    run(bucklers)
    
except bluepy.btle.BTLEDisconnectError:
    print("Disconnected. Trying to Reconnect")
    bucklers = [connect_to_peripheral(a) for a in addresses]
    run(bucklers)

finally:
    for b in bucklers:
        b.disconnect()
