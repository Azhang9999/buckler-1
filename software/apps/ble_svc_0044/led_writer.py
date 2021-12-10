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
addresses = [addr2]

# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

BUCKLER_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DATA_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"
INSTRUCTION_CHAR_UUID    = "32e6108b-2b22-4db5-a914-43ce41986c70"

def connect_to_bucklers(bucklers):
    for i in range(len(bucklers)):
        state = ""
        try:
            state = bucklers[i].getState()
        except bluepy.btle.BTLEInternalError:
            pass
        if state != 'conn':
            buckler[i].connect(addresses[i])
    return bucklers

def unpack(data):
    return struct.unpack('f', data[0:4]) + struct.unpack('f', data[4:8]) + struct.unpack('f', data[8:12])

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
            char = unpack(data)
            #char = [int(d) for d in data]
            print(char)
        for ch in instruction_chs:
            ch.write(bytearray([i, i+1, i+2]))
        i+=1

def main():
    bucklers = [Peripheral() for a in addresses]
    while True:
        try:
            print("connecting")
            bucklers = connect_to_bucklers(bucklers)
 
            run(bucklers)
            
        except bluepy.btle.BTLEDisconnectError:
            print("Disconnected. Trying to Reconnect")
            
    for b in bucklers:
        b.disconnect()

main()
           #characteristics = buckler.getCharacteristics(startHnd=0x0019, endHnd=0xFFFF, uuid=None)
            #for characteristic in characteristics:
            #    print("{}, hnd={}, supports {}".format(characteristic, hex(characteristic.handle),  characteristic.propertiesToString()))
            # Get service
            # sv = buckler.getServiceByUUID(LED_SERVICE_UUID)