#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Raspberry Pi to Arduino I2C Communication
#i2cdetect -y 1

#library
import sys
import smbus2 as smbus#,smbus2
import time
import subprocess

# Slave Addresses
I2C_SLAVE_ADDRESS = 8 #0x0b ou 11
subprocess.run(['i2cdetect', '-y', '1'])

# This function converts a string to an array of bytes.
def ConvertStringsToBytes(src):
  converted = []
  for b in src:
    converted.append(ord(b))
  return converted

def convertData(data):
    toAvoid = chr(255)
    message = ""
    for i in data:
        message = message + chr(i)
    numbers = message.split("/")
    x = float(numbers[0])
    y = float(numbers[1])
    theta = float(numbers[2].split(toAvoid)[0])
    print(x, y, theta)

def main(args):
    # Create the I2C bus
    I2Cbus = smbus.SMBus(1)
    with smbus.SMBus(1) as I2Cbus:
        slaveAddress = I2C_SLAVE_ADDRESS
        #V = float(input("Enter linear: "))
        V = 2.4
        W = -3.6
        #W = float(input("Enter angular: "))
        cmd = str(round(V,2)) + "/" + str(round(W,2))

        BytesToSend = ConvertStringsToBytes(cmd)
        print("Sent " + str(slaveAddress) + " the " + str(cmd) + " command.")
        print(BytesToSend )
        I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
        time.sleep(0.5)

        while True:
            V = float(input("Enter linear: "))
            W = float(input("Enter angular: "))
            cmd = str(round(V,2)) + "/" + str(round(W,2))

            BytesToSend = ConvertStringsToBytes(cmd)
            print("Sent " + str(slaveAddress) + " the " + str(cmd) + " command.")
            print(BytesToSend )
            I2Cbus.write_i2c_block_data(slaveAddress, 0x00, BytesToSend)
            time.sleep(0.2)
            try:
                data=I2Cbus.read_i2c_block_data(slaveAddress,0x00,20)
                #print("recieve from slave")
                #print(type(data), data)
                convertData(data)
            except Exception as e:
                print(e)
                #subprocess.run(['i2cdetect', '-y', '1'], stdout=subprocess.DEVNULL)
                #print("remote i/o error")
                #time.sleep(1.5)
            time.sleep(0.2)
    return 0

if __name__ == '__main__':
     try:
        main(sys.argv)
     except KeyboardInterrupt:
        print("program was stopped manually")
     #input()
