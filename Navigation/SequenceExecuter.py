#!/usr/bin/env python3.8
from WheelLoaderController4 import Controller, Observer
from WheelLoaderCommunication import WheelLoaderCommunication
from time import sleep
import sys

def mapValue(val, max, min):
    if val > max:
        return 0xff
    if val < min:
        return 0
    return int(0xff*(val-min)//(max-min))

def main():
    if len(sys.argv) == 1:
        sys.exit("No inputfile given")
    try:
        f = open(sys.argv[1], 'r')
    except FileNotFoundError:
        sys.exit("No such inputfile exist")
    if(len(sys.argv) == 2):
        port = "COM11"
        print("Default port set to " + port + ". Enter COM port, that the microcontroller is connected to, as argument when running the script, e.g. ")
        print("> " + sys.argv[0] + " COM11")
        print("For nvidia, ender: '/dev/ttyTHS0' as argument when using the GPIO uart pins\n")
    else:
        port = sys.argv[2]
    U = WheelLoaderCommunication(port)
    if(not U.running):
        sys.exit("Could not open serial port.")

    U.startLoggingTX(sys.argv[1] + "_TX_Log.csv")

    print("Running program: " + sys.argv[0] + ", reading sequence from: " + sys.argv[1] + ", Port: " + port)
    C = Controller(U)
    O = Observer(U)
    C.startThread()
    O.startThread()
    O.StartListning()
    O.startLogging(sys.argv[1] + "_RX_Log.csv")
    instruction = f.readline().split(',')

    while len(instruction) > 1:
        #print(instruction)
        try:
            C.velocity = mapValue(float(instruction[0]), 0.2199, -0.2199)
            C.steering_ang = mapValue(float(instruction[1]), 12.903, -12.903)
            C.bucket_ang = mapValue(float(instruction[2]), 60, -46)
            C.bucket_height = mapValue(float(instruction[3]), 477, -75)
            C.scissor_ang = mapValue(float(instruction[4]), 35, 0)
        except (ValueError):
            instruction = f.readline().split(',')
            continue

        if C.mode != 1:
            C.Run(1)

        sleep(float(instruction[5]))

        instruction = f.readline().split(',')
    U.stopLoggingTX()
    O.stopLogging()

if __name__ == '__main__':
    main()
