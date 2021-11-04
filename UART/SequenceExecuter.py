from WheelLoaderController3 import Controller, Observer
from UART import UART
from time import sleep
import sys

def mapValue(val, max, min):
    if val > max:
        return 0xff
    if val < min:
        return 0
    return int(0xff*(val-min)//(max-min))

def main():
    print(sys.argv)
    if len(sys.argv) == 1:
        sys.exit("No inputfile given")
    try:
        f = open(sys.argv[1], 'r')
    except FileNotFoundError:
        sys.exit("No such inputfile exist")

    U = UART("COM4")
    if(not U.running):
        sys.exit("Could not open serial port.")

    C = Controller(U)
    O = Observer(U)
    C.startThread()
    O.startThread()
    O.StartListning()
    instruction = f.readline().split(',')

    while len(instruction) > 1:
        print(instruction)
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

if __name__ == '__main__':
    main()
