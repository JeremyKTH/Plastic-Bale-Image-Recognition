from time import sleep
import sys
import struct
import threading
from UART import UART

pi = 3.141592


class Controller:
    """docstring for Controller.
    state   meaning
    0       stopped, no data is sent (ready in manual mode)
    1       running in manual mode
    2       ready in autonomous mode
    3       running in autonomous mode"""

    def __init__(self, uart):
        self.Uart = uart
        self.mode = self.bucket_ang = self.bucket_height = self.scissor_ang = 0
        self.TXThreadPeriod = 90
        self.TX = threading.Thread(target=self.TXThread, args=(1,), daemon=True)
        self.velocity = self.steering_ang  = 128
        

    def startThread(self):
        self.TX.start()

    def TXThread(self, name):
        global TXThreadPeriod, RXThreadPeriod
        global velocity, steering_ang, arm_ang, bucket_ang, scissor_ang
        print("TXThread started")
        while 1:
            # print(self.mode)
            if self.mode == 1:  #manual mode
                data = bytes([self.velocity, self.steering_ang, self.bucket_ang, self.bucket_height, self.scissor_ang])
                try:
                    self.Uart.SendCommand(1, data)
                    # print("sent data")
                except ValueError:
                    print("Value error! Value(s) given were out of range")
                    self.Stop()
            elif self.mode >= 2:    #ready in autonomous mode
                self.Uart.SendEnableSignal()
                if self.mode == 3:  #running in autonomous mode
                    sleep(0.02)
                    try:
                        data = bytes([self.velocity, self.steering_ang, self.bucket_ang, self.bucket_height, self.scissor_ang])
                        self.Uart.SendCommand(3, data)
                    except ValueError:
                        print("Value error! Value(s) given were out of range")
                        self.Stop()
            sleep(self.TXThreadPeriod*0.001) # Do nothing


    def Run(self, mode):
        self.mode = mode
        if self.mode:
            print("Communication started. Running mode: " + str(self.mode))
        else:
            self.Stop()

    def Stop(self):
        self.mode = 0
        print("Communication stopped")

    def LoadAllArg(self,inputArgs):
        self.velocity = int(inputArgs[1])
        self.steering_ang = int(inputArgs[2])
        self.bucket_ang = int(inputArgs[3])
        self.bucket_height = int(inputArgs[4])
        self.scissor_ang = int(inputArgs[5])
        self.TXThreadPeriod = int(inputArgs[6])
        print("All variable loaded")

class Observer:
    """docstring for Observer.
    state   meaning
    0       stopped, no data is sent (ready in manual mode)
    1       running in manual mode
    2       ready in autonomous mode
    3       running in autonomous mode"""

    def __init__(self, uart):
        self.Uart = uart
        self.listen = False
        self.RXThreadPeriod = 90
        self.RX = threading.Thread(target=self.RXThread, args=(1,), daemon=True)
        self.X = self.Y = self.velocity_actual = self.Theta = self.sens1 = self.sens2 = self.sens3 = self.mode = 0
        self.steering_ang_actual = self.bucket_ang_actual = self.bucket_height_actual = self.scissor_ang_actual = 0
    #sen1: sen2: sen3:       
        
    def startThread(self):
        self.RX.start()

    def StartListning(self):
        self.listen = True
        print("Start listening")

    def StopListning(self):
        self.listen = False
        print("Stop listening")

    def RXThread(self, name):
        self.Uart.Ser.flush()
        while 1:
            if self.listen:
                # print("listening")
                if self.Uart.ReadMessage():
                    self.X = struct.unpack('f', self.Uart.data[0:4])
                    self.Y = struct.unpack('f', self.Uart.data[4:8])
                    self.Theta = struct.unpack('f', self.Uart.data[8:12])
                    self.velocity_actual = self.Uart.data[12]
                    self.steering_ang_actual = self.Uart.data[13]
                    self.bucket_ang_actual = self.Uart.data[14]
                    self.bucket_height_actual = self.Uart.data[15]
                    self.scissor_ang_actual = self.Uart.data[16]
                    self.sens1 = struct.unpack('H', self.Uart.data[17:19])
                    self.sens2 = self.Uart.data[19]
                    self.sens3 = self.Uart.data[20]
                    self.mode = self.Uart.data[21]
                    print('Sq#' + str(self.Uart.rxSeq) + ' X={:.2f} Y={:.2f} Theta={:.2f} velocity={} steering angle={} bucket angle={} bucket height={} scissor angle={} sens1={} sens2={} sens3={} mode={}'\
                    .format(self.X[0], self.Y[0], self.Theta[0], self.velocity_actual, self.steering_ang_actual, self.bucket_ang_actual, self.bucket_height_actual, self.scissor_ang_actual, self.sens1[0], self.sens2, self.sens3, self.mode))
            else:
                sleep(self.RXThreadPeriod/1000)


def main():
    print("S - stop")
    print("R <X [m]> <Y [m]> <Theta [rad]> - Reset coordinate system")
    print("G - Get current state of vehicle")
    print("C <velocity> <steering angle> <arm angle> <bucket height> <scissor angle> <period [ms]> - send periodic control messages")
    print("A <period [ms]> - enable autonomous mode (awaits external control commands)")
    print("A <velocity [m/s]> <steering angle> <bucket angle> <bucket height> <scissor angle> <period [ms]> - send periodic AS control messages")
    print("L <period (optional)> - toggle UART listner")
    print("T <message up to 63 characters> - Send communication test message")
    print("Q - quit program")

    U = UART("COM4")
    if(not U.running):
        sys.exit("Could not open serial port.")

    C = Controller(U)
    O = Observer(U)
    C.startThread()
    O.startThread()
    command = ""
    inputArgs = []
    inputStr = ""
    while command != "Q":
        inputStr = input()
        inputArgs = inputStr.split(" ")
        command = inputArgs[0]
        if command == "S":
            C.Stop()                     # Stops the thread
        elif command == "R":
            if len(inputArgs) != 4:
                print("Invalid input. Try:")
                print("R <X [m]> <Y [m]> <Theta [rad]>")
                continue
            x, y, theta = [float(inputArgs[i]) for i in range(1,4)]
            data = struct.pack('fff', x, y, theta)
            U.SendCommand(0, data)
        elif command == "G":
            if len(inputArgs) != 1:
                print("Invalid input. Try:")
                print("G")
                continue
            U.SendCommand(4, b'')
        elif command == "C":
            if len(inputArgs) != 7:
                print("Invalid input. Try:")
                print("C <velocity [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]>")
                continue
            C.LoadAllArg(inputArgs)
            if C.mode != 1:
                C.Run(1)                                                        # Starts the thread
        elif command == "A":
            if len(inputArgs) == 2:
                C.TXThreadPeriod = int(inputArgs[1])
                C.Run(2)
            elif len(inputArgs) == 7:
                # velocity = struct.pack('i', int(inputArgs[1]))[0]
                # steering_ang = struct.pack('i', int(inputArgs[2]))[0]
                C.LoadAllArg(inputArgs)
                C.Run(3)
            else:
                print("Invalid input. Try:")
                print("A <velocity [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]> or A <period [ms]>")
                continue
        elif command == "L":
            if O.listen:
                O.StopListning()
            else:
                O.StartListning()
            if len(inputArgs) > 1:
                O.RXThreadPeriod = int(inputArgs[1])
        elif command == "T":
            if len(inputStr[2:]) > 63:
                print("Message too long")
            else:
                U.SendCommTest(inputStr[2:])

if __name__ == '__main__':
    main()
