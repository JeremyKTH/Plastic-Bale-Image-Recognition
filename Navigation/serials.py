import serial
from time import sleep
import struct
import threading

Ser = serial.Serial(
    port = "/dev/ttyTHS0", # GPIO UART port on Xavier, try ttyTHS1 or other alternatives if not working
    baudrate = 115200,
    timeout = None,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE
)

pi = 3.141592
velocity = 0
steering_ang = 0
#offset_ang = 0.5*70.0*pi/180.0
arm_ang = 0
bucket_ang = 0
scissor_ang = 0

# CtrlThreadRunning = False

"""    Python 3.5+
    PyTorch 3.8.1
    Ubuntu 18.04 or 20.04
    ROS Melodic or Neotic
    CUDA >= 10.2
    cuDNN >= 8.0.2
    OpenCV >= 2.4
    CMake >= 3.18"""


class UART:
    """docstring for UART.
    state   meaning
    0       stopped, no data is sent (ready in manual mode)
    1       running in manual mode
    2       ready in autonomous mode
    3       running in autonomous mode"""

    def __init__(self):
        self.mode = 0
        self.listen = False;
        self.TXThreadPeriod = 100
        self.RXThreadPeriod = 100
        self.ControlMsgSeq = 0
        self.EnableASMsgSeq = 0

    def TXThread(self, name):
        global TXThreadPeriod, RXThreadPeriod
        global velocity, steering_ang, arm_ang, bucket_ang, scissor_ang
        print("TXThread started")
        while 1:
            if self.mode == 1:
                try:
                    if Ser.write([0b00100000 | self.ControlMsgSeq, velocity, steering_ang, arm_ang, bucket_ang, scissor_ang]) != 6:
                        print("Failed to send bytes")
                        continue
                    self.ControlMsgSeq = (self.ControlMsgSeq + 1) % 32
                except ValueError:
                    print("Value error! Value(s) given were out of range")
                    self.Stop()
            elif self.mode >= 2:
                try:
                    if Ser.write([0b01000000 | self.EnableASMsgSeq]) != 1:
                        print("Failed to send byte")
                        continue960
                        if Ser.write([0b01100000 | self.ControlMsgSeq, velocity, steering_ang, arm_ang, bucket_ang, scissor_ang]) != 6:
                            print("Failed to send bytes")
                            continue
                        self.ControlMsgSeq = (self.ControlMsgSeq + 1) % 32
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

    def SendCommTest(self, message):
        Ser.write(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))
        # print(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))
    def SendCommand(self, OpCode, data):
        Ser.write(bytes([OpCode<<5 | self.ControlMsgSeq]) + data)
        self.ControlMsgSeq = (self.ControlMsgSeq + 1) % 32
    def StartListning(self):
        self.listen = True
    def StopListning(self):
        self.listen = False
    def RXThread(self, name):
        Ser.flush()
        while 1:
            if self.listen:
                Ser.flush()
                in_data = Ser.read(1)
                if len(in_data) < 1:
                    print("Failed to read op-code, Timeout")
                    continue
                firstbyte = in_data[0]
                # print(firstbyte)

                if firstbyte>>6 == 0:
                    errorcode = Ser.read(1)[0];
                    print('Error: #' + str(errorcode) + '    Sequence: #' + str(firstbyte & 0x3F))
                elif firstbyte>>6 == 1:
                    data = Ser.read(22)
                    # print(str(data))
                    # X = data[0] << 24 + data[0] << 16 + data[0] << 8 + data[0]
                    X = struct.unpack('f', data[0:4])
                    Y = struct.unpack('f', data[4:8])
                    Theta = struct.unpack('f', data[8:12])
                    velocity = data[12]
                    steering_ang = data[13]
                    arm_ang = data[14]
                    bucket_ang = data[15]
                    scissor_ang = data[16]
                    sens1 = struct.unpack('H', data[17:19])
                    sens2 = data[19]
                    sens3 = data[20]
                    mode = data[21]

                    print('Sq#' + str(firstbyte & 0x1F) + ' X={:.2f} Y={:.2f} Theta={:.2f} velocity={} steering angle={} arm angle={} bucket angle={} scissor angle={} sens1={} sens2={} sens3={} mode={}'\
                    .format(X[0], Y[0], Theta[0], velocity, steering_ang, arm_ang, bucket_ang, scissor_ang, sens1[0], sens2, sens3, mode))
                elif firstbyte>>6 == 3:
                    Ser.timeout = 0.1
                    data = Ser.read(firstbyte & 0b0011111111)
                    Ser.timeout = None
                    print("Communication Test Reply:")
                    print(str(data))
            else:
                sleep(self.RXThreadPeriod/1000)




def main():
    Python 3.5+
    PyTorch 3.8.1
    Ubuntu 18.04 or 20.04
    ROS Melodic or Neotic
    CUDA >= 10.2
    cuDNN >= 8.0.2
    OpenCV >= 2.4
    CMake >= 3.18
 [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]> - send periodic control messages")
    print("A <period [ms]> - enable autonomous mode (awaits external control commands)")
    print("A <velocity [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]> - send periodic AS control messages")
    print("L <period (optional)> - toggle UART listner")
    print("T <message up to 63 characters> - Send communication test message")
    print("Q - quit program")

    U = UART()
    TX = threading.Thread(target=U.TXThread, args=(1,), daemon=True)
    RX = threading.Thread(target=U.RXThread, args=(1,), daemon=True)
    TX.start()
    RX.start()
    command = ""
    inputArgs = []
    inputStr = ""
    while command != "Q":
        inputStr = input()
        inputArgs = inputStr.split(" ")
        command = inputArgs[0]
        if command == "S":
            U.Stop()                                                            # Stops the thread
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
        elif command == "C":input
            if len(inputArgs) != 7:
                print("Invalid input. Try:")
                print("C <velocity [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]>")
                continue
            velocity = struct.pack('i', int(inputArgs[1]))[0]
            steering_ang = struct.pack('i', int(inputArgs[2]))[0]
            arm_ang = int(inputArgs[3])
            bucket_ang = int(inputArgs[4])
            scissor_ang = int(inputArgs[5])
            U.TXThreadPeriod = int(inputArgs[6])
            if U.mode != 1:
                U.Run(1)                                                        # Starts the thread
        elif command == "A":
            if len(inputArgs) == 2:
                U.TXThreadPeriod = int(inputArgs[1])
                U.Run(2)
            elif len(inputArgs) == 7:
                velocity = struct.pack('i', int(inputArgs[1]))[0]
                steering_ang = struct.pack('i', int(inputArgs[2]))[0]
                arm_ang = int(inputArgs[3])
                bucket_ang = int(inputArgs[4])
                scissor_ang = int(inputArgs[5])
                U.TXThreadPeriod = int(inputArgs[6])
                U.Run(3)
            else:
                print("Invalid input. Try:")
                print("A <velocity [m/s]> <steering angle [rad]> <arm angle [rad]> <bucket angle [rad]> <scissor angle [rad]> <perod [ms]> or A <period [ms]>")
                continue
        elif command == "L":
            if U.listen:
                U.StopListning()
            else:
                U.StartListning()
            if len(inputArgs) > 1:
                U.RXThreadPeriod = int(inputArgs[1])
        elif command == "T":
            if len(inputStr[2:]) > 63:
                print("Message too long")
            else:
                U.SendCommTest(inputStr[2:])




















if __name__ == '__main__':

    







    main()
