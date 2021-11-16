from time import sleep
from datetime import datetime
import sys
import struct
import threading
from WheelLoaderCommunication import WheelLoaderCommunication

pi = 3.141592


class Controller:
    """docstring for Controller.
    The controller works like a remote controller
     - you input the states of the levers and buttons
       and the controller sends the data to the vehicle.
    Constructor takes an object of type class UART,
    which needs to be initialized beforehand.
    """

    # Modes
    STOPPED_MODE = 0
    MANUAL_MODE = 1
    READY_IN_AUTOMOMOUS_MODE = 2
    AUTONOMOUS_MODE = 3

    def __init__(self, comm):
        self.Communicator = comm
        self.mode = self.bucket_ang = self.bucket_height = self.scissor_ang = 0
        self.TXThreadPeriod = 90
        self.TX = threading.Thread(target=self.TXThread, args=(1,), daemon=True)
        self.velocity = self.steering_ang  = 128

    def startThread(self):
        """Start the transmitter thread"""
        self.TX.start()

    def TXThread(self, name):
        """Thread that handles the transmission of signals, depending on the
        controller mode. """
        print("TXThread started")
        while 1:
            # print(self.mode)
            if self.mode == self.MANUAL_MODE:
                data = bytes([self.velocity, self.steering_ang, self.bucket_ang, self.bucket_height, self.scissor_ang])
                try:
                    self.Communicator.SendCommand(1, data)
                    # print("sent data")
                except ValueError:
                    print("Value error! Value(s) given were out of range")
                    self.Stop()
            elif self.mode == self.READY_IN_AUTOMOMOUS_MODE:
                self.Communicator.SendEnableSignal()

            elif self.mode == self.AUTONOMOUS_MODE:
                try:
                    data = bytes([self.velocity, self.steering_ang, self.bucket_ang, self.bucket_height, self.scissor_ang])
                    self.Communicator.SendCommand(3, data)
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
        self.mode = self.STOPPED_MODE
        print("Communication stopped")

    def ExecuteArgs(self,inputArgs):
        """Control vehicle with these parameters"""
        self.velocity = int(inputArgs[0])
        self.steering_ang = int(inputArgs[1])
        self.bucket_ang = int(inputArgs[2])
        self.bucket_height = int(inputArgs[3])
        self.scissor_ang = int(inputArgs[4])


class Observer:
    """docstring for Observer.
    The observer sees the states of the vehicle by listening
    to its return messages. It keeps listning as long as self.listen == True
    and the listning act is blocking, i.e. i will keep waiting until
    a new message is received.
    Constructor takes an object of type class UART,
    which needs to be initialized beforehand."""

    error_lookup = (
    "MESSAGE_OK",
    "MESSAGE_LENGTH_ERROR",
    "MESSAGE_OPCODE_ERROR",
    "MESSAGE_SEQUENCE_ERROR",
    "MESSAGE_CHANNEL_IGNORED",
    "MESSAGE_PACKET_DROPPED",
    "MESSAGE_TIMEOUT",
    "MESSAGE_TIMEOUT1",
    "MESSAGE_TIMEOUT2",
    "MESSAGE_TIMEOUT3",
    "MESSAGE_TIMEOUT4"
    )

    def __init__(self, comm):
        self.Communicator = comm
        self.listen = False
        self.state_message_flag = 0
        self.last_error = []
        self.error_message_flag = 0
        self.RXThreadPeriod = 90
        self.RX = threading.Thread(target=self.RXThread, args=(1,), daemon=True)
        self.X = self.Y = self.velocity_actual = self.Theta = self.sens1 = self.sens2 = self.sens3 = self.mode = (0,)
        self.steering_ang_actual = self.bucket_ang_actual = self.bucket_height_actual = self.scissor_ang_actual = 0
        self.log_filename = "log.csv"
        self.logfile = None
        self.log_data = False
    #sen1: sen2: sen3:

    def startThread(self):
        """Start the receiver thread"""
        self.RX.start()

    def StartListning(self):
        """Start listening to incoming messages (terminal spam)"""
        self.listen = True

    def StopListning(self):
        """Stop listening to incoming messages (no more terminal spam)"""
        self.listen = False

    def RXThread(self, name):
        """Thread that handles the reception of incoming
        signals, depending on self.listen boolean."""
        self.Communicator.flush()
        while 1:
            if self.listen:
                # print("listening")
                message_type = self.Communicator.ReadMessage()
                if message_type == 1:
                    self.X = struct.unpack('f', self.Communicator.data[0:4])
                    self.Y = struct.unpack('f', self.Communicator.data[4:8])
                    self.Theta = struct.unpack('f', self.Communicator.data[8:12])
                    self.velocity_actual = self.Communicator.data[12]
                    self.steering_ang_actual = self.Communicator.data[13]
                    self.bucket_ang_actual = self.Communicator.data[14]
                    self.bucket_height_actual = self.Communicator.data[15]
                    self.scissor_ang_actual = self.Communicator.data[16]
                    self.sens1 = struct.unpack('H', self.Communicator.data[17:19])
                    self.sens2 = self.Communicator.data[19]
                    self.sens3 = self.Communicator.data[20]
                    self.mode = self.Communicator.data[21]
                    self.state_message_flag = 1
                    if(self.log_data):
                        self.logfile.write(datetime.now().strftime("%H:%M:%S.%f, ") + ' ,'.join(str(val) for val in [self.X[0], self.Y[0], self.Theta[0], self.velocity_actual, self.steering_ang_actual, self.bucket_ang_actual, self.bucket_height_actual, self.scissor_ang_actual, self.sens1[0], self.sens2, self.sens3, self.mode]) + '\n')
                    else :
                        print('Sq#' + str(self.Communicator.rxSeq) + ' X={:.2f} Y={:.2f} Theta={:.2f} velocity={} steering angle={} bucket angle={} bucket height={} scissor angle={} sens1={} sens2={} sens3={} mode={}'\
                        .format(self.X[0], self.Y[0], self.Theta[0], self.velocity_actual, self.steering_ang_actual, self.bucket_ang_actual, self.bucket_height_actual, self.scissor_ang_actual, self.sens1[0], self.sens2, self.sens3, self.mode))
                elif message_type == 0:
                    self.last_error = (self.error_lookup[self.Communicator.errorcode], self.Communicator.errorseq)
                    self.error_message_flag = 1
                    if(self.log_data):
                        self.logfile.write("Error #" + str(self.last_error[1]) + ", " + self.last_error[1] + '\n')
                    print('Error: ' + self.last_error[0] + '    Sequence: #' + str(self.last_error[1]))

            else:
                sleep(self.RXThreadPeriod/1000)
    def getStates(self):
        """Get the states in a tuple. Check state_message_flag before calling
        this method to ensure that the states are new"""
        if self.state_message_flag:
            self.state_message_flag = 0 # clear the flag
        return (self.X[0], self.Y[0], self.Theta[0], self.velocity_actual, self.steering_ang_actual, self.bucket_ang_actual, self.bucket_height_actual, self.scissor_ang_actual, self.sens1[0], self.sens2, self.sens3, self.mode)
    def getError(self):
        """Get the last error in a tuple: (error_type, Sequence number).
        Check error_message_flag before calling
        this method to ensure that the error is new"""
        if self.error_message_flag:
            self.error_message_flag = 0 # clear the flag
        return self.last_error
    def startLogging(self, filename = "log.csv"):
        self.log_filename = filename
        self.logfile = open(filename, 'w')
        self.log_data = True
    def stopLogging(self):
        self.log_data = False
        self.logfile.close()


def main():

    if(len(sys.argv) == 1):
        port = "COM4"
        print("Default port set to " + port + ". Enter COM port, that the microcontroller is connected to, as argument when running the script, e.g. ")
        print("> " + sys.argv[0] + " COM11")
        print("For nvidia, ender: '/dev/ttyTHS0' as argument when using the GPIO uart pins\n")
    else:
        port = sys.argv[1]
    U = WheelLoaderCommunication(port)
    if(not U.running):
        sys.exit("Could not open serial port.")
    print("S - stop")
    print("R <X [m]> <Y [m]> <Theta [rad]> - Reset coordinate system")
    print("G - Get current state of vehicle")
    print("C <velocity> <steering angle> <arm angle> <bucket height> <scissor angle> <period [ms]> - send periodic control messages")
    print("A <period [ms]> - enable autonomous mode (awaits external control commands)")
    print("A <velocity [m/s]> <steering angle> <bucket angle> <bucket height> <scissor angle> <period [ms]> - send periodic AS control messages")
    print("L <period (optional)> - toggle UART listner")
    print("T <message up to 63 characters> - Send communication test message")
    print("Q - quit program")

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
            controlArgs = inputArgs[1:-1]
            C.TXThreadPeriod = int(inputArgs[6])
            C.ExecuteArgs(controlArgs)
            if C.mode != 1:
                C.Run(C.MANUAL_MODE)                                                        # Starts the thread
        elif command == "A":
            if len(inputArgs) == 2:
                C.TXThreadPeriod = int(inputArgs[1])
                C.Run(C.READY_IN_AUTOMOMOUS_MODE)
            elif len(inputArgs) == 7:
                # velocity = struct.pack('i', int(inputArgs[1]))[0]
                # steering_ang = struct.pack('i', int(inputArgs[2]))[0]
                controlArgs = inputArgs[1:-1]
                C.ExecuteArgs(controlArgs)
                C.TXThreadPeriod = int(inputArgs[6])
                C.Run(C.AUTONOMOUS_MODE)
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
