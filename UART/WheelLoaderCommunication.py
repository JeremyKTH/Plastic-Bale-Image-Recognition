import serial

class WheelLoaderCommunication(serial.Serial):
    """docstring for WheelLoaderCommunication.
    This class handles the communication between the computer and the
    microcontroller (or ECU) on the wheel loader. It extends the Serial class
    as per the communication happens through UART. Only one object instance of
    this class per com port can be used"""

    def __init__(self, SerialPort):
        self.ControlMsgSeq=self.EnableASMsgSeq=self.rxSeq=self.errorseq=self.rexeseq = 0     # Message sequence numbers
        self.data = []                                                          # Data container for received messages
        self.running = False                                                    # State of the communication session
        self.TX_log_filename = "TX_log.csv"
        self.RX_log_filename = "RX_log.csv"
        self.TX_logfile = None
        self.RX_logfile = None
        self.log_TX = False
        self.log_RX = False
        self.errorcode = None
        self.remote_exe_command = None
        # self.timeout = None
        try:
            super().__init__(
                SerialPort, 115200,
                timeout = None, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.running = True
        except serial.SerialException:
            print("___Failed to open serial port " + SerialPort)

    def SendCommand(self, OpCode, data):
        # print(data)
        if self.write(bytes([OpCode<<5 | self.ControlMsgSeq]) + data) != len(data) + 1:
            print("___Failed to send bytes")
            return False
        self.ControlMsgSeq = (self.ControlMsgSeq + 1) % 32
        if self.log_TX:
            self.TX_logfile.write(', '.join(str(val) for val in list(data)) + '\n')
        return True

    def SendEnableSignal(self):
        if self.write([0b01000000 | self.EnableASMsgSeq]) != 1:
            print("___Failed to send byte")
            return False
        self.EnableASMsgSeq = (self.EnableASMsgSeq + 1) % 32
        return True

    def SendCommTest(self, message):
        self.write(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))
        # print(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))

    def ReadMessage(self):
        """Reads data and put ut in self.data
        Returns True if status message is received, False otherwise"""
        self.flush()
        in_data = self.read(1)
        if len(in_data) < 1:
            print("___Failed to read op-code, Timeout")
            return False

        firstbyte = in_data[0]
        opcode = firstbyte>>6

        if opcode == 1:
            self.data = self.read(23)
            self.rxSeq = firstbyte & 0b0011111111
            if self.log_RX:
                self.RX_logfile.write(', '.join(str(val) for val in [self.rxSeq]+ list(self.data)) + '\n')


        elif opcode == 0:
            self.errorcode = self.read(1)[0]
            self.errorseq = firstbyte & 0x3F
            if self.log_RX:
                self.RX_logfile.write(', '.join(str(val) for val in [self.errorseq, self.errorcode]) + '\n')

        elif opcode == 2:
            self.remote_exe_command = self.read(1)[0]
            self.rexeseq = firstbyte & 0x3F
            if self.log_RX:
                self.RX_logfile.write(', '.join(str(val) for val in [self.rexeseq, self.remote_exe_command]) + '\n')

        elif opcode == 3:
            self.timeout = 0.1
            self.data = self.read(firstbyte & 0b0011111111)
            self.timeout = None
            print("___Communication Test Reply:")
            print(str(self.data))

        else:
            print("___Unknown OpCode: " + str(opcode))
            return -1
        self.reset_input_buffer()
        return opcode

    def startLoggingTX(self, filename = "TX_log.csv"):
        self.TX_log_filename = filename
        self.TX_logfile = open(filename, 'w')
        self.log_TX = True

    def startLoggingRX(self, filename = "RX_log.csv"):
        self.RX_log_filename = filename
        self.RX_logfile = open(filename, 'w')
        self.log_RX = True

    def stopLoggingTX(self):
        self.log_TX = False
        self.TX_logfile.close()

    def stopLoggingRX(self):
        self.log_RX = False
        self.RX_logfile.close()

def main():
    Comm = WheelLoaderCommunication("COM7")
    if Com.running :
        print("Success!")
