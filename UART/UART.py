import serial


class UART:
    """docstring for UART."""

    def __init__(self, SerialPort):
        self.Port = SerialPort
        self.ControlMsgSeq = self.EnableASMsgSeq = self.rxSeq = 0
        self.data = []
        
        try:
            self.Ser = serial.Serial(self.Port, 115200, timeout = None)
            self.running = True
        except:
            print("Failed to open serial port " + self.Port)
            self.running = False

    def SendCommand(self, OpCode, data):
        # print(data)
        if self.Ser.write(bytes([OpCode<<5 | self.ControlMsgSeq]) + data) != len(data) + 1:
            print("Failed to send bytes")
            return False
        self.ControlMsgSeq = (self.ControlMsgSeq + 1) % 32
        return True
    
    def SendEnableSignal(self):
        if self.Ser.write([0b01000000 | self.EnableASMsgSeq]) != 1:
            print("Failed to send byte")
            return False
        self.EnableASMsgSeq = self.EnableASMsgSeq + 1
        return True
    
    def SendCommTest(self, message):
        self.Ser.write(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))
        # print(bytes([0b11000000 | len(message)]) + bytes(message, "ASCII"))

    def ReadMessage(self):
        """Reads data and put ut in self.data
        Returns True if status message is received, False otherwise"""
        self.Ser.flush()
        in_data = self.Ser.read(1)
        if len(in_data) < 1:
            print("Failed to read op-code, Timeout")
            return False

        firstbyte = in_data[0]
        opcode = firstbyte>>6

        if opcode == 1:
            self.data = self.Ser.read(22)
            self.rxSeq = firstbyte & 0b0011111111
            return True

        elif opcode == 0:
            self.errorcode = self.Ser.read(1)[0]
            self.errorseq = firstbyte & 0x3F
            print('Error: #' + str(self.errorcode) + '    Sequence: #' + str(self.errorseq))
        
        elif opcode == 3:
            self.Ser.timeout = 0.1
            self.data = self.Ser.read(firstbyte & 0b0011111111)
            self.Ser.timeout = None
            print("Communication Test Reply:")
            print(str(self.data))
        
        else:
            print("Unknown OpCode: " + str(opcode))

        return False
