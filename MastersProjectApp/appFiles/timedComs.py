from PyQt5.QtCore import pyqtSignal as Signal,  QTimer
import serial

DEBUG = True

#class for handling arduino communications
#Send messages by appending to ArduinoComs.messageQueue
class ArduinoTimedComs():
    #Constructor - sets default variables 
    #   Provide function to call on recieval of arduino coms
    #   Provide polling interval in ms
    #   Provide function to call that debug strings are sent to
    def __init__(self, responseFunc, serialPollInterval = 1, debugFunc = None):
        self.pollInterval = serialPollInterval
        self.serial = None
        self.connected = False
        self.sending = False
        self.responseFunc = responseFunc
        self.debugFunc = debugFunc
        self.timer = QTimer()
        self.timer.timeout.connect(self.checkSerialInput)

    #Connect to arduino - Must be called before messages are send
    def connect(self, comPort, baudRate):
        #Try connect to serial port
        try:
            self.serial = serial.Serial( comPort,
                                            baudRate,
                                            timeout = 0,
                                            xonxoff=False, rtscts=False, dsrdtr=False
            )
        except Exception as e:
            self.connected = False
            raise e
        #On successful connection
        self.connected = True
        self.timer.start(self.pollInterval)
    
    #Disconnect from arduino
    def disconnect(self):
        self.timer.stop()
        if self.connected:
            self.serial.close()
            self.connected = False

    #Serial polling function
    def checkSerialInput(self):
        if not self.sending:
            while self.serial.in_waiting >= 8:
                cmd = int.from_bytes(self.serial.read(4), 'little', signed = True)
                val = int.from_bytes(self.serial.read(4), 'little', signed = True)
                if self.debugFunc is not None:
                    self.debugFunc(f"Message Recieved: CMD:{cmd}, VAL:{val}")
                self.responseFunc([cmd,val])

    #Send message to arduino
    def message(self, command : int, data : int = 0) -> None:
        #Wrap function to protect from interrupt
        self.sending = True
        #inform of error, ret None if arduino not connected
        if not self.connect:
            self.debugFunc(f"Failed To Send Arduino Message: Connection Not Established")
            return
        #combine command byte and data into into serial of bytes
        message = bytearray()
        message.extend(command.to_bytes(4, 'little', signed = True))
        message.extend(data.to_bytes(4, 'little', signed = True))
        #Send message
        self.serial.write(message)
        #DEBUG
        if self.debugFunc is not None:
            self.debugFunc(f"Message Sent: CMD:{command}, VAL:{data}") 
        #release protection
        self.sending = False
