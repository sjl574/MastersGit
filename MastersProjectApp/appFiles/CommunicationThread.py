from PyQt5.QtCore import QThread, pyqtSignal as Signal, pyqtSlot as Slot
import serial
from queue import Queue
import time

DEBUG = True

#class for handling arduino communications
#Send messages by appending to ArduinoComs.messageQueue
class ArduinoComs(QThread):
    running = False     #running flag (should be turned false before shutdown)
    connected = False
    #Queue for message storing
    messageQueue = Queue() #Queue item = [command, data, replyLen, timeout]
    #Signals (internal interrupt)
    recieved_signal = Signal(list) #list[int, int]
    debug_signal = Signal(str) #str("Debug message")


    #Function override - Thread function (this will be run on class.start())
    def run(self):
        if ArduinoComs.connected == False:
            self.debug_signal.emit("Error, Arduino Not Connected!")
            return None
        self.setPriority(QThread.Priority.HighPriority)
        while ArduinoComs.running:
            #parse queue, if empty, do nothing
            if ArduinoComs.messageQueue.qsize == 0:
                time.sleep(0.01)
                continue
            #...
            #send message
            message = ArduinoComs.messageQueue.get()
            ret = self.messageArduino(message[0], message[1], message[2], message[3])
            if ret is not None:
                self.recieved_signal.emit([message[0], ret])
            

    #Connect to arduino
    def connect(self, comPort, baudRate):
        try:
            self.ardSerial = serial.Serial( comPort,
                                            baudRate,
                                            timeout = 1,
                                            xonxoff=False, rtscts=False, dsrdtr=False
            )
            ArduinoComs.connected = True
        except Exception as e:
            self.connected = False
            self.debug_signal.emit(f"Error Connecting to Arduino - {e}")
            raise e
    

    #Disconnect from arduino
    def disconnect(self):
        self.ardSerial.close()
        ArduinoComs.connected = False


    #Send message to arduino
    def messageArduino(self, command : int, data : int = 0, expectReplyLen : int = 0, replyTimeOut_s : float = 5.0) -> int | None:
        ret = None
        #inform of error, ret None if arduino not connected
        if not self.connect:
            self.debug_signal.emit(f"Failed To Send Arduino Message: Connection Not Established")
            return ret
        #Arduino connected...
        #clear any data in current serial buffers
        self.ardSerial.read_all()
        self.ardSerial.flush()

        #combine command byte and data into into serial of bytes
        message = bytearray()
        message.extend(command.to_bytes(4, 'little', signed = True))
        message.extend(data.to_bytes(4, 'little', signed = True))
        
        #DEBUG
        if DEBUG:
            self.debug_signal.emit(f"Message Sent: CMD:{command}, VAL:{data}")
        
        #Send message
        self.ardSerial.write(message)
        
        #ret immediately if expecting no reply
        if expectReplyLen == 0:
            return ret

        #Gather reply if expecting
        self.ardSerial.timeout = replyTimeOut_s
        startTime = time.time()
        while(time.time() - startTime < replyTimeOut_s):
            time.sleep(0.01)
            if expectReplyLen >= self.ardSerial.in_waiting:
                ret = self.ardSerial.read(4)
                ret = int.from_bytes(ret, 'little', signed = True)
                self.debug_signal.emit(f"Message Back: {ret}")
                break
        self.ardSerial.read() #clear any extra bytes

        #return bytes of return value, or None if none
        return ret
    
