
#List connected com ports
import serial.tools.list_ports

def getCOMPORTS():
    comportDict = {}
    comportInfoList = serial.tools.list_ports.comports()
    for comportInfo in comportInfoList:
        comportDict[comportInfo.name] = comportInfo.description
    return comportDict

print(getCOMPORTS())

##comport.device gives comport COM#