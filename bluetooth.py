import serial
import time
import sys
import signal
def signal_handler(signal, frame):
	print("closing program")
	SerialPort.close()
	sys.exit(0)

COM=input("COM5")
BAUD=input("9600")
SerialPort = serial.Serial(COM,BAUD,timeout=1)
#time.sleep(0.2)
#SerialPort.write("I")
#
while (1):
	IncomingData=SerialPort.readline()
	if(IncomingData):
		print((IncomingData).decode('utf-8'))
	time.sleep(0.01)