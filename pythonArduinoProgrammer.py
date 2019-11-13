#!/usr/bin/env python
import subprocess
import sys

# class pythonArduinoProgrammer:

def programArduino(arduinoPort,programLocation,arduinoProgram):
	#       next line must be edited to be appropriate for your PC
	#		On arch linux this is where the arduino startup script is linked to
	# 		I haven't tested this on Windows, but I would suspect the format would
	# 		be close to the same (example: C:/Programs/arduino.exe)
	# arduinoProg = "/usr/bin/arduino"
	arduinoProg = programLocation

	#		If you want to feed in the name of the file do this
	# projectFile = sys.argv[1]
	#		If you want to specify the name of the file do this
	# projectFile = "FURST_SW_003/FURST_SW_003.ino"
	projectFile = arduinoProgram

	codeFile = open(projectFile, 'r')
	startLine = codeFile.readline()[3:].strip()
	actionLine = codeFile.readline()[3:].strip()
	boardLine = codeFile.readline()[3:].strip()
	portLine = codeFile.readline()[3:].strip()
	endLine = codeFile.readline()[3:].strip()
	codeFile.close()


	portLine = arduinoPort
	#~ print projectFile
	#~ print startLine
	#~ print actionLine
	#~ print boardLine
	#~ print portLine
	#~ print endLine

	if (startLine != "python-build-start" or endLine != "python-build-end"):
		print("Sorry, can't process file, check your beginning format")
		sys.exit()

	arduinoCommand = arduinoProg + " --" + actionLine + " --board " + boardLine + " --port " + portLine + " " + projectFile

	print("\n\n -- Arduino Command --")
	print(arduinoCommand)

	print("-- Starting %s --\n" %(actionLine))

	presult = subprocess.call(arduinoCommand, shell=True)

	if presult != 0:
		print("\n Failed - result code = %s --" %(presult))
		exit()
	else:
		print("\n-- Success --")

if __name__ == "__main__":
	print("**********************************")
	print("This script is being ran by itself")
	print("**********************************")
	programArduino("/dev/ttyACM1","/usr/local/bin/arduino","FURST_message_parser/FURST_message_parser.ino")
