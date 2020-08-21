import sys
import time
import numpy
import serial
# import PySpin
# import ck_blackfly as ck
import pythonArduinoProgrammer as pyar
# import FURST_trigger as ft

#can change this if arduino is hooked into another port
#you can search this by typing in: ls /dev/
#on windows check which COM port you are connected to through device manager or arduino ide
arduinoPort = "/dev/ttyACM0"

#also which program you want to use
arduinoProgram = "FURST_Simple_camera_control/FURST_Simple_camera_control.ino"

# arduinoProgram = "FURST_message_parser/FURST_message_parser.ino"

#if you want to program the arduino on startup this is important, otherwise will fail
#Windows can use this script if the location is changed accordingly

#ubuntu Linux
programLocation = "/usr/bin/arduino"

#arch linux
#programLocation = "/usr/bin/arduino"

#is set to true when polling the user for information on camera setup
DEBUG_USERPOLL = False
DEBUG_PROGRAM  = False
STILL_SENDING  = False
afterInit      = False
initArduino    = False

# a basic command to tell the arduino a light capture with 25ms of exposure
# used in debugging
dummyMessage = "C0,light,50.0,C0"

# initilaizing a message to the arduino for testing
message = "C0,light,30.0,C0"

exposureTime = 2000.0


# cam is the initialized camera from PySpin
# arduinoPort is the usb connect to the arduino
# modeSet is a string: "open","close","trigger"
# exposureTime is exactly what the definition is


# def ZEROth_testing(cam: PySpin.CameraPtr, arduinoPort, modeSet, exposureTime, initArduino, solenoidPosition):
def ZEROth_testing():
    #need to initialize these so the python doesn't wonder where they are
    image_ptr           = []
    image_data          = []

    #this is to stay in a loop recording data for the repeated capture mode
    mode = " "
    userInput = " "

    global STILL_SENDING
    global afterInit


    previousString = " "


    ackCount = 0


    if(DEBUG_PROGRAM == False):
        print("Would you like to program the arduino? ")
        print("Make sure to specify where your arduino install is in the")
        print("pythonArduinoProgrammer.py file local to this directory")
        userInput = input("[y/n] ")


        if(userInput == "y" or initArduino):
            py_ar = pyar.programArduino(arduinoPort,programLocation,arduinoProgram)
            STILL_SENDING = False
            afterInit = False
        elif(userInput == "n"):
            print("skipping arduino program")


    s = serial.Serial()
    s.port = arduinoPort
    # s.port = '/dev/ttyACM1'
    s.baudrate = 115200
    s.timeout = 1

    s.open()
    time.sleep(1)
    # print("opening serial line")

    # toggling these lines to reset the arduino
    s.setDTR(False)
    s.setRTS(False)
    time.sleep(.2)
    s.setDTR(True)
    s.setRTS(True)



    try:
        #initializing
        arduinoSerial = ""
        loopBool = 1

        while(loopBool):

            if(s.in_waiting > 0):
                arduinoSerial = s.readline()
                if(arduinoSerial != previousString):
                    print(arduinoSerial)
                    previousString = arduinoSerial

            if(arduinoSerial[:3]  == b'rdy'):
                #this is where the arduino is waiting for a new command at the beginning

                # newMessage = ("C0," + str(solenoidPosition) + "," + modeSet + "," + str(float(exposureTime)) + ",C0")

                newMessage = ("C0," + str(2) + "," + "close" + "," + str(float(1000)) + ",C0")


                print("sending to arduino: " + newMessage + " length: " + str(len(newMessage)))

                #will make the program resend the message until the arduino gets the right command

                #after first loop boolean, not sure why I have this in here
                if(~afterInit):
                    afterInit = True

                # Before this write to camera to set exposure onit.
                # Setting exposure time if larger than a certain amount, to prevent damage to camera
                # if(exposureTime > 200):
                    # cam.ExposureTime.SetValue(float(exposureTime))  # can set exposure time with just a int
                    # test = cam.ExposureTime.GetValue()
                    # print("New exposure time: " + str(test))


                s.write(bytes(newMessage,"UTF-8"))
                time.sleep(.2)
                prevTimer = time.time()
                message = newMessage
                STILL_SENDING = True
                #an ACK based resending snippet.  If the Arduino doesn't let the python know that it got the message the python will resend the message
                while (STILL_SENDING):
                    timer = time.time()
                    print("Current Time: " + str((timer - prevTimer)))
                    arduinoSerial = s.readline()
                    print(arduinoSerial)
                    if(arduinoSerial[:3] == b'ACK'):
                        STILL_SENDING = False
                        print("Uart trasmission complete: " + str(ackCount))
                        ackCount = ackCount + 1
                    else:
                        if((timer - prevTimer) > float(1)):
                            #if the timer doesn't see a ACK within a second then resent command
                            s.write(bytes(newMessage, "UTF-8"))
                            prevTimer = timer


                #if this isn't long enough the arduino assumes that multiple sends are one line
                time.sleep(.2)

                #making sure the command only is sent once
                arduinoSerial = " "

            # if(arduinoSerial[:32] == b'Arduino: starting Camera Trigger'):
                # image_ptr, image_data = ft.trigger_acquire_single(cam)
                # return image_ptr, image_data

            if(arduinoSerial[:24] == b'Arduino: opening apeture' or arduinoSerial[:24] == b'Arduino: closing apeture'):
                #exit out of loop
                # loopBool = False
                pass


        # return image_ptr, image_data


    except KeyboardInterrupt:
        print("  existential crisis")



if __name__ == "__main__":


    if(sys.version_info[0] < 3):
        print("Using a version of python before version 3.0")
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(p)

        ZEROth_testing()

    else:
        print("Using a version of python after version 3.0")
        ZEROth_testing()
