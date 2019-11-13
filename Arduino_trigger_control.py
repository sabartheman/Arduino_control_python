import time
import numpy
import serial
import PySpin
import ck_blackfly as ck
import pythonArduinoProgrammer as pyar
import FURST_trigger as ft

#can change this if arduino is hooked into another port
#you can search this by typing in: ls /dev/
#on windows check which COM port you are connected to through device manager or arduino ide
# arduinoPort = "/dev/ttyACM1"

#also which program you want to use
arduinoProgram = "FURST_Simple_camera_control/FURST_Simple_camera_control.ino"

#if you want to program the arduino on startup this is important, otherwise will fail
#Windows can use this script if the location is changed accordingly

#ubuntu Linux
programLocation = "/usr/local/bin/arduino"

#arch linux
#programLocation = "/usr/bin/arduino"

#is set to true when polling the user for information on camera setup
DEBUG_USERPOLL = True
DEBUG_PROGRAM  = True
STILL_SENDING = False
afterInit     = False

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


def ZEROth_testing(cam: PySpin.CameraPtr, arduinoPort, modeSet, exposureTime, initArduino, solenoidPosition):
# def ZEROth_testing():
    #need to initialize these so the python doesn't wonder where they are
    image_ptr           = []
    image_data          = []

    #this is to stay in a loop recording data for the repeated capture mode
    mode = " "
    userInput = " "

    global STILL_SENDING
    global afterInit


    previousString = " "


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
    s.baudrate = 115200
    s.timeout = 5

    s.open()
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

                newMessage = ("C0," + str(solenoidPosition) + "," + modeSet + "," + str(float(exposureTime)) + ",C0")

                print("sending to arduino: " + newMessage)

                #will make the program resend the message until the arduino gets the right command
                STILL_SENDING = True;

                #after first loop boolean, not sure why I have this in here
                if(~afterInit):
                    afterInit = True

                # Before this write to camera to set exposure onit.
                # Setting exposure time if larger than a certain amount, to prevent damage to camera
                if(exposureTime > 200):
                    cam.ExposureTime.SetValue(float(exposureTime))  # can set exposure time with just a int
                    test = cam.ExposureTime.GetValue()
                    print("New exposure time: " + str(test))


                s.write(bytes(newMessage,"UTF-8"))
                message = newMessage
                #if this isn't long enough the arduino assumes that multiple sends are one line
                time.sleep(1)

                #making sure the command only is sent once
                arduinoSerial = " "

            if(arduinoSerial[:32] == b'Arduino: starting Camera Trigger'):
                image_ptr, image_data = ft.trigger_acquire_single(cam)
                return image_ptr, image_data

            if(arduinoSerial[:24] == b'Arduino: opening apeture' or arduinoSerial[:24] == b'Arduino: closing apeture'):
                #exit out of loop
                loopBool = False


        return image_ptr, image_data


    except KeyboardInterrupt:
        print("  existential crisis")



if __name__ == "__main__":

    ports = serial.tools.list_ports.comports()
    for p in ports:
        print(p)
