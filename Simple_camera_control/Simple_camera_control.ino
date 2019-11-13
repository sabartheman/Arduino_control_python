// python-build-start
// upload
// arduino:avr:mega
// /dev/ttyACM0
// python-build-end

//The aboce is only important for the pythonArduinoProgrammer script.

//Originally this file controlled a specialized camera used for narrow spectrum imaging.
//All relecant code for camera is removed from this file and others in this repo

#include "variables.h"


#define MOTORPINS 9
#define CAMERA_DATA_ACQUISITION_DELAY 1000


unsigned long loopStart;
unsigned long loopEnd;

//for the message receive function
boolean messageRecieved = false;
//for when the messages circles back in circular buffer
boolean waitingRollover = false;

int successFlag = 0;

//defines the current position the messages are inputting into a que
int waitingMessages = 0;
//defines the current position of the processed messages
int parsedMessages = 0;
//defines the current position of the processing method
int processPosition = 0;
int previousProcess = 100;

//keeping track of process execution time
unsigned long processStart;
unsigned long processCurrent;

//keeps track of how long the solenoids circuit has been active
unsigned long solenoidStart;
unsigned long solenoidCurrent;

//keeps track if a process is active
boolean processActive = false;
//keeps track of if in the process the motors have been rotated  in the beginning of a process
boolean solenoidRotated = false;
//keeps track of recieved messages
boolean noMessage = false;



boolean changeInStorage = false;


//where the messages wait until being processed
static String messageQue[100];

//holding the values of the commands send across
static String commandQue[100];
long int exposureQue[100];

//only used in the repeated capture case
long int lightCaptureQue[100];
long int darkCaptureQue[100];



void setup()
{
  //using a better baudrate than the usual 9600
  Serial.begin(115200);
  Serial.setTimeout(100);

  //initialize State Machine
  state = s_wait;

  //initliazing the pinout
  initPins();

  //Start delay counter
  // lastTime = millis();
  Serial.println("Now starting message parsing unit for FURST");


}


void simpleControl(void){
  // debug line
  // Serial.print("Current Task: "); Serial.println(commandQue[processPosition]);

  if(commandQue[processPosition].startsWith("light")){
    Serial.println("Arduino: Single Light Capture");
    simpleLightCapture();
  }
  else if(commandQue[processPosition].startsWith("dark")){
    Serial.println("Arduino: Single Dark Capture");
    simpleDarkCapture();
  }
  else if(commandQue[processPosition].startsWith("repeated")){
    Serial.println("Arduino: Starting repeated capture of light and dark");
    delay(50);
    simpleRepeatedCapture();
  }else if(commandQue[processPosition].startsWith("open")){
    //open solenoid
    Serial.println("Arduino: opening apeture");
    simpleOpenSolenoid();
  }else if(commandQue[processPosition].startsWith("close")){
    //close soleniod
    Serial.println("Arduino: closing apeture");
    simpleCloseSolenoid();
  }else if(commandQue[processPosition].startsWith("trigger")){
    //trigger camera after a small delay
    Serial.println("Arduino: starting Camera Trigger");
    simpleCameraTrigger();
  }
  processPosition++;

  // Serial.println("Arduino: Capture Sequence complete");
}

void simpleCameraTrigger(void){
  delay(5000);
  digitalWrite(3,HIGH);
  delay(50);
  digitalWrite(3,LOW);

}

void simpleOpenSolenoid(void){
  //just a simple open solenoid function
  simpleSolenoidControl(motorPin[0]);

}

void simpleCloseSolenoid(void){
  //just a simple close solenoid function
  simpleSolenoidControl(motorPin[0]);

}

void simpleLightCapture(void){

  //open the solenoid
  simpleSolenoidControl(motorPin[0]);

  delay(50);

  //just a simple trigger function
  simpleTriggerControl();


  delay(50);
  //close the solenoid
  simpleSolenoidControl(motorPin[0]+1);

  delay(50);

  Serial.println("Arduino: External Trigger finished, Light capture");


}

void simpleDarkCapture(void){
  //make sure apeture is closed
  simpleSolenoidControl(motorPin[0]+1);

  //just a simple trigger functions
  simpleTriggerControl();

  //making sure apeture is still closed
  simpleSolenoidControl(motorPin[0]+1);


  Serial.println("Arduino: External Trigger finished, Dark capture");

}

void simpleRepeatedCapture(void){
  //setting up how many times light and dark will be captured
  int i = lightCaptureQue[processPosition];
  int j = darkCaptureQue[processPosition];


  while(i > 0 || j > 0){
    if(i > 0){
      simpleLightCapture();
      i--;
    }
    //to allow the python program to grab data from the camera

    delay(CAMERA_DATA_ACQUISITION_DELAY);

    if(j > 0){
      simpleDarkCapture();
      j--;
    }
    //to allow the python program to grab the data from the camera
    delay(CAMERA_DATA_ACQUISITION_DELAY);
  }

  Serial.println("Arduino: External Trigger finished, Repeated capture");

}

//function to control the trigger in a simple manner
void simpleTriggerControl(void){
  unsigned long triggerStart;

  digitalWrite(pulseOut,HIGH);
  triggerStart = millis();
  while((millis() - triggerStart) < (exposureQue[processPosition]/1000)){
    //wait and do nothing till finishied
  }
  //Lower the trigger after a set amount of time
  digitalWrite(pulseOut,LOW);
}


void simpleSolenoidControl(int pinNumber){
  unsigned long solenoidStart = millis();

  while((millis() - solenoidStart) < d_switch){
    digitalWrite(pinNumber, HIGH);
    // Serial.print("Hit this: "); Serial.println((millis() - solenoidStart));
  }
  digitalWrite(pinNumber,LOW);
}



void loop(void){
  //will be parsing messages after they come in through the UART
  receiveSerial();

  Serial.print("storage Location for the messages: "); Serial.println(char(waitingMessages));
  //will parse the recieved messages
  parseMessage();

  simpleControl();

}





void receiveSerial(){
  String inputCMD;
  unsigned long serialStart = millis();
  boolean serialBool = true;

  Serial.println("rdy");

  while(!Serial.available()){
  }
  // wait until data is available
  if(Serial.available() > 0 && !noMessage){
    inputCMD = Serial.readString();
    // Serial.println(inputCMD);

    // if(inputCMD.startsWith("C0,") && (inputCMD.length() < 35 && (inputCMD != messageQue[waitingMessages - 1]))){
    if(inputCMD.startsWith("C0,")){
      Serial.print("Arduino: Packetized Message recieved: "); Serial.println(inputCMD);
      changeInStorage = true;
      if(waitingMessages < 100){
        messageQue[waitingMessages++] = inputCMD;
        inputCMD = " ";
      }else{
        waitingMessages = 0;
        waitingRollover = true;
        messageQue[waitingMessages++] = inputCMD;
        inputCMD = " ";
      }
    }
  }
  //once data is available send it back for verification
}



//pull a message from current position in que and process the cmd
void parseMessage(){
  //check to see if there is a new message, if not skip function
  if(waitingMessages > parsedMessages || (waitingRollover == true)){
    //run parsing methods
    // Serial.print("Arduino: Now parsing message: ");Serial.println(messageQue[waitingMessages-1]);
    //takes the input commands and parses them into respective arrays
    splitMessage();

  }
  else if (waitingRollover == true && parsedMessages == 100){
    parsedMessages = 0;
    //resetting the parsing counter now that it's caught upto the end of the official que
    //making this a circular buffer
    waitingRollover = false;
    //
    splitMessage();
  }
  else{
    //do nothing since no new messages are existing
  }


}



void splitMessage(void){
  //look at the current unparsed cmd and split it based on comma delimiters

  //this part is just b'C0', the beginning of a packet
  String part1 = getValue(messageQue[parsedMessages], ',', 0);
  // Serial.print("Found a part: ");Serial.println(part1);

  String stringCMD = getValue(messageQue[parsedMessages], ',', 1);
  // Serial.print("Found a part: ");Serial.println(stringCMD);

  String part2 = getValue(messageQue[parsedMessages], ',', 2);
  // Serial.print("Found a part: ");Serial.println(part2);

  String part3 = getValue(messageQue[parsedMessages], ',', 3);
  // Serial.print("Found a part: ");Serial.println(part3);

  String part4 = getValue(messageQue[parsedMessages], ',', 4);
  // Serial.print("Found a part: ");Serial.println(part4);

  String part5 = getValue(messageQue[parsedMessages], ',', 5);
  // Serial.print("Found a part: ");Serial.println(part5);


  // Serial.println(stringCMD);

  if(stringCMD.startsWith("light")){
      //constant, otherwise the trailing
      commandQue[parsedMessages]          = "light";
      lightCaptureQue[parsedMessages]     = 1;
      darkCaptureQue[parsedMessages]      = 0;
      //this is in microseconds
      exposureQue[parsedMessages]         = part2.toFloat();
  }else if (stringCMD.startsWith("dark")){
      commandQue[parsedMessages]          = "dark";
      lightCaptureQue[parsedMessages]     = 0;
      darkCaptureQue[parsedMessages]      = 1;
      exposureQue[parsedMessages]         = part2.toFloat();
  }else if (stringCMD.startsWith("repeated")){
      commandQue[parsedMessages]          = "repeated";
      lightCaptureQue[parsedMessages]     = part2.toFloat();
      darkCaptureQue[parsedMessages]      = part3.toFloat();
      exposureQue[parsedMessages]         = part4.toFloat();
  }else if (stringCMD.startsWith("open")){
      commandQue[parsedMessages]          = "open";
      lightCaptureQue[parsedMessages]     = 1;
      darkCaptureQue[parsedMessages]      = 0;
      exposureQue[parsedMessages]         = 0;
  }else if (stringCMD.startsWith("trigger")){
      commandQue[parsedMessages]          = "trigger";
      lightCaptureQue[parsedMessages]     = 1;
      darkCaptureQue[parsedMessages]      = 1;
      exposureQue[parsedMessages]         = part2.toFloat();
  }else if (stringCMD.startsWith("close")){
      commandQue[parsedMessages]          = "close";
      lightCaptureQue[parsedMessages]     = 0;
      darkCaptureQue[parsedMessages]      = 1;
      exposureQue[parsedMessages]         = 0;
  }
  //finally increment the parsedmessage counter
  parsedMessages++;
}


//this function splits the incoming strings nicely based on a delimiter
String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


void initPins(void)
{
  //maybe comment all unused pins out? -ST
  pinMode(motorPin[0], OUTPUT);
  pinMode(motorPin[0] + 1, OUTPUT);
  //for testing in Kandleborgs lab these arent used.
  pinMode(motorPin[1], OUTPUT);
  pinMode(motorPin[1] + 1, OUTPUT);
  pinMode(motorPin[2], OUTPUT);
  pinMode(motorPin[2] + 1, OUTPUT);
  pinMode(motorPin[3], OUTPUT);
  pinMode(motorPin[3] + 1, OUTPUT);
  pinMode(motorPin[4], OUTPUT);
  pinMode(motorPin[4] + 1, OUTPUT);
  pinMode(motorPin[5], OUTPUT);
  pinMode(motorPin[5] + 1, OUTPUT);
  pinMode(motorPin[6], OUTPUT);
  pinMode(motorPin[6] + 1, OUTPUT);
  pinMode(motorPin[7], OUTPUT);
  pinMode(motorPin[7] + 1, OUTPUT);
  pinMode(motorPin[8], OUTPUT);
  pinMode(motorPin[8] + 1, OUTPUT);
  //need to use for testing in vac chamber with heating/cooling
  pinMode(photoAmp_1, INPUT);
  pinMode(photoAmp_2, INPUT);
  pinMode(photoAmp_3, INPUT);
  pinMode(photoAmp_4, INPUT);
  //setting up the output
  pinMode(pulseOut, OUTPUT);
}
