
//delays for each state
//These are the parameters that can be changed.
const int d_wait    = 1000;
const int d_switch  = 30;
const int d_wait2   = 1000;
static int d_expose  = 23;

// Time Variables for state machine
unsigned long lastTime;
unsigned long currTime;

// pin definitions for 9 solenoids, these pins turn the motor to the ON position
// Each motor, n, has a corresponding OFF pin located at motorPin[n] + 1
const int motorPin[9] = {22, 24, 26, 28, 30, 32, 34, 36, 38};

// Track which motor is being tested
int motorCount = 0;

//Track if motor is on or off
int motorFlag = 0;

//Track the current state
int state;

const int s_wait    = 0;
const int s_switch  = 1;
const int s_wait2   = 2;
const int s_expose  = 3;

// Photo Amp Digital read pins
const int photoAmp_1 = 50;
const int photoAmp_2 = 51;
const int photoAmp_3 = 52;
const int photoAmp_4 = 53;

// Photo amp value Varriables
int amp1 = 0;
int amp2 = 0;
int amp3 = 0;
int amp4 = 0;
int amp1Last = 0;
int amp2Last = 0;
int amp3Last = 0;
int amp4Last = 0;

//Output pin to camera
const int pulseOut = 3;



//Variables for user input
static int repeatedCaptures = 0;
static int exposureDelay = 0;
static int mode;

static int messageLength = 0;       //initializing the length of input

static int inputString[200];               // a String to hold incoming data
static int inputValue = 0;                 //input value coming from the serial port

bool stringStart = false;           // whether a beginning sequence is detected on input
bool stringComplete = false;        // whether the string is complete


int prev = 0xFF;                    //initializing
int next = 0xFF;
int orderOfMag = 0;
