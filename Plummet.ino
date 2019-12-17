//  Arduino connections:
//  ---------------------
//  analog  1: Potentiometer Bit
//  digital 1: (serialTx): connect to rx of the audio player (no need to connect tx of the player to anything)
//  digital 2: (rxPrev): connect to digital 5(tx) of the previous arduino (no connection if master)
//  digital 5: (txNext): connect to digital 2(rx) of the next arduino
//  digital 7: Tone bit (not important)
//  digital 10: Servo bit
//  Ground: Servo ground, potentiometer ground, Servo Power ground, audio device ground, next & prev arduino Ground
//  5V: Potntiometer VCC (+)
//  Vin: audio device VCC (+)
// 
//  Servo power VCC (+) need to connect directly to its own power
//  
//  Help:
//
//  TODO: 
//	create an EEPROM Editor

#define PLUMMET_VERSION "0.22"

// #include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#define SoftwareSerial NeoSWSerial

#include <TimerOne.h>
//LIBSERVO #include <Servo.h>
#include <EEPROM.h>

#define rxPinPrev 2 // soft serial
// #define txPinPrev 3 // soft serial
// #define rxPinNext 4 // soft serial
#define txPinNext 5 // soft serial
#define tonePin 7 // digital
#define servoPin 10 // digital

#define potPin 1 // analog


int SYNC_MAGIC_NUMBER=-250;

int defaultLoopTime = 3167; //Palo Alto: 3080; // 3160; // 3420;
// int defaultLoopTime = 3080; // Palo Alto

#define EPPROM

// Calibrate(1): 86.00 489 711 293
// Calibrate(2): 98.00 457 665 261
// Calibrate(3): 101.00 525 351 697

boolean SERVO_VIA_TIMER1 = true;

boolean printMeasures = false; 
boolean enablePrint = true;
boolean debug = false;
boolean enableAudio = false;
int8_t audioSongNumber = 1;
int8_t audioVolume = 30;
unsigned int audioDelay = 0; // in milliseconds
unsigned int audioSnapToGrid = 10; // in milliseconds
unsigned int audioSnapToSync = 50;
boolean showLoopEvents = false;

#define EEPROM_MAGIC 5613
#define EEPROM_COMMANDS_LOC 100
#define MAX_UINT 65535

unsigned int nextCommandTime = MAX_UINT;
int nextCommandLoc = 0;
unsigned long playInitTime;
unsigned long recordInitTime;
bool isRecording =false;
bool isPlaying = false;
bool isAutoPlay = false;
bool showClock = false;
int recordingLoc;

//servo
int loopTime = defaultLoopTime;
int lastLoopTime;

double maxSpeed = 2;  // pot will move at this speed (compared to average cycle speed based on servoAmp);
double maxServoAmp = 80;
double servoAmp = maxServoAmp;
double servoCenter = 100; // 88; //100 // 95;

int potCenter = 443; // 483; // 443; // 481;
int pot50 = 650; // 700; // 650;  // 343;
int pot150 = 240; // 300; // 240; // 620;

double testPhase = 0.25;
double testAmp = 20;

unsigned long initTime;

unsigned long time;
unsigned long lastIterationTime  = millis();

float ropeMaxLeftAngle;
float ropeMaxRightAngle;

unsigned long syncInitTime;
int syncInitTimeOffset;
int syncLoopTime=0;
float syncRopeAngle;
double syncPhase;
boolean updateSlaveClock = false;

//SoftwareSerial prevSerial(rxPinPrev, txPinPrev);
//SoftwareSerial nextSerial(rxPinNext, txPinNext);
SoftwareSerial nextSerial(rxPinPrev, txPinNext);
#define prevSerial nextSerial
bool listenOnPrev = true;

#define audioSerial Serial

boolean isMaster = true;

unsigned long rightTime = 0;
unsigned long leftTime = 0;

unsigned long startTime = millis();

enum left_right_e {
  LEFT,
  RIGHT
};

left_right_e side = LEFT;

enum mode_e {
  STOP,
  STOPPING,
  HALT,
  START,
  RUNNING,
  MAINTAIN,
  MAINTAINING,
  TEST,
  TESTING,
  SYNCED_RUN,
  SYNCED_RUNNING
};
mode_e mode;


#define NOTE_A5 880
#define NOTE_G5 784
#define NOTE_G6 1568

#define sprint(s)   if (enablePrint) Serial.print(s)
#define sprintln(s) if (enablePrint) Serial.println(s)
// void sprint(String s){}; void sprint(double s) {}
// #define sprint(s) 
// #define sprintln(s)

//void debugLog(const char *x) { if (debug) Serial.print(x);		 }
#define debugLog(x) {if (debug) Serial.print(x);}


///////////////////////////
/// Utils
///////////////////////////

void(* resetArduino) (void) = 0; //declare reset function @ address 0

static int eIndex = 0;
void ewrite(unsigned int data, int index=-1) {
  if (index!=-1) eIndex = index;
  
  EEPROM.write(eIndex++, byte(data/256));
  EEPROM.write(eIndex++, byte(data%256));
}

void ewrite(const char *data, int index=-1) {
	if (index !=-1) eIndex = index;
	
	for (int i=0; data[i] != 0; i++) {
		EEPROM.write(eIndex++, data[i]);
	}
	EEPROM.write(eIndex++, 0);
}

unsigned int eread(int index=-1) {
  if (index!=-1) eIndex = index;
  
  return EEPROM.read(eIndex++)*256+EEPROM.read(eIndex++);
}

uint8_t ereadbyte(int index=-1) {
	if (index != -1) eIndex = index;
	
	return EEPROM.read(eIndex++);
}
void ewritebyte(uint8_t data, int index=-1) {
	if (index != -1) eIndex = index;
	
	EEPROM.write(eIndex++,data);
}

//Todo test this function;
void ereadstr(int index, char *p, int maxSize) { // todo: MAXSIZE
  if (index!=-1) eIndex = index;
  while (char c = EEPROM.read(eIndex++)) {
  	*(p++) = c;
  }
  *p = 0;
  return;
}

void eprintstr(int index=-1) {
  if (index!=-1) eIndex = index;
  while (char c = EEPROM.read(eIndex++)) {
  	sprint(c);
  }
}
//////////////////////////////
// Audio
//////ֿ////////////////////////

void sendAudioCommand(int8_t command, int16_t dat) {
	sendAudioCommand(command, (int8_t)(dat >>8), (int8_t)(dat));
}

void sendAudioCommand(int8_t command, int8_t datah, int8_t datal) {
//7E FF 06 22 00 0F 02 EF --> Set the volume to 15(0x0f is 15) and play the second song

  static int8_t Send_Audio_buf[8] = {0x7e, 0xff, 0x06, 0, 0, 0, 0, 0xef} ;

  debugLog("audio");
  Send_Audio_buf[0] = 0x7e; //starting byte
  Send_Audio_buf[1] = 0xff; //version
  Send_Audio_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_Audio_buf[3] = command; //
  Send_Audio_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  Send_Audio_buf[5] = datah;//datah
  Send_Audio_buf[6] = datal; //datal
  Send_Audio_buf[7] = 0xef; //ending byte
  for(uint8_t i=0; i<8; i++)//
  {
	audioSerial.write(Send_Audio_buf[i]) ;
  }
  audioSerial.write('\r');
}

void playSong(int8_t songNumber=1, int8_t volume=30) {
	if (!enableAudio) return;
	sendAudioCommand(0x22, volume, songNumber);
}


///////////////////////////
/// SERVO
///////////////////////////
//LIBSERVO: Servo myservo;  // create servo object to control a servo
double lastServoWriteValue = 95;
#define SERVO_PWM_RATE 3040
boolean servoAttached = false;
static int originalTCCR1A = 0;

double smoothWrite(double desiredPosition) {
  //myservowrite(desiredPosition); return desiredPosition;
  double maxServoMove = double(time-lastIterationTime)/defaultLoopTime * max(servoAmp,5) * 2 * maxSpeed; // Max 3 times average speed under current servo.
  double servoPosition_new = max(min(desiredPosition, lastServoWriteValue+maxServoMove), lastServoWriteValue-maxServoMove);
//  if (servoPosition_new != desiredPosition) {
	//sprintln("^^^ desiredPosition="+String(desiredPosition)+" lastServoWriteValue="+String(lastServoWriteValue)+" MaxMove="+String(maxServoMove) + "  newpos="+String(servoPosition_new));
//  }
  myservowrite(servoPosition_new);
  return servoPosition_new;
}

void myservowrite(double pos) {
  if (SERVO_VIA_TIMER1 && !servoAttached) return;
  lastServoWriteValue = pos;
  //Serial.println(duty);

  if (SERVO_VIA_TIMER1) {
	int duty = int(double(map(int(pos), 0,180,544.0,2400.0))/SERVO_PWM_RATE*1024);
	Timer1.pwm(servoPin, duty);
  } else {
	//LIBServo: myservo.write(int(pos));
  }
}

double myservoread() {
  if (SERVO_VIA_TIMER1) {
	return lastServoWriteValue;
  } else {
	//LIBServo: return myservo.read();
  }
}

void myservoattach(int pin) {
  if (SERVO_VIA_TIMER1) {
	if (originalTCCR1A == 0) {
	  pinMode(pin, OUTPUT);
	  Timer1.initialize(SERVO_PWM_RATE);
	} else {
	  TCCR1A = originalTCCR1A;
	}
	servoAttached = true;
  } else {
	//LIBServo: myservo.attach(pin);
  }
}

boolean myservoattached() {
  if (SERVO_VIA_TIMER1) {
	return servoAttached;
  } else {
	//LIBServo: return myservo.attached();
  }
}
void myservodetach() {
  if (SERVO_VIA_TIMER1) {
	originalTCCR1A = TCCR1A;
	Timer1.disablePwm(servoPin);
	servoAttached = false;
  } else {
	//LIBServo: myservo.detach();
  }
}

void smoothMove(double desiredPosition, int totalTime = 2000) {
  double sl = myservoread();
  int iterations = totalTime/20;
  for (int i=0; i<=iterations; i++) {
	 double np = sl*double(iterations-i)/iterations + desiredPosition*double(i)/iterations;
	 myservowrite(np);
	 debugLog("moving Servo from ");debugLog(sl); debugLog(" to "); debugLog(desiredPosition);debugLog(" Step(");debugLog(i);debugLog(" of ");debugLog(iterations);debugLog("): "); debugLog(np); debugLog("\n\r");
	 delay(20);
  }
}

float getServoAngle() {
  return -(0.0+myservoread()-servoCenter)/100*(PI/2);
}

int angleToServo(float angle) {
  return angle/(PI/2)*100+servoCenter;
}

double getOcsilatorPos(){ 
  return sin(((time-initTime)%loopTime)*2*PI/loopTime - PI)*servoAmp/2+servoCenter;
}

///////////////////////////
/// Potentiometer
///////////////////////////

// Potentiometer position history
#define POSITIONS_STACK_SIZE 10
double positions[POSITIONS_STACK_SIZE]; 
unsigned int positionsPTR = POSITIONS_STACK_SIZE;
#define push(p) positions[(++positionsPTR)%POSITIONS_STACK_SIZE] = p
#define getPos(i) positions[(positionsPTR-i)%POSITIONS_STACK_SIZE]
#define lastPos getPos(0)


double potentiometerRead() {
  push(double(analogRead(potPin)));
  return posAvg();
}

float getPotAngle() {
  double p = potentiometerRead();
  return (0.0+p-potCenter)/(pot150-pot50)*(PI/2);
//  p-pot150
}


int posAvg() {
  int s = 0;
  for (int i=0; i< POSITIONS_STACK_SIZE; i++)
  {
	  s += positions[i];
  }
  return s/POSITIONS_STACK_SIZE;
}



//////////////////////////////
// Calibrate
//////////////////////////////

double waitForSteadiness(double threshold, int steadyTime=5000) {
  double potRead, maxRead, minRead;
  unsigned long t;
  do {
	potRead = potentiometerRead();
	maxRead=minRead=potRead;
	t = millis();
	while ((millis() < t + steadyTime))
	{
	  potRead = potentiometerRead();
	  minRead = min(minRead, potRead);
	  maxRead = max(maxRead, potRead);
	  delay(20);
	}
	sprint("Potentiometer is ");
	sprint((maxRead+minRead)/2);
	sprint("+-");
	sprint((maxRead-minRead)/2);
	sprint(". Threshold is ");
	sprintln(threshold);
  } while ((maxRead-minRead)/2 > threshold);
  return (maxRead+minRead)/2;
}

void calibrate() {
  //myservo.detach();
  servoAmp = 0;

  //String oldCalibration = String(servoCenter) + " " + String(potCenter) + " " + String(pot50) + " " + String(pot150) + " " + loopTime;
  sprint("Old ");sprint("servoCenter: "); sprintln(servoCenter);
  servoCenter = myservoread();
  sprint("servoCenter: "); sprintln(servoCenter);
  
  sprint("Old ");sprint("PotCenter: "); sprintln(potCenter);
  potCenter = waitForSteadiness(1,6000);  
  sprint("PotCenter:  "); sprintln(potCenter);

  smoothMove(servoCenter-50,4000); 
  sprint("Old ");sprint("Pot50: ");sprintln(pot50);
  pot50 = waitForSteadiness(10,6000);  
  sprint("Pot50:  ");sprintln(pot50);

  smoothMove(servoCenter+50,8000); 
  sprint("Old ");sprint("Pot150: "); sprintln(pot150);
  pot150 = waitForSteadiness(10,6000); 
  sprint("Pot150:  ");sprintln(pot150);
  
  calibrateLoopTime();

  smoothMove(servoCenter);
  sprint("Calibrate is:  ");
  sprint(servoCenter); sprint(" ");
  sprint(potCenter); sprint(" ");
  sprint(pot50); sprint(" ");
  sprint(pot150); sprint(" ");
  sprint(loopTime); sprintln("");
}

void calibrateLoopTime() { 
  sprintln("");
  sprint("Old LoopTime "); sprintln(loopTime);
  smoothMove(servoCenter-maxServoAmp);
  initTime = millis();
  sprintln("Speed up");
  unsigned long t = millis();
  mode = START;
  while (millis() < t + 9000) { loop();  }
  mode = HALT;
  loop();

  //sprintln("Move to center");
  smoothMove(servoCenter);

  sprintln("Delay 10 seconds");
  delay(10000);
  double potRead;
  int cycles = 0;
  int nCycles = 5;
  unsigned long initLeftTime = 0;
  side = LEFT;
  
  sprintln("Analyzing movement");
  do {
	 time = millis();
	 potRead = potentiometerRead();
	 if ((side==RIGHT) && (potRead < potCenter) && (time-rightTime>loopTime/5) ) {
		side = LEFT;
		leftTime = time;
		if (initLeftTime == 0) {
		   initLeftTime = time;
		   sprintln("Starting cycle analysis");
		} else {
		  cycles ++;
		  sprint("Cycle ");sprint(cycles);sprint(" of ");sprint(nCycles);sprint(" complete. Average loop time: "); sprintln((time-initLeftTime)/cycles);
		}
	 } else if ((side==LEFT) && (potRead > potCenter) && (time-leftTime>loopTime/5) ) {
		//sprintln("Right side");
		side = RIGHT;
		rightTime = time;
	 }
  } while ((cycles < nCycles)); // TIMEOUT:  && (millis() < initTime + 6000 + 3000 + 4000*(cycles+2)));

  defaultLoopTime = (cycles == nCycles) ? (time-initLeftTime)/(cycles) : loopTime;
  loopTime = defaultLoopTime;

  sprint("loopTime Calibration ("); sprint(cycles); sprint("): "); sprintln(loopTime); 
}

void writeCalibration() {
  ewrite(EEPROM_MAGIC,0); //magic number
  ewrite(2); // calibration version
  ewrite(servoCenter);
  ewrite(potCenter);
  ewrite(pot50);
  ewrite(pot150);
  ewrite(loopTime);
  ewrite((unsigned int)0);
  ewrite((unsigned int)0);
  ewrite((unsigned int)0);
  ewrite((unsigned int)0);
  
  sprint("EEPROM: ");
  printCurrentCalibration();
}

void printCurrentCalibration() {
  sprint("ServoCenter="); sprint(servoCenter);
  sprint(" potCenter="); sprint(potCenter);
  sprint(" pot50="); sprint(pot50);
  sprint(" pot150="); sprint(pot150);
  sprint(" loopTime="); sprint(loopTime);
  sprintln(""); 
}

void readCalibration() {
  if (eread(0) == EEPROM_MAGIC) { // confirm magic number
	eread(); // calibration version
	servoCenter = eread();
	potCenter = eread();
	pot50 = eread();
	pot150 = eread();
	loopTime = eread(); defaultLoopTime = loopTime;
  } else {
	sprintln("No EEPROM");
	//servoCenter = eread(1);
  }
  if (servoCenter>1000) servoCenter = 95;
  printCurrentCalibration();
}

///////////////////////////
/// User Data
///////////////////////////
#define KBSIZE 33
char KB[KBSIZE];
char *CMD=KB;

boolean commandAvailable() {
  return (isPlaying && (nextCommandTime != MAX_UINT) && (millis() > nextCommandTime*1000+playInitTime));
}


byte readByte() {
  unsigned long timeout = millis() + 100;
  char b;
  while (millis() < timeout) {
	if (Serial.available()) {
	  b = Serial.read();
	  //sprint(b);
	  return b;
	} else if (listenOnPrev && prevSerial.available()){
	  isMaster = false;
	  char b = prevSerial.read();
//	  if (b=='~') { Serial.write("Error:"); delay(100); while (prevSerial.available()) Serial.print("~"+String(int(prevSerial.read()))); Serial.println("");}
	  //sprint(b);
	  return b;
	}
  }
sprint("?");
}

bool readByteAvailable() {
  return (Serial.available() || (listenOnPrev && prevSerial.available()));
}



int find(char *str, char c) {
	for (int index = 0; str[index] != 0; index++) {
		if (str[index] == c) return index;
	}
	return -1;
}

char * forwardCommand() {
	if (KB[0]==0) return;
	if (isRecording) {
	   ewrite((millis()-recordInitTime)/1000, recordingLoc);
	   ewrite(KB);
	   recordingLoc = eIndex;
	   ewrite(MAX_UINT);
	}

	char k = KB[0];
	if (k==':') {
		 int index = 1+find(KB+1, ':');
		 if ((index==0) || (KB[index+1]==0)) {
		 	sprint("Bad KB: ");
		 	sprintln(KB);
		 	KB[0]=0; CMD=KB;
		 }
		 CMD = KB + index + 1;
		 if (KB[1]=='e') {
		 	KB[1] = 'o';
		 	nextSerial.println(KB);
		 } else if (KB[1]=='o') {
		 	KB[1] = 'e';
		 	nextSerial.println(KB);
		 	KB[0] = 0; CMD=KB; //ignore this command as it is not for this arduino.
		 } else if (KB[1]=='w') {
		 	nextSerial.println(KB);
		 	delay(50);
		 	nextSerial.println(CMD);
		 } else {
			 int id = atoi(KB+1);
			 if (id>0){
			   // id ==> id-1
			   itoa(id-1, KB+1, 10);
			   KB[strlen(KB)] = ' '; // remove the null terminated created by itoa;
			   CMD[-1] = ':'; // return the ':' if deleted by atoi			   
			   // notify other arduinos
			   nextSerial.println(KB);
			   KB[0] = 0; CMD = KB;
		 	} else {
			   // do not notify other arduinos
			   //sprintln("Command is directed to me only");
			 }
		 }
	 }
	 int s;
	 switch (k) {
	 // Do not forward the following commands
	   case 't':
	   case 's': /* will be send via the s command with the right synclooptime */
		  s = atoi(KB+1); 
		  if (s==0) {
		  	itoa(defaultLoopTime+30, KB+1, 10);
		  	sprint(KB+1);
		  	s = strlen(KB);
		  	KB[s++] = '\n';
		  	KB[s] = 0;
		  }
		  nextSerial.println(KB);
		  break;
	   case 'u': /* will be send via the u command with the right synclooptime */
		  s = atoi(KB+1); 
		  if (s==0) {
		  	itoa(int((millis()-(syncInitTime+syncInitTimeOffset)) % syncLoopTime), KB+1, 10);
		  	sprint(KB+1);
		  	s = strlen(KB);
		  	KB[s++] = '\n';
		  	KB[s] = 0;
		  }
		  nextSerial.println(KB);
		  break;
	   case ':':
	   case 'U':
	   case 'e':
	   case 'E':
	   case 'p':
	   case 'd':
	   case 'L':
	   case '=':
	   case '+':
	   case '-':
	   case 'P':
	   case 'R':
	   case 'Y':
	   case ' ':
	   case '\n':
		 break;
	   default:
		 nextSerial.println(KB);
		 break;
	  }
}

void handleKeyboardInput() {
	////////////
	// input
	bool cmdOriginFromRecordedSequence = false;
	int kblength = strlen(KB);
	char inByte = KB[kblength-1];
	if (commandAvailable() && (inByte != '\n')) {
	  ereadstr(nextCommandLoc, KB+kblength,KBSIZE-kblength);
	  kblength = strlen(KB);
	  nextCommandTime = eread();
	  nextCommandLoc = eIndex;
	  sprintln(KB);
	  if (kblength==0) {
   	  	return;
   	  }
  	  forwardCommand();
	  kblength = strlen(KB);
  	  KB[kblength++] = '\n';
  	  KB[kblength] = 0;
	  cmdOriginFromRecordedSequence = true;
	} else {
	  while (readByteAvailable() && (inByte != '\n')) {
		 inByte = readByte();
		 if ((inByte=='\r') || (inByte=='\n')) {
		   if (kblength>0) {
			   forwardCommand();
			   kblength = strlen(KB);
			   //convert \r to \n
			   inByte = '\n';
			   KB[kblength++] = inByte;
			   KB[kblength] = 0;
			   if ((kblength==2) && (KB[0]=='T')) {
			   	// don't print the T (update clock command);
			   	sprint("\r");
			   } else {
	  			   sprintln("");
	  		   }

			}
		 } else if (inByte == 127) {
		   //handle backspace
		   if (kblength > 0) {
			 KB[--kblength] = 0;
			 inByte = KB[kblength-1];
			 sprint("\b\b\b   \r");
			 sprint(KB);
		   } else {
			 inByte = 0;
		   }
		 } else if (inByte == ' ') {
		   // disregard spaces
		   inByte = KB[kblength-1];
		 } else {
		   KB[kblength++] = inByte;
		   KB[kblength] = 0;
		   sprint(inByte);
		 }
	  }
	}


  
  int CMDlength = strlen(CMD);
  if ((CMDlength == 0) || (CMD[CMDlength-1] != '\n')) {
  	CMD=KB;
	return;
  }
  char cmd = *(CMD++);
  
  if (cmd == '\n') {
	KB[0] = 0;CMD=KB;
	return;
  } 

  int s,p;
  switch (cmd) {
	case 'e': /* Enable output */
	  enablePrint = true;
	  break;
	case 'E': /* Disable output */
	  enablePrint = false;
	  break;
	case 'd': /* Debug */
	  debug = !debug;
	  enablePrint = true;
	  sprint("debug="); sprintln(debug ? "on" : "off");
	  break;
	case 'p': // Print measurements (potentiometer, servo, etc.)
	  printMeasures = !printMeasures;
	  enablePrint = true;
//	  sprint("printMeasures="); sprintln(printMeasures ? "on" : "off");
	  break;
	case 'L': // Print loop events (move from RIGHT to left)
	  showLoopEvents = !showLoopEvents;
	  enablePrint = true;
//	  sprint("showLoopEvents="); sprintln(showLoopEvents ? "on" : "off");
	  break;
	case '0': /* temporary move servo to center */
	  smoothMove(servoCenter);
	  break;
	case '1': // START 
	  mode = START; sprintln("START");
	  break;
	case '2': // STOP
	  mode = STOP; sprintln("STOP");
	  break;
	case '9': // HALT: STOP and dont move anymore;
	  mode = HALT; sprintln("HALT");
	  smoothMove(servoCenter);
	  break;
	case 'm': // MAINTAIN
	  mode = MAINTAIN; sprintln("MAINTAIN");
	  break;
	case 't': // TEST 
	  syncLoopTime = atoi(CMD); KB[0] = 0; CMD=KB;
	  syncInitTime = millis();
	  syncInitTimeOffset = 0;
	  syncRopeAngle = 0.2;

	  mode = TEST; sprintln("TEST");
	  break;
	case ']': /* Increase TEST Phase */
	  testPhase = (int((testPhase + 0.05)*100) % 100) /100.0;
	  sprint("tPhase="); sprintln(testPhase);

	  break;
	case '[': /* Decrease TEST phase */
	  testPhase = (int((testPhase - 0.05)*100) % 100) /100.0;
	  sprint("tPhase="); sprintln(testPhase);
	  break;
	case '>': /* Increase TEST amplitude */
	  testAmp = min(testAmp + 1,100);
	  sprint("tAmp="); sprintln(testAmp);
	  break;
	case '<': /* Decrease TEST amplitude */
	  testAmp = max(testAmp -1,0);
	  sprint("tAmp="); sprintln(testAmp);
	  break;
	case 's': // SYNC
	  syncLoopTime = atoi(CMD); KB[0] = 0; CMD=KB;

	  syncInitTime = millis();
	  syncInitTimeOffset = 0;
	  syncRopeAngle = 0.3;
	  //servoAmp = 20;
	  mode = SYNCED_RUN; sprint("SYNC");sprintln(syncLoopTime);
	  break;
	case 'T': /* Set the clock for SYNC */
	  syncInitTime = millis();
	  break;
	case 'u': // Print phase compared to sync clock
	  s = atoi(CMD); KB[0] = 0; CMD=KB;
//	  sprint ("clock: "); sprint(millis()); sprint ("; sIT: "); sprint(syncInitTime); sprint("; sITO"); sprint(syncInitTimeOffset); sprint("; sLT:");sprintln(syncLoopTime);
//	  sprint ("clock-sync: "); sprintln(millis()-syncInitTime);
	  sprint("sync: ");
	  sprint(int((millis()-(syncInitTime+syncInitTimeOffset) - s) % syncLoopTime));
	  break;
	case 'U': // Update slaves on current Sync Clock
	  updateSlaveClock = true;
	  break;
	case '{': // Offset the SYNC clock forward (1/32 of a loop)
	  syncInitTimeOffset -= syncLoopTime/32;
	  mode = SYNCED_RUN; 
	  break;
	case '}': // Offset the SYNC clock backwards (1/32 of a loop)
	  syncInitTimeOffset += syncLoopTime/32;
	  mode = SYNCED_RUN; 
	  break;
	case 'h': // Offset the Sync clock by half loop
	  syncInitTimeOffset -= syncLoopTime/2;
	  mode = SYNCED_RUN;
	  break;
	case 'r': // Randomize the SYNC clock 
	  syncInitTimeOffset = random(0,defaultLoopTime);
	  //servoAmp = 20;
	  mode = SYNCED_RUN; 
	  break;
	case 'S': /* SYNC to an already set clock (don't update the clock) */
	  // s = atoi(CMD); KB[0]=0; CMD=KB;
	  //	if (s!=0) syncLoopTime = s;
	  mode = SYNCED_RUN; 
	  syncInitTimeOffset = 0;
	  break;
/*
	case 'w': / Create a wave 
	 nextSerial.write("{");
	  mode = SYNCED_RUN; 
	  break;
	case 'W': / Create a backward wave
	 nextSerial.write("}");
	  mode = SYNCED_RUN; 
	  break;
*/
	case 'M': // Set magic number
	  sprint("Old ");
	  sprint("Magic=");
	  sprintln(SYNC_MAGIC_NUMBER);
	  s = atoi(CMD); KB[0]=0; CMD=KB;
	  SYNC_MAGIC_NUMBER = s;
	  sprint("Magic=");
	  sprintln(SYNC_MAGIC_NUMBER);
	  break;
	  
	case '=': // Move servo to specific location
	  s = atoi(CMD); KB[0]=0; CMD=KB;
	  if (s!=0) smoothMove(s);
	  sprint("servo="); sprintln(myservoread());	
	  // myservo.write(n);
	  break;
	case '+': // Move servo one step forwards
	  s = myservoread(); 
	  sprint("Old "); sprint("servo=");sprintln(s);
	  myservowrite(s+1);
	  sprint("servo="); sprintln(myservoread());	
	  break;
	case '-': // Move servo one step backwards
	  s = myservoread();
	  sprint("Old "); sprint("servo=");sprintln(s);
	  myservowrite(s-1);
	  sprint("servo="); sprintln(myservoread());	
	  break;
	case '_': // Detach or reattach servo
	  if (myservoattached()) {
		myservodetach();
	  } else {
		 myservoattach(servoPin);
	  }
	  sprint("servo="); sprintln(myservoattached() ? "attached" : "detached");
	  break;
   case '~': /* Change Servo type (Timer1 vs Servo library) */
	 myservodetach();
	 SERVO_VIA_TIMER1 = !SERVO_VIA_TIMER1;
	 myservoattach(servoPin);
	 sprintln(SERVO_VIA_TIMER1 ? "Using Timer1" : "Using Servo lib");
	 break; 
	case 'b': /* Beep */
	  tone(7, NOTE_A5, 1000);
	  break;
	case 'B': // Are you a master?
	  Serial.print("I am a ");
	  Serial.println(isMaster ? "master" : "slave"); 
	  if (isMaster) tone(7, NOTE_A5, 1000);
	  break;
	case 'c': // Calibrate
	  calibrate();
	  break;
	case 'C': // Save calibration
	  writeCalibration();
	  break;
	case 'l': /* Calibrate loop time */
	  calibrateLoopTime();
	  break;
	case '&': // Print current calibration
	  printCurrentCalibration();
	case 'a': // play audio
	  playSong(audioSongNumber, audioVolume);
	  break;
	case 'A': // Audio config for example A2,25 (song 2 volume 25)
	  if (CMD[0]=='\n'){
	  	enableAudio = !enableAudio;
	  }
	  s = atoi(CMD);
	  if (s>0) {
	  	audioSongNumber = s;
	  } 
	  p = find(CMD, ',');
	  if (p!=-1) {
		 s = atoi(CMD+p+1);
		 audioVolume = s;
	  }
	  KB[0] = 0; CMD=KB;
	  sprint("Audio "); sprint(enableAudio ? "on" : "off"); sprint(" song="); sprint(audioSongNumber); sprint(" vol="); sprintln(audioVolume);
	  break;
	case 'P': // Play
	  if (!isPlaying) {
		startPlaySequence();
	  } else {
		stopPlaySequence();
	  }
	  break;
	case 'R': // Record
	  if (cmdOriginFromRecordedSequence) {
	  	if (isPlaying) {
	  		if (isAutoPlay) {
	  			startPlaySequence();
	  		} else {
	  			stopPlaySequence();
	  		}
	  	} else {sprintln("this is weird");}
	  } else {
		if (isPlaying) stopPlaySequence();
		isRecording = !isRecording;
		sprint("Recording "); sprintln(isRecording ? "on" : "off");
		ewrite(EEPROM_MAGIC, EEPROM_COMMANDS_LOC - 2);
		recordingLoc = EEPROM_COMMANDS_LOC;
		recordInitTime = millis();
	  }
	  break;
	case 'Y': // Autoplay and loop on restart
	  isAutoPlay = !isAutoPlay;
	  sprint("auto play is ");
	  sprintln(isAutoPlay ? "on" : "off");
	  ewrite((int)isAutoPlay, EEPROM_COMMANDS_LOC - 4);
	  break;
	case '*': // Show Clock
	  showClock = !showClock;
	  break;
	case '@': // eprom access
	  s = atoi(CMD);
	  p = find(CMD, '=');
	  if (p==-1) {
	  	sprint("EPROM[");sprint(s);sprint("] = ");sprint((char) ereadbyte(s)); sprint(" <"); sprint(ereadbyte(s)); sprintln(">");
	  } else {
		ewritebyte(atoi(CMD+p+1),s);
	  	sprint("Set EPROM[");sprint(s); sprint("] to ");sprint((char) ereadbyte(s)); sprint(" <"); sprint(ereadbyte(s)); sprintln(">");
	  }
	  KB[0] = 0; CMD=KB;
	  break;
	
	default:
	  break;
  }
}


void startPlaySequence() {
	if (eread(EEPROM_COMMANDS_LOC-2) != EEPROM_MAGIC) {
		sprint("No recording saved");
	}
	// Print commands:
	unsigned int commandTime = eread(EEPROM_COMMANDS_LOC);
  	sprintln("Playing Sequence:");
  	while (commandTime != MAX_UINT) {
  		 sprint(commandTime);
  		 sprint("s: ");
  		 eprintstr();
  		 sprintln("");
  		 commandTime = eread();
	}
	
	nextCommandTime = eread(EEPROM_COMMANDS_LOC);
	nextCommandLoc = eIndex;
	if (nextCommandTime == MAX_UINT) {
		sprintln("No "); sprintln("Recorded Commands");
	} else {
	  	isPlaying = true;
	  	playInitTime = millis();
	}
}

void stopPlaySequence() {
  	sprint("Playback "); sprintln("stopped");
  	nextCommandTime = MAX_UINT;
  	isPlaying = false;
}
///////////////////////////
/// Modes
///////////////////////////

/*
void updateAmpAndTimeForStopping() {
  loopTime = defaultLoopTime;
  initTime = millis()-loopTime*(side==LEFT ? 0.25 : 0.75) + SYNC_MAGIC_NUMBER;

  servoAmp = (angleToServo(ropeMaxRightAngle)-angleToServo(ropeMaxLeftAngle));
  servoAmp = servoAmp*1.5;
  servoAmp = max(min(maxServoAmp,servoAmp),0);
//  servoAmp = servoAmp*servoAmp/40;
  if (servoAmp < 25) { servoAmp = servoAmp/2; } // was servoAmp/2

  if (servoAmp < 10) { servoAmp = 0; }		// was <10
  
  sprint("- Update ServoAmp: maxRight("); sprint(ropeMaxRightAngle); sprint(")-maxLeft("); sprint(ropeMaxLeftAngle);
  sprint(")="); sprint(ropeMaxRightAngle-ropeMaxLeftAngle);
  sprint(" ==> New servoAmp: "); sprint(servoAmp); sprintln(" -");  
}*/

void updateAmpAndTimeForMaintaining() {
  loopTime = defaultLoopTime;
//  initTime = millis()-loopTime*(side==LEFT ? 0.75 : 0.25) + SYNC_MAGIC_NUMBER;
  initTime = millis()-loopTime*(side==LEFT ? 0.5 : 0) + SYNC_MAGIC_NUMBER;
  // Maybe add MAINTAIN looptime:::  if (time-leftTime >2000) { loopTime = time-leftTime;}
  float desiredAngle = 0.4;
  float ropeAngle = ropeMaxRightAngle-ropeMaxLeftAngle;
  if ( ropeAngle > desiredAngle*1.2) {
	servoAmp = 0;
  } else if (ropeAngle > desiredAngle*1.1) {
	servoAmp = (angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8;
  }  else {
	servoAmp = (angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8 + (1 - ropeAngle/desiredAngle)*maxServoAmp;
  }
	sprint("ServoAmp: ");sprint(servoAmp); sprint(" baseline: "); sprintln((angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8);
}

void updateAmpAndTimeForRunning() {
  loopTime = defaultLoopTime;
  initTime = millis()-loopTime*(side==LEFT ? 0.75 : 0.25) + SYNC_MAGIC_NUMBER;
  servoAmp = maxServoAmp;
}

#define MAP(v,fromL,fromH,toL,toH) ((v-fromL)/(fromH-fromL)*(toH-toL) + toL)

//void updateAmpAndTimeForTesting() {
void updateAmpAndTimeForStopping() {
  double ropeAmp;
  static double lastRopeAmp=0;
  loopTime = defaultLoopTime;
//  servoAmp = testAmp;
//
//  initTime = millis()-loopTime-loopTime*(side==LEFT ? 0.5+testPhase : testPhase) + SYNC_MAGIC_NUMBER;
//  if (initTime == 0) { 
	 initTime = millis()-loopTime*(side==LEFT ? 0.25 : 0.75) + SYNC_MAGIC_NUMBER;
//	 sprintln("initTime set");
// }
/*  if ((millis()-initTime)> loopTime*5) {
	 mode = HALT;
	 sprintln("HALT");
  }*/
  
  sprint("- Stopping: maxRight("); sprint(ropeMaxRightAngle); sprint(")-maxLeft("); sprint(ropeMaxLeftAngle);
  sprint(")="); sprint(ropeMaxRightAngle-ropeMaxLeftAngle);
  ropeAmp = (angleToServo(ropeMaxRightAngle)-angleToServo(ropeMaxLeftAngle));
  sprint (" | a="); sprint(ropeAmp);
  
  if (lastRopeAmp > ropeAmp) {
  	servoAmp = max(0,ropeAmp-(lastRopeAmp-ropeAmp));
	sprint (" | c="); sprint(servoAmp);
  } else {
  	servoAmp = ropeAmp;
  }
  lastRopeAmp = ropeAmp;

  servoAmp = servoAmp * 1.5;
  
  if (servoAmp>=40) { 
	 servoAmp = maxServoAmp;
  } else if (servoAmp>=35) {
  	 servoAmp = MAP(servoAmp, 35, 40, 25, maxServoAmp);
   } else if (servoAmp>=10) {
	 servoAmp = MAP(servoAmp,10,35,10,25);
  } else if (servoAmp>=3) {
  	servoAmp = MAP(servoAmp, 3, 10, 0, 10); 
  } else {
  	servoAmp = 0;
  }

/*  servoAmp = servoAmp*1.5;
//  servoAmp = servoAmp*servoAmp/40;
  if (servoAmp < 25) { servoAmp = servoAmp; } // was 25,servoAmp/2

  if (servoAmp < 10) { servoAmp = max(0,servoAmp-2); }		// was <10,0
  */

  servoAmp = servoAmp*testAmp/20;

  servoAmp = max(min(maxServoAmp,servoAmp),0);
  sprint(" ==> New servoAmp: "); sprint(servoAmp); sprintln(" -");  
//  if (ropeMaxRightAngle-ropeMaxLeftAngle < 0.4) {
//	servoAmp = 10;
//  } else {
//	servoAmp = 0;
//  }
}
#define PREDICT(x, y) (x + (x-y))
#define PREDICT2(x, y) (x + (x-y)/2)

void updateAmpAndTimeForTesting() {
  double ropeAmp;
  static double lastRopeAmp = 0;
  float offset, phaseOffset;
  static float lastPhaseOffset = 0;
  bool tooStrong = false;
  int desiredServoAmp;
  
  double FASTER=0.6; double SLOWER=0.9;
  // update loopTime
  loopTime = syncLoopTime;

  // speed up or slow down (only do this when getting to right side - just to reduce amount of updates).
  if (side==RIGHT) {
	ropeAmp = ropeMaxRightAngle-ropeMaxLeftAngle;
	tooStrong = PREDICT(ropeAmp, lastRopeAmp) > syncRopeAngle;
	sprint(tooStrong ? "*" : ".");
	
	phaseOffset = ((millis()-(syncInitTime+syncInitTimeOffset))%syncLoopTime) / float (syncLoopTime);
	if (phaseOffset > 0.5) phaseOffset = phaseOffset-1;

	// predict next phase offset based on lastphase offset.
	offset = PREDICT(phaseOffset, lastPhaseOffset);
	// TODO What if phaseOffset is irregular???
	
	// Handle cyclic movement of offset. 
	if (offset > 0.5) offset = offset -1;
	if (offset < -0.5) offset = offset + 1;

	syncPhase = (offset > 0) ? FASTER : SLOWER;
	if (abs(offset) > 0.3) {
		servoAmp = (servoAmp+maxServoAmp)/2; // todo: if amp is too high and on direction of speeding up reduce servoAmp;
	} else if (abs(offset) > 0.15) {
		servoAmp = (servoAmp+maxServoAmp/4)/2; // todo: if amp is too high and on direction of speeding up reduce servoAmp;
	} else if (abs(offset) > 0.10) {
		servoAmp = (servoAmp+maxServoAmp/4)/2; // todo: if amp is too high and on direction of speeding up reduce servoAmp;
	} else if (abs(offset) > 0.05) {
		servoAmp = (3 * servoAmp + abs(PREDICT(ropeAmp,lastRopeAmp)-syncRopeAngle)*100 / 2 ) / 4;
		servoAmp = max(3,min(10, servoAmp));
	} else if (abs(offset) > 0.02) {
		// Linear calculation, offset:0==>phase:0.25; offset:0.05==>0.5; offset:-0.05==> 0; trim for phase to be between 0 to 0.5;
		syncPhase = max(min(0.25 + phaseOffset/0.05*0.25, 0.5), 0);

/* 		if (tooStrong) {
			syncPhase = max(min(0.75 + phaseOffset/0.05*0.2, 1), 0);
			syncPhase = 0.75;
		} else {
			syncPhase = max(min(0.25 + phaseOffset/0.05*0.2, 1), 0);
			syncPhase = 0.25;
		} */
		servoAmp = (3*servoAmp + max(min(abs(PREDICT(ropeAmp,lastRopeAmp)-syncRopeAngle)*100 / 2 , 6), 3)) / 4;
		
		//syncPhase = (phaseOffset > 0) ? 0.6 : 0.9;
		//servoAmp = ((ropeAmp-(lastRopeAmp-ropeAmp))>syncRopeAngle) ? 3 : 10;
	} else {
		// todo: still do minor fixes.
		//syncPhase = tooStrong ? 0.75 : 0.25;
		syncPhase = 0.5;
		servoAmp = (3*servoAmp + max(min(abs(PREDICT(ropeAmp,lastRopeAmp)-syncRopeAngle)*100 , 6) , 0)) / 4;
	}
	
	lastRopeAmp = ropeAmp;
	lastPhaseOffset = phaseOffset;

//	syncPhase = (desiredPhase*0.5 + syncPhase*0.5);
	initTime = millis()-loopTime*(side==LEFT ? syncPhase+0.5 : syncPhase);
	
	sprint("Testing: ropeAmp("); sprint(ropeAmp);
	sprint(") Phase(");sprint(int(phaseOffset*syncLoopTime)); sprint("ms / "); sprint(100*phaseOffset); sprint ("%");
	sprint(")/Offset(");sprint(int(offset*syncLoopTime));sprint("ms / "); sprint(100*offset); sprint("%");
	sprint(")  ==>  ServoAmp="); sprint(servoAmp);
	sprint("; syncPhase="); sprintln(syncPhase);
//	sprint("(wanted"); sprint(desiredPhase);
//	sprintln(")");
  }
}

void updateAmpAndTimeForSyncedRunning() {  

  // update loopTime
  //loopTime = syncLoopTime;

  // update servoAmp
  float offsetRopeAngle = ropeMaxRightAngle-ropeMaxLeftAngle - syncRopeAngle;
  if (offsetRopeAngle < 0) {
	servoAmp = min(servoAmp + 1, maxServoAmp);
  } else {
	servoAmp = max(servoAmp - 1, 3);
  }

//	servoAmp = max(min(servoAmp + min(10,offsetRopeAngle*100) ,maxServoAmp),3);

/*
  if (abs(offsetRopeAngle)>0.02) {
	servoAmp = max(min( servoAmp - offsetRopeAngle * 100 ,maxServoAmp),3);
  }
*/
  // speed up or slow down (only do this when getting to right side - just to reduce amount of updates).
  if (side==RIGHT) {
	double offset = ((millis()-(syncInitTime+syncInitTimeOffset))%syncLoopTime) / double(syncLoopTime);
	if (offset > 0.5) offset = offset-1;
	
	double desiredPhase = 0.25;
	if (abs(offset) <0.15) {
	  if (servoAmp>maxServoAmp-10)  {
		servoAmp = 20;
	  }
	  if (abs(offset) < 0.02) {
		desiredPhase = 0.25;
	  } else {
	  //	  // Linear calculation, offset:0==>phase:0.25; offset:0.05==>0.5; offset:-0.05==> 0; trim for phase to be between 0 to 0.5;
		desiredPhase = max(min(0.25 + offset/0.05*0.25, 0.5), 0);
	  }
	} else if (offset>0) {
	  desiredPhase = 0.6;
	  servoAmp = maxServoAmp;
	} else if (offset<0) {
	  desiredPhase = 0.9;
	  servoAmp = maxServoAmp;
	}
//	syncPhase = (desiredPhase*0.5 + syncPhase*0.5);
	syncPhase = desiredPhase;
	initTime = millis()-loopTime*(side==LEFT ? syncPhase+0.5 : syncPhase);
	
	sprint("Syncing: Offset("); sprint(offset);sprint(" ==> "); sprint(int(offset*syncLoopTime));sprint("ms), ropeAmp("); sprint(ropeMaxRightAngle-ropeMaxLeftAngle);
	sprint(") ==> loopTime="); sprint(syncLoopTime);
	sprint("; ServoAmp="); sprint(servoAmp);
	sprint("; phase="); sprint(syncPhase);
	sprint("(wanted"); sprint(desiredPhase);
	sprintln(")");
  }
}


void updateAmpAndTime() {
	if (mode == RUNNING)		{ updateAmpAndTimeForRunning();		  }
	if (mode == MAINTAINING)	{ updateAmpAndTimeForMaintaining();	  }
	if (mode == TESTING)		{ updateAmpAndTimeForTesting();		  }
	if (mode == STOPPING)	   { updateAmpAndTimeForStopping();		 }
	if (mode == SYNCED_RUNNING) { updateAmpAndTimeForSyncedRunning();	}
}



//////////////////////////////
// Main Code
//////////////////////////////
void setup() {
  KB[0] = 0; CMD=KB;
  pinMode(13, OUTPUT); digitalWrite(13, HIGH);
  Serial.begin(9600);
  Serial.print("v");
  Serial.print(PLUMMET_VERSION);

  nextSerial.begin(9600);
  prevSerial.begin(9600);

  nextSerial.println(" "); // let the following arduino know you are here;

  //prevSerial.attachInterrupt(t);
  delay(200);
  sendAudioCommand(0X09, 0X02); // Select TF Card
  sprintln("");
  readCalibration();
  sendAudioCommand(0X22, 0X1E01);
  
  myservoattach(servoPin);
  smoothMove(servoCenter);
  tone(7, NOTE_G5, 100);
  
  // Read all prevSerial data
  if (prevSerial.available()) {
  	  sprintln("prevdata?");
	  while (prevSerial.available()) {prevSerial.read(); };
	  delay(300);
	  if (prevSerial.available()) {
	  	sprintln("Noisy! Cancelling listen mode");
	  	listenOnPrev = false;
	  }
  }
  sprintln("Done.");
  
  mode=HALT;
  initTime = millis();
  randomSeed((initTime + int(potentiometerRead()*100)) % 30000);
  updateAmpAndTime();

  if (listenOnPrev) {
	  // wait 1.5 seconds to see if you are a slave
	  while ((millis() < initTime + 1500) && isMaster) {
		 if (prevSerial.available()) {isMaster = false; } 
	  }
	  Serial.println(isMaster ? "I am master" : "I am slave"); 
  } else {
	  Serial.println("I am probably master");
  }
  
  if ( eread(EEPROM_COMMANDS_LOC-2) == EEPROM_MAGIC) {
  	isAutoPlay = eread(EEPROM_COMMANDS_LOC-4);
  }
  if (isAutoPlay) startPlaySequence();

  pinMode(13, OUTPUT); digitalWrite(13, LOW);
}

unsigned long keepalive = 0;

left_right_e direction = RIGHT;
#define itIsTime(x) ((time>=x) && (lastIterationTime<x))

unsigned long audioTime=0;
void playAudioIn(int phase, int syncedPhase) {
	audioTime = time + phase;
	// Snap to Sync
	int offsetToSync = (audioTime - (syncInitTime+syncInitTimeOffset)) % syncLoopTime - syncedPhase;
	//sprint("offSetToSync: ");
	//sprintln(offsetToSync);
	if (abs(offsetToSync) <= audioSnapToSync) {
		audioTime = audioTime - offsetToSync;
	} else {
		int offsetToGrid = ((audioTime - (syncInitTime+syncInitTimeOffset)) % syncLoopTime) % audioSnapToGrid;
		//sprint("offsetToGrid: ");
		//sprintln(offsetToGrid);

		// new audioTime is either audioTime-offsetToGrid or audioTime-offsetToGrid+snapToGrid;
		if (offsetToGrid < audioSnapToGrid/2) {
			audioTime = audioTime - offsetToGrid;
		} else {
			audioTime = audioTime - offsetToGrid + audioSnapToGrid;
		}
		
	}
	//sprint("*** audio in ");
	//sprintln(audioTime - time);
}

void loop(){
  static unsigned long lastClock = 0;
  static int lastClockIter = 0;
  if (showClock) {
	if (millis() > lastClock + 3000) {
		lastClockIter = 0;
	}
  	if (millis() > lastClock + 100) {
  		lastClock= millis();
  		sprint(double(lastClockIter++)/10); sprint("  \r");
  	}
  }
  if (time > keepalive) { nextSerial.print(" "); keepalive = time + 1000; }  // inform slaves they are slaves every 1 seconds;
  if (listenOnPrev && prevSerial.available()) {if (isMaster) Serial.println("I am now a slave"); isMaster = false;} // inform 
  // if (isMaster) { tone(7, NOTE_F5,100); }   // If master make noise
  //if (millis()-lastIterationTime<50) {
  //	delay(50-(millis()-lastIterationTime));
  //}
  time = millis();
  
  //sprint(time-lastIterationTime);
  //sprint("...\r");

  handleKeyboardInput();

  // Update clock of slaves
  if (updateSlaveClock || (isMaster && ((mode==SYNCED_RUNNING) || (mode==TESTING)))) {
	if ((time-syncInitTime)%syncLoopTime  < (lastIterationTime-syncInitTime) % syncLoopTime) {
	  // this means we just got to the init time frame;
	  if (updateSlaveClock) {
		  updateSlaveClock = false;
		  sprint("Sync moved:");
	  	  sprintln((millis()-syncInitTime) % syncLoopTime);
	  }
	  syncInitTime = millis();
	  nextSerial.println("T"); // update the clock...	 
	}
  }


  int potRead = potentiometerRead();
  double currentServoPos = myservoread();
  float potAngle = getPotAngle();
  float servoAngle = getServoAngle();
  float ropeAngle = potAngle+servoAngle;

  ropeMaxRightAngle = max(ropeAngle,ropeMaxRightAngle);
  ropeMaxLeftAngle = min(ropeAngle,ropeMaxLeftAngle);
  
/*  if ((direction == RIGHT) && (ropeAngle < ropeMaxRightAngle)) {
  	direction = LEFT;
  	sprintln ("right peak");
  	ropeMaxLeftAngle = ropeAngle;
  } else if ((direction == LEFT) && (ropeAngle > ropeMaxLeftAngle)) {
  	direction = RIGHT;
  	sprintln ("left peak");
  	ropeMaxRightAngle = ropeAngle;
  }
  */

  if (printMeasures) {
	sprint(" potRead: "); sprint(potRead);
	sprint(" ServoPos: "); sprint(currentServoPos);
	sprint(" potAngle: "); sprint(potAngle);
	sprint(" servoAngle: "); sprint(servoAngle);
	sprint(" ropeAngle: "); sprint(ropeAngle);
	sprintln("");
  }
  
  switch (mode) {
	case SYNCED_RUN:
	  mode = SYNCED_RUNNING;
	  break;
	case TEST:
	  initTime = 0;
	  mode = TESTING;
	  updateAmpAndTime();
	  sprint("testPhase: "); sprintln(testPhase);
	  break;
	case START:
	  mode = RUNNING;
	  updateAmpAndTime();
	  break;
	case STOP:
	  mode = STOPPING;
	  updateAmpAndTime();
	  break;
	case MAINTAIN:
	  mode = MAINTAINING;
	  updateAmpAndTime();
	  break;
	default:
	  break;
  }

  if (itIsTime(audioTime+audioDelay) &&
	  (ropeMaxRightAngle>0.05) && (ropeMaxLeftAngle<-0.05)) {
	  playSong(audioSongNumber,audioVolume);
  }

  if ((ropeAngle<0) && (side==RIGHT) && (time-rightTime>loopTime/4)) {
	  // This section is if we are now moving to the left side;
	side = LEFT;
	lastLoopTime = time-leftTime;
	leftTime = time;
	if (showLoopEvents) {
		sprint("# Loop: Time(");sprint(lastLoopTime);sprint(")"); sprint(side==LEFT ? " [L] :" : " [R]:");
		sprint("maxRight(");sprint(ropeMaxRightAngle); sprint(")-maxLeft("); sprint(ropeMaxLeftAngle);
		sprint(")=");sprint(ropeMaxRightAngle-ropeMaxLeftAngle); sprintln(" #");
	}
	tone(7, NOTE_G6, 100);

	if (mode != RUNNING) updateAmpAndTime();

	ropeMaxLeftAngle = 0;
	  
  } else if ((ropeAngle>0) && (side==LEFT)&& (time-leftTime>loopTime/4)) {
	// This section is if we are now moving to the right side;
	side = RIGHT;
	lastLoopTime = time-rightTime;
	rightTime = time;
	playAudioIn(loopTime/4,syncLoopTime/4);
	if (showLoopEvents) {
		sprint("# Loop: Time(");sprint(lastLoopTime);sprint(")"); sprint(side==LEFT ? " [L] :" : " [R]:");
		sprint("maxRight(");sprint(ropeMaxRightAngle); sprint(")-maxLeft("); sprint(ropeMaxLeftAngle);
		sprint(")=");sprint(ropeMaxRightAngle-ropeMaxLeftAngle); sprintln(" #");
	}
	tone(7, NOTE_G5, 100);
	
	if (mode != RUNNING) updateAmpAndTime();
	
	ropeMaxRightAngle = 0;
  }
  
/*
  debugLog("Mode: "); debugLog(String(mode)); debugLog("  ");
  debugLog("currentServoPos: "); debugLog(String(currentServoPos)); debugLog("  ");
  debugLog("servoAngle: "); debugLog(String(servoAngle)); debugLog("  ");
  debugLog("pot: "); debugLog(String(potRead)); debugLog("  ");
  debugLog("potAngle: "); debugLog(String(potAngle)); debugLog("  ");
  debugLog("ropeAngle: "); debugLog(String(ropeAngle)); debugLog("  ");
*/
  // Set oscilator movement if needed;
  if ((mode==STOPPING) || (mode==RUNNING) || (mode==MAINTAINING) || (mode==TESTING) || (mode==SYNCED_RUNNING)) {
	double desiredServoPos = getOcsilatorPos();
	smoothWrite(desiredServoPos);
	//myservowrite(desiredServoPos);
	//myservo.writeMicroseconds(sin(((time-initTime)%loopTime)*2*PI/loopTime - PI)*100/2+1500);
/*
	debugLog("desiredServoPos: "); debugLog(String(desiredServoPos)); debugLog("  ");
*/
  }

/*
   debugLog("\n\r");
*/

   lastIterationTime = time;

}

