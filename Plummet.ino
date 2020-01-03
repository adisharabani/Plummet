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
//	Make 1 work for 6 seconds
//  Make sync faster - predict future movements
//  Update stop based on new sync

// TODO: Better master detection
// TODO: ML for circular phase options.
// TODO: start move even if potCenter is not at the right place.
// TODO: autoupdate potCenter
// TODO: better speed up in calibrate loop time
// TODO: Sync clock even if not master
// TODO: Stop calibration
// TODO: Git pull different versions


#define PLUMMET_VERSION "0.22"
#define USE_SERVOLIB_NOT

// #include <SoftwareSerial.h>
#include <NeoSWSerial.h>
#define SoftwareSerial NeoSWSerial

#include <TimerOne.h>

#ifdef USE_SERVOLIB
#include <Servo.h>
#endif

#include <EEPROM.h>

#define rxPinPrev 2 // soft serial
// #define txPinPrev 3 // soft serial
// #define rxPinNext 4 // soft serial
#define txPinNext 5 // soft serial
#define tonePin 7 // digital
#define servoPin 10 // digital

#define potPin 1 // analog


int8_t myID = -1;
int SYNC_MAGIC_NUMBER=-250;

int defaultLoopTime = 3167; //Palo Alto: 3080; // 3160; // 3420;
// int defaultLoopTime = 3080; // Palo Alto


#define EPPROM

// Calibrate(1): 86.00 489 711 293
// Calibrate(2): 98.00 457 665 261
// Calibrate(3): 101.00 525 351 697
#ifdef USE_SERVOLIB
boolean SERVO_VIA_TIMER1 = false;
#else
boolean SERVO_VIA_TIMER1 = true;
#endif

boolean printMeasures = false; 
boolean enablePrint = true;
boolean debug = false;
boolean showShift = true;
boolean enableAudio = false;
int8_t audioSongNumber = 1;
int8_t maxAudioVolume = 30;
int8_t audioVolume = maxAudioVolume;
bool audioVolumeAdaptive = true;
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

double maxSpeed = 2;  // PI/2 should be enough- pot will move at max this speed (compared to average cycle speed based on servoAmp);
double maxServoAmp = 80;
double servoAmp = maxServoAmp;
double servoCenter = 100; // 88; //100 // 95;

int potCenter = 443; // 483; // 443; // 481;
int pot50 = 650; // 700; // 650;  // 343;
int pot150 = 240; // 300; // 240; // 620;

double testPhase = 0.25;
double testAmp = 20;

int oAmp = maxServoAmp;
double oPhase = 0.25; // like stopping

double mPhase = 0;
int mAmp = 5;
double mPhaseFrom = 0;
double mPhaseTo = 0.30;
double mPhaseJump = 0.02;
int mAmpFrom = 0;
int mAmpTo = 40;
int mAmpJump = 2;

unsigned long initTime;

unsigned long time;
unsigned long lastIterationTime  = millis();

int requestedNLoops = 0;
bool requestedMoveStarted = false;


double ropeMaxLeftAngle = 0;
double ropeMaxRightAngle = 0;

unsigned long syncInitTime = 0;
int syncInitTimeOffset = 0;
int syncLoopTime=3501;
double syncRopeAngle=0.25;
double syncPhase;
boolean updateSlaveClock = false;

//SoftwareSerial prevSerial(rxPinPrev, txPinPrev);
//SoftwareSerial nextSerial(rxPinNext, txPinNext);
SoftwareSerial nextSerial(rxPinPrev, txPinNext);
#define prevSerial nextSerial

#define audioSerial Serial

boolean isMaster = false;

unsigned long rightTime = 0;
unsigned long leftTime = 0;

unsigned long startTime = millis();

enum left_right_e {
  LEFT,
  RIGHT
};

left_right_e side = LEFT;

enum mode_e {
  STOPPING,
  HALT,
  RUNNING,
  MAINTAINING,
  TESTING,
  SYNCED_RUNNING,
  ANALYZING
};
mode_e mode;

void updateAmpAndTime(bool runNow=false); 
void setMode(mode_e m) {
	mode = m;
	// halt should stop 
    if ( mode==HALT ) {
    	requestedNLoops = 0;
    	requestedMoveStarted = false;
    }
	updateAmpAndTime(true);
}


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
	sendAudioCommand(0x22, volume, songNumber);
}


///////////////////////////
/// SERVO
///////////////////////////
#ifdef USE_SERVOLIB
Servo myservo;  // create servo object to control a servo
#endif
double lastServoWriteValue = 95;
#define SERVO_PWM_RATE 6040
boolean servoAttached = false;
static int originalTCCR1A = 0;

double smoothWrite(double desiredPosition) {
  static double lastWrite = desiredPosition;
  static unsigned long lastWriteTime = 0;
  //myservowrite(desiredPosition); return desiredPosition;
  double maxServoMove = double(millis()-lastWriteTime) * maxServoAmp*2.0 / defaultLoopTime * maxSpeed; // Max 3 times average speed under current servo.
  maxServoMove = min(maxServoMove,1);
  double servoPosition_new = max(min(desiredPosition, lastWrite+maxServoMove), lastWrite-maxServoMove);
//  if (servoPosition_new != desiredPosition) {
	//sprintln("^^^ desiredPosition="+String(desiredPosition)+" lastServoWriteValue="+String(lastServoWriteValue)+" MaxMove="+String(maxServoMove) + "  newpos="+String(servoPosition_new));
//  }
  myservowrite(servoPosition_new);
  
  lastWrite = servoPosition_new;
  lastWriteTime = millis();
  
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
#ifdef USE_SERVOLIB
	myservo.write(int(pos));
#endif
  }
}

double myservoread() {
  if (SERVO_VIA_TIMER1) {
	return lastServoWriteValue;
  } else {
#ifdef USE_SERVOLIB
	return myservo.read();
#endif

  }
}

void myservoattach(int pin) {
  if (SERVO_VIA_TIMER1) {
	if (originalTCCR1A == 0) {
	  pinMode(pin, OUTPUT);
	  Timer1.initialize(SERVO_PWM_RATE);
	} else {
	  TCCR1A = originalTCCR1A;
	  Timer1.resume();
	}
	servoAttached = true;
  } else {
#ifdef USE_SERVOLIB
	myservo.attach(pin);
#endif
  }
}

boolean myservoattached() {
  if (SERVO_VIA_TIMER1) {
	return servoAttached;
  } else {
#ifdef USE_SERVOLIB
	return myservo.attached();
#endif
  }
}
void myservodetach() {
  if (SERVO_VIA_TIMER1) {
	originalTCCR1A = TCCR1A;
	Timer1.disablePwm(servoPin);
	Timer1.stop();
	servoAttached = false;
  } else {
#ifdef USE_SERVOLIB
	myservo.detach();
#endif
  }
}

void smoothMove(double desiredPosition, int totalTime = 1000) {
  double sl = myservoread();
  int iterations = totalTime/20;
  for (int i=0; i<=iterations; i++) {
	 double np = sl*double(iterations-i)/iterations + desiredPosition*double(i)/iterations;
	 myservowrite(np);
	 debugLog("moving Servo from ");debugLog(sl); debugLog(" to "); debugLog(desiredPosition);debugLog(" Step(");debugLog(i);debugLog(" of ");debugLog(iterations);debugLog("): "); debugLog(np); debugLog("\n\r");
	 delay(20);
  }
}

double getServoAngle() {
  return -(0.0+myservoread()-servoCenter)/100*(PI/2);
}

int angleToServo(double angle) {
  return angle/(PI/2)*100+servoCenter;
}

double getOcsilatorPos(unsigned long time) {
  return sin(getOcsilatorPhase(time))*servoAmp/2+servoCenter;
}
double getOcsilatorPhase(unsigned long time) {
	return ((time-initTime)%loopTime)*2*PI/loopTime - PI;
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

double getPotAngle() {
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
  sprint("PotCenter: "); sprintln(potCenter);

  smoothMove(servoCenter-50,4000); 
  sprint("Old ");sprint("Pot50: ");sprintln(pot50);
  pot50 = waitForSteadiness(4,6000);  
  sprint("Pot50: ");sprintln(pot50);

  smoothMove(servoCenter+50,8000); 
  sprint("Old ");sprint("Pot150: "); sprintln(pot150);
  pot150 = waitForSteadiness(4,6000); 
  sprint("Pot150: ");sprintln(pot150);
  
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
  sprint("Old LoopTime "); sprintln(defaultLoopTime);
  smoothMove(servoCenter);
  initTime = millis();
  sprintln("Speed up");
  unsigned long t = millis();
  setMode(RUNNING);
  while (millis() < t + 15000) { loop();  }
  setMode(HALT);
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
  ewrite(defaultLoopTime);
  ewrite(isMaster ? 0 : 1);
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
  sprint(" loopTime="); sprint(defaultLoopTime);
  sprint(" isMaster="); sprint(isMaster);
  sprintln(""); 
}

void readCalibration() {
  if (eread(0) == EEPROM_MAGIC) { // confirm magic number
	eread(); // calibration version
	servoCenter = eread();
	potCenter = eread();
	pot50 = eread();
	pot150 = eread();
	defaultLoopTime = eread(); loopTime = defaultLoopTime;
	isMaster = (eread() == 0);
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
	} else if (!isMaster && prevSerial.available()){
	  char b = prevSerial.read();
//	  if (b=='~') { Serial.write("Error:"); delay(100); while (prevSerial.available()) Serial.print("~"+String(int(prevSerial.read()))); Serial.println("");}
	  //sprint(b);
	  return b;
	}
  }
sprint("?");
}

bool readByteAvailable() {
  return (Serial.available() || (!isMaster && prevSerial.available()));
}

int find(char *str, char c, int startingIndex=0) {
	for (int index = startingIndex; str[index] != 0; index++) {
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
		 } else if (KB[1]=='m') {
                        if (!isMaster) {
                           KB[0] = 0; CMD=KB; //ignore command if you are not master
                        }
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
	   	  break;
	   case 's':
 		  // add defaultLoop if not set
 		  s = atoi(KB+1); 
		  if (s==0) {
		  	itoa(defaultLoopTime, KB+1, 10);
		  	sprint(KB+1);
		  	s = strlen(KB);
		  	KB[s++] = '\n';
		  	KB[s] = 0;
		  }
		  nextSerial.println(KB);
		  break;
	   case 'u':
	       // Current device phase so that everyone will show the clock compared to this.
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
	   case 'B':
	      s = atoi(KB+1);
	   	  if (isMaster || s>0) {
	   	     myID = s;
	   	     itoa(s+1, KB+1, 10);
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
	   case '@':
	   case ' ':
	   case '^':
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
	  ereadstr(nextCommandLoc, KB, KBSIZE);
	  kblength = strlen(KB);
	  nextCommandTime = eread();
	  nextCommandLoc = eIndex;
	  sprint("command: "); sprint(KB);
	  if (nextCommandTime != MAX_UINT) {
		  sprint(" (in "); sprint(nextCommandTime-(millis()-playInitTime)/1000); sprint("s:");eprintstr();sprint(")");
	  }
	  sprintln("");
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
			   /*if ((kblength==2) && (KB[0]=='T')) {
			   	// don't print the T (update clock command);
			   	sprint("\r \r");
			   } else {
	  			   sprintln("");
	  		   }*/ sprintln("");

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
		 } else if (inByte == 'T') {
		   // Handle clock sync
		   int s = (millis()-syncInitTime) % syncLoopTime;
		   if (s>syncLoopTime/2) s = s-syncLoopTime;
		   syncInitTime = millis();
		   nextSerial.write("T");
		   if ((abs(s) >= 20) && showShift) {
		   	 sprint("["); sprint(syncInitTime); sprint (" / "); sprint(syncLoopTime); sprint("] shift: "); sprint(s); sprint("     \r");
		   }
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
  double d;
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
	  if (CMD[0]=='\n') {
		  printMeasures = !printMeasures;
		  enablePrint = true;
	  } else {
	  	if (CMD[0]=='=') {
	  		sprint("Old ");
	  		sprint("PotCenter: "); sprintln(potCenter);
	  		potCenter = atoi(CMD+1);
	  		sprint("PotCenter: "); sprintln(potCenter);
	  			  		
	  		KB[0]=0;CMD=KB;
	  	} 
	  }
//	  sprint("printMeasures="); sprintln(printMeasures ? "on" : "off");
	  break;
	case 'L': // Print loop events (move from RIGHT to left)
	  showLoopEvents = !showLoopEvents;
	  enablePrint = true;
	  if (CMD[0]=='L') {
	  	sprintLoopEvents();
	  }
	  if (CMD[0]=='0') {
	  	ropeMaxRightAngle = 0;
	  	ropeMaxLeftAngle = 0;
	  }
//	  sprint("showLoopEvents="); sprintln(showLoopEvents ? "on" : "off");
	  break;
	case '0': /* temporary move servo to center */
	  smoothMove(servoCenter);
	  break;
	case '1': // START 
	  setMode(RUNNING); sprintln("START");
	  break;
	case '2': // STOP
	  setMode(STOPPING); sprintln("STOP");
	  break;
	case '9': // HALT: STOP and dont move anymore;
	  setMode(HALT); sprintln("HALT");
	  smoothMove(servoCenter);
	  break;
	case 'm': // MAINTAIN
      if (CMD[0]!='\n') mPhaseFrom = atof(CMD);
	  p = find(CMD, ',');
	  if (p!=-1) mPhaseTo = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mPhaseJump = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mAmpFrom = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mAmpTo = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mAmpJump = atof(CMD+p+1);
	  KB[0] = 0; CMD=KB;

	  mAmp = mAmpFrom;
	  mPhase = mPhaseFrom;
	  setMode(MAINTAINING); sprint("ML phase("); sprint(mPhaseFrom);sprint("-");sprint(mPhaseTo);sprint(" +");sprint(mPhaseJump);sprint(") amp(");sprint(mAmpFrom);sprint("-");sprint(mAmpTo);sprint(" +");sprint(mAmpJump); sprintln(")");
	  break;
	case 'o': // ANALYZE Amp,Phase
	  if (CMD[0]!='\n'){
		  oAmp = atoi(CMD);
		  p = find(CMD, ',');
		  if (p!=-1) oPhase = atof(CMD+p+1);
	  }
	  setMode(ANALYZING);
	  KB[0] = 0; CMD=KB;
	  break;
	case 't': // TEST 
	  if (CMD[0]!='\n'){
		  testAmp = atoi(CMD);
		  p = find(CMD, ',');
		  if (p!=-1) testPhase = atof(CMD+p+1);
	  }
	  KB[0] = 0; CMD=KB;

	  setMode(TESTING); sprint("TEST");
	  break;
	case 's': // SYNC
	  syncLoopTime = atoi(CMD); 
	  if (syncLoopTime == 0) syncLoopTime = defaultLoopTime;
	  syncInitTime = millis();
	  syncInitTimeOffset = 0;
	  p = find(CMD, ',');
	  
	  if (p!=-1) {
	  	d = atof(CMD+p+1);
	  	if (d != 0) syncRopeAngle = d;
	  }
	  
	  KB[0] = 0; CMD=KB;
	  setMode(SYNCED_RUNNING); sprint("SYNC");sprint(syncLoopTime); sprint(","); sprintln(syncRopeAngle);
	  break;
	case '[':
	  syncRopeAngle = min(max(syncRopeAngle-0.01,0),0.4);
	  sprint("SyncRopeAngle: ");
	  sprintln(syncRopeAngle);
	  break;
	case ']':
	  syncRopeAngle = min(max(syncRopeAngle+0.01,0),0.4);
	  sprint("SyncRopeAngle: ");
	  sprintln(syncRopeAngle);
	  break;
    case '%': // disable clock update
      showShift = !showShift;
      sprintln(showShift);
      break;
	case 'u': // Print phase compared to sync clock
	  s = atoi(CMD); KB[0] = 0; CMD=KB;
	  sprint("sync: ");
	  s = int((millis()-(syncInitTime+syncInitTimeOffset) - s) % syncLoopTime);
	  if (s>syncLoopTime/2) s=s-syncLoopTime;
	  sprintln(s);
	  break;
	case 'U': // Update slaves on current Sync Clock
	  updateSlaveClock = true;
	  break;
	case '{': // Offset the SYNC clock forward (1/32 of a loop)
	  syncInitTimeOffset -= syncLoopTime/32;
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  setMode(SYNCED_RUNNING); 
	  break;
	case '}': // Offset the SYNC clock backwards (1/32 of a loop)
	  syncInitTimeOffset += syncLoopTime/32;
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  setMode(SYNCED_RUNNING);
	  break;
	case 'h': // Offset the Sync clock by half loop
	  syncInitTimeOffset -= syncLoopTime/2;
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  setMode(SYNCED_RUNNING);
	  break;
	case 'r': // Randomize the SYNC clock 
	  syncInitTimeOffset = random(0,defaultLoopTime);
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  //servoAmp = 20;
	  setMode(SYNCED_RUNNING); 
	  break;
	case 'S': /* SYNC to an already set clock (don't update the clock) */
	  syncInitTimeOffset = atoi(CMD); KB[0]=0; CMD=KB;
	  //	if (s!=0) syncLoopTime = s;
      setMode(SYNCED_RUNNING);
	  break;
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
	  Serial.print("I am ");
	  Serial.print(isMaster ? "master" : "slave"); 
	  Serial.print(" #");
	  Serial.println(myID);
      KB[0] = 0; CMD = KB;

	  if (isMaster) tone(7, NOTE_A5, 1000);
	  break;
	case '^': // toggle master/slave
	  isMaster = !isMaster;
	  ewrite(isMaster ? 0 : 1,14);
	  Serial.print("I am ");
	  Serial.println(isMaster ? "master" : "slave");
	  break; 
	case 'i':
	  Serial.print("I am ");
	  Serial.print("#");
	  Serial.println(myID);
	  if (isMaster && (myID==-1)) {
	  	myID = 0;
	  	nextSerial.print("B1");
	  }
	  break;
	case 'c': // Calibrate
	  calibrate();
	  break;
	case 'C': // Save calibration
	  writeCalibration();
	  break;
	case 'l': /* Calibrate loop time */
	  if (CMD[0] == '=') {
	  	  defaultLoopTime = atoi(CMD+1);
	  	  loopTime = defaultLoopTime;
	  	  sprint("loopTime=");sprintln(loopTime);
	  } else {
		  calibrateLoopTime();
	  }
	  KB[0] = 0; CMD = KB;
	  break;
	case '&': // Print current calibration
	  printCurrentCalibration();
	case 'a': // play audio
	  playSong(audioSongNumber, maxAudioVolume);
	  break;
	case 'A': // Audio config #,Vol,STG,STS,delay,adaptive?
	  if (CMD[0]=='\n'){
	  	enableAudio = !enableAudio;
	  }
	  s = atoi(CMD);
	  if (s>0) audioSongNumber = s;

	  p = find(CMD, ',');
	  if (p!=-1) maxAudioVolume = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioSnapToGrid = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioSnapToSync = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioDelay = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioVolumeAdaptive = CMD[p+1]!='0';
	  KB[0] = 0; CMD=KB;
	  sprint("Audio "); sprint(enableAudio ? "on" : "off"); sprint(" song="); sprint(audioSongNumber); sprint(" vol="); sprint(maxAudioVolume); sprint(" stg=");sprint(audioSnapToGrid);sprint(" sts=");sprint(audioSnapToSync); sprint(" delay=");sprint(audioDelay); sprint(" adaptive="); sprintln(audioVolumeAdaptive);
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
	  if (CMD[0]=='@') {
	  	readCalibration();
	  } else {
		  s = atoi(CMD);
		  p = find(CMD, '=');
		  if (p==-1) {
		  	sprint("EPROM[");sprint(s);sprint("] = ");sprint((char) ereadbyte(s)); sprint(" <"); sprint(ereadbyte(s)); sprintln(">");
		  } else {
			ewritebyte(atoi(CMD+p+1),s);
		  	sprint("Set EPROM[");sprint(s); sprint("] to ");sprint((char) ereadbyte(s)); sprint(" <"); sprint(ereadbyte(s)); sprintln(">");
		  }
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

double axisToAngle(double x, double y) {
  if (x == 0 && y == 0) {
  	return 0;
  } else if (x == 0 && 0 < y) {
    return PI / 2;
  } else if (x == 0 && y < 0) {
    return PI * 3 / 2;
  } else if (x < 0) { //x != 0
    return atan(y / x) + PI;
  } else if (y < 0) {
    return atan(y / x) + 2 * PI;
  } else {
    return atan(y / x);
  }
}

#define PREDICT(x, y) (x + (x-y))
#define PREDICT2(x, y) (x + (x-y)/2)


void updateAmpAndTime(bool runNow=false) {
	if (mode==HALT) return;
	
	static const uint8_t LOOP_INTERVAL = 3;
	static double ML_ROPE_ANGLE_DECREASE = 0.05; // per cycle.  
	static double ML_MAX_ROPE_SHIFT_IN_CYCLE = 0.18;
	static double ML_MAX_OFFSET_SHIFT_IN_CYCLE = 250.0;

	static int waitLoops=0;
	static double lastOffsetAxis = 0;
	static double lastRopeOffsetAxis = 0;
	static unsigned long lastTime = 0;
	static double lastRopeAngle = 0;
	static boolean inTest = false;

	double tPhase;
	double ropeAngle = (ropeMaxRightAngle-ropeMaxLeftAngle);
	double ropeAngleOffset = ropeAngle-syncRopeAngle;
	int offset = (rightTime+syncLoopTime-(syncInitTime+syncInitTimeOffset)) % syncLoopTime;

	if (offset > syncLoopTime/2) offset = offset - syncLoopTime;

	
	if (runNow) {
//		waitLoops = 0;
	}

	if (((side==RIGHT) && ((mode!=TESTING) && (mode != STOPPING))) ||
	    ((side==LEFT)  && ((mode==TESTING) || (mode == STOPPING))) ||
	    runNow) {
		sprint(waitLoops ? "\x1b[0;37m" : "\x1b[0;31m");
		sprint ("["); sprint(lastLoopTime); sprint("]: offset="); sprint(offset); sprint("ms ropeAngle="); 	sprint(ropeAngle); sprint((ropeAngleOffset > 0) ? "(+" : "(");sprint(ropeAngleOffset); sprint(")");
		sprint("\x1b[0m");
		
		if (requestedMoveStarted && !runNow) { sprint("in motion [");sprint(requestedNLoops);sprintln("]"); return; }
		if (waitLoops > 0) { sprint("wait ["); sprint(waitLoops--); sprintln("]"); /*servoAmp = 0;*/ return; }
		
		double offsetAxis = offset / ML_MAX_OFFSET_SHIFT_IN_CYCLE;
		double ropeOffsetAxis = ropeAngleOffset / ML_MAX_ROPE_SHIFT_IN_CYCLE;

		if (false) { // show analysis
			double tetaLastTarget = axisToAngle(-lastOffsetAxis,-lastRopeOffsetAxis)/2/PI;
			double rLastTarget = sqrt(pow(lastOffsetAxis,2) + pow(lastRopeOffsetAxis,2));
	
	        double tetaLastActual = axisToAngle(offsetAxis - lastOffsetAxis, ropeOffsetAxis-lastRopeOffsetAxis)/2/PI;
			double rLastActual = sqrt(pow(offsetAxis-lastOffsetAxis,2) + pow(ropeOffsetAxis - lastRopeOffsetAxis,2));
			sprintln("");
			sprint("Aimed("); sprint(-lastOffsetAxis); sprint(","); sprint(-lastRopeOffsetAxis);
			sprint(") @(");    sprint(rLastTarget);            sprint(","); sprint(tetaLastTarget);
			sprint(") Actual(");  sprint(-offsetAxis);     sprint(","); sprint(-ropeOffsetAxis);
			sprint(") @(");    sprint(rLastActual);            sprint(","); sprint(tetaLastActual);
			sprint(")  ==>  Delta("); sprint(-offsetAxis+lastOffsetAxis);sprint(","); sprint(-ropeOffsetAxis+lastRopeOffsetAxis);
			sprint(") @("); sprint(rLastActual/rLastTarget); sprint(","); sprint(tetaLastActual-tetaLastTarget); 
			sprint(")");
		}
		sprintln("");


		//predict
		offsetAxis = offsetAxis + (defaultLoopTime-syncLoopTime) * LOOP_INTERVAL / ML_MAX_OFFSET_SHIFT_IN_CYCLE; //predict increase in offset in offsetAxis grid (hence devide by ML_MAX..)

        ropeOffsetAxis = ropeOffsetAxis - ML_ROPE_ANGLE_DECREASE*ropeAngle * LOOP_INTERVAL / ML_MAX_ROPE_SHIFT_IN_CYCLE ; //predict decrease in rope Angle in percentage
		
		// sprint("** "); sprint(offsetAxis); sprint(",");sprintln(ropeOffsetAxis);
		//if synced running:
		
		// if there might be more than 1 cycle change the ropeOffset so that system will slow things down.
		
		tPhase = axisToAngle(-offsetAxis,-ropeOffsetAxis)/2/PI;
		double tRadial = sqrt(offsetAxis*offsetAxis + ropeOffsetAxis*ropeOffsetAxis);
		if ((tRadial > 2) && (abs(offsetAxis) > 2)) {
			tPhase = axisToAngle(-offsetAxis, -0.3)/2/PI;
		}
		waitLoops=LOOP_INTERVAL-2;
		loopTime = defaultLoopTime;
	
		int M = 0;
		if (mode==TESTING) { //STOPPING
			tPhase = 0.75;
			tRadial = ropeAngle/ML_MAX_ROPE_SHIFT_IN_CYCLE;
			waitLoops=2;
			M=SYNC_MAGIC_NUMBER;
		} else if (mode==STOPPING) { //STOPPING
			tPhase = 0.75;
			tRadial = ropeAngle/ML_MAX_ROPE_SHIFT_IN_CYCLE;
 			if (ropeAngle<=0.04) {
				tRadial = ropeAngle*2;
			} 
			M=SYNC_MAGIC_NUMBER;
			waitLoops=2;
		} else if (mode == RUNNING) {
			tPhase = 0.25;
			tRadial = max(-ropeOffsetAxis,0); // only increase speed up to limit
			waitLoops=1;
		} else if (mode == ANALYZING) {
			tPhase = oPhase;
			tRadial = double(oAmp)/maxServoAmp;
			waitLoops = 5;
			syncInitTime = rightTime;
			syncInitTimeOffset = 0;
			syncLoopTime = defaultLoopTime;
		} 

		// HERE
		servoAmp = int(min(1,tRadial)*maxServoAmp);
		requestedNLoops = min(max(int(tRadial-0.01),1),3) ;
				
		//if ((mode != STOPPING) && (mode != TESTING)) requestedNLoops = 1 ;
		loopTime = defaultLoopTime - offset / max(1,tRadial) / LOOP_INTERVAL;
		 

		if (mode == MAINTAINING) {
			if (inTest) {
				int relativeOffset = (time-lastTime) % syncLoopTime;
				if (relativeOffset > syncLoopTime/2) relativeOffset = relativeOffset - syncLoopTime;
				sprint("\x1b[0;34mMachineLearning "); sprint(lastRopeAngle); sprint(": f("); sprint(mPhase); sprint(", "); sprint(mAmp); sprint(") = ("); sprint(relativeOffset); sprint("ms, "); sprint(ropeAngle); sprint(") ");
				sprint ((ropeAngleOffset < -0.005 ? "<" : (ropeAngleOffset > 0.005 ? ">" : "=")));
				if ((ropeAngleOffset < 0.04) && (mAmp < mAmpTo)) {
					// not wide enough, next time try harder
					if (mAmp < mAmpTo) {
						mAmp = min(mAmp + mAmpJump, mAmpTo);
					}
				} else {
					mPhase = mPhase + mPhaseJump;
					mAmp = mAmpFrom;
				}
				
				sprintln("\x1b[0m");
				inTest = false;
				if (mPhase > mPhaseTo) {
					setMode(HALT);
				}
			}
			
			if (ropeAngleOffset < -0.005) {
				// speed up
				tPhase = 0.25;
				servoAmp = 20;
				loopTime = defaultLoopTime;
				requestedNLoops = 1;
				waitLoops = LOOP_INTERVAL-2;
				inTest = false;
			} else if (ropeAngleOffset > 0.005) {
				requestedNLoops = 0; // just wait...
				waitLoops = 0;
				inTest = false;
				return;
			} else { // start a test;
				tPhase = mPhase;
				servoAmp = mAmp;
				loopTime = syncLoopTime;
				requestedNLoops = 1;
				waitLoops = LOOP_INTERVAL-2;
				inTest = true;
				sprint ("&& ");
			}
			//lastRopeAngle: f(lastTPhase, servoAmp) = relativeOffset, ropeAngle (if ropeAngle==syncRopeAngle: ==)
		}

		//loopTime = defaultLoopTime + sin(tPhase*2*PI)*ML_MAX_OFFSET_SHIFT_IN_CYCLE;
		if (side == RIGHT) {
			initTime = rightTime - tPhase * loopTime + M;
		} else {
			initTime = leftTime  - (0.5+tPhase) * loopTime + M;
		}
		//rightTime - loopTime*(side==LEFT ? 0.5 + tPhase : tPhase);
		
		
		sprint("   ==>   servoAmp=");sprint(servoAmp); sprint(" phase=");sprint(tPhase); sprint(" nLoop="); sprint(requestedNLoops); sprint(" LT="); sprintln(loopTime);

		lastOffsetAxis = offsetAxis;
		lastRopeOffsetAxis = ropeOffsetAxis;
		lastTime = time;
		lastRopeAngle = ropeAngle;
	}	
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

  nextSerial.println("      "); // let the following arduino know you are here;

  delay(200);
  sendAudioCommand(0X09, 0X02); // Select TF Card
  sprintln("");
  readCalibration();
  sendAudioCommand(0X22, 0X1E01);
  
  myservoattach(servoPin);
  myservowrite(servoCenter);
  tone(7, NOTE_G5, 100);
  
  initTime = millis();
  randomSeed((initTime + int(potentiometerRead()*100)) % 30000);
  
  setMode(HALT);
  smoothMove(servoCenter);

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
	double ropeAngle = (ropeMaxRightAngle-ropeMaxLeftAngle);
	audioVolume = (audioVolumeAdaptive ? min(max(ropeAngle/syncRopeAngle,0),1) : 1.0) * maxAudioVolume;

	audioTime = time + phase;
	// Snap to Sync
	int offsetToSync = (audioTime - (syncInitTime+syncInitTimeOffset)) % syncLoopTime - syncedPhase;
	if (abs(offsetToSync) <= audioSnapToSync) {
		audioTime = audioTime - offsetToSync;
	} else {
		int offsetToGrid = ((audioTime - (syncInitTime+syncInitTimeOffset)) % syncLoopTime) % audioSnapToGrid;

		// new audioTime is either audioTime-offsetToGrid or audioTime-offsetToGrid+snapToGrid;
		if (offsetToGrid < audioSnapToGrid/2) {
			audioTime = audioTime - offsetToGrid;
		} else {
			audioTime = audioTime - offsetToGrid + audioSnapToGrid;
		}
		
	}
}

void showClockIfNeeded() {
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
}

void sprintLoopEvents() {
	if (showLoopEvents) {
		sprint("# Loop: Time(");sprint(lastLoopTime);sprint(")"); sprint(side==LEFT ? " [L] :" : " [R]:");
		sprint("maxRight(");sprint(ropeMaxRightAngle); sprint(")-maxLeft("); sprint(ropeMaxLeftAngle);
		sprint(")=");sprint(ropeMaxRightAngle-ropeMaxLeftAngle); sprintln(" #");
	}
}


void loop(){
  showClockIfNeeded();
  if (time > keepalive) { nextSerial.print("      "); keepalive = time + 1000; }  // inform slaves they are slaves every 1 seconds;

  time = millis();
  
  handleKeyboardInput();

  // Update clock of slaves
  if (updateSlaveClock || isMaster) {
	if ((time-syncInitTime)%syncLoopTime  < (lastIterationTime-syncInitTime) % syncLoopTime) {
	  // this means we just got to the init time frame;
	  if (updateSlaveClock) {
		  updateSlaveClock = false;
		  sprint("Sync moved:");
	  	  sprintln((millis()-syncInitTime) % syncLoopTime);
	  }
	  syncInitTime = millis();
	  nextSerial.write("T"); // update the clock...	 
	}
  }

  // Read input
  int potRead = potentiometerRead();
  double currentServoPos = myservoread();
  double potAngle = getPotAngle();
  double servoAngle = getServoAngle();
  double ropeAngle = potAngle+servoAngle;
  
  if (potAngle < -1) {
  	sprint("POT0   \r");
  	delay(1000);
  	return;
  }
  ropeMaxRightAngle = max(ropeAngle,ropeMaxRightAngle);
  ropeMaxLeftAngle = min(ropeAngle,ropeMaxLeftAngle);
  
  // Print measures
  if (printMeasures) {
	sprint(" potRead: "); sprint(potRead);
	sprint(" ServoPos: "); sprint(currentServoPos);
	sprint(" potAngle: "); sprint(potAngle);
	sprint(" servoAngle: "); sprint(servoAngle);
	sprint(" ropeAngle: "); sprint(ropeAngle);
	sprintln("");
  }
  
  // Play Audio
  if (enableAudio && itIsTime(audioTime+audioDelay) &&
	  (ropeMaxRightAngle>0.05) && (ropeMaxLeftAngle<-0.05)) {
	  playSong(audioSongNumber,audioVolume);
  }

  // Analysis when pendulum is at the center
  if ((ropeAngle<0) && (side==RIGHT) && (time-rightTime>loopTime/4)) {
	  // This section is if we are now moving to the left side;
	side = LEFT;
	lastLoopTime = time-leftTime;
	leftTime = time;
	
	sprintLoopEvents();
	tone(7, NOTE_G6, 100);

	updateAmpAndTime();

	ropeMaxLeftAngle = 0;
	  
  } else if ((ropeAngle>0) && (side==LEFT)&& (time-leftTime>loopTime/4)) {
	// This section is if we are now moving to the right side;
	side = RIGHT;
	lastLoopTime = time-rightTime;
	rightTime = time;
	playAudioIn(loopTime/4,syncLoopTime/4);

	sprintLoopEvents();
	tone(7, NOTE_G5, 100);
	
	updateAmpAndTime();
	
	ropeMaxRightAngle = 0;
  }
  
  // Wait for ocsilator phase to begin 
  if ((getOcsilatorPos(time) >= servoCenter) && (getOcsilatorPos(lastIterationTime) < servoCenter)) {
  		if ((requestedNLoops == 0) && requestedMoveStarted) {
  			requestedMoveStarted = false;
  			myservowrite(servoCenter);
  		}
  		if (requestedNLoops > 0) {
  			requestedNLoops--;
  			requestedMoveStarted = true;
  		}
  }
  
  if (requestedMoveStarted) {
	// Set oscilator movement if needed;
	double pos = getOcsilatorPos(time);
	smoothWrite(pos);
  }

   lastIterationTime = time;
}

