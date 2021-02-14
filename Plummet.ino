//  Arduino connections:
//  ---------------------
//  analog  1: Potentiometer Bit
//  digital 1: (serialTx): connect to rx of the audio player (no need to connect tx of the player to anything)
//  digital 2: (rxPrev): connect to digital 5(tx) of the previous arduino (no connection if master)
//  digital 5: (txNext): connect to digital 2(rx) of the next arduino
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

#define PLUMMET_VERSION "0.30"

#define BAUD_RATE 9600
#define BAUD_RATE 38400

////// WHAT SERVO LIB TO USE
#define USE_TIMER1
//#define USE_SERVOLIB
//#define USE_SERVO2LIB

////// WHAT SERIAL LIB TO USE
#define USE_NEOSWSERIAL
// #define USE_ALTSOFTSERIAL
// #define USE_SOFTWARESERIAL

#ifdef USE_SOFTWARESERIAL
#include <SoftwareSerial.h>
#endif

#ifdef USE_NEOSWSERIAL
#include <NeoSWSerial.h>
#define SoftwareSerial NeoSWSerial
#endif

#ifdef USE_ALTSOFTSERIAL
#include <AltSoftSerial.h>
#define SoftwareSerial AltSoftSerial
#endif

#ifdef USE_TIMER1
#include <TimerOne.h>
#define SERVO_PWM_RATE 20000
// 6040
#endif

#ifdef USE_SERVOLIB
#ifdef USE_SERVO2LIB
#include <Servo2.h>
#else
#include <Servo.h>
#endif
// #include <PWMServo.h>
// #define Servo PWMServo
#endif

#ifdef USE_SERVO2LIB
#endif

#include <EEPROM.h>

#define rxPinPrev 2 // soft serial
// #define txPinPrev 3 // soft serial
// #define rxPinNext 4 // soft serial
#define txPinNext 5 // soft serial
// #define tonePin 7 // digital
#define servoPin 10 // digital

#define potPin 1 // analog


int8_t myID = -1;
int SYNC_MAGIC_NUMBER=-250;

//int defaultLoopTime = 3564; // Orig Work: 3654 (3.30 meter)//Palo Alto: 3080; // 3160; // 3420;
// int defaultLoopTime = 3080; // Palo Alto
int defaultLoopTime = 3200; //3020;

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
boolean avoidShiftUpdate = false;
boolean updateClock = true;
boolean updatePotCenter = false;
boolean enableAudio = false;
int8_t audioSongNumber = 1;
int8_t maxAudioVolume = 15;
int8_t audioVolume = maxAudioVolume;
bool audioVolumeAdaptive = false;
int audioDelay = 0; // in milliseconds
int audioSnapToGrid = 10; // in milliseconds
int audioSnapToSync = 50;
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
double clockShift = 1;
unsigned long clockShiftCalibration = 0;

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
double oPhase = 0.75; // like stopping
int oLoopTime = defaultLoopTime;
int oNLoops = 2;
int oWaitLoops = 0;

int sLoopDelta = 300;
float sAmpMult = 400;

double mPhase = 0;
int mAmp = 5;
double mPhaseFrom = 0;
double mPhaseTo = 0.50;
double mPhaseJump = 0.02;
int mAmpFrom = 0;
int mAmpTo = 40;
int mAmpJump = 2;
int mLoopTimeFrom = 0;
int mLoopTimeTo = 0;
int mLoopTimeJump = 10;
int mLoopTime = 0;
int mNLoops = 1;
bool updateMLModel = false;

//ML DATA
const uint8_t LOOP_INTERVAL = 3;
double ML_loop_mult;
double ML_angle_mult;
double ML_loop_default;
double ML_angle_default;
double avg_l, avg_x, avg_xx, avg_xl;
double avg_r, avg_y, avg_yy, avg_yr;
int ML_count=100;
bool updateMachineLearning = false;

unsigned long initTime;

unsigned long time = 0;
unsigned long lastIterationTime =0;


int requestedNLoops = 0;
int lastRequestedNLoops = 0;
bool requestedMoveStarted = false;

double ropeMaxLeftAngle = 0;
double ropeMaxRightAngle = 0;

double maxPotRead = 0;
double minPotRead = 1024;

unsigned long syncInitTime = 3000;
int syncInitTimeOffset = 0;
int syncLoopTime=defaultLoopTime; //3020;//3564;
double syncRopeAngle=0.3;
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


enum left_right_e {
  LEFT,
  RIGHT
};

left_right_e side = LEFT;

enum mode_e {
  STOPPING,
  HALT,
  RUNNING,
  MACHINE_LEARNING,
  TESTING,
  SYNCED_RUNNING,
  ANALYZING
};
mode_e mode = HALT;

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
#define sprint2(s,f)   if (enablePrint) Serial.print(s,f)
#define sprintln(s) if (enablePrint) Serial.println(s)
#define sprintline() if (enablePrint) Serial.println()

#define sprint(s)    Serial.print(s)
#define sprint2(s,f)    Serial.print(s,f)
#define sprintln(s)  Serial.println(s)
#define sprintline()  Serial.println()

// void sprint(String s){}; void sprint(double s) {}
// #define sprint(s) 
// #define sprintln(s)

//void debugLog(const char *x) { if (debug) Serial.print(x);		 }
// #define debugLog(x) {if (debug) Serial.print(x);}
#define debugLog(x) {}

unsigned long millis2() {
	return clockShiftCalibration + (millis()-clockShiftCalibration) / clockShift;
}

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

void ewritefloat(float data, int index=-1) {
	if (index!=-1) eIndex = index;

	byte* p = (byte*)(void*)&data;
   for (int i = 0; i < sizeof(data); i++) {
       EEPROM.write(eIndex++, *p++);
    }
}

float ereadfloat(int index=-1) {

	float data;
	byte *p = (byte *)(void *)&data;
    for (int i = 0; i < sizeof(data); i++) {
       *p++ = EEPROM.read(eIndex++);
    }
    return data;
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

void eprintstr(int index=-1,bool isPrint=true) {
  if (index!=-1) eIndex = index;
  while (char c = EEPROM.read(eIndex++)) {
  	if (isPrint) sprint(c);
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
//  Send_Audio_buf[0] = 0x7e; //starting byte
//  Send_Audio_buf[1] = 0xff; //version
//  Send_Audio_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_Audio_buf[3] = command; //
//  Send_Audio_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  Send_Audio_buf[5] = datah;//datah
  Send_Audio_buf[6] = datal; //datal
//  Send_Audio_buf[7] = 0xef; //ending byte
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
boolean servoAttached = false;
int originalTCCR1A = 0;

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
#ifdef USE_TIMER1
	int duty = int(double(map(int(pos), 0,180,544.0,2400.0))/SERVO_PWM_RATE*1024);
	Timer1.pwm(servoPin, duty);
#endif
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
#ifdef USE_TIMER1
	if (!servoAttached) {
		if (originalTCCR1A == 0) {
		  pinMode(pin, OUTPUT);
		  Timer1.initialize(SERVO_PWM_RATE);
		} else {
		  TCCR1A = originalTCCR1A;
		  Timer1.resume();
		}
	}
#endif
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
	boolean ret = myservo.attached();
	myservo.write(servoCenter);
	return ret;
#endif
  }
}
void myservodetach() {
  if (SERVO_VIA_TIMER1) {
#ifdef USE_TIMER1
    if (servoAttached) {sprint("tccr");
		originalTCCR1A = TCCR1A;
		Timer1.disablePwm(servoPin);
		Timer1.stop();
	}
#endif
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

//int angleToServo(double angle) {
//  return angle/(PI/2)*100+servoCenter;
//}

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

int rawPotentiometerRead() {
  return analogRead(potPin);
}
double potentiometerAvg(double p) {
  push(p);
  return posAvg();
}
double potentiometerRead() {
  push(double(analogRead(potPin)));
  return posAvg();
}

double getPotAngle(double p) {
//  double p = potentiometerRead();
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
  potCenter = waitForSteadiness(3,6000);  
  sprint("PotCenter: "); sprintln(potCenter);

  smoothMove(servoCenter-50,4000); 
  sprint("Old ");sprint("Pot50: ");sprintln(pot50);
  pot50 = waitForSteadiness(4,6000);  
  sprint("Pot50: ");sprintln(pot50);

  smoothMove(servoCenter+50,8000); 
  sprint("Old ");sprint("Pot150: "); sprintln(pot150);
  pot150 = waitForSteadiness(4,6000); 
  sprint("Pot150: ");sprintln(pot150);
  
  //calibrateLoopTime();

  smoothMove(servoCenter);
  sprint("Calibrate is:  ");
  sprint(servoCenter); sprint(" ");
  sprint(potCenter); sprint(" ");
  sprint(pot50); sprint(" ");
  sprint(pot150); sprint(" ");
  sprintln(loopTime); 
}
/*
void calibrateLoopTime() { 
  sprintline();
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
*/

void writeCalibration() {
  ewrite(EEPROM_MAGIC,0); //magic number
  ewrite(4); // calibration version
  ewrite(servoCenter);
  ewrite(potCenter);
  ewrite(pot50);
  ewrite(pot150);
  ewrite(defaultLoopTime);
  ewrite(isMaster);
  ewrite((unsigned int)0);
  ewrite((unsigned int)0);

  ewritefloat(ML_loop_default);
  ewritefloat(ML_loop_mult);
  ewritefloat(ML_angle_default);
  ewritefloat(ML_angle_mult);
  ewritefloat((unsigned int)0);
 
  ewritefloat(clockShift); 
  sprint("EEPROM: ");
  printCurrentCalibration();
}

void printMLData() {
	sprint("MLData ");sprint(ML_loop_mult); sprint(",");sprint(ML_loop_default); sprint(" "); sprint2(ML_angle_mult,6); sprint(" "); sprint2(ML_angle_default,6);sprint(" #"); sprintln(ML_count);
}

void printCurrentCalibration() {
  sprint("ServoCenter="); sprint(servoCenter);
  sprint(" potCenter="); sprint(potCenter);
  sprint(" pot50="); sprint(pot50);
  sprint(" pot150="); sprint(pot150);
  sprint(" loopTime="); sprint(defaultLoopTime);
  sprint(" clockS="); sprint2(clockShift,5);
  sprint(" isMaster="); sprintln(isMaster);
  printMLData(); 
}

void printMLPoint(double phase, int amp) {
	  double loop = ML_loop_mult * amp * cos(phase*2*PI) + ML_loop_default;
	  double angle = ML_angle_mult * amp * sin(phase*2*PI) + ML_angle_default;

	  sprint("f("); sprint(phase); sprint(", "); sprint(amp); sprint(") = ("); 
	  sprint(loop); sprint("ms, "); 
	  sprint(angle); sprint(") ");
	  sprint("aLT:"); sprintln(loop * LOOP_INTERVAL - (LOOP_INTERVAL-1)*ML_loop_default);
}

void setDefaultCalibration() {
		// 2.28,3227.39 0.001328 -0.033839
		ML_loop_mult = 2.35; // 1.34; //1.27;//1.28;
		ML_loop_default = 3211; //3020;//3567;
		ML_angle_mult = 0.002167; //0.001957; //0.0224;//0.00199;
		ML_angle_default = -0.055560; //-0.09367; //-0.01765;//-0.02483;
		ML_count = 2;
		fakeAvg();
}

void readCalibration() {
	int version = 0;
	if (eread(0) == EEPROM_MAGIC) { // confirm magic number
		version = eread(); // calibration version
		servoCenter = eread();
		potCenter = eread();
		pot50 = eread();
		pot150 = eread();
		defaultLoopTime = eread(); loopTime = defaultLoopTime;syncLoopTime = loopTime;
		isMaster = eread();
	} else {
		sprintln("No EEPROM");
		//servoCenter = eread(1);
	}
	eread();
	eread();
	
	if (servoCenter>1000) servoCenter = 95;
	
	if (version > 2) {
		ML_loop_default = ereadfloat();
		ML_loop_mult = ereadfloat();
		ML_angle_default = ereadfloat();
		ML_angle_mult = ereadfloat();
		ML_count = 100;
 		fakeAvg(); 	
	} else {
		setDefaultCalibration();
	}

	if (version > 3) {
		ereadfloat();
		clockShift = ereadfloat();
	}


	printCurrentCalibration();
}
void fakeAvg() {
	avg_l = ML_loop_default / 2.0; // loop default;
	avg_x = -avg_l / ML_loop_mult; // 1.273 - loop_mult (ratio between servo and offset per cycle)
	avg_xx = avg_x*avg_x * 2;
	avg_xl = 0;
	
	avg_r = ML_angle_default / 2; // angle change after 3 cycles
	avg_y = -avg_r / ML_angle_mult; // 0.0016 - angle multiplier (ratio between servo and rope angle offset per cycle)
	avg_yy = avg_y*avg_y * 2;
	avg_yr = 0;
}

///////////////////////////
/// User Data
///////////////////////////
#define KBSIZE 50
char KB[KBSIZE];
char *CMD=KB;
char c10[10];

boolean commandAvailable() {
  return (isPlaying && (nextCommandTime != MAX_UINT) && (nextCommandTime < (millis()-playInitTime)/1000));
}

byte COMSTART[] = {111,110,112};
int8_t prevSerialState = 0;
void nextSerialPrintln(char *s, bool cr=true) {
   for (int8_t i = 0; i<sizeof(COMSTART); i++) {
	nextSerial.write(COMSTART[i]);
   }
   if (cr) {
        nextSerial.println(s);
   } else {
	nextSerial.print(s);
   }
   // sprint("_");
}

bool prevSerialAvailable() {
       while (prevSerial.available() && (prevSerialState<sizeof(COMSTART))) {
          char b = prevSerial.read();
	  if (b == COMSTART[prevSerialState]) {
		prevSerialState ++;
		//sprint("-");sprint(prevSerialState);
	  } else {
		sprint(".");sprint((int)b);
	  }
       }
       return (prevSerial.available() && (prevSerialState == sizeof(COMSTART)));
}

byte readByte() {
  unsigned long timeout = millis() + 100;
  char b;
  while (millis() < timeout) {
	if (Serial.available()) {
	  b = Serial.read();
	  //sprint(b);
	  return b;
	} else if (!isMaster && prevSerialAvailable()){
	  char b = prevSerial.read();
//	  if (b=='~') { Serial.write("Error:"); delay(100); while (prevSerial.available()) Serial.print("~"+String(int(prevSerial.read()))); Serial.println("");}
	  //sprint(b);
	  if (b=='\n') prevSerialState = 0;
	  return b;
	}
  }
sprint("?");
}


bool readByteAvailable() {
  return (Serial.available() || (!isMaster && prevSerialAvailable()));
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
		 	nextSerialPrintln(KB);
		 } else if (KB[1]=='o') {
		 	KB[1] = 'e';
		 	nextSerialPrintln(KB);
		 	KB[0] = 0; CMD=KB; //ignore this command as it is not for this arduino.
		 } else if (KB[1]=='w') {
		 	nextSerialPrintln(KB);
		 	delay(50);
		 	nextSerialPrintln(CMD);
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
			   nextSerialPrintln(KB);
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
		  nextSerialPrintln(KB);
		  break;
	   case 'u':
	       // Current device phase so that everyone will show the clock compared to this.
	      s = atoi(KB+1); 
		  if (s==0) {
		  	itoa(int((time-(syncInitTime+syncInitTimeOffset)) % syncLoopTime), KB+1, 10);
		  	sprint(KB+1);
		  	s = strlen(KB);
		  	KB[s++] = '\n';
		  	KB[s] = 0;
		  }
		  nextSerialPrintln(KB);
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
	   	  nextSerialPrintln(KB);
	      break;
	   case 'c':
	   case 'C':
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
	   case '_':
	   case 'P':
	   case 'R':
	   case 'Y':
	   case '@':
	   case ' ':
	   case '^':
	   case '\n':
		 break;
	   default:
		 nextSerialPrintln(KB);
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
	  sprintline();
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
		 if (kblength >= (sizeof(KB)-2)) {
		 	sprintln("TOO LONG");
		 	KB[0] = 0; kblength =0;
		 }
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
	  			   sprintline();
	  		   }*/ sprintline();

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
	  	   nextSerialPrintln("T", false); // update the clock...	 

		   // Handle clock sync
		   int s = (time-syncInitTime ) % syncLoopTime;
		   if (s>syncLoopTime/2) s = s-syncLoopTime;

		   prevSerialState = 0;
		   if ((updateClock) && (!avoidShiftUpdate) && (abs(s)>2)) {
			syncInitTime = time-s+min(max(s,-5),5) - syncLoopTime;
		   	if (showShift && (abs(s)>10)) {
		   		sprint("["); sprint(time); sprint (" / ");sprint(syncInitTime); sprint (" / "); sprint(syncLoopTime); sprint("] shift: "); sprintln(s); //sprint(" ");
				// sprintln (avoidShiftUpdate ? "AV" : "VV");
			}
		   }
		avoidShiftUpdate = false;

//		 } else if (inByte == 'k') { /* todo remove keepalive */
//		   prevSerialState = 0;
		 } else if (inByte == 'Q') {
		   // Delete command
		   KB[0] = 0; kblength =0; inByte =0;
		   sprint("\r        \r");
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
	case 'L': /* Print loop events (move from RIGHT to left) */
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
	case '?': 
	  sprint("Mode="); sprintln(mode);
	  break;
	case '0':
	  updatePotCenter = true;
	  break;
	case '1': // START 
	  setMode(RUNNING); sprintln("START");
	  break;
	case '2': // STOP
	  if (CMD[0]=='=')  {
		sLoopDelta = atoi(CMD+1);
		p = find(CMD,',');
		if (p!=-1) sAmpMult = atof(CMD+p+1);
	 }
	setMode(STOPPING); sprint("STOP ");sprint(sLoopDelta); sprint (","); sprintln(sAmpMult);
	  KB[0] = 0; CMD=KB;
	  break;
	case '9': // HALT: STOP and dont move anymore;
	  setMode(HALT); sprintln("HALT");
	  smoothMove(servoCenter);
	  break;
	case 'M': /* ML Calibration */
	  //ML_count = 10;
	  mPhase = mPhaseFrom = 0; mPhaseTo=1; mPhaseJump = 0.25;
	  mAmp = mAmpFrom = 0; mAmpTo = 40; mAmpJump = 20;
	  mLoopTime = mLoopTimeFrom = mLoopTimeTo = 0;
	  mNLoops = 1;
	  setMode(MACHINE_LEARNING);
	  sprint("ML Calibrate");
	  KB[0] = 0; CMD=KB;
	  break;

	case 'm': /* MACHINE_LEARNING */
	//Quick Learn m0,0.5,0.1,0,40,20
	//Stop Learn m0.75,0.75,0.1,45,60,2,3000,3000,50,2
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
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mLoopTimeFrom = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mLoopTimeTo = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mLoopTimeJump = atof(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) mNLoops = atof(CMD+p+1);
	  KB[0] = 0; CMD=KB;

	  mAmp = mAmpFrom;
	  mPhase = mPhaseFrom;
	  mLoopTime = mLoopTimeFrom;
	  setMode(MACHINE_LEARNING); sprint("ML ");sprint(syncRopeAngle);
	  sprint(": phase("); sprint(mPhaseFrom);sprint("-");sprint(mPhaseTo);sprint(" +");sprint(mPhaseJump);
	  sprint(") amp(");sprint(mAmpFrom);sprint("-");sprint(mAmpTo);sprint(" +");sprint(mAmpJump);
	  sprint(") loopTime(");sprint(mLoopTimeFrom);sprint("-");sprint(mLoopTimeTo);sprint(" +");sprint(mLoopTimeJump);
	  sprint(") mNLoops(");sprint(mNLoops);
	  sprintln(")");
	  break;
	case 'o':
	case 'k':
		KB[0] = 0; CMD = KB;
		break;
	case 'O': /* ANALYZE Amp,Phase, loopTime, nLoops, oWaitLoops */
	  if (CMD[0]!='\n'){
		  oAmp = atoi(CMD);
		  p = find(CMD, ',');
		  if (p!=-1) oPhase = atof(CMD+p+1);
		  if (p!=-1) p = find(CMD, ',', p+1);
		  if (p!=-1) oLoopTime = atoi(CMD+p+1);
		  if (p!=-1) p = find(CMD, ',', p+1);
		  if (p!=-1) oNLoops = atoi(CMD+p+1);
		  if (p!=-1) p = find(CMD, ',', p+1);
		  if (p!=-1) oWaitLoops = atoi(CMD+p+1);
	  }
	  sprint(oAmp);sprint(",");sprint(oPhase);sprint(",");sprint(oLoopTime);sprint(",");sprint(oNLoops);sprint(",");sprintln(oWaitLoops);
	  setMode(ANALYZING);
	  KB[0] = 0; CMD=KB;
	  break;
	case 't': /* TEST */
	  /*
syncLoopTime = atoi(CMD); 
	  if (syncLoopTime == 0) syncLoopTime = defaultLoopTime;
	  syncInitTime = time;
	  syncInitTimeOffset = 0;
	  p = find(CMD, ',');
	  
	  if (p!=-1) {
	  	d = atof(CMD+p+1);
	  	if (d != 0) syncRopeAngle = d;
	  }
	  
	  KB[0] = 0; CMD=KB;

	  setMode(TESTING); sprintln("TEST");sprint(syncLoopTime); sprint(","); sprintln(syncRopeAngle);
*/
	  break;
	case 's': // Set SYNC clock
	  syncLoopTime = atoi(CMD); 
	  if (syncLoopTime == 0) syncLoopTime = defaultLoopTime;
	  syncInitTimeOffset = 0;
	  if (isMaster) {
		syncInitTime = millis2();
	  } else { 
		syncInitTime = time;
		avoidShiftUpdate = true;
	  }
	  p = find(CMD, ',');
	  
	  if (p!=-1) {
	  	d = atof(CMD+p+1);
	  	if (d != 0) syncRopeAngle = d;
	  }
	  
	  KB[0] = 0; CMD=KB;
	  sprint("SYNC");sprint(syncLoopTime); sprint(","); sprintln(syncRopeAngle);
	  break;
/*	case '[':
	  d = atof(CMD);
	  if (d>0) {
		syncRopeAngle=d;
		KB[0]=0;CMD=KB;
	  } else {
	  	syncRopeAngle = min(max(syncRopeAngle-0.01,0),0.4);
	  }
	  sprint("SyncAngle: ");
	  sprintln(syncRopeAngle);
	  break;
	case ']':
	  syncRopeAngle = min(max(syncRopeAngle+0.01,0),0.4);
	  sprint("SyncAngle: ");
	  sprintln(syncRopeAngle);
	  break;

*/
    case '%': /* disable clock update */
      showShift = !showShift;
      sprintln(showShift);
      break;
    case '$': /* Don't update clock */
      updateClock = !updateClock;
      sprintln(updateClock);
      break;
	case 'u': /* Just print phase compared to sync clock */
	  s = atoi(CMD); KB[0] = 0; CMD=KB;
	  sprint("sync: ");
	  s = int((time-(syncInitTime+syncInitTimeOffset) - s) % syncLoopTime);
	  if (s>syncLoopTime/2) s=s-syncLoopTime;
	  sprintln(s);
	  break;
	case 'U': /* Update slaves on current Sync Clock */
	  updateSlaveClock = true;
	  break;

	case '!': /* Toggle Machine Learning functionality */
	  updateMachineLearning = !updateMachineLearning ;
	  sprint("ML "); sprint(updateMachineLearning ? "on" : "off");
	  break;
	case 'h': // Offset the Sync clock by half loop
	  if (CMD[0]!='\n') {
	  	d = atof(CMD);
	  } else {
		d = 0.5;
	  }
	  KB[0] = 0; CMD=KB;
	  syncInitTimeOffset -= syncLoopTime * d;
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  break;
	case 'r': // Randomize the SYNC clock 
	  syncInitTimeOffset = random(0,defaultLoopTime);
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  //servoAmp = 20;
	  break;
	case 'S': // SYNC to an already set clock (don't update the clock) 
	  if(CMD[0]!='\n') syncInitTimeOffset = atoi(CMD);
	  sprint ("Offset:");sprintln(syncInitTimeOffset);
	  KB[0]=0; CMD=KB;
	  //	if (s!=0) syncLoopTime = s;
      setMode(SYNCED_RUNNING);
	  break;
    case '#': // Show age of ML module
	  if (CMD[0] == '0') setDefaultCalibration();
	  s = atoi(CMD); KB[0] = 0; CMD=KB;
	  if (s>=2) {
	  	ML_count = s;
	  }
	  printMLData();
	  break;
//	case 'X': /* calc ML for phase,amp */
//	  d = atof(CMD);
//	  p = find(CMD,',');
//	  if (p!=-1) s = atoi(CMD+p+1);
	  
//	  printMLPoint(d,s);
	  
//	  KB[0] = 0; CMD=KB;
//	  break;
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
	  // __ to attach _ to detach
	  if (CMD[0]=='_') myservoattach(servoPin); // __ to attach
	  if (CMD[0]!='_') myservodetach();
	  KB[0] = 0; CMD= KB;
	  sprint("servo="); sprintln(myservoattached() ? "attached" : "detached");
	  break;
   case '~': /* Change Servo type (Timer1 vs Servo library) */
// #ifdef USE_SERVOLIB
//	 myservodetach();
//	 SERVO_VIA_TIMER1 = !SERVO_VIA_TIMER1;
//	 myservoattach(servoPin);
// #endif
	 sprintln(SERVO_VIA_TIMER1 ? "Using T1" : "Using Servolib");
	 break; 
	case 'B': // which device is this 
	  Serial.print("I am ");
	  Serial.print(isMaster ? "master" : "slave"); 
	  Serial.print(" #");
	  Serial.println(myID);
      KB[0] = 0; CMD = KB;

//	  if (isMaster) tone(7, NOTE_A5, 1000);
	  break;
	case '^': /* toggle master/slave */
	  isMaster = !isMaster;
	  ewrite(isMaster,14);
	  Serial.print("I am ");
	  Serial.println(isMaster ? "master" : "slave");
	  break; 
	case 'i':
	  Serial.print("iI am ");
	  Serial.print("#");
	  Serial.println(myID);
	  if (isMaster && (myID==-1)) {
	  	myID = 0;
	  	nextSerialPrintln("B1");
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
		  ///calibrateLoopTime();
	  }
	  KB[0] = 0; CMD = KB;
	  break;
	case '&': // Print current calibration
	  printCurrentCalibration();
	  break;
	case 'a': // play audio
	  playSong(audioSongNumber, maxAudioVolume);
	  break;
	case 'V': // set audio volume
	  if (CMD[0]!='\n') maxAudioVolume = atoi(CMD);
	  sprint("Volume "); sprintln(maxAudioVolume);
	  KB[0] = 0;CMD=KB;
	  break;
	case 'A': // Audio config #,Vol,STG,STS,delay,adaptive?
	  if (CMD[0]=='+'){
	  	enableAudio = true; CMD +=1;
	  } else if (CMD[0]=='-') {
		enableAudio = false; CMD +=1;
	  }
	  s = atoi(CMD);
	  if (s>0) audioSongNumber = s;

	  p = find(CMD, ',');
	  if (p!=-1) audioSnapToGrid = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioSnapToSync = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioDelay = atoi(CMD+p+1);
	  if (p!=-1) p = find(CMD, ',', p+1);
	  if (p!=-1) audioVolumeAdaptive = CMD[p+1]!='0';
	  KB[0] = 0; CMD=KB;
	  sprint("Audio "); sprint(enableAudio ? "on" : "off"); sprint(" song="); sprint(audioSongNumber); sprint(" stg=");sprint(audioSnapToGrid);sprint(" sts=");sprint(audioSnapToSync); sprint(" delay=");sprint(audioDelay); sprint(" adaptive="); sprintln(audioVolumeAdaptive);
	  break;
	case 'P': // Play
	  if (isMaster) {
		if (CMD[0] == 'P') {
			printCommands();
		  } else if (CMD[0]=='+') {
			startPlaySequence();
		  } else if (CMD[0]=='-') {
			stopPlaySequence();
		  } else if (!isPlaying) {
			startPlaySequence();
		  } else {
			stopPlaySequence();
		  }
	  }
	  KB[0] = 0; CMD = KB;
	  break;
	case 'R': // Record
	  if (CMD[0] == 'R') {
	  	importCommand(CMD+1);
		KB[0] = 0; CMD = KB;
	  } else if (cmdOriginFromRecordedSequence) {
	  	if (isPlaying) {
	  		if (isAutoPlay) {
	  			startPlaySequence();
	  		} else {
	  			stopPlaySequence();
	  		}
	  	} else {sprintln("Weird");}
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
	  if (isMaster) { 
		  isAutoPlay = !isAutoPlay;
		  if (CMD[0]=='+') isAutoPlay = true;
		  if (CMD[0]=='-') isAutoPlay = false;
		  sprint("auto play is ");
		  sprintln(isAutoPlay ? "on" : "off");
		  ewrite((int)isAutoPlay, EEPROM_COMMANDS_LOC - 4);
	  }
	  KB[0] = 0; CMD= KB;
	  break;
	case '*': /* Show Clock. ** calibrate Clock */
	  if (CMD[0]=='c') {
	    if (isMaster) {
	  	sprintln("30s cali");
		nextSerialPrintln("*B");
		delay(100000);
		nextSerialPrintln("*E");
	    } 
	  } else if (CMD[0]=='B') {
		clockShiftCalibration = millis();
		sprint(millis()); sprint (";"); sprint (millis2()); sprint(";"); sprintln(millis());
	  } else if (CMD[0]=='E') {
		unsigned long dt = (millis()-clockShiftCalibration);
		sprint (" ++="); sprint(dt);
		clockShift = (millis()-clockShiftCalibration)/((double)100000.0);
		if ((clockShift >1.1) || (clockShift < 0.9)) {
			clockShift = 1;
		}
		sprint(" **="); sprint2(clockShift,5);
		sprint((unsigned long ) ((millis()-clockShiftCalibration) * clockShift));
		sprintline();
	  } else {
	  	showClock = !showClock;
	  }

	  KB[0] = 0; CMD = KB;
	  break;
	case '@': /* eprom access */
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

	if (!isMaster) return;
	printCommands();
	
	nextCommandTime = eread(EEPROM_COMMANDS_LOC);
	nextCommandLoc = eIndex;
	if (nextCommandTime == MAX_UINT) {
		sprintln("No "); sprintln("Recorded Commands");
	} else {
	  	isPlaying = true;
	  	playInitTime = millis();
	}
}

void importCommand(char *cmd) {
	// TODO: Write eprommagic;
	eIndex = EEPROM_COMMANDS_LOC;
	
	for (int i = atoi(cmd); i>0; i--) {
		if (eread() == MAX_UINT) {
			sprintln("Bad");
			return;
		}
		eprintstr(-1,false); 
	}

	int p = find(cmd, ':');
	if (p==-1) { sprintln("BAd");return;}
	unsigned int commandTime = atoi(cmd+p+1);

	p = find(cmd,',');	
	if (p==-1) { sprintln("BAD");return;}
	cmd = cmd+p+1;

	cmd[strcspn(cmd,"\r\n")] = 0; // trim newline

	ewrite(commandTime);
	ewrite(cmd);
	ewrite(MAX_UINT);
}

void printCommands() {
	// Print commands:
	unsigned int commandTime = eread(EEPROM_COMMANDS_LOC);
	sprintln("Recorded Commands");
	int i =0;
	while (commandTime != MAX_UINT) {
		 sprint("RR"); sprint(i++);sprint(":");
		 sprint(commandTime);
		 sprint(",");
		 eprintstr();
		 sprintline();
		 commandTime = eread();
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


void updateAmpAndTime(bool runNow=false) {
	
	static bool isFirstIter = true;
	if (runNow) isFirstIter = true;
	
	static double ML_ROPE_ANGLE_DECREASE = 0.05; // per cycle.  
	static double ML_MAX_ROPE_SHIFT_IN_CYCLE = 0.18;
	static double ML_MAX_OFFSET_SHIFT_IN_CYCLE = 250.0;


	int mlLoopTime;
	double mlRopeOffset;

	static int waitLoops=0;
	static unsigned long waitForTime=0;
	static unsigned long lastTime = 0;
	static double lastRopeAngle = 0;
	static boolean inTest = false;

	double tPhase;
	double tRadial;
	double ropeAngle = (ropeMaxRightAngle-ropeMaxLeftAngle);
	double ropeAngleOffset = ropeAngle-syncRopeAngle;
	int offset = (rightTime+syncLoopTime-(syncInitTime+syncInitTimeOffset)) % syncLoopTime;

	
	double X, Y;
	
	if (offset > syncLoopTime/2) offset = offset - syncLoopTime;

	if ((side == RIGHT) && (!runNow)) {
		playAudioIn(loopTime/4,offset);
	}


	if (runNow) {
	//	waitLoops = 0;
		updateMLModel = false;
	}

	if ((side==RIGHT) || runNow) {
		if (mode != HALT) {
			sprint((waitLoops || time<waitForTime) ? "\x1b[0;37m" : "\x1b[0;31m");
			sprint(time); sprint("|");
			sprint ("["); sprint(lastLoopTime); sprint("]: s="); sprint(syncLoopTime); sprint(",");sprint(syncInitTime); sprint(" offset="); sprint(offset); sprint("ms ropeAngle="); 	sprint2(ropeAngle,3); sprint((ropeAngleOffset > 0) ? "(+" : "(");sprint2(ropeAngleOffset,3); sprint(")");
			sprint("\x1b[0m");
		}
		if (requestedMoveStarted && !runNow) { sprint("in motion [");sprint(requestedNLoops);sprintln("]"); return; }
		if (waitLoops > 0) { sprint("wait ["); sprint(waitLoops--); sprintln("]"); /*servoAmp = 0;*/ return; }
		if (!runNow && (time < waitForTime)) { sprint("waiting "); sprint(waitForTime - time); sprintln("ms"); return;}
		
		if ((mode == STOPPING) && (lastLoopTime >defaultLoopTime*1.5)) { sprintln("long"); return;}
		if (mode==HALT) {return;}

		if (mode == ANALYZING) {
			tPhase = oPhase;
			servoAmp = oAmp;
			loopTime = oLoopTime == 0 ? defaultLoopTime : oLoopTime;
			requestedNLoops = oNLoops ;

			waitLoops = oWaitLoops;
			syncInitTime = rightTime;
			syncInitTimeOffset = 0;
			syncLoopTime = defaultLoopTime;
			updateMLModel = false;
		} 

				
		//if ((mode != STOPPING) && (mode != TESTING)) requestedNLoops = 1 ;
		//loopTime = defaultLoopTime - offset / max(1,tRadial) / LOOP_INTERVAL;
		// learn
		if ((mode == RUNNING) || (mode==SYNCED_RUNNING) || ((mode==MACHINE_LEARNING) && inTest)) {
			// last input
			X = mAmp * cos(mPhase*2*PI);
			Y = mAmp * sin(mPhase*2*PI);

			// last results
			// mlLoopTime = (time-lastTime) / (LOOP_INTERVAL + lastRequestedNLoops-1);
			mlLoopTime = ((time-lastTime) - ML_loop_default * (LOOP_INTERVAL-1)) / (lastRequestedNLoops>0 ? lastRequestedNLoops : 1);
			mlRopeOffset = ropeAngle-lastRopeAngle ;

			sprintline();
			sprint("\x1b[0;34mMachineLearning "); sprint2(lastRopeAngle,3); sprint(": f("); sprint(mPhase); sprint(", "); sprint(mAmp); sprint(") = ("); sprint(mlLoopTime); sprint("ms, "); sprint2(mlRopeOffset,3); sprint(") "); sprint("LT:"); sprint(mLoopTime);sprint(" ");
			
#define ML_UPDATE(a,b) a = a*(ML_count/(ML_count+1.0)) + b/(ML_count+1.0)
			//learn:
			if (updateMLModel && !isFirstIter && (mAmp < maxServoAmp) && (ropeAngle<syncRopeAngle + 0.08) && (ropeAngle>0.2)) {
				sprint(" *");
				if (updateMachineLearning) {
					sprint ("*");	
					// Update 
					ML_UPDATE(avg_l, mlLoopTime); ML_UPDATE(avg_x, X); ML_UPDATE(avg_xx,X*X); ML_UPDATE(avg_xl,X*mlLoopTime);
					ML_UPDATE(avg_r, mlRopeOffset); ML_UPDATE(avg_y, Y); ML_UPDATE(avg_yy,Y*Y); ML_UPDATE(avg_yr,Y*mlRopeOffset);
					ML_count = min(ML_count+1, 10000);
					updateMLModel = false;

					// calculate ML models:
					ML_loop_mult = (avg_xl - avg_x*avg_l) / (avg_xx - avg_x*avg_x);
					ML_angle_mult = (avg_yr - avg_y*avg_r) / (avg_yy - avg_y*avg_y);
					ML_loop_default = avg_l - ML_loop_mult*avg_x;
					ML_angle_default = avg_r - ML_angle_mult*avg_y;
				}
				sprint(" ");
			}


			// print ML models
			//sprint("X("); sprint(X);sprint(",");sprint(mlLoopTime);sprint(") Y(");sprint(Y);sprint(",");sprint(mlRopeOffset);sprint(") ML ");
			printMLData();
		}
		 
		if ((mode == RUNNING) || (mode == SYNCED_RUNNING) || (mode == STOPPING)) { // synced_running
			waitLoops = 0;			

			// calculate desired phase and amp (aim)
			// mlLoopTime = syncLoopTime - offset / LOOP_INTERVAL; // desired loopTime
			mlLoopTime = syncLoopTime * LOOP_INTERVAL - ML_loop_default * (LOOP_INTERVAL-1) - offset;
			mlRopeOffset = (syncRopeAngle-ropeAngle); //desired RopeOffset
			
			if (mode == RUNNING) {
				mlLoopTime = ML_loop_default; // if running - we set the offset to make X=0 so that we just update the amp and not the offset
			}

			if (mode == STOPPING) {
				mlRopeOffset = -ropeAngle; // desired to stop
				mlLoopTime = ML_loop_default;
			}

			//sprintln("");
			//sprint("-");sprint(mlLoopTime);sprint(",");sprint(mlRopeOffset);
			X = (mlLoopTime - ML_loop_default) / ML_loop_mult;
			Y = (mlRopeOffset - ML_angle_default) / ML_angle_mult;

			tPhase = axisToAngle(X,Y)/2/PI;
			servoAmp = int(sqrt(X*X+Y*Y));
			requestedNLoops = 1;
			
			if (servoAmp > maxServoAmp) {
				sprint("wanted "); sprint(servoAmp);
				//if (ropeAngle < syncRopeAngle - 0.1) {
					servoAmp = min(servoAmp,maxServoAmp*3);
				//}
				requestedNLoops = int(servoAmp/maxServoAmp + 1);


				// recalculate aim
				mlLoopTime = (syncLoopTime * (LOOP_INTERVAL+requestedNLoops-1) - ML_loop_default * (LOOP_INTERVAL-1) - offset) / requestedNLoops;;
				X = (mlLoopTime - ML_loop_default) / ML_loop_mult;
				Y = (mlRopeOffset - ML_angle_default) / ML_angle_mult;

				tPhase = axisToAngle(X,Y)/2/PI;
				servoAmp = int(sqrt(X*X+Y*Y));

				servoAmp = min(servoAmp/requestedNLoops,maxServoAmp);
				X = servoAmp * cos(tPhase*2*PI);
				Y = servoAmp * sin(tPhase*2*PI);
				mlLoopTime = ML_loop_mult * X + ML_loop_default;
				mlRopeOffset = ML_angle_mult * Y + ML_angle_default; 
				sprint("+++");//sprint(offset - (syncLoopTime-mlLoopTime)*LOOP_INTERVAL); sprint(",");sprint(ropeAngle+mlRopeOffset*LOOP_INTERVAL);
				updateMLModel = false;
				
			} else {
				updateMLModel = true;
			}
			//sprint(" X");sprint(X);sprint(",");sprint(Y);sprint(";");sprint(tPhase);
			//loopTime = mlLoopTime * LOOP_INTERVAL - (LOOP_INTERVAL-1)*ML_loop_default;
			loopTime = mlLoopTime;

			if ((mode == STOPPING)) {
				tPhase = 0.75;
				if (ropeAngle>syncRopeAngle - 0.08) {
					loopTime = ML_loop_default - 100;
					servoAmp = 70;
					requestedNLoops = 2;
				} else if (ropeAngle > 0.01){
					loopTime = ML_loop_default-sLoopDelta;
					//servoAmp = (ropeAngle) / ML_angle_mult;
					servoAmp = sAmpMult * ropeAngle ;
					sprint("S");sprint(servoAmp);
					servoAmp = min(max(servoAmp,0),  maxServoAmp);
					requestedNLoops = 1;
sprint("LLL");sprintln(loopTime);
				} else {
					servoAmp = 0;
					requestedNLoops = 0;
				}
				updateMLModel = false;
			}

			//waitForTime = millis() + (LOOP_INTERVAL-0.5)*defaultLoopTime;
			waitForTime = time + requestedNLoops * loopTime + (LOOP_INTERVAL-1.5) * ML_loop_default;

						
			mAmp = servoAmp;
			mPhase = tPhase;
			mLoopTime = loopTime;
			
			sprintln("\x1b[0m");
		}
		
		if (mode == MACHINE_LEARNING) {
			if (inTest) {
				
				// Set up for next test
				if ((mLoopTime + mLoopTimeJump) < mLoopTimeTo) {
					// next loopTime
					mLoopTime += mLoopTimeJump;
				} else if (((mAmp + mAmpJump) < mAmpTo) && (ropeAngleOffset < 0.04)) {
					// next Amp
					mAmp = mAmp + mAmpJump;
					mLoopTime = mLoopTimeFrom;
				} else {
				    // next phase
					mPhase = mPhase + mPhaseJump;
					mLoopTime = mLoopTimeFrom;
				}
				
				inTest = false;
				if (mPhase > mPhaseTo) {
					setMode(HALT);
				}
			}
			waitLoops = 0;
			if (ropeAngleOffset < -0.005) {
				// speed up
				tPhase = 0.25;
				servoAmp = int(min(max(-ropeAngleOffset*20*maxServoAmp, 20),maxServoAmp));
				loopTime = defaultLoopTime;
				requestedNLoops = 1;
				waitForTime = time + (LOOP_INTERVAL-0.5)*defaultLoopTime;
				inTest = false;
				updateMLModel = true;
			} else if (ropeAngleOffset > 0.005) {
				requestedNLoops = 0; // just wait for next iteration...
				inTest = false;
				sprintln("\x1b[0m");
				//TODO: return is not nice...
				
				return;
			} else { // start a test;
				tPhase = mPhase;
				servoAmp = mAmp;
				loopTime = (mLoopTime==0 ? defaultLoopTime : mLoopTime);
				waitForTime = time + (LOOP_INTERVAL-0.5)*defaultLoopTime;
				
				requestedNLoops = mNLoops;
				waitForTime = waitForTime + (mNLoops-1)*defaultLoopTime;
				inTest = true;
				updateMLModel = true;
				sprint ("&& ");
			}
			sprintln("\x1b[0m");
		}

		//loopTime = defaultLoopTime + sin(tPhase*2*PI)*ML_MAX_OFFSET_SHIFT_IN_CYCLE;
		if (side == RIGHT) {
			initTime = rightTime - tPhase * loopTime;
		} else {
			initTime = leftTime  - (0.5+tPhase) * loopTime;
		}
		//rightTime - loopTime*(side==LEFT ? 0.5 + tPhase : tPhase);


		if (((mode == RUNNING) || (mode == SYNCED_RUNNING)) && 
		    (ropeAngle < 0.05)) {
			sprint("Quick Start");
			servoAmp = maxServoAmp;
			loopTime = syncLoopTime;
			initTime = syncInitTime + syncInitTimeOffset + 0.75 * syncLoopTime;
			requestedNLoops = 2;
			waitForTime = time + requestedNLoops * loopTime + (LOOP_INTERVAL-1.5) * ML_loop_default;
						
			mAmp = servoAmp;
			mPhase = 0.25;
			mLoopTime = loopTime;
			updateMLModel = false;
		}
		
		
		sprint(" => servoAmp=");sprint(servoAmp); sprint(" phs=");sprint(tPhase); sprint(" LT="); sprint(loopTime); sprint(" nL="); sprintln(requestedNLoops);

		lastTime = time;
		lastRopeAngle = ropeAngle;
		lastRequestedNLoops = requestedNLoops;
		isFirstIter = false;
	}	
}




//////////////////////////////
// Main Code
//////////////////////////////
void setup() {
  KB[0] = 0; CMD=KB;
  //pinMode(13, OUTPUT); digitalWrite(13, HIGH);
  Serial.begin(BAUD_RATE);
  Serial.print("v");
  Serial.print(PLUMMET_VERSION);

  nextSerial.begin(9600);
  prevSerial.begin(9600);

  nextSerialPrintln("k",false); // let the following arduino know you are here;

  delay(200);
  sendAudioCommand(0X09, 0X02); // Select TF Card
  sprintline();
  readCalibration();
  sendAudioCommand(0X22, audioVolume);
  
  myservoattach(servoPin);
  myservowrite(servoCenter);
//  tone(7, NOTE_G5, 100);
  
  time = initTime = millis2();

  randomSeed((initTime + int(potentiometerRead()*100)) % 30000);
  
  setMode(HALT);
  smoothMove(servoCenter);

  if ( eread(EEPROM_COMMANDS_LOC-2) == EEPROM_MAGIC) {
  	isAutoPlay = eread(EEPROM_COMMANDS_LOC-4);
  }

  if (isMaster && isAutoPlay) startPlaySequence();

  //pinMode(13, OUTPUT); digitalWrite(13, LOW);
}

//unsigned long keepalive = 0;

left_right_e direction = RIGHT;
#define itIsTime(x) ((time>=x) && (lastIterationTime<x))

unsigned long audioTime=0;
void playAudioIn(int phase, int offset) {
	double ropeAngle = (ropeMaxRightAngle-ropeMaxLeftAngle);
	audioVolume = (audioVolumeAdaptive ? min(max(ropeAngle/syncRopeAngle,0),1) : 1.0) * maxAudioVolume;

	audioTime = time+ phase;
	// Snap to Sync
	if (abs(offset) <= audioSnapToSync) {
		audioTime = audioTime - offset;
	} else {
		audioTime = audioTime - offset + int(offset/audioSnapToGrid)*audioSnapToGrid;
	}
	//sprint("A:");sprintln(audioTime-time);
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
		sprint("[");sprint(lastLoopTime);sprint("] LOOP "); sprint(side==LEFT ? "[L]:" : "[R]:");
		sprint("maxR(");sprint(ropeMaxRightAngle); sprint(")-maxL("); sprint(ropeMaxLeftAngle);
		sprint(")=");sprint(ropeMaxRightAngle-ropeMaxLeftAngle); sprintln(" #");
	}
}


void loop(){
  time = millis2();

  showClockIfNeeded();
//  if (time > keepalive) { nextSerialPrintln("k",false); keepalive = time + 1000; }  // inform slaves they are slaves every 1 seconds;

  
  handleKeyboardInput();

  time = millis2();
  // Update clock of slaves
  if (updateSlaveClock || isMaster) { 
	if ((time-syncInitTime)%syncLoopTime  < (lastIterationTime-syncInitTime) % syncLoopTime) {
	  // this means we just got to the init time frame;
	  if (updateSlaveClock) {
		  updateSlaveClock = false;
		  sprint("Sync moved:");
	  	  sprintln((time-syncInitTime) % syncLoopTime);
	  }
	  int delta = (time-syncInitTime) % syncLoopTime;
	  if (delta <5) {
		syncInitTime = time-delta;
	  	nextSerialPrintln("T",false); // update the clock...	 
		// sprint("T<");sprintln(delta);
	  } else { sprint("longOp "); sprintln(delta);}
	}
  }

  // Read input
  int potRead = rawPotentiometerRead();

  static unsigned long POT0Time = 0;

  if (millis() < POT0Time + 300) return;

  if ((potRead < 20) || (potRead > 980)){
  	sprint("POT0   \r");
	POT0Time = millis();
  	return;
  }

  potRead = potentiometerAvg(potRead);
  double currentServoPos = myservoread();
  double potAngle = getPotAngle(potRead);
  double servoAngle = getServoAngle();
  double ropeAngle = potAngle+servoAngle;
  

  ropeMaxRightAngle = max(ropeAngle,ropeMaxRightAngle);
  ropeMaxLeftAngle = min(ropeAngle,ropeMaxLeftAngle);
 
  maxPotRead = max(potRead,maxPotRead);
  minPotRead = min(potRead, minPotRead); 
  // Print measures
  if (printMeasures) {
	sprint("pot: "); sprint(potRead);
	sprint(" Servo: "); sprint(currentServoPos);
	sprint(" potAng: "); sprint(potAngle);
	sprint(" servoAng: "); sprint(servoAngle);
	sprint(" ropeAng: "); sprintln(ropeAngle);
  }
  
  // Play Audio
  if (enableAudio && itIsTime(audioTime+audioDelay) &&
	  (ropeMaxRightAngle>0.05) && (ropeMaxLeftAngle<-0.05)) {
	  playSong(audioSongNumber,audioVolume);
  }

  static unsigned long lastPosCenterUpdate = millis2();
  if ((mode == HALT) && // (lastPosCenterUpdate + 6000 < time) && 
      (time-lastPosCenterUpdate > loopTime * 1.5) && updatePotCenter) {
	if ( (maxPotRead>=minPotRead) && (maxPotRead-minPotRead<30)) {
	  sprint("pot update "); sprint(potCenter); sprint (" -> "); 
	  potCenter = (maxPotRead+minPotRead)/2;
	  sprint(potCenter);
	  sprintln("\r");
	  updatePotCenter = false;
        } else { sprint("&&&");}
	maxPotRead = 0; minPotRead = 1024; lastPosCenterUpdate = time;
  }
  // Analysis when pendulum is at the center
  if ((ropeAngle<0) && (side==RIGHT) && (time-rightTime>loopTime/4)) {
	  // This section is if we are now moving to the left side;
	side = LEFT;
	lastLoopTime = time-leftTime;
	leftTime = time;
	
	sprintLoopEvents();
//	tone(7, NOTE_G6, 100);

	updateAmpAndTime();


	ropeMaxLeftAngle = 0;
	  
  } else if ((ropeAngle>0) && (side==LEFT)&& (time-leftTime>loopTime/4)) {
	// This section is if we are now moving to the right side;
	side = RIGHT;
	lastLoopTime = time-rightTime;
	rightTime = time;

	sprintLoopEvents();
//	tone(7, NOTE_G5, 100);
	
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

