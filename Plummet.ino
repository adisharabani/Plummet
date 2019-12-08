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

#define PLUMMET_VERSION "0.21"

#include <SoftwareSerial.h>
#include <Servo.h>
#include <EEPROM.h>

#define rxPinPrev 2 // soft serial
#define txPinPrev 3 // soft serial
#define rxPinNext 4 // soft serial
#define txPinNext 5 // soft serial
#define tonePin 7 // digital
#define servoPin 10 // digital
#define AUDIO_RX 11 //should connect to TX of the Serial MP3 Player module
#define AUDIO_TX 12 //connect to RX of the module

#define potPin 1 // analog


#define SYNC_MAGIC_NUMBER 0
#define AUDIO_DELAY 0

int defaultLoopTime = 3167; //Palo Alto: 3080; // 3160; // 3420;
// int defaultLoopTime = 3080; // Palo Alto

#define EPPROM

// Calibrate(1): 86.00 489 711 293
// Calibrate(2): 98.00 457 665 261
// Calibrate(3): 101.00 525 351 697

boolean SERVO_VIA_TIMER1 = false;

boolean printMeasures = false; 
boolean enablePrint = true;
boolean debug = false;
boolean enableAudio = true;
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

double ropeMaxLeftAngle;
double ropeMaxRightAngle;

unsigned long syncInitTime;
int syncInitTimeOffset;
int syncLoopTime=0;
double syncRopeAngle;
double syncPhase;
boolean updateSlaveClock = false;

SoftwareSerial prevSerial(rxPinPrev, txPinPrev);
SoftwareSerial nextSerial(rxPinNext, txPinNext);

//SoftwareSerial audioSerial(AUDIO_RX, AUDIO_TX);
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


#define NOTE_A5 880
#define NOTE_G5 784
#define NOTE_G6 1568



#define sprint(s)   if (enablePrint) Serial.print(s)
// void sprint(String s){}; void sprint(double s) {}

#define sprintln(s) if (enablePrint) Serial.println(s)

void debugLog(String x) { if (debug) Serial.print(x);         }



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

void ewrite(String data, int index=-1) {
	if (index !=-1) eIndex = index;
	
	for (int i=0; i<data.length(); i++) {
		EEPROM.write(eIndex++, data[i]);
	}
	EEPROM.write(eIndex++, 0);
}

unsigned int eread(int index=-1) {
  if (index!=-1) eIndex = index;
  
  return EEPROM.read(eIndex++)*256+EEPROM.read(eIndex++);
}

char ereadchar(int index=-1) {
	if (index != -1) eIndex = index;
	
	return EEPROM.read(eIndex++);
}

String ereadstr(int index=-1) {
  if (index!=-1) eIndex = index;
  
  String s = "";
  char c;
  while (c = EEPROM.read(eIndex++)) {
  	s += c;
  }
  return s;
}

///////////////////////////
/// SERVO
///////////////////////////
Servo myservo;  // create servo object to control a servo
double lastServoWriteValue = 95;
#define SERVO_PWM_RATE 3040
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
  lastServoWriteValue = pos;
  //Serial.println(duty);

  if (SERVO_VIA_TIMER1) {
    int duty = int(double(map(int(pos), 0,180,544.0,2400.0))/SERVO_PWM_RATE*1024);
//LIB    Timer1.pwm(servoPin, duty);
  } else {
    myservo.write(int(pos));
  }
}

double myservoread() {
  if (SERVO_VIA_TIMER1) {
    return lastServoWriteValue;
  } else {
    return myservo.read();
  }
}

boolean servoAttached = false;
static int originalTCCR1A = 0;
void myservoattach(int pin) {
  if (SERVO_VIA_TIMER1) {
    if (originalTCCR1A == 0) {
      pinMode(pin, OUTPUT);
//LIB      Timer1.initialize(SERVO_PWM_RATE);
    } else {
      TCCR1A = originalTCCR1A;
    }
    servoAttached = true;
  } else {
    myservo.attach(pin);
  }
}

boolean myservoattached() {
  if (SERVO_VIA_TIMER1) {
    return servoAttached;
  } else {
    return myservo.attached();
  }
}
void myservodetach() {
  if (SERVO_VIA_TIMER1) {
    originalTCCR1A = TCCR1A;
//LIB    Timer1.disablePwm(servoPin);
    servoAttached = false;
  } else {
    myservo.detach();
  }
}

void smoothMove(double desiredPosition, int totalTime = 2000) {
  double sl = myservoread();
  int iterations = totalTime/20;
  for (int i=0; i<=iterations; i++) {
     double np = sl*double(iterations-i)/iterations + desiredPosition*double(i)/iterations;
     myservowrite(np);
     debugLog("moving Servo from "+String(sl)+ " to "+String(desiredPosition) + " Step("+String(i)+" of "+String(iterations)+"): "+np+"\n\r");
     delay(20);
  }
}

double getServoAngle() {
  return -(0.0+myservoread()-servoCenter)/100*(PI/2);
}

int angleToServo(double angle) {
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
int positions[POSITIONS_STACK_SIZE]; 
unsigned int positionsPTR = POSITIONS_STACK_SIZE;
#define push(p) positions[(++positionsPTR)%POSITIONS_STACK_SIZE] = p
#define getPos(i) positions[(positionsPTR-i)%POSITIONS_STACK_SIZE]
#define lastPos getPos(0)


int potentiometerRead() {
  push(int(analogRead(potPin)));
  return posAvg();
}

double getPotAngle() {
  int p = potentiometerRead();
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

///////////////////////////
/// Modes
///////////////////////////
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
  sprint("Old ");sprintln("servoCenter: "+String(servoCenter));
  servoCenter = myservoread();
  sprintln("servoCenter: "+String(servoCenter));
  
  sprint("Old ");sprintln("PotCenter: "+String(potCenter));
  potCenter = waitForSteadiness(1,6000);  
  sprintln("PotCenter:  "+String(potCenter));

  smoothMove(servoCenter-50,4000); delay(1000);
  sprint("Old ");sprintln("Pot50: "+String(pot50));
  pot50 = waitForSteadiness(10,6000);  
  sprintln("Pot50:  "+String(pot50));

  smoothMove(servoCenter+50,8000); delay(1000);
  sprint("Old ");sprintln("Pot150: "+String(pot150));
  pot150 = waitForSteadiness(10,6000); 
  sprintln("Pot150:  "+String(pot150));
  
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
  sprintln("Old LoopTime " + String(loopTime));
  smoothMove(servoCenter-maxServoAmp);
  initTime = millis();
  sprintln("Speed up");
  unsigned long t = millis();
  mode = START;
  while (millis() < t + 9000) {
    loop();
  }
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
          sprintln("Cycle "+String(cycles)+" of "+String(nCycles) +" complete. Average loop time: "+String((time-initLeftTime)/cycles));
        }
     } else if ((side==LEFT) && (potRead > potCenter) && (time-leftTime>loopTime/5) ) {
        //sprintln("Right side");
        side = RIGHT;
        rightTime = time;
     }
  } while ((cycles < nCycles)); // TIMEOUT:  && (millis() < initTime + 6000 + 3000 + 4000*(cycles+2)));

  defaultLoopTime = (cycles == nCycles) ? (time-initLeftTime)/(cycles) : loopTime;
  loopTime = defaultLoopTime;

  sprintln("loopTime Calibration ("+String(cycles)+"): " + String(loopTime));
}

void writeCalibration() {
  ewrite(EEPROM_MAGIC,0); //magic number
  ewrite(2); // calibration version
  ewrite(servoCenter);
  ewrite(potCenter);
  ewrite(pot50);
  ewrite(pot150);
  ewrite(loopTime);
  ewrite(0);
  ewrite(0);
  ewrite(0);
  ewrite(0);

  sprintln("EEPROM: " +String(servoCenter) + " " + String(potCenter) + " " + String(pot50) + " " + String(pot150) + " " + String(loopTime));
}
 

void readCalibration() {
  if (eread(0) == EEPROM_MAGIC) { // confirm magic number
    eread(); // calibration version
    servoCenter = eread();
    potCenter = eread();
    pot50 = eread();
    pot150 = eread();
    loopTime = eread(); defaultLoopTime = loopTime;
    sprintln("EEPROM: " +String(servoCenter) + " " + String(potCenter) + " " + String(pot50) + " " + String(pot150) + " " + String(loopTime));
  } else {
    sprintln("No EEPROM");
    sprintln("ServoCenter?="+String(eread(1)));
    servoCenter = eread(1);
  }
}


void updateAmpAndTimeForStopping() {
  loopTime = defaultLoopTime;
  initTime = millis()-loopTime*(side==LEFT ? 0.25 : 0.75) + SYNC_MAGIC_NUMBER;

  servoAmp = (angleToServo(ropeMaxRightAngle)-angleToServo(ropeMaxLeftAngle));
  servoAmp = servoAmp*1.5;
  servoAmp = max(min(maxServoAmp,servoAmp),0);
//  servoAmp = servoAmp*servoAmp/40;
  if (servoAmp < 25) { servoAmp = servoAmp/2; } // was servoAmp/2

  if (servoAmp < 10) { servoAmp = 0; }        // was <10
  
  if (showLoopEvents) sprintln("- Update ServoAmp: maxRight("+String(ropeMaxRightAngle)+")-maxLeft(" + String(ropeMaxLeftAngle) + ")="+ String(ropeMaxRightAngle-ropeMaxLeftAngle) + " ==> New servoAmp: "+String(servoAmp) + " -");
}

void updateAmpAndTimeForMaintaining() {
  loopTime = defaultLoopTime;
//  initTime = millis()-loopTime*(side==LEFT ? 0.75 : 0.25) + SYNC_MAGIC_NUMBER;
  initTime = millis()-loopTime*(side==LEFT ? 0.5 : 0) + SYNC_MAGIC_NUMBER;
  // Maybe add MAINTAIN looptime:::  if (time-leftTime >2000) { loopTime = time-leftTime;}
  double desiredAngle = 0.4;
  double ropeAngle = ropeMaxRightAngle-ropeMaxLeftAngle;
  if ( ropeAngle > desiredAngle*1.2) {
    servoAmp = 0;
  } else if (ropeAngle > desiredAngle*1.1) {
    servoAmp = (angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8;
  }  else {
    servoAmp = (angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8 + (1 - ropeAngle/desiredAngle)*maxServoAmp;
  }
    sprintln("ServoAmp: "+ String(servoAmp) +" baseline: "+ String((angleToServo(desiredAngle/2)-servoCenter)*2 * 0.8));
}

void updateAmpAndTimeForRunning() {
  loopTime = defaultLoopTime;
  initTime = millis()-loopTime*(side==LEFT ? 0.75 : 0.25) + SYNC_MAGIC_NUMBER;
  servoAmp = maxServoAmp;
}


void updateAmpAndTimeForTesting() {
  loopTime = defaultLoopTime;
  servoAmp = testAmp;
//
  initTime = millis()-loopTime-loopTime*(side==LEFT ? 0.5+testPhase : testPhase) + SYNC_MAGIC_NUMBER;
//  if (ropeMaxRightAngle-ropeMaxLeftAngle < 0.4) {
//    servoAmp = 10;
//  } else {
//    servoAmp = 0;
//  }
}

//int q = 0;
void updateAmpAndTimeForSyncedRunning() {  

  // update loopTime
  //loopTime = syncLoopTime;

  // update servoAmp
  double offsetRopeAngle = ropeMaxRightAngle-ropeMaxLeftAngle - syncRopeAngle;
  if (offsetRopeAngle < 0) {
    servoAmp = min(servoAmp + 1, maxServoAmp);
  } else {
    servoAmp = max(servoAmp - 1, 3);
  }

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
      //      // Linear calculation, offset:0==>phase:0.25; offset:0.05==>0.5; offset:-0.05==> 0; trim for phase to be between 0 to 0.5;
        desiredPhase = max(min(0.25 + offset/0.05*0.25, 0.5), 0);
      }
    } else if (offset>0) {
      desiredPhase = 0.6;
      servoAmp = maxServoAmp;
    } else if (offset<0) {
      desiredPhase = 0.9;
      servoAmp = maxServoAmp;
    }
//    syncPhase = (desiredPhase*0.5 + syncPhase*0.5);
    syncPhase = desiredPhase;
    initTime = millis()-loopTime*(side==LEFT ? syncPhase+0.5 : syncPhase);
    
    sprint("Syncing: Offset(" + String(offset) + "), ropeAmp("+ String(ropeMaxRightAngle-ropeMaxLeftAngle));
    sprint(") ==> loopTime=" + String(syncLoopTime));
    sprint("; ServoAmp=" + String(servoAmp));
    sprint("; phase="+String(syncPhase));
    sprintln("(wanted"+String(desiredPhase)+")" );
  }
}

void updateAmpAndTime() {
    if (mode == RUNNING)        { updateAmpAndTimeForRunning();      }
    if (mode == MAINTAINING)    { updateAmpAndTimeForMaintaining();      }
    if (mode == TESTING)        { updateAmpAndTimeForTesting();          }
    if (mode == STOPPING)       { updateAmpAndTimeForStopping();         }
    if (mode == SYNCED_RUNNING) { updateAmpAndTimeForSyncedRunning();    }
}

///////////////////////////
/// User Data
///////////////////////////

boolean commandAvailable() {
  return (isPlaying && (nextCommandTime != MAX_UINT) && (millis() > nextCommandTime*1000+playInitTime));
}

String readCommand() {
	String command = ereadstr(nextCommandLoc);
	nextCommandTime = eread();
	nextCommandLoc = eIndex;
	return command;
}


byte readByte() {
  unsigned long timeout = millis() + 100;
  char b;
  while (millis() < timeout) {
    if (Serial.available()) {
      b = Serial.read();
      //sprint(b);
      return b;
    } else if (prevSerial.available()){
      isMaster = false;
      char b = prevSerial.read();
//      if (b=='~') { Serial.write("Error:"); delay(100); while (prevSerial.available()) Serial.print("~"+String(int(prevSerial.read()))); Serial.println("");}
      //sprint(b);
      return b;
    }
  }
sprint("?");
}

bool readByteAvailable() {
  return (Serial.available() || prevSerial.available());
}

String keyboardBuffer = "";

void handleKeyboardInput() {
  ////////////
  // input
  char inByte = 0;
  if (keyboardBuffer.length() > 0) {
     inByte = keyboardBuffer[keyboardBuffer.length()-1];
  }
  if (commandAvailable()) {
     String cmd = readCommand();
     sprintln(cmd);
   	 keyboardBuffer += (cmd + "\n");
	 if ((keyboardBuffer.length()==0) || (keyboardBuffer[keyboardBuffer.length()-1] != '\n')) {
  		sprint("BAD COMMAND:");
  		sprintln(keyboardBuffer);
  		keyboardBuffer = "";
  		return;
  	}
  	inByte = '\n';
  } else {
	  while (readByteAvailable() && (inByte != '\n')) {
	     inByte = readByte();
	     if ((inByte=='\r') || (inByte=='\n')) {
	       //convert \r to \n
	       inByte = '\n';
	       if (keyboardBuffer.length()>0) {
	       	 sprintln("");
             if (isRecording) {
	           ewrite((millis()-recordInitTime)/1000, recordingLoc);
	           ewrite(keyboardBuffer);
	           recordingLoc = eIndex;
	           ewrite(MAX_UINT);
	         }
             keyboardBuffer += inByte;
             byte k = keyboardBuffer[0];
             int id;
             if (k==':') {
			     String who = keyboardBuffer.substring(1,keyboardBuffer.indexOf(":",1));
			     keyboardBuffer = keyboardBuffer.substring(keyboardBuffer.indexOf(":",1)+1,keyboardBuffer.length());
			     if (who=="e") {
			     	nextSerial.println(String(":o:")+keyboardBuffer);
			     } else if (who=="o") {
			     	nextSerial.println(String(":e:")+keyboardBuffer);
			     	inByte = 0; keyboardBuffer = ""; //ignore this command as it is not for this arduino.
			     } else if (who=="w") {
			     	nextSerial.println(String(":w:")+keyboardBuffer);
			     	delay(50);
			     	nextSerial.println(keyboardBuffer);
			     } else {
				     id = who.toInt();
				     if (id>0){
				       // notify other arduinos
				       nextSerial.println(String(":") + String(id-1)+String(":") + keyboardBuffer);
//				       String wr = String(":") + String(id-1) + String(":") + keyboardBuffer;
				       //sprintln("Command forwarded to the next device");
//				       nextSerial.println(wr);
				       inByte = 0; keyboardBuffer = ""; //ignore this command as it is not for this arduino.
			     	} else {
				       // do not notify other arduinos
				       //sprintln("Command is directed to me only");
				     }
			     }
			 }
			 switch (k) {
 			   case ':':
			   case 'e':
			   case 'E':
			   case 'p':
			   case 'd':
			   case 'L':
			   case '$':
			   case '%':
			   case '?':
			   case ' ':
			   case '\n':
			     break;
			   default:
			     nextSerial.print(keyboardBuffer);
			     break;
			  }
			  inByte = '\n';
	       }
	     } else if (inByte == 127) {
	       //handle backspace
	       if (keyboardBuffer.length()>0) {
	         keyboardBuffer = keyboardBuffer.substring(0,keyboardBuffer.length()-1);
	         inByte = keyboardBuffer[keyboardBuffer.length()-1];
	         sprint("\b\b\b   \r");
	         sprint(keyboardBuffer);
	       } else {
	         inByte = 0;
	       }
	     } else if (inByte == ' ') {
	       // disregard spaces
	       inByte = keyboardBuffer[keyboardBuffer.length()-1];
	     }else {
	       keyboardBuffer += String(inByte);
	       sprint(inByte);
	     }
	  }
  }
  if (inByte != '\n') {
    return;
  }
  if (keyboardBuffer.length() == 0) return;

  inByte = keyboardBuffer[0]; keyboardBuffer = keyboardBuffer.substring(1);

  if (inByte == '\n') {return; } 

  int s;
  switch (inByte) {
    case 'e': // Enable output
      enablePrint = true;
      break;
    case 'E': // Disable output
      enablePrint = false;
      break;
    case 'd': // Debug
      debug = !debug;
      enablePrint = true;
      sprintln("debug is "+ String(debug ? "on" : "off"));
      break;
    case 'p': // Print measurements (potentiometer, servo, etc.)
      printMeasures = !printMeasures;
      enablePrint = true;
      sprintln("printMeasures is "+ String(printMeasures ? "on" : "off"));
      break;
    case 'L': // Print loop events (move from RIGHT to left)
      showLoopEvents = !showLoopEvents;
      enablePrint = true;
      sprintln("showLoopEvents is "+ String(showLoopEvents ? "on" : "off"));
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
    case 't': /* TEST */
      mode = TEST; sprintln("TEST");
      break;
    case ']': /* Increase TEST Phase */
      testPhase = (int((testPhase + 0.05)*100) % 100) /100.0;
      sprintln("Testing phase is "+String(testPhase));

      break;
    case '[': /* Decrease TEST phase */
      testPhase = (int((testPhase - 0.05)*100) % 100) /100.0;
      sprintln("Testing phase is "+String(testPhase));
      break;
    case '>': /* Increase TEST amplitude */
      testAmp = min(testAmp + 5,100);
      sprintln("Testing amp is "+String(testAmp));
      break;
    case '<': /* Decrease TEST amplitude */
      testAmp = max(testAmp -5,0);
      sprintln("Testing amp is "+String(testAmp));
      break;
    case 's': // SYNC
      s = keyboardBuffer.toInt(); keyboardBuffer = "";
      syncLoopTime = (s!=0) ? s : 3185; //defaultLoopTime+16; // RONEN=3174
      syncInitTime = millis();
      syncInitTimeOffset = 0;
      syncRopeAngle = 0.3;
      servoAmp = 20;
      mode = SYNCED_RUN; sprintln("SYNC" + String(syncLoopTime));
      break;
    case 'S': // SYNC to an already set clock (don't update the clock)
      mode = SYNCED_RUN; 
      break;
    case 'T': // Set the clock for SYNC
      syncInitTime = millis();
      break;
    case 'u': // How long was the syncInitTime ago?
      sprintln("syncInitTime was "+ String(millis()-syncInitTime) + " milliseconds ago");
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
    case 'r': // Randomize the SYNC clock 
      syncInitTime = millis() - random(0,defaultLoopTime);
      syncInitTimeOffset = 0;
      servoAmp = 20;
      mode = SYNCED_RUN; 
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
    case '=': // Move servo to specific location
      s = keyboardBuffer.toInt(); keyboardBuffer = "";
      if (s!=0) smoothMove(s);
      // myservo.write(n);
      break;
    case '+': // Move servo one step forwards
      s = myservoread(); 
      sprint("Old "); sprintln("servo is " + String(s));
      myservowrite(s+1);
      sprintln("servo is " + String(myservoread()));    
      break;
    case '-': // Move servo one step backwards
      s = myservoread();
      sprint("Old "); sprintln("servo is " + String(s));
      myservowrite(s-1);
      sprintln("servo is " + String(myservoread()));    
      break;
    case '_': // Detach or reattach servo
      if (myservoattached()) {
        myservodetach();
      } else {
         myservoattach(servoPin);
      }
      break;
/*   case '~': 
	 // Change Servo type (Timer1 vs Servo library)
     myservodetach();
     SERVO_VIA_TIMER1 = !SERVO_VIA_TIMER1;
     myservoattach(servoPin);
     sprintln(SERVO_VIA_TIMER1 ? "Using Timer1" : "Using Servo lib");
     break; */
    case 'b': /* Beep */
      tone(7, NOTE_A5, 1000);
      break;
    case 'B': // Are you a master?
      sprintln(isMaster ? "I am a master" : "I am a slave"); 
      if (isMaster) tone(7, NOTE_A5, 1000);
      break;
    case 'c': // Calibrate
      calibrate();
      break;
    case 'C': // Save calibration
      writeCalibration();
      break;
    case 'l': // Calibrate loop time
      calibrateLoopTime();
      break;
    case 'a': // play audio
      sendAudioCommand(0X22, 0X1E01);
      break;
    case 'A': // enable/disable audio
      enableAudio = !enableAudio;
      sprintln(String("Audio ")+(enableAudio ? "enabled" : "disabled"));
      break;
    case '$': //Play
      if (!isPlaying) {
        nextCommandTime = eread(EEPROM_COMMANDS_LOC);
        nextCommandLoc = eIndex;
        if (nextCommandTime == MAX_UINT) {
        	sprintln("No recorded actions");
        } else {
	      	isPlaying = true;
    	  	playInitTime = millis();
	        sprint("Next action in ");
	        sprint(nextCommandTime);
	        sprintln(" seconds");
        }
      } else {
      	sprintln("Playback stopped");
      	nextCommandTime = MAX_UINT;
      	isPlaying = false;
      }
      break;
    case '%': //Record
      if (isPlaying) {
      	sprintln("Playback finished");
      	isPlaying = false;
        nextCommandTime = MAX_UINT; // stop playback if needed;
      } else {
        isRecording = !isRecording;
        sprintln(isRecording ? "Recording on" : "Recording off");
        recordingLoc = EEPROM_COMMANDS_LOC;
        recordInitTime = millis();
      }
      break;
    case '?': //Show recorded commands;
      unsigned int commandTime = eread(EEPROM_COMMANDS_LOC);
      sprintln("Recorded Commands:");
      while (commandTime != MAX_UINT) {
      	 sprint(commandTime);
      	 sprint("s: ");
      	 sprint(ereadstr());
      	 commandTime = eread();
      }
	  break;
    default:
      break;
  }
}


//////////////////////////////
// Audio
//////ֿ////////////////////////

void sendAudioCommand(int8_t command, int16_t dat)
{
  static int8_t Send_Audio_buf[8] = {0x7e, 0xff, 0x06, 0, 0, 0, 0, 0xef} ;

  if (!enableAudio) return;
  debugLog("audio");
  Send_Audio_buf[0] = 0x7e; //starting byte
  Send_Audio_buf[1] = 0xff; //version
  Send_Audio_buf[2] = 0x06; //the number of bytes of the command without starting byte and ending byte
  Send_Audio_buf[3] = command; //
  Send_Audio_buf[4] = 0x00;//0x00 = no feedback, 0x01 = feedback
  Send_Audio_buf[5] = (int8_t)(dat >> 8);//datah
  Send_Audio_buf[6] = (int8_t)(dat); //datal
  Send_Audio_buf[7] = 0xef; //ending byte
  for(uint8_t i=0; i<8; i++)//
  {
    audioSerial.write(Send_Audio_buf[i]) ;
  }
}

//////////////////////////////
// Main Code
//////////////////////////////


static void handleRxChar( uint8_t c ) {}

void setup() {  

  Serial.begin(9600);
  Serial.println(String("v") + String(PLUMMET_VERSION));

  nextSerial.begin(9600);
  prevSerial.begin(9600);

  nextSerial.println("9"); // let the following arduino know you are here and set on HALT mode;

  //prevSerial.attachInterrupt(t);
  
  sendAudioCommand(0X09, 0X02);
  sprintln("");
  readCalibration();

  myservoattach(servoPin);
  smoothMove(servoCenter);
  tone(7, NOTE_G5, 100);
  

  mode=HALT;
  initTime = millis();
  randomSeed((initTime + potentiometerRead()*10) % 30000);
  updateAmpAndTime();

  // wait 3 seconds to see if you are a slave
  while ((millis() < initTime + 3000) && isMaster) {
     if (prevSerial.available()) {isMaster = false; } 
  }
  sprintln(isMaster ? "I am master" : "I am slave"); 
}

unsigned long keepalive = 0;
void loop(){
  if (time > keepalive) { nextSerial.print(" "); keepalive = time + 1000; }  // inform slaves they are slaves every 1 seconds;
  if (prevSerial.available()) {if (isMaster) sprintln("I am now a slave"); isMaster = false;} // inform 
  // if (isMaster) { tone(7, NOTE_F5,100); }   // If master make noise
  
  time = millis();

  handleKeyboardInput();

  int potRead = potentiometerRead();
  double currentServoPos = myservoread();
  double potAngle = getPotAngle();
  double servoAngle = getServoAngle();
  double ropeAngle = potAngle+servoAngle;

  ropeMaxRightAngle = max(ropeAngle,ropeMaxRightAngle);
  ropeMaxLeftAngle = min(ropeAngle,ropeMaxLeftAngle);
/*
  if (printMeasures) {
    sprint("potRead: "+String(potRead) + "  ");
    sprint("currentServoPos: "+String(currentServoPos) + "  ");
    sprint("potAngle: "+String(potAngle) + "  ");
    sprint("servoAngle: "+String(servoAngle) + "  ");
    sprint("ropeAngle: "+String(ropeAngle) + "  ");
    sprintln("");
  }
  */
  
  switch (mode) {
    case SYNCED_RUN:
      mode = SYNCED_RUNNING;
      break;
    case TEST:
      initTime = time;
      mode = TESTING;
      updateAmpAndTime();
      sprintln("testPhase is: "+String(testPhase));
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

  if ((time-rightTime+AUDIO_DELAY >= loopTime/4) && 
      (lastIterationTime-rightTime+AUDIO_DELAY < loopTime/4) &&
      (ropeMaxRightAngle>0.05) && (ropeMaxLeftAngle<-0.05)) {
    sendAudioCommand(0X22, 0X1E01);
  }

  if ((ropeAngle<0) && (side==RIGHT) && (time-rightTime>loopTime/5)) {
      // This section is if we are now moving to the left side;
    side = LEFT;
    lastLoopTime = time-leftTime;
    leftTime = time;
    if (showLoopEvents) sprintln("# Loop: Time("+String(lastLoopTime)+")" + (side==LEFT ? " [L] :" : " [R:") + "maxRight("+String(ropeMaxRightAngle)+")-maxLeft(" + String(ropeMaxLeftAngle) + ")="+ String(ropeMaxRightAngle-ropeMaxLeftAngle)+ " #");
    tone(7, NOTE_G6, 100);

    if (mode != RUNNING) updateAmpAndTime();

    ropeMaxLeftAngle = 0;
      
  } else if ((ropeAngle>0) && (side==LEFT)&& (time-leftTime>loopTime/5)) {
    // This section is if we are now moving to the right side;
    side = RIGHT;
    lastLoopTime = time-rightTime;
    rightTime = time;
    if (showLoopEvents) sprintln("# Loop: Time("+String(lastLoopTime)+")" + (side==LEFT ? " [L] :" : " [R]:") + "maxRight("+String(ropeMaxRightAngle)+")-maxLeft(" + String(ropeMaxLeftAngle) + ")="+ String(ropeMaxRightAngle-ropeMaxLeftAngle)+ " #");
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

  // Update clock of slaves
  if (updateSlaveClock && isMaster && (mode == SYNCED_RUNNING)) {
    if ((time-syncInitTime)%syncLoopTime  < (lastIterationTime-syncInitTime) % syncLoopTime) {
      // this means we just got to the init time frame;
      updateSlaveClock == false;
      nextSerial.println("T"); // update the clock...     
    }
  }

   lastIterationTime = time;

}

