#include "projectConsts.h"
#include "BasicStepperDriver.h"
#include "DFRobot_LIDAR07.h"

//pin13 test led
#define LED_PIN 13
void flashLED(uint32_t flashes = 5);

//Create stepper motor driver instances
BasicStepperDriver upperStepper(UPPER_MOTOR_SPR, UPPER_DIR_PIN, UPPER_STEP_PIN);
BasicStepperDriver lowerStepper(LOWER_MOTOR_SPR, LOWER_DIR_PIN, LOWER_STEP_PIN);
//constants
const uint32_t upperDTS = uint32_t( UPPER_GR * (UPPER_MOTOR_SPR * UPPER_MICROSTEPS) / 360.0 );
const uint32_t lowerDTS = uint32_t( LOWER_GR * (LOWER_MOTOR_SPR * LOWER_MICROSTEPS) / 360.0 );
//flags
bool upperClockwise, lowerClockwise = false;
//func declarations
void moveUpper(float deg);
void moveLower(float deg);


//beam break sensors
volatile bool bbUpperFlag, bbLowerFlag = false;

//MOTOR CONTROL
void upperBBISR();
void lowerBBISR();

//firing mechanism
void fireProjectile();

//setup lidar
DFRobot_LIDAR07_IIC  lidar;
void lidarCapture();

//Serial
void processSerial();


void setup() {
  //Setup debug / test led
  pinMode(LED_PIN, OUTPUT);

  //setup Serial comms
  Serial.begin(115200);
  while(!Serial);

  //Init steppers
  upperStepper.begin(UPPER_MAX_RPM, UPPER_MICROSTEPS);
  lowerStepper.begin(LOWER_MAX_RPM, LOWER_MICROSTEPS);
  //Set stepper acceleration profiles
  upperStepper.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, UPPER_DEFAULT_ACCEL, UPPER_DEFAULT_ACCEL);
  lowerStepper.setSpeedProfile(BasicStepperDriver::LINEAR_SPEED, LOWER_DEFAULT_ACCEL, LOWER_DEFAULT_ACCEL);


  //setup beam break interrupts
  pinMode(UPPER_BB_PIN, INPUT);
  pinMode(LOWER_BB_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(UPPER_BB_PIN), upperBBISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LOWER_BB_PIN), lowerBBISR, RISING);

  //Setup firing mechanism
  //.....

  // //Setup Lidar
  // if(!lidar.begin()){
  //   flashLED(0);
  //   while(1);
  // }
  // lidar.startFilter();
  // lidar.setMeasureMode(lidar.eLidar07Single);

  //Setup End
}

void loop() {
  //main setup
  flashLED(3);
  //main loop
  while(1){
    //check serial comms for message
    processSerial();

    // //check flags for motion failures (bb flags, etc)
    // if(bbLowerFlag){
    //   lowerMotorClearance();
    // }
    // if(bbUpperFlag){
    //   upperMotorClearance();
    // }
    
  }//main loop end
}//arduino loop end



//-------------------------SERIAL STUFF
//Serial message processing
void processSerial(){
  // //check for message over serial
  if (Serial.available() < 8) {
    return;
  }
  //message ready -> read and process
  char buffer[8] = {0,0,0,0,0,0,0,0}; 
  Serial.readBytes(buffer, 8); 
  int32_t cmd = * ((int32_t*) &buffer[0]);;
  int32_t val = * ((int32_t*) &buffer[4]);

  // debug echo command
  // Serial.write((char*)&cmd, 4);
  Serial.write((char*)&val, 4);
  //THIS IS NEEDED AS PYTHON ADDS 0xF0,0xF0 and i dont know why???
  delay(50);
  while(Serial.available()){Serial.read();} //clear any left over bytes

  //select appropriate command
  switch(cmd){ 
    case CMD_MOTOR_X: //upper Stepper Motion
      upperStepper.move(val * DEG_DECIMAL_SHIFT * upperDTS);
      break;
    case CMD_MOTOR_Y: //lower stepper motion
      lowerStepper.move(val * DEG_DECIMAL_SHIFT * lowerDTS);
      break;
    case CMD_FIRE: //fire projectile
      fireProjectile(val);
      break;
    case CMD_GET_LIDAR:
      // sendLidar();
      break;
  }
}

//send an integer value back to PC over serial
void sendSerialInt(uint32_t* dataPtr){
  Serial.write((char*) dataPtr, 4); //type cast int to char(byte), send 4 bytes(one int)
}



//-----------------------------------------------COMMAND ACTION FUNCTIONS

//---------------------PROJECTILE FIRING
void fireProjectile(uint32_t val){
  //Temp func, flash led 5 times
  flashLED(val);
}


//---------------------MOTOR CONTROLLS
//motor motion wrappers
void moveUpper(float deg){
  //Set motion direction flags
  if(deg > 0){
    upperClockwise = true;
  }else{
    upperClockwise = false;
  }
  //Move motor
  upperStepper.move(deg * upperDTS);
}
void moveLower(float deg){
  //Set motion direction flags
  if(deg > 0){
    lowerClockwise = true;
  }else{
    lowerClockwise = false;
  }
  //Move motor
  lowerStepper.move(deg * upperDTS);
}

//Interrupt serivce routines for motor limit reaches
/*@Brief ISR that flips stored state of beam break sensor upon it changing physical state
  Stops motor that was running to set of sensor
*/
void upperBBISR(){
  //if not already noticed (dont want to interrupt clearnce procedure (itself kinda))
  if(!bbUpperFlag){
    bbUpperFlag = true;
    upperStepper.stop();
  }
}
void lowerBBISR(){
  //if not already noticed (dont want to interrupt clearnce procedure (itself kinda))
  if(!bbLowerFlag){
    bbLowerFlag = true;
    lowerStepper.stop();
  }
}

//Motor jerkback routines for limit clearance
void lowerMotorClearance(){
  //check direction moving when interrupt occured
  if(lowerClockwise){
    lowerStepper.move(LOWER_CLEARANCE_DEG * lowerDTS);
  }else{
    lowerStepper.move(-1 * LOWER_CLEARANCE_DEG * lowerDTS);
  }
  //should be clear now, reset flag
  bbLowerFlag = false;
}

void upperMotorClearance(){
  //check direction moving when interrupt occured
  if(upperClockwise){
    upperStepper.move(UPPER_CLEARANCE_DEG * upperDTS);
  }else{
    upperStepper.move(-1 * UPPER_CLEARANCE_DEG * upperDTS);
  }
  //should be clear now, reset flag
  bbUpperFlag = false;
}


//---------------------LIDAR FUNCTIONS
//capture lidar value and send back over serial
void sendLidar(){
  uint32_t lidarmm;
  lidarCapture(&lidarmm);
}

void lidarCapture(uint32_t* valPtr){
  lidar.startMeasure();
  lidar.getValue();
  *valPtr = lidar.getDistanceMM();
}




//---------------------OTHER

//flash onboard led, if flashes set to 0, will flash indefinetly
void flashLED(uint32_t flashes = 5){
  if(!flashes){
    while(1){
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  }else{
    for(uint32_t i = 0; i < flashes; i++){
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  }
}



