/*
Huge thanks to those who have derived the kinematic conrrols for omni-directional robots. This hackpack comes with an optional educational reading that I highly reccomend: (Siradjuddin, Indrazno. "Kinematics and control a three wheeled omnidirectional mobile robot." Int. J. Electr. Electron. Eng 6.12 (2019): 1-6.) [https://www.internationaljournalssrg.org/IJEEE/2019/Volume6-Issue12/IJEEE-V6I12P101.pdf] 

Amazingly the relationship of the wheel speeds of the robot to its overall speed vector is algebraically linear.  The formula also works for any relative wheel angle and can easily be expanded to include more wheels. This gives a mathematically deterministic way to get the robot from A to B defined by two translation variables and one rotation variable, a powerful tool. 

Combining this with a gyroscope allows for a true field oriented drive-- where  joystick commands always will move the robot forwards relative to you, despite which way the robot is facing. For those of you willing to take on the challenge, this is a great hack to try!
*/
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "config.h"
#include "sbus.h"
#include "FrSkySBUS.h"

#define M1_DIR 5
#define M1_PWM 6
#define M2_DIR 8
#define M2_PWM 7
#define M3_DIR 9
#define M3_PWM 10

#define LIFT_DIR 3
#define LIFT_PWM 4


//>>>>>>>>>>>>>>>>>>>>>>>>>> KEY ROBOT VARIABLES <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//SBUS Setup ----------------------------------------------------
// SBUS setup using SoftwareSerial on pin D12
SoftwareSerial sbusSerial(SBUS_RX_PIN, SBUS_TX_PIN);
bfs::SbusRx sbus_rx(&sbusSerial);
SBUSManager rc;

// Channel mapping
#define CH_FORWARD_BACK  0  // Channel 1: Forward/Backward drive
#define CH_STRAFE        1  // Channel 2: Left/Right strafe
#define CH_ROTATE        2  // Channel 3: Rotate left/right
#define CH_LIFT          3  // Channel 4: Lift up/down

//Remote Commands ----------------------------------------------------
// remote command: drive vector (in velocities) {v_x, v_y, omega}
int vSpeed[3];
// remote command: for forklift, continuous control -100 to +100
int liftControl = 0;

//Robot Physical Parameters -------------------------------------
double angleWheel1 = 90.0;
double angleWheel2 = 210.0;
double angleWheel3 = 330.0;

// Equation puts wheel 1 facing forwards. This rotates the robot's "front" ccw.
double localAngle = 60.0;
//double gyroAngle;
// Radius of wheel center to robot center
double botRadius = 100.0;
// Radius of omniwheel
double wheelRadius = 35.0;

// stores each calculated wheel speed for a given commanded vector {w1, w2, w3}.
double wheelSpeeds[3];

// tuning paratmeter to get linearized motor speed.
double tune = 1; 

// strafing timer variable for catching odd remote behavior
unsigned long lastStrafe = 0;


// pre-define FUNCTIONS ---------------------------------------------
void moveBot();
void getWheelSpeeds(float* wheelList);
void driveWheels(float* speeds, float maxSpeed);
void driveLift();


// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SETUP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void setup() {
  Serial.begin(115200);
  Serial.println("OMNIB V1.0.0 SBUS ..... (01/11/2026)");

  // Initialize SBUS receiver on SoftwareSerial
  sbus_rx.Begin();

  // Setup motor pins
  pinMode(M1_DIR, OUTPUT);
  pinMode(M1_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(LIFT_DIR, OUTPUT);
  pinMode(LIFT_PWM, OUTPUT);
}



// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LOOP <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
void loop() {
  // Read SBUS data
  if (sbus_rx.Read()) {
    rc.update(sbus_rx.data());

    // Map SBUS channels to velocity vector
    // Use toPercent() to get -100 to +100 range
    int forward_back = rc[CH_FORWARD_BACK].toPercent();   // -100 to +100
    int strafe = rc[CH_STRAFE].toPercent();                // -100 to +100
    int rotate = rc[CH_ROTATE].toPercent();                // -100 to +100
    int lift = rc[CH_LIFT].toPercent();                    // -100 to +100

    // Convert to velocity vector (scale to ±maxSpeed range)
    // Note: vSpeed values are normalized (-1 to +1) for kinematics, not PWM values
    vSpeed[0] = strafe;        // X velocity (strafe left/right)
    vSpeed[1] = forward_back;  // Y velocity (forward/back)
    vSpeed[2] = rotate;        // Omega (rotation)

    // Update lift control
    liftControl = lift;

    // Failsafe handling
    if (sbus_rx.data().failsafe || sbus_rx.data().lost_frame) {
      // Stop all motors on signal loss
      vSpeed[0] = 0;
      vSpeed[1] = 0;
      vSpeed[2] = 0;
      liftControl = 0;
      Serial.println("FAILSAFE: Signal lost!");
    }
  }

  // Execute motor control
  moveBot();
}


// >>>>>>>>>>>>>>>>>>>>>>>> ROBOT ACTION FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// prepare motors to coorinate bot movements
void moveBot(){
  float wheelSpeeds[3];
  getWheelSpeeds(wheelSpeeds);
  driveWheels(wheelSpeeds, maxSpeed);
  driveLift();
}

// Addresses motor controllers to drive wheels at caclulated speeds
void driveWheels(float* speeds, float maxSpeed){  
  //drive all motors at calculated speeds
  float topWheelSpeed = max(max(abs(speeds[0]), abs(speeds[1])), abs(speeds[2]));

  wheelSpeeds[0] = speeds[0] / topWheelSpeed;
  wheelSpeeds[1] = speeds[1] / topWheelSpeed; 
  wheelSpeeds[2] = speeds[2] / topWheelSpeed;

  if(speeds[0] < 0){
    wheelSpeeds[0] = - pow(abs(wheelSpeeds[0]), tune) * maxSpeed;
  } else {
    wheelSpeeds[0] = pow(wheelSpeeds[0], tune) * maxSpeed;
  }
  if(speeds[1] < 0){
    wheelSpeeds[1] = - pow(abs(wheelSpeeds[1]), tune) * maxSpeed;
  } else {
    wheelSpeeds[1] = pow(wheelSpeeds[1], tune) * maxSpeed;
  }
  if(speeds[2] < 0){
    wheelSpeeds[2] = - pow(abs(wheelSpeeds[2]), tune) * maxSpeed;
  } else {
    wheelSpeeds[2] = pow(wheelSpeeds[2], tune) * maxSpeed;
  }

  if(flipM1){
     wheelSpeeds[0] = - wheelSpeeds[0];
  }
  if(flipM2){
     wheelSpeeds[1] = - wheelSpeeds[1];
  }
  if(flipM3){
     wheelSpeeds[2] = - wheelSpeeds[2];
  }
  

  if(wheelSpeeds[0] < 0){
    digitalWrite(M1_DIR, LOW);
  } else {
    digitalWrite(M1_DIR, HIGH);
  }
  analogWrite(M1_PWM, int(abs(wheelSpeeds[0])));

  if(wheelSpeeds[1] < 0){
    digitalWrite(M2_DIR, LOW);
  } else {
    digitalWrite(M2_DIR, HIGH);
  }
  analogWrite(M2_PWM, int(abs(wheelSpeeds[1])));

  if(wheelSpeeds[2] < 0){
    digitalWrite(M3_DIR, LOW);
  } else {
    digitalWrite(M3_DIR, HIGH);
  }
  analogWrite(M3_PWM, int(abs(wheelSpeeds[2])));
}

// Moves lift motor with proportional speed control based on SBUS input
void driveLift(){
  if(abs(liftControl) > SBUS_DEADBAND) {  // Outside deadband
    // Calculate proportional speed (map 10-100 to minimum speed to maxSpeed)
    int speed = map(abs(liftControl), SBUS_DEADBAND, 100, 100, maxSpeed);

    if(flipM4){
      if(liftControl > 0){  // Lift up
        digitalWrite(LIFT_DIR, LOW);
        analogWrite(LIFT_PWM, speed);
      } else {  // Lift down
        digitalWrite(LIFT_DIR, HIGH);
        analogWrite(LIFT_PWM, speed);
      }
    } else {
      if(liftControl > 0){  // Lift up
        digitalWrite(LIFT_DIR, HIGH);
        analogWrite(LIFT_PWM, speed);
      } else {  // Lift down
        digitalWrite(LIFT_DIR, LOW);
        analogWrite(LIFT_PWM, speed);
      }
    }
  } else {  // Within deadband - stop motor
    analogWrite(LIFT_PWM, 0);
  }
}



// >>>>>>>>>>>>>>>>>>>>>>>>>> INVERSE KINEMATICS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// calculates speed of each wheel based of commanded speed vector. Forumlas are in the linked reading!
void getWheelSpeeds(float* wheelList) {
  double botAngle = localAngle; //+ gyroAngle;

  // 0.0174533 converts deg to rad 
  wheelList[0] = (-sin((botAngle + angleWheel1) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel1) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2]) / wheelRadius;
  
  wheelList[1] = (-sin((botAngle + angleWheel2) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel2) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2])/ wheelRadius;
  wheelList[2] = (-sin((botAngle + angleWheel3) * 0.0174533) 
                * cos(botAngle * 0.0174533) * vSpeed[0] + cos((botAngle + angleWheel3) * 0.0174533) * cos(botAngle * 0.0174533) * vSpeed[1] + botRadius * vSpeed[2])/ wheelRadius;         
}