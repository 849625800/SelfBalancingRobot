#include <SPI.h>
#include <MPU6050_tockn.h>

#define PWMA 11
#define AIN2 8
#define AIN1 7
#define STBY 9
#define BIN1 6
#define BIN2 5
#define PWMB 12

float angleY, angleZ, gyroY, gyroZ;
float eAngleY;
float bPWM, Pluse_L, Pluse_R;
long loop_timer;
MPU6050 mpu6050(Wire);

void setup() {
  // put your setup code here, to run once:

  TCCR1B = (TCCR0B & 0xF8) | 0x01;   //set frquency 63kHZ.
//  ICR1 = 0x0fff;                     //set resolution to 12bits.
  OCR1A = 0;
  OCR1B = 0;

  Serial.begin(115200);
  
  motorInit();

  loop_timer = 0;
  
  Wire.begin();
  mpu6050.begin();
//  mpu6050.calcGyroOffsets(true);
  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:
  mpu6050.update();
  angleY = mpu6050.getAngleY();                                            //get angle and gyro value
  angleZ = mpu6050.getAngleZ();
  gyroY = mpu6050.getGyroY();
  gyroZ = mpu6050.getGyroZ();

  balancing_PD(angleY, gyroY);
  Pluse_L = bPWM;                       //get pluse of Motors for PWM wave.
  Pluse_R = bPWM;

  if (abs(angleY) < 30)                //if angleY is between -20 and 20 ,run it
    {
      if ((Pluse_L) < 0)
      {
        moveForwardL();
        analogWrite(PWMA,abs(Pluse_L));
      }
      else
      {
        moveBackwardL();
        analogWrite(PWMA,abs(Pluse_L));
      }
  
      if ((Pluse_R) < 0)
      {
        moveBackwardR();
        analogWrite(PWMB,abs(Pluse_R));
      }
      else
      {
        moveForwardR();  
        analogWrite(PWMB,abs(Pluse_R));
      }
    }
    else
    {
      robotStop();
      reset(); 
    }

//Serial.print("angleY: "); Serial.print(angleY); Serial.print("  ");
//  Serial.print("PWM: "); Serial.print(OCR1A); Serial.print("  ");
//  Serial.print("BPWM: "); Serial.print(bPWM); Serial.print("  ");
//  Serial.println("");
  
  while (micros() - loop_timer < 5000);                                     //wait for 5000us.
  loop_timer = micros();                                                    //Set the timer for the next loop.
}
//-------------------helper functions-------------------
/**
 * This function returns the pwm for balancing.
 */
float balancing_PD(float angleY, float gyroY) {
  static float Kp = - 18, Kd = -0.18;
  eAngleY = angleY - 2.0;
  bPWM = Kp * eAngleY + Kd * gyroY;
  bPWM = constrain(bPWM, -255, 255);
  return bPWM;
}

void moveForwardL() {
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,HIGH);
}

void moveBackwardL() {
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,LOW);
}

void moveForwardR() {
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,LOW);
}

void moveBackwardR() {
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,HIGH);
}

void motorInit() {
  pinMode(PWMA,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(STBY,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  digitalWrite(AIN1,LOW);
  digitalWrite(AIN2,LOW);
  digitalWrite(BIN1,LOW);
  digitalWrite(BIN2,LOW);
  digitalWrite(PWMA,LOW);
  digitalWrite(PWMB,LOW);
  digitalWrite(STBY,HIGH);
}

void robotStop() {
  digitalWrite(AIN1,HIGH);
  digitalWrite(AIN2,HIGH);
  digitalWrite(BIN1,HIGH);
  digitalWrite(BIN2,HIGH);
  digitalWrite(PWMA,LOW);
  digitalWrite(PWMB,LOW);
}

void reset(){
  bPWM = 0;
}
