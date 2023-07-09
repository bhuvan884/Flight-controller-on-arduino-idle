#include <Servo.h>
Servo esc_signal0;
Servo esc_signal1;
Servo esc_signal2;
Servo esc_signal3;
#define USE_PWM_RX
#define USE_MPU6050_I2C 
#define GYRO_250DPS 
#define ACCEL_2G 
#include <Wire.h>     //I2c communication
#include <SPI.h>      //SPI communication
#include <PWMServo.h> 
 #include "src/MPU6050/MPU6050.h"
  MPU6050 mpu6050;
#define GYRO_FS_SEL_250    MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500    MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000   MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000   MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2     MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4     MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8     MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16    MPU6050_ACCEL_FS_16
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
unsigned long channel_1_fs = 1000; //thro
unsigned long channel_2_fs = 1500; //ail
unsigned long channel_3_fs = 1500; //elev
unsigned long channel_4_fs = 1500; //rudd
unsigned long channel_5_fs = 2000; //gear, greater than 1500 = throttle cut

//Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  //Madgwick filter parameter
float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14)
float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1)
float B_mag = 1.0,ratee,dega=0;        //Magnetometer LP filter parameter
//IMU calibration parameters 
float AccErrorX = 0.0;
float AccErrorY = 0.0;
float AccErrorZ = 0.0;
float GyroErrorX = 0.0;
float GyroErrorY= 0.0;
float GyroErrorZ = 0.0;

//Controller parameters 
float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode 
float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;     //Max yaw rate in deg/sec

float Kp_roll_angle = 0.45;    //Roll P-gain - angle mode 
float Ki_roll_angle = 0.35;    //Roll I-gain - angle mode
float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode 
float B_loop_roll = 0.9;      //Roll damping  lower is more damping (must be between 0 to 1)
float Kp_pitch_angle = 0.45;   //Pitch P-gain - angle mode
float Ki_pitch_angle = 0.35;//Pitch I-gain - angle mode
float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode 
float B_loop_pitch = 0.9;     //Pitch damping  lower is more damping (must be between 0 to 1)

float Kp_roll_rate = 0.135;    //Roll P-gain - rate mode
float Ki_roll_rate = 0.2;     //Roll I-gain - rate mode
float Kd_roll_rate = 0.0036;  //Roll D-gain - rate mode 
float Kp_pitch_rate = 0.135;   //Pitch P-gain - rate mode
float Ki_pitch_rate = 0.2;    //Pitch I-gain - rate mode
float Kd_pitch_rate = 0.0036; //Pitch D-gain - rate mode 

float Kp_yaw = 0.3;           //Yaw P-gain
float Ki_yaw =0.05;          //Yaw I-gain
float Kd_yaw = 0.015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
const int ch1Pin = 17; //throttle
const int ch2Pin = 15; //ail
const int ch3Pin = 16; //ele
const int ch4Pin = 20; //rudd
const int ch5Pin = 21; //gear (throttle cut)
//bldc motar pwm pins
const int m1Pin = 0;
const int m2Pin = 1;
const int m3Pin = 2;
const int m4Pin = 3;
const int m5Pin = 4;
const int m6Pin = 5;

float dt;
unsigned long current_time, prev_time;
unsigned long print_counter, serial_counter;
unsigned long blink_counter, blink_delay;
bool blinkAlternate;

//Radio communication:
unsigned long channel_1_pwm, channel_2_pwm, channel_3_pwm, channel_4_pwm, channel_5_pwm=0, channel_6_pwm;
unsigned long channel_1_pwm_prev, channel_2_pwm_prev, channel_3_pwm_prev, channel_4_pwm_prev;
//IMU:
float AccX, AccY, AccZ;
float AccX_prev, AccY_prev, AccZ_prev;
float GyroX, GyroY, GyroZ,corr;
float GyroX_prev, GyroY_prev, GyroZ_prev;
float MagX, MagY, MagZ;
float MagX_prev, MagY_prev, MagZ_prev;
float roll_IMU, pitch_IMU, yaw_IMU;
float roll_IMU_prev, pitch_IMU_prev;
float q0 = 1.0f; //Initialize quaternion for madgwick filter
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float thro_des, roll_des, pitch_des, yaw_des;
float roll_passthru, pitch_passthru, yaw_passthru;

//Controller:
float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;

//Mixer
float m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
int m1_command_PWM=0, m2_command_PWM=0, m3_command_PWM=0, m4_command_PWM=0, m5_command_PWM, m6_command_PWM;
float s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;
int s1_command_PWM, s2_command_PWM, s3_command_PWM, s4_command_PWM, s5_command_PWM, s6_command_PWM, s7_command_PWM;



//========================================================================================================================//
//                                                      VOID SETUP                                                        //                           
//========================================================================================================================//

void setup() {
  Serial.begin(500000);
  //Serial1.begin(500000, SERIAL_8N1, 7, 8); //USB serial
  //Initialize all pins
  pinMode(13, OUTPUT); //Pin 13 LED blinker on board, do not modify 
  pinMode(m1Pin, OUTPUT);
  pinMode(m2Pin, OUTPUT);
  pinMode(m3Pin, OUTPUT);
  pinMode(m4Pin, OUTPUT);
  pinMode(m5Pin, OUTPUT);
  pinMode(m6Pin, OUTPUT);
  //Set built in LED to turn on to signal startup
  digitalWrite(13, HIGH);

  delay(5);

  //Initialize radio communication
  radioSetup();
  
  //Set radio channels to default (safe) values before entering main loop
  channel_1_pwm = channel_1_fs;
  channel_2_pwm = channel_2_fs;
  channel_3_pwm = channel_3_fs;
  channel_4_pwm = channel_4_fs;

  //Initialize IMU communication
  IMUinit();

  delay(5);

  //Get IMU error to zero accelerometer and gyro readings, assuming vehicle is level when powered up
  calculate_IMU_error(); 
  
  delay(5);
 esc_signal0.attach(m1Pin);  
  esc_signal0.write(30);   
   esc_signal1.attach(m2Pin);  
  esc_signal1.write(30);
   esc_signal3.attach(m4Pin);  
  esc_signal3.write(30);
   esc_signal2.attach(m3Pin);  
  esc_signal2.write(30);
  calibrateAttitude();   
  
  //Indicate entering main loop with 3 quick blinks
  setupBlink(3,160,70); //numBlinks, upTime (ms), downTime (ms)

}



//========================================================================================================================//
//                                                       MAIN LOOP                                                        //                           
//========================================================================================================================//
                                                  
void loop() {
  //track of what time it is and how much time has elapsed since the last loop
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  loopBlink(); //Indicate we are in main loop with short blink every 1.5 seconds

  //Print data at 100hz 
  //printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
  //printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec 
  //printGyroData();      //Prints filtered gyro data direct from IMU 
  //printAccelData();     //Prints filtered accelerometer data direct from IMU 
  //printMagData();       //Prints filtered magnetometer data direct from IMU 
  //printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter 
  //printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint 
  printMotorCommands(); //Prints the values being written to the motors 
  //printLoopRate();      //Prints the time between loops in microseconds 

  //Get vehicle state
  //dega=dega+ratee*(current_time - prev_time);
  //Serial.print(dega);
  //Serial.print(" ");
  //Serial.println(yaw_IMU);
  getIMUdata(); //Pulls raw gyro, accelerometer, and magnetometer
  Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  //yaw_IMU=yaw_IMU-dega;

  //Compute desired state
  getDesState(); //Convert raw commands to normalized values 

   
  //PID Controller
  //controlANGLE(); //Stabilize on angle setpoint
  controlANGLE2(); //Stabilize on angle setpoint using cascaded pid
  //controlRATE(); //Stabilize on rate setpoint

  //mixing and scaling to PWM values
  controlMixer(); //Mixes PID outputs 
  scaleCommands(); //Scales motor commands 
  //Serial.print(roll_des);
  //Serial.print(",");
  //Serial.println(roll_IMU);
  //Throttle cut check
  throttleCut(); //sets motor commands to low based on state of ch5


  commandMotors(); //Sends command pulses to each motor pin using pwm
  getCommands(); //Pulls current available radio commands
  failSafe(); //Prevent failures in event of bad receiver connection, defaults to failsafe values assigned in setup

  //Regulate loop rate
  loopRate(2000); //Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}



//========================================================================================================================//
//                                                      FUNCTIONS                                                         //                           
//========================================================================================================================//



void controlMixer() {
  //DESCRIPTION: Mixes scaled commands from PID controller 
  m1_command_scaled = thro_des+ constrain(-pitch_PID + roll_PID + yaw_PID,-0.2*thro_des,0.2*thro_des); //Front Left
  m2_command_scaled = thro_des+ constrain(- pitch_PID - roll_PID - yaw_PID,-0.2*thro_des,0.2*thro_des); //Front Right
  m3_command_scaled = thro_des+ constrain(+ pitch_PID - roll_PID + yaw_PID,-0.2*thro_des,0.2*thro_des); //Back Right
  m4_command_scaled = thro_des+ constrain(+ pitch_PID + roll_PID - yaw_PID,-0.2*thro_des,0.2*thro_des); //Back Left
  m5_command_scaled = 0;
  m6_command_scaled = 0; 
}

void IMUinit() {
  // Initialize IMU
    Wire.begin();
    Wire.setClock(1000000); 
    
    mpu6050.initialize();
    
    if (mpu6050.testConnection() == false) {
      Serial.println("MPU6050 initialization unsuccessful");
      Serial.println("Check MPU6050 wiring or try cycling power");
      while(1) {}
    }
    mpu6050.setFullScaleGyroRange(GYRO_SCALE);
    mpu6050.setFullScaleAccelRange(ACCEL_SCALE);
}

void getIMUdata() {
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

 //Accelerometer
  AccX = AcX / ACCEL_SCALE_FACTOR; //G's
  AccY = AcY / ACCEL_SCALE_FACTOR;
  AccZ = AcZ / ACCEL_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  AccX = AccX - AccErrorX;
  AccY = AccY - AccErrorY;
  AccZ = AccZ - AccErrorZ;
  //LP filter accelerometer data
  AccX = (1.0 - B_accel)*AccX_prev + B_accel*AccX;
  AccY = (1.0 - B_accel)*AccY_prev + B_accel*AccY;
  AccZ = (1.0 - B_accel)*AccZ_prev + B_accel*AccZ;
  AccX_prev = AccX;
  AccY_prev = AccY;
  AccZ_prev = AccZ;

  //Gyro
  GyroX = GyX / GYRO_SCALE_FACTOR; //deg/sec
  GyroY = GyY / GYRO_SCALE_FACTOR;
  GyroZ = GyZ / GYRO_SCALE_FACTOR;
  //Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY;
  GyroZ = GyroZ - GyroErrorZ;
  //LP filter gyro data
  GyroX = (1.0 - B_gyro)*GyroX_prev + B_gyro*GyroX;
  GyroY = (1.0 - B_gyro)*GyroY_prev + B_gyro*GyroY;
  GyroZ = (1.0 - B_gyro)*GyroZ_prev + B_gyro*GyroZ;
  GyroX_prev = GyroX;
  GyroY_prev = GyroY;
  GyroZ_prev = GyroZ;
}

void calculate_IMU_error() {
  // Computes IMU accelerometer and gyro error 
  int16_t AcX,AcY,AcZ,GyX,GyY,GyZ,MgX,MgY,MgZ;
  AccErrorX = 0.0;
  AccErrorY = 0.0;
  AccErrorZ = 0.0;
  GyroErrorX = 0.0;
  GyroErrorY= 0.0;
  GyroErrorZ = 0.0;
  
  //Read IMU values 12000 times
  int c = 0;
  while (c < 12000) {
    #if defined USE_MPU6050_I2C
      mpu6050.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
    #elif defined USE_MPU9250_SPI
      mpu9250.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
    #endif
    
    AccX  = AcX / ACCEL_SCALE_FACTOR;
    AccY  = AcY / ACCEL_SCALE_FACTOR;
    AccZ  = AcZ / ACCEL_SCALE_FACTOR;
    GyroX = GyX / GYRO_SCALE_FACTOR;
    GyroY = GyY / GYRO_SCALE_FACTOR;
    GyroZ = GyZ / GYRO_SCALE_FACTOR;
    
    //Sum all readings
    AccErrorX  = AccErrorX + AccX;
    AccErrorY  = AccErrorY + AccY;
    AccErrorZ  = AccErrorZ + AccZ;
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  //Divide the sum by 12000 to get the error value
  AccErrorX  = AccErrorX / c;
  AccErrorY  = AccErrorY / c;
  AccErrorZ  = AccErrorZ / c - 1.0;
  GyroErrorX = GyroErrorX / c;
  GyroErrorY = GyroErrorY / c;
  GyroErrorZ = GyroErrorZ / c;

  Serial.print("float AccErrorX = ");
  Serial.print(AccErrorX);
  Serial.println(";");
  Serial.print("float AccErrorY = ");
  Serial.print(AccErrorY);
  Serial.println(";");
  Serial.print("float AccErrorZ = ");
  Serial.print(AccErrorZ);
  Serial.println(";");
  
  Serial.print("float GyroErrorX = ");
  Serial.print(GyroErrorX);
  Serial.println(";");
  Serial.print("float GyroErrorY = ");
  Serial.print(GyroErrorY);
  Serial.println(";");
  Serial.print("float GyroErrorZ = ");
  Serial.print(GyroErrorZ);
  Serial.println(";");

  Serial.println("Paste these values in user specified variables section and comment out calculate_IMU_error() in void setup.");
}

void calibrateAttitude() {
  //Warm up IMU and madgwick filter in simulated main loop
  //upper define kra hai ratee variable
  double yawpi,yawpf,talle=micros();
  for (int i = 0; i <= 20000; i++) {
    prev_time = current_time;      
    current_time = micros();      
    dt = (current_time - prev_time)/1000000.0; 
    getIMUdata();
    Madgwick(GyroX, -GyroY, -GyroZ, -AccX, AccY, AccZ, dt);
    if(i==0)
    {yawpi=yaw_IMU;}
    loopRate(2000); //do not exceed 2000Hz
  }
  yawpf=yaw_IMU;
  ratee=(yawpf-yawpi)/(micros()-talle);
}



void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq) {
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

  //Convert gyroscope degrees/sec to radians/sec
  gx *= 0.0174533f;
  gy *= 0.0174533f;
  gz *= 0.0174533f;

  //Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _4q0 = 4.0f * q0;
    _4q1 = 4.0f * q1;
    _4q2 = 4.0f * q2;
    _8q1 = 8.0f * q1;
    _8q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    //Gradient decent algorithm corrective step
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    //feedback step
    qDot1 -= B_madgwick * s0;
    qDot2 -= B_madgwick * s1;
    qDot3 -= B_madgwick * s2;
    qDot4 -= B_madgwick * s3;
  }

  //Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * invSampleFreq;
  q1 += qDot2 * invSampleFreq;
  q2 += qDot3 * invSampleFreq;
  q3 += qDot4 * invSampleFreq;

  //Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

  //Compute angles
  pitch_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
  roll_IMU = asin(-2.0f * (q1*q3 - q0*q2))*57.29577951; //degrees
  yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
 // Serial.print(roll_IMU);
 // Serial.print("/");
  //Serial.print(pitch_IMU);
  //Serial.print("/");
  //Serial.print(yaw_IMU);
  //Serial.println("/");
}

void getDesState() {
  
  thro_des = (channel_1_pwm - 1000.0)/1000.0; //Between 0 and 1
  roll_des = (channel_2_pwm - 1500.0)/500.0; //Between -1 and 1
  pitch_des = (channel_3_pwm - 1500.0)/500.0; //Between -1 and 1
  yaw_des = (channel_4_pwm - 1500.0)/500.0; //Between -1 and 1
  roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
  pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
  yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
  
  //Constrain within normalized bounds
  thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
  roll_des = constrain(roll_des, -1.0, 1.0)*maxRoll; //Between -maxRoll and +maxRoll
  pitch_des = constrain(pitch_des, -1.0, 1.0)*maxPitch; //Between -maxPitch and +maxPitch
  yaw_des = constrain(yaw_des, -1.0, 1.0)*maxYaw; //Between -maxYaw and +maxYaw
  //roll_passthru = constrain(roll_passthru, -0.5, 0.5);
  //pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
  //yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
}

void controlANGLE() {
 //adjustments
  corr=GyroX;
  GyroX=-GyroY;
  GyroY=corr;
  GyroZ=GyroZ;
  
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX;
  roll_PID = 0.01*(Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll - Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_PID = .01*(Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch - Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = (.016/3.0)*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  integral_roll_prev = integral_roll;
  //Update pitch variables
  integral_pitch_prev = integral_pitch;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void controlANGLE2() {
  corr=GyroX;
  GyroX=-GyroY;
  GyroY=corr;
  GyroZ=GyroZ;
  //Outer loop - PID on angle
  float roll_des_ol, pitch_des_ol;
  //Roll
  error_roll = roll_des - roll_IMU;
  integral_roll_ol = integral_roll_prev_ol + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_ol = 0;
  }
  integral_roll_ol = constrain(integral_roll_ol, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = GyroX; 
  roll_des_ol = Kp_roll_angle*error_roll + Ki_roll_angle*integral_roll_ol- Kd_roll_angle*derivative_roll;

  //Pitch
  error_pitch = pitch_des - pitch_IMU;
  integral_pitch_ol = integral_pitch_prev_ol + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_ol = 0;
  }
  integral_pitch_ol = constrain(integral_pitch_ol, -i_limit, i_limit); //saturate integrator to prevent unsafe buildup
  derivative_pitch = GyroY;
  pitch_des_ol = Kp_pitch_angle*error_pitch + Ki_pitch_angle*integral_pitch_ol - Kd_pitch_angle*derivative_pitch;

  // artificial damping
  float Kl = 18.0;
  roll_des_ol = Kl*roll_des_ol;
  pitch_des_ol = Kl*pitch_des_ol;
  roll_des_ol = constrain(roll_des_ol, -240.0, 240.0);
  pitch_des_ol = constrain(pitch_des_ol, -240.0, 240.0);
  roll_des_ol = (1.0 - B_loop_roll)*roll_des_prev + B_loop_roll*roll_des_ol;
  pitch_des_ol = (1.0 - B_loop_pitch)*pitch_des_prev + B_loop_pitch*pitch_des_ol;

  //Inner loop - PID on rate
  //Roll
  error_roll = roll_des_ol - GyroX;
  integral_roll_il = integral_roll_prev_il + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll_il = 0;
  }
  integral_roll_il = constrain(integral_roll_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = ((.01/32.0)*26.0)*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll_il + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des_ol - GyroY;
  integral_pitch_il = integral_pitch_prev_il + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch_il = 0;
  }
  integral_pitch_il = constrain(integral_pitch_il, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = ((.01/32.0)*26.0)*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch_il + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range
  
  //Yaw
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = ((.01/47.0)*30.0)*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range
  
  //Update roll variables
  integral_roll_prev_ol = integral_roll_ol;
  integral_roll_prev_il = integral_roll_il;
  error_roll_prev = error_roll;
  roll_IMU_prev = roll_IMU;
  roll_des_prev = roll_des_ol;
  //Update pitch variables
  integral_pitch_prev_ol = integral_pitch_ol;
  integral_pitch_prev_il = integral_pitch_il;
  error_pitch_prev = error_pitch;
  pitch_IMU_prev = pitch_IMU;
  pitch_des_prev = pitch_des_ol;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
  //Serial.print(pitch_des_ol);
  //Serial.println(",");
  //Serial.print(roll_des_ol);

}

void controlRATE() {
  //Roll
  error_roll = roll_des - GyroX;
  integral_roll = integral_roll_prev + error_roll*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_roll = 0;
  }
  integral_roll = constrain(integral_roll, -i_limit, i_limit); //Saturate integrator 
  derivative_roll = (error_roll - error_roll_prev)/dt; 
  roll_PID = .01*(Kp_roll_rate*error_roll + Ki_roll_rate*integral_roll + Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

  //Pitch
  error_pitch = pitch_des - GyroY;
  integral_pitch = integral_pitch_prev + error_pitch*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_pitch = 0;
  }
  integral_pitch = constrain(integral_pitch, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_pitch = (error_pitch - error_pitch_prev)/dt; 
  pitch_PID = .01*(Kp_pitch_rate*error_pitch + Ki_pitch_rate*integral_pitch + Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

  //Yaw, stablize on rate from GyroZ
  error_yaw = yaw_des - GyroZ;
  integral_yaw = integral_yaw_prev + error_yaw*dt;
  if (channel_1_pwm < 1060) {   //Don't let integrator build if throttle is too low
    integral_yaw = 0;
  }
  integral_yaw = constrain(integral_yaw, -i_limit, i_limit); //Saturate integrator to prevent unsafe buildup
  derivative_yaw = (error_yaw - error_yaw_prev)/dt; 
  yaw_PID = .01*(Kp_yaw*error_yaw + Ki_yaw*integral_yaw + Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

  //Update roll variables
  error_roll_prev = error_roll;
  integral_roll_prev = integral_roll;
  GyroX_prev = GyroX;
  //Update pitch variables
  error_pitch_prev = error_pitch;
  integral_pitch_prev = integral_pitch;
  GyroY_prev = GyroY;
  //Update yaw variables
  error_yaw_prev = error_yaw;
  integral_yaw_prev = integral_yaw;
}

void scaleCommands() {
  m1_command_PWM = m1_command_scaled*72 + 58;
  m2_command_PWM = m2_command_scaled*72 + 58;
  m3_command_PWM = m3_command_scaled*72 + 58;
  m4_command_PWM = m4_command_scaled*72 + 58;
  m1_command_PWM = constrain(m1_command_PWM, 58, 130);
  m2_command_PWM = constrain(m2_command_PWM, 58, 130);
  m3_command_PWM = constrain(m3_command_PWM, 58, 130);
  m4_command_PWM = constrain(m4_command_PWM, 58, 130);
  s1_command_PWM = s1_command_scaled*180;
  s2_command_PWM = s2_command_scaled*180;
  s3_command_PWM = s3_command_scaled*180;
  s4_command_PWM = s4_command_scaled*180;
  s5_command_PWM = s5_command_scaled*180;
  s6_command_PWM = s6_command_scaled*180;
  s7_command_PWM = s7_command_scaled*180;
  //Constrain commands to servos within servo library bounds
  s1_command_PWM = constrain(s1_command_PWM, 0, 180);
  s2_command_PWM = constrain(s2_command_PWM, 0, 180);
  s3_command_PWM = constrain(s3_command_PWM, 0, 180);
  s4_command_PWM = constrain(s4_command_PWM, 0, 180);
  s5_command_PWM = constrain(s5_command_PWM, 0, 180);
  s6_command_PWM = constrain(s6_command_PWM, 0, 180);
  s7_command_PWM = constrain(s7_command_PWM, 0, 180);

}

void getCommands() {
  //radioSetup(); uncomment kardo agar radiovalues connstant aa rhi ho toh

    channel_1_pwm = getRadioPWM(1);
    channel_2_pwm = getRadioPWM(2);
    channel_3_pwm = getRadioPWM(3);
    channel_4_pwm = getRadioPWM(4);
    channel_5_pwm = getRadioPWM(5);
    channel_6_pwm = getRadioPWM(6);
    
 
  float b = 0.7; //Lower=slower, higher=noiser
  channel_1_pwm = (1.0 - b)*channel_1_pwm_prev + b*channel_1_pwm;
  channel_2_pwm = (1.0 - b)*channel_2_pwm_prev + b*channel_2_pwm;
  channel_3_pwm = (1.0 - b)*channel_3_pwm_prev + b*channel_3_pwm;
  channel_4_pwm = (1.0 - b)*channel_4_pwm_prev + b*channel_4_pwm;
  channel_1_pwm_prev = channel_1_pwm;
  channel_2_pwm_prev = channel_2_pwm;
  channel_3_pwm_prev = channel_3_pwm;
  channel_4_pwm_prev = channel_4_pwm;
}

void failSafe() {
  unsigned minVal = 800;
  unsigned maxVal = 2200;
  int check1 = 0;
  int check2 = 0;
  int check3 = 0;
  int check4 = 0;
  int check5 = 0;
  int check6 = 0;

  //Triggers for failure criteria
  if (channel_1_pwm > maxVal || channel_1_pwm < minVal) check1 = 1;
  if (channel_2_pwm > maxVal || channel_2_pwm < minVal) check2 = 1;
  if (channel_3_pwm > maxVal || channel_3_pwm < minVal) check3 = 1;
  if (channel_4_pwm > maxVal || channel_4_pwm < minVal) check4 = 1;
  if (channel_5_pwm > maxVal || channel_5_pwm < minVal) check5 = 1;

  //If any failures, set to default failsafe values
  if ((check1 + check2 + check3 + check4 + check5) > 0) {
    channel_1_pwm = channel_1_fs;
    channel_2_pwm = channel_2_fs;
    channel_3_pwm = channel_3_fs;
    channel_4_pwm = channel_4_fs;
    
  }
}

void commandMotors() {
esc_signal0.write(m1_command_PWM);
esc_signal1.write(m2_command_PWM);
esc_signal2.write(m3_command_PWM);
esc_signal3.write(m4_command_PWM);
}







void throttleCut() {
 
  if (channel_5_pwm > 1800||channel_5_pwm==0) {
    m1_command_PWM = 0;
    m2_command_PWM = 0;
    m3_command_PWM = 0;
    m4_command_PWM = 0;
    
  }
}


void loopRate(int freq) {
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void loopBlink() {

  if (current_time - blink_counter > blink_delay) {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate); //Pin 13 is built in LED
    
    if (blinkAlternate == 1) {
      blinkAlternate = 0;
      blink_delay = 100000;
      }
    else if (blinkAlternate == 0) {
      blinkAlternate = 1;
      blink_delay = 2000000;
      }
  }
}

void setupBlink(int numBlinks,int upTime, int downTime) {
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j<= numBlinks; j++) {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

void printRadioData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F(" CH1: "));
    Serial.print(channel_1_pwm);
    Serial.print(F(" CH2: "));
    Serial.print(channel_2_pwm);
    Serial.print(F(" CH3: "));
    Serial.print(channel_3_pwm);
    Serial.print(F(" CH4: "));
    Serial.print(channel_4_pwm);
    Serial.print(F(" CH5: "));
    Serial.print(channel_5_pwm);
    Serial.print(F(" CH6: "));
    Serial.println(channel_6_pwm);
  }
}

void printDesiredState() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("thro_des: "));
    Serial.print(thro_des);
    Serial.print(F(" roll_des: "));
    Serial.print(roll_des);
    Serial.print(F(" pitch_des: "));
    Serial.print(pitch_des);
    Serial.print(F(" yaw_des: "));
    Serial.println(yaw_des);
  }
}

void printGyroData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("GyroX: "));
    Serial.print(GyroX);
    Serial.print(F(" GyroY: "));
    Serial.print(GyroY);
    Serial.print(F(" GyroZ: "));
    Serial.println(GyroZ);
  }
}

void printAccelData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("AccX: "));
    Serial.print(AccX);
    Serial.print(F(" AccY: "));
    Serial.print(AccY);
    Serial.print(F(" AccZ: "));
    Serial.println(AccZ);
  }
}

void printMagData() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("MagX: "));
    Serial.print(MagX);
    Serial.print(F(" MagY: "));
    Serial.print(MagY);
    Serial.print(F(" MagZ: "));
    Serial.println(MagZ);
  }
}

void printRollPitchYaw() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll: "));
    Serial.print(roll_IMU);
    Serial.print(F(" pitch: "));
    Serial.print(pitch_IMU);
    Serial.print(F(" yaw: "));
    Serial.println(yaw_IMU);
  }
}

void printPIDoutput() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("roll_PID: "));
    Serial.print(roll_PID);
    Serial.print(F(" pitch_PID: "));
    Serial.print(pitch_PID);
    Serial.print(F(" yaw_PID: "));
    Serial.println(yaw_PID);
  }
}

void printMotorCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("m1_command: "));
    Serial.print(m1_command_PWM);
    Serial.print(F(" m2_command: "));
    Serial.print(m2_command_PWM);
    Serial.print(F(" m3_command: "));
    Serial.print(m3_command_PWM);
    Serial.print(F(" m4_command: "));
    Serial.println(m4_command_PWM);
  }
}

void printServoCommands() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("s1_command: "));
    Serial.print(s1_command_PWM);
    Serial.print(F(" s2_command: "));
    Serial.print(s2_command_PWM);
    Serial.print(F(" s3_command: "));
    Serial.print(s3_command_PWM);
    Serial.print(F(" s4_command: "));
    Serial.print(s4_command_PWM);
    Serial.print(F(" s5_command: "));
    Serial.print(s5_command_PWM);
    Serial.print(F(" s6_command: "));
    Serial.print(s6_command_PWM);
    Serial.print(F(" s7_command: "));
    Serial.println(s7_command_PWM);
  }
}

void printLoopRate() {
  if (current_time - print_counter > 10000) {
    print_counter = micros();
    Serial.print(F("dt = "));
    Serial.println(dt*1000000.0);
  }
}
float invSqrt(float x) {
 
  return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
}
