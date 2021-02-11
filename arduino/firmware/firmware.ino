
#include <Arduino.h>
#include "ros.h"
#include "ros/time.h"
//header file for cmd_subscribing to "cmd_vel"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/Imu.h"

//header file for pid server

#include "math.h"

//#define ENCODER_OPTIMIZE_INTERRUPTS // comment this out on Non-Teensy boards
#define ENCODER_USE_INTERRUPTS
#include "lib/encoder/Encoder.h"

#include "lib/imu/imu.h"

Encoder_internal_state_t * Encoder::interruptArgs[];

#define IMU_PUBLISH_RATE 0.5 //hz
#define COMMAND_RATE 20 //hz
#define DEBUG_RATE 0.1
#define DEBUG 1
#define PWM_MIN 50
#define PWMRANGE 255

Encoder motor1_encoder(2, 50);
Encoder motor2_encoder(19, 51);
Encoder motor3_encoder(3, 52);
Encoder motor4_encoder(18, 53);

//Controller motor1_controller(Controller::MOTOR_DRIVER, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
//Controller motor2_controller(Controller::MOTOR_DRIVER, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
//Controller motor3_controller(Controller::MOTOR_DRIVER, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
//Controller motor4_controller(Controller::MOTOR_DRIVER, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

//PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
//PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

// Pins
const uint8_t R_FORW_PWM = 11;
const uint8_t R_BACK_PWM = 10;
const uint8_t L_BACK_PWM = 13;
const uint8_t L_FORW_PWM = 12;


//Kinematics kinematics(Kinematics::LINO_BASE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);

float g_req_linear_vel_x = 0;
float g_req_linear_vel_y = 0;
float g_req_angular_vel_z = 0;

unsigned long g_prev_command_time = 0;

//callback function prototypes
void commandCallback(const geometry_msgs::Twist& cmd_msg);
//void PIDCallback(const lino_msgs::PID& pid);
float mapPwm(float x, float out_min, float out_max);
void calibrateIMUCbk(const std_msgs::Empty& calibrate_msg);

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
//ros::Subscriber<lino_msgs::PID> pid_sub("pid", PIDCallback);
ros::Subscriber<std_msgs::Empty> imu_calib_sub("calibration_imu", &calibrateIMUCbk );

sensor_msgs::Imu raw_imu_msg;
ros::Publisher raw_imu_pub("raw_imu", &raw_imu_msg);

//lino_msgs::Velocities raw_vel_msg;
//ros::Publisher raw_vel_pub("raw_vel", &raw_vel_msg);

void setup()
{
  //    steering_servo.attach(STEERING_PIN);
  //    steering_servo.write(90);

  nh.initNode();
  nh.getHardware()->setBaud(115200);
  // nh.subscribe(pid_sub);
  nh.subscribe(cmd_sub);
  nh.subscribe(imu_calib_sub);

  //nh.advertise(raw_vel_pub);
   nh.advertise(raw_imu_pub);

  if (initIMU())
    nh.loginfo("IMU Initialized");
  else
    nh.logfatal("IMU failed to initialize. Check your IMU connection.");

  pinMode(R_FORW_PWM, OUTPUT);
  pinMode(R_BACK_PWM, OUTPUT);
  pinMode(L_BACK_PWM, OUTPUT);
  pinMode(L_FORW_PWM, OUTPUT);

  while (!nh.connected())
  {
    nh.spinOnce();
  }
  nh.loginfo("HACHIBOT CONNECTED");
  delay(1);
}

void loop()
{
  static unsigned long prev_control_time = 0;
  static unsigned long prev_imu_time = 0;
  static unsigned long prev_debug_time = 0;

  //this block publishes the IMU data based on defined rate
  if ((millis() - prev_imu_time) >= (1000 / IMU_PUBLISH_RATE))
  {
    publishIMU();

    prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if (DEBUG)
  {
    if ((millis() - prev_debug_time) >= (1000 / DEBUG_RATE))
    {
      printDebug();
      prev_debug_time = millis();
    }
  }
  //call all the callbacks waiting to be called
  nh.spinOnce();
}

//void PIDCallback(const lino_msgs::PID& pid)
//{
//    //callback function every time PID constants are received from lino_pid for tuning
//    //this callback receives pid object where P,I, and D constants are stored
//    motor1_pid.updateConstants(pid.p, pid.i, pid.d);
//    motor2_pid.updateConstants(pid.p, pid.i, pid.d);
//    motor3_pid.updateConstants(pid.p, pid.i, pid.d);
//    motor4_pid.updateConstants(pid.p, pid.i, pid.d);
//}

void commandCallback(const geometry_msgs::Twist& cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  g_req_linear_vel_x = cmd_msg.linear.x;
  g_req_linear_vel_y = cmd_msg.linear.y;
  g_req_angular_vel_z = cmd_msg.angular.z;


  // Cap values at [-1 .. 1]
  float x = max(min(cmd_msg.linear.x, 1.0f), -1.0f);
  float z = max(min(cmd_msg.angular.z, 1.0f), -1.0f);

  // Calculate the intensity of left and right wheels. Simple version.
  // Taken from https://hackernoon.com/unicycle-to-differential-drive-courseras-control-of-mobile-robots-with-ros-and-rosbots-part-2-6d27d15f2010#1e59
  float l = (cmd_msg.linear.x - cmd_msg.angular.z) / 2;
  float r = (cmd_msg.linear.x + cmd_msg.angular.z) / 2;

  // Then map those values to PWM intensities. PWMRANGE = full speed, while PWM_MIN = the minimal amount of power at which the motors begin moving.
  uint16_t lPwm = mapPwm(fabs(l), PWM_MIN, PWMRANGE);
  uint16_t rPwm = mapPwm(fabs(r), PWM_MIN, PWMRANGE);

  // Set direction pins and PWM
  analogWrite(L_FORW_PWM, l > 0 ? lPwm : 0);
  analogWrite(L_BACK_PWM, l < 0 ? lPwm : 0);

  analogWrite(R_FORW_PWM, r > 0 ? rPwm : 0);
  analogWrite(R_BACK_PWM, r < 0 ? rPwm : 0);


  g_prev_command_time = millis();
}

void moveBase()
{
  //get the required rpm for each motor based on required velocities, and base used
  //    Kinematics::rpm req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);
  //
  //    //get the current speed of each motor
  //    int current_rpm1 = motor1_encoder.getRPM();
  //    int current_rpm2 = motor2_encoder.getRPM();
  //    int current_rpm3 = motor3_encoder.getRPM();
  //    int current_rpm4 = motor4_encoder.getRPM();
  //
  //    //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //    //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  //    motor1_controller.spin(motor1_pid.compute(req_rpm.motor1, current_rpm1));
  //    motor2_controller.spin(motor2_pid.compute(req_rpm.motor2, current_rpm2));
  //    motor3_controller.spin(motor3_pid.compute(req_rpm.motor3, current_rpm3));
  //    motor4_controller.spin(motor4_pid.compute(req_rpm.motor4, current_rpm4));
  //
  //    Kinematics::velocities current_vel;
  //
  //    if(kinematics.base_platform == Kinematics::ACKERMANN || kinematics.base_platform == Kinematics::ACKERMANN1)
  //    {
  //        float current_steering_angle;
  //
  //        current_steering_angle = steer(g_req_angular_vel_z);
  //        current_vel = kinematics.getVelocities(current_steering_angle, current_rpm1, current_rpm2);
  //    }
  //    else
  //    {
  //        current_vel = kinematics.getVelocities(current_rpm1, current_rpm2, current_rpm3, current_rpm4);
  //    }
  //
  //    //pass velocities to publisher object
  //    raw_vel_msg.linear_x = current_vel.linear_x;
  //    raw_vel_msg.linear_y = current_vel.linear_y;
  //    raw_vel_msg.angular_z = current_vel.angular_z;
  //
  //    //publish raw_vel_msg
  //    raw_vel_pub.publish(&raw_vel_msg);
}

void stopBase()
{
  g_req_linear_vel_x = 0;
  g_req_linear_vel_y = 0;
  g_req_angular_vel_z = 0;
}

void publishIMU()
{
  raw_imu_msg.header.stamp = nh.now();
  raw_imu_msg.header.frame_id = "imu_link";

  if (updateIMU())
  {

    //raw_imu_msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    //raw_imu_msg.angular_velocity_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
    //raw_imu_msg.linear_acceleration_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};

    //pass accelerometer data to imu object
    raw_imu_msg.linear_acceleration = readLinearAcceleration();

    //pass gyroscope data to imu object
    raw_imu_msg.angular_velocity = readAngularVelocity();

    //pass orientation data to imu object
    raw_imu_msg.orientation = readOrientation();

    nh.loginfo("Publish IMU data");

    //publish raw_imu_msg
    raw_imu_pub.publish(&raw_imu_msg);
  }

}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Map x value from [0 .. 1] to [out_min .. out_max]
float mapPwm(float x, float out_min, float out_max)
{
  return x * (out_max - out_min) + out_min;
}

void calibrateIMUCbk( const std_msgs::Empty& calibrate_msg)
{
 // calibrate anytime you want to
    nh.loginfo("Accel Gyro calibration will start in 5sec.");
    nh.loginfo("Please leave the device still on the flat plane.");
    
    delay(5000);

    nh.spinOnce();

    calibrateAccelGyro();

    nh.loginfo("Mag calibration will start in 5sec.");
    nh.loginfo("Please Wave device in a figure eight until done.");

    delay(5000);

    nh.spinOnce();

    calibrateMag();

    nh.loginfo("Done calibration!");
}


void printDebug()
{
  char buffer[50];

  sprintf (buffer, "Encoder FrontLeft  : %ld", motor1_encoder.read());
  nh.loginfo(buffer);
  sprintf (buffer, "Encoder FrontRight : %ld", motor2_encoder.read());
  nh.loginfo(buffer);
  sprintf (buffer, "Encoder RearLeft   : %ld", motor3_encoder.read());
  nh.loginfo(buffer);
  sprintf (buffer, "Encoder RearRight  : %ld", motor4_encoder.read());
  nh.loginfo(buffer);
}
