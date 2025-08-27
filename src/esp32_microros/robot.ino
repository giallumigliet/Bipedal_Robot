#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <math.h>

//micro-ros and ros2
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <bipedal_robot/msg/imudata.h>
#include <bipedal_robot/msg/jointsangles.h>
//**********************************************************************************************************

#define DEBUG 


#define BAUDRATE 115200
#define IMU_TIMER 10
#define EXECUTOR_TIMER 10 // 10ms


#define SDA_2 33
#define SCL_2 32


#define SERVOMIN_sg90 130 // pwm
#define SERVOMAX_sg90 530 // pwm

#define SERVOMIN_mg996r 120 // pwm
#define SERVOMAX_mg996r 600 // pwm

#define SERVO_NUMBER 2 

#define SERVO_KNEE_R 1
#define SERVO_KNEE_L 2
#define SERVO_HIP_R 3
#define SERVO_HIP_L 4
#define SERVO_LATERALHIP_R 5
#define SERVO_LATERALHIP_L 6
#define SERVO_NECK1 7
#define SERVO_NECK2 8
#define SERVO_TAIL 9
//**********************************************************************************************************

struct Angles {
  float roll;
  float pitch;
};


//
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver(0x40, Wire1);


rclc_support_t support;
rclc_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

rcl_publisher_t IMU_publisher;
rcl_timer_t timer;
bipedal_robot__msg__IMUData imu_msg; 

rcl_subscriber_t servo_subscriber;
bipedal_robot__msg__JointsAngles joints_msg; 


//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************
void timer_callback(rcl_timer_t* timer, int64_t last_call_time){
  if(timer != NULL) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp); //a [m/s^2]; g [rad/s]

    imu_msg.a_x = a.acceleration.x;
    imu_msg.a_y = a.acceleration.y;
    imu_msg.a_z = a.acceleration.z;
    imu_msg.v_roll = g.gyro.x;
    imu_msg.v_pitch = g.gyro.y;
    imu_msg.v_yaw = g.gyro.z;
    //compute roll and pitch
    Angles RollPitch;
    RollPitch = getRollPitch(a.acceleration.x, a.acceleration.y, a.acceleration.z);
  
    imu_msg.roll = RollPitch.roll;
    imu_msg.pitch = RollPitch.pitch;
    
    #ifdef DEBUG
      Serial.print("Acceleration X: ");
      Serial.print(imu_msg.a_x);
      Serial.print(", Acceleration Y: ");
      Serial.print(imu_msg.a_y);
      Serial.print(", Acceleration Z: ");
      Serial.println(imu_msg.a_z);
        
      Serial.print("Roll angular velocity: ");
      Serial.print(imu_msg.v_roll);
      Serial.print(", Pitch angular velocity: ");
      Serial.print(imu_msg.v_pitch);
      Serial.print(", Yaw angular velocity: ");
      Serial.println(imu_msg.v_yaw);
      
      Serial.print("Roll: ");
      Serial.print(imu_msg.roll);
      Serial.print(", Pitch: ");
      Serial.println(imu_msg.pitch);
    
      Serial.println("");
      Serial.println("");
    #endif
    
    rcl_publish(&IMU_publisher, &imu_msg, NULL);
  }
}

void subscription_callback(const void *msgin){
  const bipedal_robot__msg__JointsAngles  *joints_msg = (const bipedal_robot__msg__JointsAngles *)msgin;

  size_t n_joints = joints_msg->name.size; // numero di giunti
  for (size_t i = 0; i < n_joints; i++) {
      const char * joint_name = joints_msg->name.data[i].data;
      double position = joints_msg->position.data[i];

      // esempio: muovo il servo corrispondente
      servoDriver.setPWM(i, 0, angleToPulse(position));
  }
}


Angles getRollPitch(float Ax, float Ay, float Az){ //gets ax,ay,az and returns roll and pitch in degrees;
  Angles ang;
  ang.roll  = atan2(Ay, Az) * 180.0 / PI;
  ang.pitch = atan2(-Ax, sqrt(Ay*Ay + Az*Az)) * 180.0 / PI;
  
  return ang;
}


int angleToPulse(double angle){  //gets target angle in degrees and returns the pulse width
   double pulse = (angle / 180.0) * (SERVOMAX_mg996r - SERVOMIN_mg996r) + SERVOMIN_mg996r; // map angle of 0 to 180 to Servo min and Servo max 
   //double pulse = (angle / 180.0) * (SERVOMAX_sg90 - SERVOMIN_sg90) + SERVOMIN_sg90;
   #ifdef DEBUG
     Serial.print("Angle: ");Serial.println(angle);
     Serial.print(" pulse: ");Serial.println(pulse);
   #endif
   
   return (int)pulse;
}

//**********************************************************************************************************
//**********************************************************************************************************
//**********************************************************************************************************

// put your setup code here, to run once:
void setup() {
  Serial.begin(BAUDRATE);
  
  set_microros_serial_transports(Serial); // could be  set_microros_transports();
  delay(2000);

  Wire.begin();
  Wire1.begin(SDA_2, SCL_2);
  
  // Try to initialize MPU6050!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);  // resta bloccato qui ma senza reset continui
    }
  } else Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // set accelerometer range to +-8G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // set gyro range to +- 500 deg/s
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // set filter bandwidth to 21 Hz
  Serial.println("MPU6050 ready");

  // Try to initialize PCA9685 driver board!
  if (!servoDriver.begin()) {
    Serial.println("Failed to find PCA9685 driver board");
    while (1) {
      delay(10);  // resta bloccato qui ma senza reset continui
    }
  } else Serial.println("PCA9685 Found!");
  
  servoDriver.setPWMFreq(60); 
  Serial.println("PCA9685 frequency OK");



  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator); 
  rclc_node_init_default(&node, "robot_esp32_node", "", &support);
  rclc_subscription_init_default(&servo_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(bipedal_robot, msg, JointsAngles), "joints_angles");
  rclc_publisher_init_default(&IMU_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(bipedal_robot,msg,IMUData), "imu_data");
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(IMU_TIMER), timer_callback);
  
  rclc_executor_init(&executor, &support.context, 2, &allocator); // 1 executor, with 2 handles (timer + subscriber)
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &servo_subscriber, &joints_msg, &subscription_callback, ON_NEW_DATA);

  imu_msg.a_x = 0;
  imu_msg.a_y = 0;
  imu_msg.a_z = 0;
  imu_msg.v_roll = 0;
  imu_msg.v_pitch = 0;
  imu_msg.v_yaw = 0;
  imu_msg.roll = 0;
  imu_msg.pitch = 0;
}


//**********************************************************************************************************
// put your main code here, to run repeatedly:
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(EXECUTOR_TIMER));
}
