#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/accel_stamped.h>
#include "MPU6050.h"
#include <Wire.h>
#include <math.h>
#include <string>

MPU6050 imu_;
int16_t ax_, ay_, az_;
int16_t gx_, gy_, gz_;
double pitch_ = 0;
double roll_ = 0;
double yaw_ = 0;
double prev_time_ = 0;
HardwareSerial SerialDebug(2);
const double g_ = 9.80665;
const unsigned int callback_duration_ms_ = 14; // given lage size msg, callback cannot be finished within that time
const char* ros_node_name_ = "mpu6050";
const char* ros_namespace_ = "gyro";
rcl_publisher_t publisher_;
geometry_msgs__msg__AccelStamped msg_;
rclc_executor_t executor_;
rclc_support_t support_;
rcl_allocator_t allocator_;
rcl_node_t node_;
rcl_timer_t timer_;
rcl_time_point_value_t now_ns_;
rcl_clock_t ros_clock_;
rcl_ret_t rc_;

#define LED_PIN 13
#define I2C_SDA 21
#define I2C_SCL 22
#define UART_RX 16
#define UART_TX 17
#define SERIAL_RATE 921600
#define I2C_FREQ 400000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer_, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer_ != NULL) {
    imu_.getMotion6(&ax_, &ay_, &az_, &gx_, &gy_, &gz_);
  
    int64_t time_ns = rmw_uros_epoch_nanos();
    double cur_time = (double)time_ns / 1000000000.0;
    msg_.header.stamp.sec = (int32_t)(cur_time);
    msg_.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
    
    msg_.accel.linear.x = rawacc2radsacc(ax_); 
    msg_.accel.linear.y = rawacc2radsacc(ay_);
    msg_.accel.linear.z = rawacc2radsacc(az_);

    msg_.accel.angular.x = rawgyr2radsgyr(gx_);
    msg_.accel.angular.y = rawgyr2radsgyr(gy_);
    msg_.accel.angular.z = rawgyr2radsgyr(gz_);

    RCSOFTCHECK(rcl_publish(&publisher_, &msg_, NULL));

    debug_values(ax_, ay_, az_, gx_, gy_, gz_, cur_time - prev_time_);
    prev_time_ = cur_time;
  }
}

void setup() {
  SerialDebug.begin(SERIAL_RATE, SERIAL_8N1, UART_RX, UART_TX);
  SerialDebug.println("START");
  
  set_microros_transports();
  rc_ = rcl_clock_init(RCL_ROS_TIME, &ros_clock_, &allocator_);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  imu_.initialize();
  msg_ = init_accel_msg(ros_node_name_);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator_ = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support_, 0, NULL, &allocator_));

  // create node_
  RCCHECK(rclc_node_init_default(&node_, ros_node_name_, ros_namespace_, &support_));

  // create publisher_
  RCCHECK(rclc_publisher_init_default(
    &publisher_,
    &node_,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, AccelStamped),
    (std::string(ros_node_name_) + "/data").c_str()));

  // create timer_,
  RCCHECK(rclc_timer_init_default(
    &timer_,
    &support_,
    RCL_MS_TO_NS(callback_duration_ms_),
    timer_callback));

  // create executor_
  RCCHECK(rclc_executor_init(&executor_, &support_.context, 1, &allocator_));
  RCCHECK(rclc_executor_add_timer(&executor_, &timer_));

  rmw_uros_sync_session(1000);

  int64_t time_ns = rmw_uros_epoch_nanos();
  prev_time_ = (double)time_ns / 1000000000.0;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1)));
}



/*
 * convert a to b
 */
double rawacc2radsacc(const int16_t raw_acc_from_mpu6050){return (double)raw_acc_from_mpu6050 / 16384.0 * g_;} // 1G = 16384LSB
double rawgyr2radsgyr(const int16_t raw_gyr_from_mpu6050){return deg2rad((double)raw_gyr_from_mpu6050 / 131.0); }// 1deg/s = 131LSB
double deg2rad(const double deg){return deg * PI / 180.0;}
geometry_msgs__msg__AccelStamped init_accel_msg(const char* frame_id)
{
  geometry_msgs__msg__AccelStamped msg;
  msg.header.frame_id.data = (char*)frame_id;
  msg.header.frame_id.size = strlen(frame_id);
  msg.header.frame_id.capacity = msg.header.frame_id.size + 1;

  return msg;
}

/*
 * debug output
 */
void debug_values(
  const int16_t ax, const int16_t ay, const int16_t az, 
  const int16_t gx, const int16_t gy, const int16_t gz, 
  const double dt
){
//  SerialDebug.print("raw ");
//  SerialDebug.print(ax); SerialDebug.print(", ");
//  SerialDebug.print(ay); SerialDebug.print(", ");
//  SerialDebug.print(az); SerialDebug.print(" : ");
//  SerialDebug.print(gx); SerialDebug.print(", ");
//  SerialDebug.print(gy); SerialDebug.print(", ");
//  SerialDebug.print(gz); SerialDebug.print("\t");
  SerialDebug.print("proc ");
  SerialDebug.print(rawacc2radsacc(ax)); SerialDebug.print(", ");
  SerialDebug.print(rawacc2radsacc(ay)); SerialDebug.print(", ");
  SerialDebug.print(rawacc2radsacc(az)); SerialDebug.print(" : ");
  SerialDebug.print(rawgyr2radsgyr(gx)); SerialDebug.print(", ");
  SerialDebug.print(rawgyr2radsgyr(gy)); SerialDebug.print(", ");
  SerialDebug.print(rawgyr2radsgyr(gz)); SerialDebug.print(", ");
  SerialDebug.print("\tdt "); SerialDebug.print(dt); 
  SerialDebug.println();
}
