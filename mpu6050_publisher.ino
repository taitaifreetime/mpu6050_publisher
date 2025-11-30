#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/quaternion.h>
#include "MPU6050.h"
#include <Wire.h>
#include <math.h>

MPU6050 imu_;
int16_t ax_, ay_, az_;
int16_t gx_, gy_, gz_;
double pitch_ = 0;
double roll_ = 0;
double yaw_ = 0;
double prev_time_ = 0;
const double g_ = 9.80665;
const unsigned int callback_duration_ms_ = 33; // up to 30hz ????
const char* ros_node_name_ = "mpu6050";
const char* ros_namespace_ = "imu";
rcl_publisher_t publisher_;
sensor_msgs__msg__Imu imu_msg_;
rclc_executor_t executor_;
rclc_support_t support_;
rcl_allocator_t allocator_;
rcl_node_t node_;
rcl_timer_t timer_;
rcl_time_point_value_t now_ns_;
rcl_clock_t ros_clock_;
rcl_ret_t rc_;

#define LED_PIN 13

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
    double dt = cur_time - prev_time_;
    imu_msg_.header.stamp.sec = (int32_t)(cur_time);
    imu_msg_.header.stamp.nanosec = (uint32_t)(time_ns % 1000000000);
    
    imu_msg_.linear_acceleration.x = rawacc2radsacc(ax_); 
    imu_msg_.linear_acceleration.y = rawacc2radsacc(ay_);
    imu_msg_.linear_acceleration.z = rawacc2radsacc(az_);

    imu_msg_.angular_velocity.x = rawgyr2radsgyr(gx_);
    imu_msg_.angular_velocity.y = rawgyr2radsgyr(gy_);
    imu_msg_.angular_velocity.z = rawgyr2radsgyr(gz_);

    pitch_ += imu_msg_.angular_velocity.x * dt;
    roll_ += imu_msg_.angular_velocity.y * dt;
    yaw_ += imu_msg_.angular_velocity.z * dt;
    imu_msg_.orientation = euler2quaternion(pitch_, roll_, yaw_);

    RCSOFTCHECK(rcl_publish(&publisher_, &imu_msg_, NULL));

    prev_time_ = cur_time;
  }
}

void setup() {
  set_microros_transports();
  rc_ = rcl_clock_init(RCL_ROS_TIME, &ros_clock_, &allocator_);
  
  Wire.begin(21, 22);
  imu_.initialize();
  imu_msg_ = init_imu_msg(ros_node_name_);
  
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu/data"));

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

double rawacc2radsacc(const int16_t raw_acc_from_mpu6050)
{
  return (double)raw_acc_from_mpu6050 / 16384.0 * g_; // 1G = 16384LSB
}

double rawgyr2radsgyr(const int16_t raw_gyr_from_mpu6050)
{
  return deg2rad((double)raw_gyr_from_mpu6050 / 131.0); // 1deg/s = 131LSB
}

double deg2rad(const double deg)
{
  return deg * PI / 180.0;
}

sensor_msgs__msg__Imu init_imu_msg(const char* frame_id)
{
  sensor_msgs__msg__Imu msg;
  msg.header.frame_id.data = (char*)frame_id;
  msg.header.frame_id.size = strlen(frame_id);
  msg.header.frame_id.capacity = imu_msg_.header.frame_id.size + 1;
  for (size_t i = 0; i < 9; i++)
  {
    msg.orientation_covariance[i] = -1;
    msg.angular_velocity_covariance[i] = -1;
    msg.linear_acceleration_covariance[i] = -1;
  }

  return msg;
}

geometry_msgs__msg__Quaternion euler2quaternion(const double pitch, const double roll, const double yaw)
{
  geometry_msgs__msg__Quaternion q;

  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

void debug_values(const int16_t ax, const int16_t ay, const int16_t az, const int16_t gx, const int16_t gy, const int16_t gz)
{
  Serial.print("RAW");
  Serial.print(ax); Serial.print(", ");
  Serial.print(ay); Serial.print(", ");
  Serial.print(az); Serial.print(" : ");
  Serial.print(gx); Serial.print(", ");
  Serial.print(gy); Serial.print(", ");
  Serial.print(gz); 
  Serial.println();
  Serial.print("PREPROC");
  Serial.print(rawacc2radsacc(ax)); Serial.print(", ");
  Serial.print(rawacc2radsacc(ay)); Serial.print(", ");
  Serial.print(rawacc2radsacc(az)); Serial.print(" : ");
  Serial.print(rawgyr2radsgyr(gx)); Serial.print(", ");
  Serial.print(rawgyr2radsgyr(gy)); Serial.print(", ");
  Serial.print(rawgyr2radsgyr(gz)); 
  Serial.println();
}
