// #include <MPU6050.h>
// #include "MPU6050_6Axis_MotionApps20.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <sensor_msgs/msg/imu.h>
rcl_publisher_t publisher;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;
rcl_publisher_t publisher5;

sensor_msgs__msg__Imu msg;
sensor_msgs__msg__Imu msg2;
sensor_msgs__msg__Imu msg3;
sensor_msgs__msg__Imu msg4;
sensor_msgs__msg__Imu msg5;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// reconnect stuff
bool micro_ros_init_successful;

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#define LED_PIN 13

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

#include <Adafruit_MPU6050.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>

#define TCAADDR 0x70

/* Assign a unique ID to this sensor at the same time */
Adafruit_MPU6050 mpu;
Adafruit_MPU6050 mpu2;
Adafruit_MPU6050 mpu3;
Adafruit_MPU6050 mpu4;
Adafruit_MPU6050 mpu5;
sensors_event_t accel, gyro, temp; // Declares an object "event"

void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void publishIMUData(Adafruit_MPU6050 &mpu, sensor_msgs__msg__Imu &msg, rcl_publisher_t &publisher)
{
  mpu.getEvent(&accel, &gyro, &temp);
  msg.angular_velocity.x = gyro.gyro.x;
  msg.angular_velocity.y = gyro.gyro.y;
  msg.angular_velocity.z = gyro.gyro.z;
  msg.linear_acceleration.x = accel.acceleration.x;
  msg.linear_acceleration.y = accel.acceleration.y;
  msg.linear_acceleration.z = accel.acceleration.z;
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  if (state == AGENT_CONNECTED)
  {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
      sensors_event_t event;

      // tcaselect(0);
      publishIMUData(mpu, msg, publisher);
      // tcaselect(1);
      // publishIMUData(mpu2, msg2, publisher2);
      // tcaselect(2);
      // publishIMUData(mpu3, msg3, publisher3);
      // tcaselect(3);
      // publishIMUData(mpu4, msg4, publisher4);
      // tcaselect(4);
      // publishIMUData(mpu5, msg5, publisher5);
    }
  }
}

bool create_entities()
{
  // Code to initialize ROS entities
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_imu_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "micro_ros_arduino_imu_publisher"));

  // RCCHECK(rclc_publisher_init_default(
  //     &publisher2,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //     "micro_ros_arduino_imu_publisher2"));

  // RCCHECK(rclc_publisher_init_default(
  //     &publisher3,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //     "micro_ros_arduino_imu_publisher3"));

  // RCCHECK(rclc_publisher_init_default(
  //     &publisher4,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //     "micro_ros_arduino_imu_publisher4"));

  // RCCHECK(rclc_publisher_init_default(
  //     &publisher5,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
  //     "micro_ros_arduino_imu_publisher5"));


  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 6, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&publisher2, &node);
  rcl_publisher_fini(&publisher3, &node);
  rcl_publisher_fini(&publisher4, &node);
  rcl_publisher_fini(&publisher5, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  state = WAITING_AGENT;
  Wire.begin();
//select mux addr
//set up the IMU
  // tcaselect(0);
  mpu.begin();
  // tcaselect(1);
  // mpu2.begin();
  // tcaselect(2);
  // mpu3.begin();
  // tcaselect(3);
  // mpu4.begin();
  // tcaselect(4);
  // mpu5.begin();


  set_microros_transports();

//teensy led
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

 

  msg.header.frame_id.data = "imu_link";
  msg2.header.frame_id.data = "imu2_link";
  msg3.header.frame_id.data = "imu3_link";
  msg4.header.frame_id.data = "imu4_link";
  msg5.header.frame_id.data = "imu5_link";
}

void loop()
{
  
  switch (state)
  {
  case WAITING_AGENT:
    if (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_AVAILABLE;
    }
    break;
  case AGENT_AVAILABLE:
    if (create_entities())
    {
      state = AGENT_CONNECTED;
    }
    else
    {
      state = WAITING_AGENT;
    }
    break;
  case AGENT_CONNECTED:
    if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
    {
      state = AGENT_DISCONNECTED;
    }
    break;
  case AGENT_DISCONNECTED:
    destroy_entities();
    state = WAITING_AGENT;
    break;
  }

  if (state == AGENT_CONNECTED)
  {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1000000)));
  }
}
