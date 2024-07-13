// #include <MPU6050.h>
#include "MPU6050_6Axis_MotionApps20.h"

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
sensor_msgs__msg__Imu msg;
std_msgs__msg__Float32 msg2;

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

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z] //the ros2 quat message expects xyzw        quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gg;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorInt16 ggWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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

MPU6050 mpu;
// MPU6050 imu2(0x69);
// int16_t gx1, gy1, gz1;
// int16_t gx, gy, gz;

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  if (state == AGENT_CONNECTED)
  {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
      mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGyro(&gg, fifoBuffer);

      msg.orientation.w = q.w;
      msg.orientation.x = q.x;
      msg.orientation.y = q.y;
      msg.orientation.z = q.z;
      msg.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg.angular_velocity.y = (float)aa.y;
      msg.angular_velocity.z = (float)aa.z;
      msg.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg.linear_acceleration.y = (float)gg.y;
      msg.linear_acceleration.z = (float)gg.z;

      RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

      //    msg2.data = (float)gx;  // Assign IMU data to message
      ///     RCSOFTCHECK(rcl_publish(&publisher2, &msg2, NULL));
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

  RCCHECK(rclc_publisher_init_default(
      &publisher2,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "micro_ros_arduino_imu_publisher2"));

  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(timer_timeout),
      timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher, &node);
  rcl_publisher_fini(&publisher2, &node);
  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  state = WAITING_AGENT;
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-156);
  mpu.setYGyroOffset(-11);
  mpu.setZGyroOffset(-14);
  mpu.setXAccelOffset(-3699);
  mpu.setYAccelOffset(-2519);
  mpu.setZAccelOffset(1391); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {

    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }

  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  msg2.data = 0;

  msg.header.frame_id.data = "imu_link";
}

void loop()
{
  delay(100);
  if (!dmpReady)
    return;

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
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }
}
