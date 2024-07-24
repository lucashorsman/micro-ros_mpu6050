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
rcl_publisher_t publisher0;
rcl_publisher_t publisher1;
rcl_publisher_t publisher2;
rcl_publisher_t publisher3;
rcl_publisher_t publisher4;
sensor_msgs__msg__Imu msg0;
sensor_msgs__msg__Imu msg1;
sensor_msgs__msg__Imu msg2;
sensor_msgs__msg__Imu msg3;
sensor_msgs__msg__Imu msg4;


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

#define TCAADDR 0x70
void tcaselect(uint8_t i)
{
  if (i > 7)
    return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}
MPU6050 mpu0;
MPU6050 mpu1;
MPU6050 mpu2;
MPU6050 mpu3;
MPU6050 mpu4;
// MPU6050 imu1(0x68);
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
      tcaselect(0);
      mpu0.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu0.dmpGetQuaternion(&q, fifoBuffer);
      mpu0.dmpGetGravity(&gravity, &q);
      mpu0.dmpGetAccel(&aa, fifoBuffer);
      mpu0.dmpGetGyro(&gg, fifoBuffer);
      msg0.orientation.w = q.w;
      msg0.orientation.x = q.x;
      msg0.orientation.y = q.y;
      msg0.orientation.z = q.z;
      msg0.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg0.angular_velocity.y = (float)aa.y;
      msg0.angular_velocity.z = (float)aa.z;
      msg0.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg0.linear_acceleration.y = (float)gg.y;
      msg0.linear_acceleration.z = (float)gg.z;

      msg0.header.stamp.sec = millis() / 1000;
      RCSOFTCHECK(rcl_publish(&publisher0, &msg0, NULL));

       tcaselect(1);
      mpu1.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu1.dmpGetQuaternion(&q, fifoBuffer);
      mpu1.dmpGetGravity(&gravity, &q);
      mpu1.dmpGetAccel(&aa, fifoBuffer);
      mpu1.dmpGetGyro(&gg, fifoBuffer);
      msg1.orientation.w = q.w;
      msg1.orientation.x = q.x;
      msg1.orientation.y = q.y;
      msg1.orientation.z = q.z;
      msg1.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg1.angular_velocity.y = (float)aa.y;
      msg1.angular_velocity.z = (float)aa.z;
      msg1.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg1.linear_acceleration.y = (float)gg.y;
      msg1.linear_acceleration.z = (float)gg.z;

      msg1.header.stamp.sec = millis() / 1000;
      RCSOFTCHECK(rcl_publish(&publisher1, &msg1, NULL));
       tcaselect(2);
      mpu2.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu2.dmpGetQuaternion(&q, fifoBuffer);
      mpu2.dmpGetGravity(&gravity, &q);
      mpu2.dmpGetAccel(&aa, fifoBuffer);
      mpu2.dmpGetGyro(&gg, fifoBuffer);
      msg2.orientation.w = q.w;
      msg2.orientation.x = q.x;
      msg2.orientation.y = q.y;
      msg2.orientation.z = q.z;
      msg2.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg2.angular_velocity.y = (float)aa.y;
      msg2.angular_velocity.z = (float)aa.z;
      msg2.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg2.linear_acceleration.y = (float)gg.y;
      msg2.linear_acceleration.z = (float)gg.z;

      msg2.header.stamp.sec = millis() / 1000;
      RCSOFTCHECK(rcl_publish(&publisher2, &msg2, NULL));
 tcaselect(3);
      mpu3.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu3.dmpGetQuaternion(&q, fifoBuffer);
      mpu3.dmpGetGravity(&gravity, &q);
      mpu3.dmpGetAccel(&aa, fifoBuffer);
      mpu3.dmpGetGyro(&gg, fifoBuffer);
      msg3.orientation.w = q.w;
      msg3.orientation.x = q.x;
      msg3.orientation.y = q.y;
      msg3.orientation.z = q.z;
      msg3.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg3.angular_velocity.y = (float)aa.y;
      msg3.angular_velocity.z = (float)aa.z;
      msg3.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg3.linear_acceleration.y = (float)gg.y;
      msg3.linear_acceleration.z = (float)gg.z;

      msg3.header.stamp.sec = millis() / 1000;
      RCSOFTCHECK(rcl_publish(&publisher3, &msg3, NULL));
 tcaselect(4);
      mpu4.dmpGetCurrentFIFOPacket(fifoBuffer);
      mpu4.dmpGetQuaternion(&q, fifoBuffer);
      mpu4.dmpGetGravity(&gravity, &q);
      mpu4.dmpGetAccel(&aa, fifoBuffer);
      mpu4.dmpGetGyro(&gg, fifoBuffer);
      msg4.orientation.w = q.w;
      msg4.orientation.x = q.x;
      msg4.orientation.y = q.y;
      msg4.orientation.z = q.z;
      msg4.angular_velocity.x = (float)aa.x; // these might be gg not aa
      msg4.angular_velocity.y = (float)aa.y;
      msg4.angular_velocity.z = (float)aa.z;
      msg4.linear_acceleration.x = (float)gg.x; // these might be aa not gg
      msg4.linear_acceleration.y = (float)gg.y;
      msg4.linear_acceleration.z = (float)gg.z;

      msg4.header.stamp.sec = millis() / 1000;
      RCSOFTCHECK(rcl_publish(&publisher4, &msg4, NULL));

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

  RCCHECK(rclc_publisher_init_best_effort(
            &publisher0,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_imu_publisher"));

  RCCHECK(rclc_publisher_init_best_effort(
            &publisher1,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_imu_publisher1"));

  RCCHECK(rclc_publisher_init_best_effort(
            &publisher2,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_imu_publisher2"));

  RCCHECK(rclc_publisher_init_best_effort(
            &publisher3,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_imu_publisher3"));

  RCCHECK(rclc_publisher_init_best_effort(
            &publisher4,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "micro_ros_arduino_imu_publisher4"));



  //
  //  RCCHECK(rclc_publisher_init_default(
  //      &publisher2,
  //      &node,
  //      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  //      "micro_ros_arduino_imu_publisher2"));

  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
            &timer,
            &support,
            RCL_MS_TO_NS(timer_timeout),
            timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&publisher0, &node);
  rcl_publisher_fini(&publisher1, &node);
  rcl_publisher_fini(&publisher2, &node);
  rcl_publisher_fini(&publisher3, &node);
  rcl_publisher_fini(&publisher4, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  state = WAITING_AGENT;
  Wire.begin();

  mpu0.initialize();

  devStatus = mpu0.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu0.setXGyroOffset(-156);
  mpu0.setYGyroOffset(-11);
  mpu0.setZGyroOffset(-14);
  mpu0.setXAccelOffset(-3699);
  mpu0.setYAccelOffset(-2519);
  mpu0.setZAccelOffset(1391); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {

    mpu0.CalibrateAccel(6);
    mpu0.CalibrateGyro(6);
    mpu0.PrintActiveOffsets();
    mpu0.setDMPEnabled(true);

    mpuIntStatus = mpu0.getIntStatus();

    dmpReady = true;
    packetSize = mpu0.dmpGetFIFOPacketSize();
  }

  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  //
  //  delay(2000);


}

void loop()
{
  //  delay(100);
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
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  }
}
