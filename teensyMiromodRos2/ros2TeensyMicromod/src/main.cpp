#include <Arduino.h>
#include "TeensyThreads.h"
// #include "ChRt.h"

#include <micro_ros_platformio.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

// Define the UART pins for Teensy (RX, TX)
#define RX_PIN 0
#define TX_PIN 1

#define HWSERIAL1 Serial1
#define HWSERIAL2 Serial2
// #define HWSERIAL3 Serial6
// #define HWSERIAL4 Serial3

// Define the node and publisher
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_publisher_t publisher;
// std_msgs__msg__String msg;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;

unsigned long long time_offset = 0;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      rclErrorLoop();            \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

volatile int blinkcode = 0;
bool lock1 = false;

uint8_t data[10] = {0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void Ros2pub()
{

  while (true)
  {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
    threads.delay(1000);

    threads.yield();
  }
}

void rclErrorLoop()
{
  while (1)
  {
    // digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

void serialThread()
{

  uint8_t buffer[10];
  while (1)
  {
    if (lock1 == false)
    {
      lock1 = true;
      HWSERIAL1.write(data, sizeof(data));
      int ok = 0;
      while (HWSERIAL1.available() < 10)
      {
      }
      for (int n = 0; n < 10; n++)
        buffer[n] = HWSERIAL1.read();

      lock1 = false;
    }
    Serial.println("uart1");
    threads.delay(50);

    threads.yield();
  }
}

void blinkthread()
{
  while (1)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    threads.delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    threads.delay(50);
    threads.yield();
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  HWSERIAL1.begin(115200);
  HWSERIAL2.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);
  threads.addThread(blinkthread);
  threads.addThread(serialThread);
  threads.addThread(Ros2pub);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "micro_ros_platformio_node_publisher"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));


  msg.data = 0;
}

void loop()
{
}