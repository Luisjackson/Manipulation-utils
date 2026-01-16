   
// Versão para varios IDS e posicao atual e função de apontar 

// ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 

// ros2 topic pub /motor_position std_msgs/msg/Int32 "{data: 1100}"

// Sendo que o primeiro digito é o ID, e o resto é a posicao desejada
// Ex data:5800 -> ID 5 POSICAO 

// SERVO 1: 1510 DEFAULT
// SERVO 2 : 2570 DEFAULT
// SERVO 3 : 3515 DEFAULT
// SERVO 4 : 4505 DEFAULT
// SERVO 5 : 5600 DEFAULT
// SERVO 6 : 6207 DEFAULT

#include <micro_ros_arduino.h>
#include <DynamixelShield.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // RX, TX
#define DEBUG_SERIAL soft_serial
#else
#define DEBUG_SERIAL Serial
#endif

#define DXL_PROTOCOL_VERSION 1.0
DynamixelShield dxl;
using namespace ControlTableItem;

#define DXL_MOVING_SPEED 50 

int motor_ids[] = {1, 2, 3, 4, 5, 6};
const int num_motors = sizeof(motor_ids) / sizeof(motor_ids[0]);

// ROS2 Configuração
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Publicadores ROS2
rcl_publisher_t status_publisher;
rcl_publisher_t current_position_publisher;
std_msgs__msg__Int32 status_msg;
std_msgs__msg__Int32 current_position_msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    delay(100);
  }
}

bool motor_exists(int id) {
  for (int i = 0; i < num_motors; ++i) {
    if (motor_ids[i] == id) {
      return true;
    }
  }
  return false;
}

void move_motor(int id, int pos){
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
    dxl.setGoalPosition(id, pos);
}

void defaultPosition(){
  DEBUG_SERIAL.println("Movendo todos os motores para a posição default...");
  dxl.setGoalPosition(2, 570);
  delay(100);
  dxl.setGoalPosition(3, 515);
  delay(100);
  dxl.setGoalPosition(4, 505);
  delay(1500);
  dxl.setGoalPosition(5, 600);
  delay(1500);
  dxl.setGoalPosition(6, 207);

}

void apontarFrente(){
    DEBUG_SERIAL.println("Apontando para frente");
    dxl.setGoalPosition(1, 500);
    delay(1500);
    dxl.setGoalPosition(2, 570);
    delay(100);
    dxl.setGoalPosition(3, 515);
    delay(100);
    dxl.setGoalPosition(4, 815);
    delay(1500);
    dxl.setGoalPosition(5, 600);
    delay(1500);
    dxl.setGoalPosition(6, 207);
}

void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  int valor = msg->data;

  move_motor(1, valor);
  delay(1500);

  apontarFrente();
}

void setup() {
  DEBUG_SERIAL.begin(115200);
  set_microros_transports();

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 0; i < num_motors; ++i) {
    dxl.ping(motor_ids[i]);
    dxl.setGoalVelocity(motor_ids[i], DXL_MOVING_SPEED);
}

  
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_dynamixel_node", "", &support));

  RCCHECK(rclc_subscription_init_default(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
            "motor_position"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  DEBUG_SERIAL.println("Setup completo. Aguardando mensagem no tópico /motor_position...");

}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Publicar a posição atual de cada motor
  for (int i = 0; i < num_motors; ++i) {
    int id = motor_ids[i];
    int current_position = dxl.getPresentPosition(id);

    if (current_position != -1) { // -1 indica erro de leitura
      current_position_msg.data = id * 1000 + current_position;
      RCSOFTCHECK(rcl_publish(&current_position_publisher, &current_position_msg, NULL));
    }
  }

  delay(10);
}