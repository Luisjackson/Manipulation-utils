#include <DynamixelShield.h>

using namespace ControlTableItem; 

DynamixelShield dxl; 

const int JOINT_COUNT = 5;
const uint8_t DXL_IDS[JOINT_COUNT] = {6, 2, 3, 4, 5}; // IDs dos servos

char serial_buffer[100];
int buffer_pos = 0;

// Variáveis para controle de tempo do envio de estado
unsigned long last_status_update_time = 0;
const int STATUS_UPDATE_INTERVAL_MS = 100;
void setup() {
  // Inicia a comunicação serial com o PC
  Serial.begin(115200);

  // Inicia a comunicação com os servos Dynamixel
  dxl.begin(1000000); 
  dxl.setPortProtocolVersion(1.0);

  // Configura cada servo
  for (int i = 0; i < JOINT_COUNT; i++) {
    if (dxl.ping(DXL_IDS[i])) {
      dxl.setOperatingMode(DXL_IDS[i], OP_POSITION);
      dxl.torqueOn(DXL_IDS[i]);
    }
  }
}

void loop() {
  // proocessar os dados que chegam do ROS2.
  process_serial_commands();

  // envia o estado atual para o ros2
  publish_current_state();
}

void process_serial_commands() {
  while (Serial.available() > 0) {
    char in_char = Serial.read();

    // Se recebermos uma quebra de linha, o comando está completo.
    if (in_char == '\n') {
      serial_buffer[buffer_pos] = '\0';

      // Verifica se o comando é de Posição (P)
      if (serial_buffer[0] == 'P' && serial_buffer[1] == ' ') {
        int p[JOINT_COUNT];
        // extrair números de uma string.
        int items_parsed = sscanf(serial_buffer + 2, "%d %d %d %d %d", &p[0], &p[1], &p[2], &p[3], &p[4]);
        
        // Se conseguimos ler todos os 5 valores, move os servos.
        if (items_parsed == JOINT_COUNT) {
          for (int i = 0; i < JOINT_COUNT; i++) {
            dxl.setGoalPosition(DXL_IDS[i], p[i]);
          }
        }
      }
      
      buffer_pos = 0; // Reseta o buffer para o próximo comando.
      break;
    } 
    // Se não for quebra de linha, adiciona o caractere ao buffer.
    else {
      if (buffer_pos < sizeof(serial_buffer) - 1) {
        serial_buffer[buffer_pos++] = in_char;
      }
    }
  }
}

void publish_current_state() {
  if (millis() - last_status_update_time >= STATUS_UPDATE_INTERVAL_MS) {
    last_status_update_time = millis();

    // Monta e envia a string de estado "S pos1 pos2"
    Serial.print("S ");
    for (int i = 0; i < JOINT_COUNT; i++) {
      Serial.print(dxl.getPresentPosition(DXL_IDS[i]));
      if (i < JOINT_COUNT - 1) {
        Serial.print(" ");
      }
    }
    Serial.println(); 
  }
}
