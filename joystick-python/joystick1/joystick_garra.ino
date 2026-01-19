#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8);
  #define DEBUG_SERIAL soft_serial
#else
  #define DEBUG_SERIAL Serial
#endif

using namespace ControlTableItem;

DynamixelShield dxl;
const float DXL_PROTOCOL_VERSION = 1.0;

// IDs
const uint8_t ID_BASE   = 6;
const uint8_t ID_OMBRO  = 2;
const uint8_t ID_COTOV  = 3;
const uint8_t ID_PUNHO  = 4;
const uint8_t ID_GARRA  = 5;

// Ganhos (provavelmente tem que mudar)
float ganhoBase  = 4; 
float ganhoOmbro = 3;
float ganhoCotov = 3;
float ganhoPunho = 3;

// Limites
const int LIM_MIN = 0;
const int LIM_MAX = 1023;

// Posições (tem que ajustar)
int posBase  = 228;
int posOmbro = 756;
int posCotov = 512;
int posPunho = 831;

// Buffer serial
char buffer[12];
uint8_t idx = 0;

// FUNÇÕES
void configurarServo(uint8_t id) {
  dxl.ping(id);
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
}

void moverAnalogico(uint8_t id, int &pos, int desloc, float ganho) {
  if (desloc == 0) return;
  pos += desloc * ganho;
  pos = constrain(pos, LIM_MIN, LIM_MAX);
  dxl.setGoalPosition(id, pos);
}

void posicaoBase() { // tem que arrumar
  posBase  = 228;
  posOmbro = 756;
  posCotov = 512;
  posPunho = 831;

  dxl.setGoalPosition(ID_BASE,  posBase);
  dxl.setGoalPosition(ID_OMBRO, posOmbro);
  dxl.setGoalPosition(ID_COTOV, posCotov);
  dxl.setGoalPosition(ID_PUNHO, posPunho);
  dxl.setGoalPosition(ID_GARRA, 600);
}

void abrirGarra()  { dxl.setGoalPosition(ID_GARRA, 500); }
void fecharGarra() { dxl.setGoalPosition(ID_GARRA, 920); }

// SETUP 
void setup() {
  Serial.begin(115200);
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  configurarServo(ID_BASE);
  configurarServo(ID_OMBRO);
  configurarServo(ID_COTOV);
  configurarServo(ID_PUNHO);
  configurarServo(ID_GARRA);

  posicaoBase();
}

// LOOP
void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      buffer[idx] = '\0';
      
      if (idx > 0) {
        char cmd = buffer[0];
        int valor = atoi(&buffer[1]);

      // Botões
      if (cmd == 'Y') posicaoBase();
      else if (cmd == 'X') abrirGarra();
      else if (cmd == 'B') fecharGarra();

      // Analógicos
      else if (cmd == 'U') moverAnalogico(ID_BASE,  posBase,  valor, ganhoBase);
      else if (cmd == 'V') moverAnalogico(ID_OMBRO, posOmbro, valor, ganhoOmbro);
      else if (cmd == 'W') moverAnalogico(ID_COTOV, posCotov, valor, ganhoCotov);
      else if (cmd == 'Z') moverAnalogico(ID_PUNHO, posPunho, valor, ganhoPunho);
    }
      
      idx = 0; // Reseta o índice para a próxima mensagem
    }
    
    else if (idx < sizeof(buffer) - 1) {
      if (c >= 32) { // Ignora caracteres invisíveis/ruídos
        buffer[idx++] = c;
      }
    }
    
    // Estouro de buffer
    else {
      idx = 0; 
    }
  }
}