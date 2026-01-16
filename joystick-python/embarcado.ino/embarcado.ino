#include <DynamixelShield.h>

const uint8_t DXL_06_OMBRO     = 6;
const uint8_t DXL_02_ANTEBRACO = 2;
const uint8_t DXL_03_COTOVELO  = 3;
const uint8_t DXL_04_PULSO     = 4;
const uint8_t DXL_05_MAO       = 5;
const float DXL_PROTOCOL_VERSION = 1.0;

DynamixelShield dxl;

class ArmController {
private:
    DynamixelShield* dxl;

public:

    ArmController(DynamixelShield* d) {
        dxl = d;
    }

    void moveArmSmooth(
        int target_ombro,
        int target_antebraco,
        int target_cotovelo,
        int target_pulso,
        int target_mao,
        int step = 3,
        int delay_ms = 10)
    {
        int cur_ombro     = dxl->getPresentPosition(DXL_06_OMBRO);
        int cur_antebraco = dxl->getPresentPosition(DXL_02_ANTEBRACO);
        int cur_cotovelo  = dxl->getPresentPosition(DXL_03_COTOVELO);
        int cur_pulso     = dxl->getPresentPosition(DXL_04_PULSO);
        int cur_mao       = dxl->getPresentPosition(DXL_05_MAO);

        bool done = false;

        while (!done) {
            done = true;

            if (abs(cur_ombro - target_ombro) > step) {
                cur_ombro += (cur_ombro < target_ombro ? step : -step);
                dxl->setGoalPosition(DXL_06_OMBRO, cur_ombro);
                done = false;
            }

            if (abs(cur_antebraco - target_antebraco) > step) {
                cur_antebraco += (cur_antebraco < target_antebraco ? step : -step);
                dxl->setGoalPosition(DXL_02_ANTEBRACO, cur_antebraco);
                done = false;
            }

            if (abs(cur_cotovelo - target_cotovelo) > step) {
                cur_cotovelo += (cur_cotovelo < target_cotovelo ? step : -step);
                dxl->setGoalPosition(DXL_03_COTOVELO, cur_cotovelo);
                done = false;
            }

            if (abs(cur_pulso - target_pulso) > step) {
                cur_pulso += (cur_pulso < target_pulso ? step : -step);
                dxl->setGoalPosition(DXL_04_PULSO, cur_pulso);
                done = false;
            }

            if (abs(cur_mao - target_mao) > step) {
                cur_mao += (cur_mao < target_mao ? step : -step);
                dxl->setGoalPosition(DXL_05_MAO, cur_mao);
                done = false;
            }

            delay(delay_ms);
        }

        dxl->setGoalPosition(DXL_06_OMBRO, target_ombro);
        dxl->setGoalPosition(DXL_02_ANTEBRACO, target_antebraco);
        dxl->setGoalPosition(DXL_03_COTOVELO, target_cotovelo);
        dxl->setGoalPosition(DXL_04_PULSO, target_pulso);
        dxl->setGoalPosition(DXL_05_MAO, target_mao);

        
    }

    // Poses
    void moveToHome() { moveArmSmooth(228, 756, 4, 831, 600);}
    void moveToReady() {moveArmSmooth(512, 756, 3, 829, 600);}
    void moveToPrePick() {moveArmSmooth(517, 1012, 8, 845, 592);}
    void moveToPick() {moveArmSmooth(499, 1011, 300, 531, 390);}
    void moveToPosPick() {moveArmSmooth(512, 1012, 3, 830, 600);}
    void moveToPlace(){moveArmSmooth(532, 1014, 197, 654, 600);}
    void moveToFront() {moveArmSmooth(547, 950, 300, 530, 600);}
    void moveToPlaceAlto() { moveArmSmooth(525, 561, 600, 683, 600);}

    // Controle de Um servo só
    void openHand() {
        moveArmSmooth(
            dxl->getPresentPosition(DXL_06_OMBRO),
            dxl->getPresentPosition(DXL_02_ANTEBRACO),
            dxl->getPresentPosition(DXL_03_COTOVELO),
            dxl->getPresentPosition(DXL_04_PULSO),
            390, 2, 5
        );
    }

    void openHandObject() {
      moveArmSmooth(
          dxl->getPresentPosition(DXL_06_OMBRO),
          dxl->getPresentPosition(DXL_02_ANTEBRACO),
          dxl->getPresentPosition(DXL_03_COTOVELO),
          dxl->getPresentPosition(DXL_04_PULSO),
          390,
          35,   
          5     
      );
    }

    void closeHand() {
        moveArmSmooth(
            dxl->getPresentPosition(DXL_06_OMBRO),
            dxl->getPresentPosition(DXL_02_ANTEBRACO),
            dxl->getPresentPosition(DXL_03_COTOVELO),
            dxl->getPresentPosition(DXL_04_PULSO),
            550, 2, 5
        );
    }

    void descerGarra() {
        moveArmSmooth(
            dxl->getPresentPosition(DXL_06_OMBRO),
            dxl->getPresentPosition(DXL_02_ANTEBRACO),
            dxl->getPresentPosition(DXL_03_COTOVELO),
            700,
            dxl->getPresentPosition(DXL_05_MAO),
            2, 5
        );
    }

    void subirGarra() {
        moveArmSmooth(
            dxl->getPresentPosition(DXL_06_OMBRO),
            dxl->getPresentPosition(DXL_02_ANTEBRACO),
            dxl->getPresentPosition(DXL_03_COTOVELO),
            195,
            dxl->getPresentPosition(DXL_05_MAO),
            2, 5
        );
    }

    
    void pick() {
        moveToHome();     delay(2500);
        moveToReady();    delay(2500);
        moveToPrePick();  delay(4000);
        openHand();      delay(2000);
        moveToPick();     delay(1200);
        closeHand();      delay(1000);
        moveToPosPick();  delay(1000);
    }

    void place(){
        moveToPlace(); delay(3500);
        descerGarra(); delay(500);
        openHandObject(); delay(2000);
    }
};

ArmController arm(&dxl);

void configurarServo(uint8_t id) {
  dxl.ping(id);
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
}

void setup() {
  // Inicializa a Serial na mesma velocidade do Python
  Serial.begin(115200); 

  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  configurarServo(DXL_06_OMBRO);
  configurarServo(DXL_02_ANTEBRACO);
  configurarServo(DXL_03_COTOVELO);
  configurarServo(DXL_04_PULSO);
  configurarServo(DXL_05_MAO);

  arm.moveToHome();
}

void loop() {
  // Verifica se chegou algo via Serial
  if (Serial.available() > 0) {
    char command = Serial.read(); // Lê o caractere vindo do Python

    switch (command) {
      case 'B': // Mapeado para LB
        arm.moveToPrePick();
        break;
      case 'K': // Mapeado para RB 
        arm.moveToPick();
        break;
      case 'P': // Pick
        arm.pick();
        break;
      case 'L': // Place
        arm.place();
        break;
      case 'H': // Home
        arm.moveToHome();
        break;
      case 'C': // Close Hand
        arm.closeHand();
        break;
      case 'O': // Open Hand
        arm.openHand();
        break;
      case 'R': // Ready
        arm.moveToReady();
        break;
      case 'D': // Descer
        arm.descerGarra();
        break;
      case 'M':
        arm.moveToPlaceAlto();
        break;
    }
  }
}