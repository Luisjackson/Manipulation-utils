/* 
junta0 - motor0 - lift
junta1 - motor1 - giro
junta2 - motor2 - braço
junta2 - motor3 - braço
junta3 - motor4 - antebraço
junta3 - motor5 - antebraço
junta4 - motor6 - 1 servo
junta5 - motor7 - 1 
junta6 - motor8 - 1 
junta7 - motor9 - 1 gripper
*/

#include <Arduino.h>
#include "LX16A-bus.h"
#include "smoothmotor.h"
#include "smoothservo.h"

LX16A motor0(0, Serial); // lift
LX16A motor1(1, Serial); // Junta 1 (base/giro)
LX16A motor2(2, Serial); // junta2
LX16A motor3(3, Serial); // junta2
LX16A motor4(4, Serial); // junta3
LX16A motor5(5, Serial); // junta3
LX16A motor6(6, Serial);  // junta 4
LX16A motor7(7, Serial);  // junta 5
LX16A motor8(8, Serial);  // junta 6
LX16A motor9(9, Serial);  // junta 7

// Smooth motor mode operation:
SmoothMotor smotor0(motor0);  //Elevador

const int botaoPin = 19; // Pino onde o botão esta conectado
const int botaoFimCursoSuperiorPin = 12; 
const int botaoFimCursoInferiorPin = 26;                
const int buzzerPin = 16;
const int ledPin = 2;               
int temp;

// Mapeamento de cada posição
int posMotor1 = 105; 
int posJunta2 = 110; // Motor 2 e 3
int posJunta3 = 80; // Motor 4 e 5
int posMotor6 = 110;
int posMotor7 = 25;
int posMotor8 = 145;
int posMotor9 = 110;

// Motor mapeado inicialmente
int motorSelecionado = 1;

void homePosition() {
  digitalWrite(2, LOW);         // Apaga o Led
  tone(buzzerPin, 2600, 1500);  // // Toca 1500 Hz por 400ms

  motor7.move(25, 500);
  posMotor7 = 25;
  delay(1000);

  motor2.move(110, 500);
  motor3.move(240 - 110, 500);
  posJunta2 = 110;
  delay(1000);

  motor4.move(80, 500);
  motor5.move(240 - 80 - 15, 500);  // 15 e "ajuste mecanico"
  posJunta3 = 80;
  delay(1000);

  motor1.move(105, 500);  // Giro, 105 posiçao default, no meio virado para a frente
  posMotor1 = 105;
  delay(1000);

  motor6.move(110, 500);
  posMotor6 = 110;
  delay(1000);

  motor8.move(145, 500);
  posMotor8 = 145;
  delay(2000);

  motor9.move(110, 500);
  posMotor9 = 110;
  delay(1000);
}

void openHand() {
  motor9.move(110); 
}

void closeHand() {
  motor9.move(35); 
}

void setup(){
  Serial.begin(115200);

  motor0.initialize(); motor0.enableTorque();
  motor1.initialize(); motor1.enableTorque();
  motor2.initialize(); motor2.enableTorque();
  motor3.initialize(); motor3.enableTorque();
  motor4.initialize(); motor4.enableTorque();
  motor5.initialize(); motor5.enableTorque();
  motor6.initialize(); motor6.enableTorque();
  motor7.initialize(); motor7.enableTorque();
  motor8.initialize(); motor8.enableTorque();
  motor9.initialize(); motor9.enableTorque();

  motor0.setMotorMode(0);
  motor1.setServoMode();
  motor2.setServoMode();
  motor3.setServoMode();
  motor4.setServoMode();
  motor5.setServoMode();
  motor6.setServoMode();
  motor7.setServoMode();
  motor8.setServoMode();
  motor9.setServoMode();

  pinMode(botaoPin, INPUT_PULLUP);                  // Configura o pino do botão com pull-up interno
  pinMode(botaoFimCursoSuperiorPin, INPUT_PULLUP);  // Configura o pino do botão com pull-up interno
  pinMode(botaoFimCursoInferiorPin, INPUT_PULLUP);  // Configura o pino do botão com pull-up interno

  pinMode(buzzerPin, OUTPUT);  // Configura o pino do buzzer como saída
  pinMode(ledPin, OUTPUT);     //Define a porta do led como saida

  smotor0.start(1000);

  homePosition();  //Posiçao default inicial
  delay(1000);

}

void loop() {

  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'H': homePosition(); break;
      case 'O': openHand(); break;
      case 'C': closeHand(); break;

      case '1': motorSelecionado = 1; break;

      case '2': motorSelecionado = 2; break; // Seleciona a Junta 2 (motores 2 e 3)
      case '4': motorSelecionado = 4; break;// Seleciona Antebraço (Motores 4 e 5)
      case '6': motorSelecionado = 6; break;
      case '7': motorSelecionado = 7; break; 
      case '9': motorSelecionado = 9; break;


      case 'B': // INCREMENTAR
        if (motorSelecionado == 1) {
            posMotor1 += 10;
            if (posMotor1 > 240) posMotor1 = 240;
            motor1.move(posMotor1, 500);
        } 
        else if (motorSelecionado == 2) {
            posJunta2 += 10;
            if (posJunta2 > 240) posJunta2 = 240;
            motor2.move(posJunta2, 500);
            motor3.move(240 - posJunta2, 500); 
        }
        else if (motorSelecionado == 4) {
            posJunta3 += 10;
            if (posJunta3 > 240) posJunta3 = 240;
            motor4.move(posJunta3, 500);
            motor5.move(240 - posJunta3 - 15, 500);
        }
        else if (motorSelecionado == 6) {
            posMotor6 += 10;
            if (posMotor6 > 240) posMotor6 = 240;
            motor6.move(posMotor6, 500);
        }
        else if (motorSelecionado == 7) { 
            posMotor7 += 10;
            if (posMotor7 > 240) posMotor7 = 240;
            motor7.move(posMotor7, 500);
        }
        else if (motorSelecionado == 9) {
            posMotor9 += 10;
            if (posMotor9 > 240) posMotor9 = 240;
            motor9.move(posMotor9, 500);
        }
        break;

      case 'K': // DECREMENTAR
        if (motorSelecionado == 1) {
            posMotor1 -= 10;
            if (posMotor1 < 0) posMotor1 = 0;
            motor1.move(posMotor1, 500);
        } 
        else if (motorSelecionado == 2) { 
            posJunta2 -= 10;
            if (posJunta2 < 0) posJunta2 = 0;
            motor2.move(posJunta2, 500);
            motor3.move(240 - posJunta2, 500);
        }
        else if (motorSelecionado == 4) {
            posJunta3 -= 10;
            if (posJunta3 < 0) posJunta3 = 0;
            motor4.move(posJunta3, 500);
            motor5.move(240 - posJunta3 - 15, 500); 
        }
        else if (motorSelecionado == 6) { 
            posMotor6 -= 10;
            if (posMotor6 < 0) posMotor6 = 0;
            motor6.move(posMotor6, 500);
        }
        else if (motorSelecionado == 7) {
            posMotor7 -= 10;
            if (posMotor7 < 0) posMotor7 = 0;
            motor7.move(posMotor7, 500);
        }
        else if (motorSelecionado == 9) {
            posMotor9 -= 10;
            if (posMotor9 < 0) posMotor9 = 0;
            motor9.move(posMotor9, 500);
        }
        break;
    }
  }
}