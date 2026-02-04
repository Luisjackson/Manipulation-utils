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
SmoothMotor smotor0(motor0);  //Lift

const int botaoPin = 19; // Pino onde o botão esta conectado
const int botaoFimCursoSuperiorPin = 12;  // Fim de Curso Superior
const int botaoFimCursoInferiorPin = 26;  // Fim de curso Inferiorconst int buzzerPin = 16;                 // Pino onde o buzzer está conectado
const int buzzerPin = 16;
const int ledPin = 2;                     // Pino onde o buzzer está conectado
int temp;

//Home homePosition
void homePosition() {
  digitalWrite(2, LOW);         // Apaga o Led
  tone(buzzerPin, 2600, 1500);  // // Toca 1500 Hz por 400ms

  motor7.move(25);
  delay(1000);

  motor2.move(110);
  motor3.move(240 - 110);
  delay(1000);

  motor4.move(80);
  motor5.move(240 - 80 - 15);  // 15 e "ajuste mecanico"
  delay(1000);

  motor1.move(105);  // Giro, 105 posiçao default, no meio virado para a frente
  delay(1000);

  motor6.move(110);
  delay(1000);

  motor8.move(145);
  delay(2000);

  motor9.move(110);
  delay(1000);
}

void openHand() {
  // Exemplo: Motor 9 é o gripper
  motor9.move(110); 
}

void closeHand() {
  motor9.move(35); 
}

 // TODO - ESTA FORA DE ORDEM. ORGANIZAR. 


 // OPEN HAND
 // MOVE TO PICK(MOVE PRA PERTO DO OBJETO)
 // APPROACH OBJECT (PEGAR OBJETO)
 // CLOSE HAND
 // PICK POSITION
 // RETREAT
 // RETURN HOME
 
void setup(){
  Serial.begin(115200);

  motor0.initialize();
  motor0.enableTorque();

  motor1.initialize();
  motor1.enableTorque();

  motor2.initialize();
  motor2.enableTorque();

  motor3.initialize();
  motor3.enableTorque();

  motor4.initialize();
  motor4.enableTorque();

  motor5.initialize();
  motor5.enableTorque();

  motor6.initialize();
  motor6.enableTorque();

  motor7.initialize();
  motor7.enableTorque();

  motor8.initialize();
  motor8.enableTorque();

  motor9.initialize();
  motor9.enableTorque();

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
  unsigned long ms = millis();

  // ATUALIZAÇÃO DOS MOVIMENTOS (Obrigatório para a suavidade funcionar)
  smotor0.update(ms);
  // sGiro.update(ms);
  // sBracoA.update(ms);
  // sBracoB.update(ms);
  // sAnteA.update(ms);
  // sAnteB.update(ms);

  if (Serial.available() > 0) {
    char command = Serial.read();

    switch (command) {
      case 'H': // Home
        homePosition();
        break;

      case 'O': // Open Hand
        openHand();
        break;

      case 'C': // Close Hand
        closeHand();
        break;

    }
  }
}