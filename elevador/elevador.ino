// Definição dos pinos (GPIOs da ESP32)
const int IN1 = 18;
const int IN2 = 19;
const int IN3 = 27;
const int IN4 = 26;

// Velocidade: quanto MENOR o número, MAIS RÁPIDO o motor gira.
// Com 19V, você pode tentar valores entre 5 e 20.
int tempo = 2; 

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void passoA() {
  digitalWrite(IN1, 1); digitalWrite(IN2, 0); digitalWrite(IN3, 0); digitalWrite(IN4, 1);
  delay(tempo);
}

void passoB() {
  digitalWrite(IN1, 0); digitalWrite(IN2, 1); digitalWrite(IN3, 0); digitalWrite(IN4, 1);
  delay(tempo);
}

void passoC() {
  digitalWrite(IN1, 0); digitalWrite(IN2, 1); digitalWrite(IN3, 1); digitalWrite(IN4, 0);
  delay(tempo);
}

void passoD() {
  digitalWrite(IN1, 1); digitalWrite(IN2, 0); digitalWrite(IN3, 1); digitalWrite(IN4, 0);
  delay(tempo);
}

void loop() {
  // Para manter o motor girando continuamente para a mesma direção,
  // basta chamar a sequência de passos infinitamente.
  passoA();
  passoB();
  passoC();
  passoD();
}