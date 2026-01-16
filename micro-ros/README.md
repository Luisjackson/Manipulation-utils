# Manipulação - Controlador micro-ROS (Dynamixel)

Este código embarcado transforma o Arduino em um nó do ROS2, permitindo o controle de múltiplos motores Dynamixel e o monitoramento de suas posições em tempo real.

## 🏗️ Arquitetura do Sistema

O fluxo de comunicação segue este modelo:
**PC (ROS2)** ↔️ `micro_ros_agent` ↔️ **Arduino (micro-ROS Node)** ↔️ **Dynamixel Shield** ↔️ **Motores**

## 🚀 Como rodar

### 1. Preparação do Hardware

* Placa: **Arduino MKR Zero** (ou compatível).
* Shield: **Dynamixel Shield** conectado aos motores ID 1 a 6.
* Protocolo: Dynamixel **1.0**.

### 2. No Computador (ROS2)

Primeiro, é necessário rodar o agente para estabelecer a ponte de comunicação:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

```

### 3. Comandos de Operação

O nó escuta o tópico `/motor_position`. O comando utiliza um formato de "ID concatenado" para simplificar o envio via `Int32`:

* **Fórmula:** `(ID * 1000) + Posição desejada`
* **Exemplo:** Para mover o motor **5** para a posição **800**, envie `5800`.

**Publicar comando:**

```bash
ros2 topic pub /motor_position std_msgs/msg/Int32 "{data: 5800}"

```

## 🎮 Funções Pré-programadas

O código contém rotinas automáticas que facilitam testes de trajetória:

* **`defaultPosition()`**: Move os servos para a postura padrão de repouso.
* **`apontarFrente()`**: Sequência de movimentos para estender o braço à frente.

## 📊 Telemetria (Feedback)

O robô publica continuamente a posição atualizada de todos os motores no tópico:

* **Tópico:** `current_motor_position`
* **Formato:** O mesmo formato de ID concatenado (`ID1000 + POS`).

Para visualizar a posição em tempo real:

```bash
ros2 topic echo /current_motor_position

```

---

## 🛠️ Configurações Padrão

* **Baud Rate Dynamixel:** 1.000.000 (1 Mbps).
* **Velocidade de Movimento:** 50 (Escala 1-1023).
* **IDs Monitorados:** 1, 2, 3, 4, 5, 6.

---
