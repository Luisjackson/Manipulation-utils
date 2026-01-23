## 🛠️ Controle da Garra via Joystick

Este fluxo permite realizar algumas posições pré-definidas do braço 6DOF utilizando um controle IPEGA.

### 📋 Pré-requisitos

Antes de começar, instale as dependências necessárias no seu sistema:

```bash
# Instalar ferramentas de teste de joystick
sudo apt-get install jstest-gtk

# Instalar bibliotecas Python
pip install pygame pyserial

```
### 🕹️ Configuração do Joystick (IPEGA 9076)

1. **Ligar no modo correto:** Pressione simultaneamente as teclas **LB + SELECT + HOME**.
2. **Conexão:** Conecte o dongle ou pareie via Bluetooth ao computador.
3. **Validação:** Abra o terminal e rode `jstest-gtk` para garantir que o computador está reconhecendo os eixos e botões do controle.

### 🔌 Conexão do Hardware

1. Conecte a placa da garra via USB ao computador.
2. Carregue o arquivo `embarcado.ino` na placa utilizando a IDE do Arduino.
3. Conecte o braço robótico à sua fonte de energia externa.

### 🚀 Como executar

Com o hardware pronto e o joystick conectado, execute o script principal:

```bash
python3 control_joystick.py

```

### 🎮 Mapeamento de Comandos

| Botão | Ação | Comando Serial |
| --- | --- | --- |
| **Y** | Abre a garra | `O` |
| **X** | Fecha a garra | `C` |
| **A** | Executa sequência PICK | `P` |
| **B** | Executa sequência PLACE | `L` |
| **LB** | Move para PRE-PICK | `B` |
| **RB** | Move para PICK | `K` |
| **Start** | Posição HOME | `H` |
| **Select** | Posição READY | `R` |
| **Botão ON/OFF** | Posição PLACE ALTO | `M` |

---
