import pygame
import serial
import time

# Pygame 
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Nenhum joystick encontrado.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print("Joystick conectado:", joystick.get_name())

clock = pygame.time.Clock()

# Serial
arduino = serial.Serial("/dev/ttyACM0", 115200, timeout=0)
time.sleep(2)

# Config
DEADZONE = 0.25  # talvez precise mudar
ESCALA   = 6     # talvez precise mudar
FPS      = 15    # provavel de precisar mudar
MAX_STEP = 1

ultimo = {'U':0, 'V':0, 'W':0, 'Z':0}

# Funções
def map_analog(v):
    if abs(v) < DEADZONE:
        return 0
    return int(v * ESCALA)

def limit_step(old, new):
    if new > old:
        return min(old + MAX_STEP, new)
    if new < old:
        return max(old - MAX_STEP, new)
    return new

def enviar(cmd, valor=None):
    if valor is None:
        arduino.write(f"{cmd}\n".encode())
    else:
        arduino.write(f"{cmd}{valor}\n".encode())

# Loop
try:
    while True:
        clock.tick(FPS)
        pygame.event.pump()

        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 3:
                    enviar('Y')   # posição base
                elif event.button == 2:
                    enviar('X')   # abrir garra
                elif event.button == 1:
                    enviar('B')   # fechar garra

        eixos = {
            'U': joystick.get_axis(0),
            'V': joystick.get_axis(1),
            'W': joystick.get_axis(4),
            'Z': joystick.get_axis(3)
        }

        for cmd, val in eixos.items():
            desloc = map_analog(val)
            desloc = limit_step(ultimo[cmd], desloc)

            if desloc != ultimo[cmd]:
                ultimo[cmd] = desloc
                enviar(cmd, desloc)

except KeyboardInterrupt:
    pass
finally:
    arduino.close()
