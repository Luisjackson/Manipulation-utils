import pygame
import serial
import time

# Configurações
PORTA_SERIAL = "/dev/ttyACM0" 
BAUD_RATE = 115200

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick conectado: {joystick.get_name()}")
else:
    print("Erro: Nenhum joystick encontrado.")
    exit()

try:
    arduino = serial.Serial(PORTA_SERIAL, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"Conectado ao Arduino em {PORTA_SERIAL}")
except Exception as e:
    print(f"Erro de conexão: {e}")
    exit()

botoes_continuos = set()

print("\n--- Controle  Ativo ---")
print("Botões A, B, X, Y, LB, RB, START, SELECT: Clique único")
print("Analógicos 9 e 10: Segure para mover continuamente")
print("Pressione CTRL+C para sair.\n")

try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:

                if event.button == 0:   arduino.write(b'P'); print("Ação: Sequência PICK") #botao A
                elif event.button == 1: arduino.write(b'L'); print("Ação: Sequência PLACE") #botao B
                elif event.button == 2: arduino.write(b'C'); print("Ação: Fecha Garra") #botao X
                elif event.button == 3: arduino.write(b'O'); print("Ação: Abre Garra") #botao Y
                elif event.button == 4: arduino.write(b'B'); print("Ação: Pre-Pick") #botao LB
                elif event.button == 5: arduino.write(b'K'); print("Ação: Pick Position") #botao RB
                elif event.button == 6: arduino.write(b'R'); print("Ação: Pre home alto") #botao SELECT
                elif event.button == 7: arduino.write(b'H'); print("Ação: Home") #botao START
                elif event.button == 8: arduino.write(b'M'); print("Ação: Place Alto") 

                if event.button in [9, 10]:
                    botoes_continuos.add(event.button)

            elif event.type == pygame.JOYBUTTONUP:
                if event.button in [9, 10]:
                    botoes_continuos.discard(event.button)

        for botao in botoes_continuos:
            if botao == 9: 
                arduino.write(b'1')
            elif botao == 10: 
                arduino.write(b'2')

        time.sleep(0.05) 

except KeyboardInterrupt:
    print("\nEncerrando...")
finally:
    arduino.close()
    pygame.quit()