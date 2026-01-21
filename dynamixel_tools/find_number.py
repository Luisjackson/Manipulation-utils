import pygame

# Inicializar o Pygame e o joystick
pygame.init()
pygame.joystick.init()

# Verificar se o joystick está disponível
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Joystick conectado.")
else:
    print("Nenhum joystick encontrado.")
    exit()

# Laço para capturar os eventos do joystick
try:
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:  # Quando um botão é pressionado
                print(f"Botão {event.button} pressionado")  # Exibe o número do botão pressionado

except KeyboardInterrupt:
    print("Programa interrompido.")
finally:
    pygame.quit()
