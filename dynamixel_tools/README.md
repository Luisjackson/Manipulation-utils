## ⚙️ Utilidades Dynamixel

Estes scripts são ferramentas de baixo nível para configuração e diagnóstico dos motores Dynamixel do braço.

### 📋 Scripts Disponíveis

1. **`scan_dynamixel.ino`**:
* **Função:** Varre o barramento serial para encontrar todos os IDs conectados.
* **Uso:** Útil para verificar se todos os motores estão sendo reconhecidos ou se há mau contato nos cabos.

2. **`set_id.py`**:
* **Função:** Altera o ID de um motor específico.
* **Uso:** Utilize este script ao substituir um motor novo (que geralmente vem com ID 1 de fábrica) para o ID correspondente à sua posição no braço.

3. **`reset_factory.py`**:
* **Função:** Retorna o motor para as configurações de fábrica (Baud rate 57600, ID 1).
* **Uso:** Use em casos de erro crítico de EEPROM ou quando você perder a comunicação com o motor por configurações de Baud rate incompatíveis.



### ⚠️ Avisos Importantes

* **Conexão Única:** Ao rodar o `set_id.py` ou `reset_factory.py`, certifique-se de que **apenas um motor** esteja conectado à controladora para evitar conflitos de ID ou reset acidental de todos os motores simultaneamente.
* **Permissões:** Garanta que seu usuário tenha permissão de acesso à porta serial:
```bash
sudo usermod -a -G dialout $USER

