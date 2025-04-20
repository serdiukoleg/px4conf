from pymavlink import mavutil
import time

# Підключення до системи
serial_port = 'COM10'  # Заміни на свій порт
baud_rate = 115200
master = mavutil.mavlink_connection(serial_port, baud=baud_rate)
master.wait_heartbeat()
print("Heartbeat received from system")

# Функція для розблокування (arm) двигунів
def arm_drone():
    print("Arming the drone (starting engines)...")
    # MAV_CMD_COMPONENT_ARM_DISARM - команда для армії/дисарму
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # команда для армії
        0,  # Confirmation (0 = no confirmation)
        1,  # Армування (1 = arm)
        0, 0, 0, 0, 0, 0  # Додаткові параметри
    )
    time.sleep(1)
    print("Drone armed. Engines should now be running.")

# Функція для розблокування (disarm) двигунів
def disarm_drone():
    print("Disarming the drone (stopping engines)...")
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # команда для армії
        0,  # Confirmation (0 = no confirmation)
        0,  # Дисармуємо (0 = disarm)
        0, 0, 0, 0, 0, 0  # Додаткові параметри
    )
    time.sleep(1)
    print("Drone disarmed. Engines should now be stopped.")

def send_text_message(message):
    print(f"Sending message: {message}")
    master.mav.statustext_send(
        mavutil.mavlink.MAV_SEVERITY_NOTICE,  # Тип повідомлення
        message.encode('utf-8')
    )
    time.sleep(1)
    print("Message sent!")

# Запуск двигунів (arm)
arm_drone()

# Затримка 10 секунд, а потім дисарм
time.sleep(10)

# Відключення двигунів (disarm)
disarm_drone()

send_text_message("Engine start initiated. All systems are normal.")
