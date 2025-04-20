from pymavlink import mavutil
import time


# Функція для отримання параметра та перевірки його типу
def get_param(name, timeout=5):
    name_bytes = name.encode('utf-8')
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name_bytes,
        -1  # index = -1, якщо шукаємо по імені
    )
    start_time = time.time()
    while time.time() - start_time < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg and msg.param_id.strip('\x00') == name:
            return msg.param_value, msg.param_type
    return None, None


# Функція для встановлення параметра з підтвердженням
def set_param(name, value, param_type, retries=10):
    type = 1;
    for i in range(retries):
        print(f"\033[94mAttempt {i + 1}: Setting {name} to {value} (type: {type})\033[0m")
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            value,
            type
        )
        time.sleep(1)
        
        confirmed_value, confirmed_type = get_param(name)
        
        if confirmed_value is not None:
            if abs(confirmed_value - value) < 0.01 and confirmed_type == type:
                print(f"\033[92m✅ {name} set successfully to {confirmed_value} (type: {type})\033[0m")
                return True
            else:
                print(f"\033[93m⚠️ Attempt {i + 1} failed, got {confirmed_value} (type: {confirmed_type})\033[0m")
        else:
            print(f"\033[91m❌ Unable to fetch {name} after setting.\033[0m")
        type = type + 1
        
    return False


# Підключення до PX4
serial_port = 'COM10'        # Заміни на свій порт
baud_rate = 115200
print(f"\033[94mConnecting to {serial_port} at {baud_rate} baud...\033[0m")
master = mavutil.mavlink_connection(serial_port, baud=baud_rate)
master.wait_heartbeat()
print(f"\033[92mHeartbeat received from system {master.target_system}, component {master.target_component}\033[0m")

# Параметри для встановлення
param_name = "SENS_GPS_EN"
param_value = 1.401298464324817e-45 # Встановлюємо значення (можна змінити на бажане)
param_type = 2  # Тип параметра

# Виклик функції для встановлення параметра
set_param(param_name, param_value, param_type)

# Перевірка значення після установки
gps_value, gps_type = get_param(param_name)
if gps_value is not None:
    print(f"\033[92m{param_name} = {gps_value} (type: {gps_type})\033[0m")
else:
    print(f"\033[91mFailed to fetch {param_name} after attempt.\033[0m")
