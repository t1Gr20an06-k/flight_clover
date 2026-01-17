#!/usr/bin/env python
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('simple_aruco_check')

# Подключаем сервисы
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
land = rospy.ServiceProxy('land', Trigger)

def check_aruco():
    """Просто проверяет, видит ли коптер ArUco"""
    try:
        # Пробуем получить координаты в системе aruco_map
        telemetry = get_telemetry(frame_id='aruco_map')
        
        # Если координаты не NaN и не нулевые - маркер виден
        if (not math.isnan(telemetry.x) and 
            not math.isnan(telemetry.y) and 
            not math.isnan(telemetry.z)):
            return True
        else:
            return False
    except:
        return False

def wait_aruco():
    """Ждет, пока появится ArUco"""
    print("Жду ArUco...")
    while not rospy.is_shutdown():
        if check_aruco():
            print("ArUco найден!")
            return True
        rospy.sleep(0.5)  # проверяем каждые 0.5 секунды
    return False

def goto_point(x, y, z):
    """Летит в точку с проверкой ArUco"""
    print(f"Летим в: X={x}, Y={y}")
    
    # Команда на полет
    navigate(x=x, y=y, z=z, frame_id='aruco_map')
    
    # Контролируем полет
    while not rospy.is_shutdown():
        # Проверяем ArUco
        if not check_aruco():
            print("Потерял ArUco! Останавливаюсь...")
            
            # Останавливаемся
            telemetry = get_telemetry(frame_id='aruco_map')
            navigate(x=telemetry.x, y=telemetry.y, z=telemetry.z, frame_id='aruco_map')
            
            # Ждем ArUco
            wait_aruco()
            
            # Продолжаем
            navigate(x=x, y=y, z=z, frame_id='aruco_map')
        
        # Проверяем, достигли ли точки
        telemetry = get_telemetry(frame_id='aruco_map')
        distance = math.sqrt((telemetry.x - x)**2 + (telemetry.y - y)**2)
        
        if distance < 0.1:  # достигли с точностью 10 см
            print("Точка достигнута!")
            return True
        
        rospy.sleep(0.5)  # проверяем каждые 0.5 секунды
    
    return False

# ===== ОСНОВНАЯ ПРОГРАММА =====

# 1. Взлет
print("Взлетаем...")
navigate(x=0, y=0, z=1.5, frame_id='body', auto_arm=True)
rospy.sleep(7)

# 2. Ждем ArUco после взлета
if not wait_aruco():
    print("Не вижу ArUco! Сажусь...")
    land()
    exit()

# 3. Список точек для посещения (в метрах)
points = [
    (0.5, 0, 1.5),
    (1.0, 0, 1.5),
    (1.0, 0.5, 1.5),
    (0.5, 0.5, 1.5),
    (0, 0, 1.5)  # возврат в начало
]

# 4. Облетаем все точки
for i, (x, y, z) in enumerate(points):
    print(f"\nТочка {i+1}/{len(points)}")
    
    if not goto_point(x, y, z):
        print("Не удалось достичь точки")
        break
    
    rospy.sleep(1)  # пауза на точке

# 5. Посадка
print("\nЗавершение, посадка...")
land()