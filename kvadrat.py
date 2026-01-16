#!/usr/bin/env python3
import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('square_test')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

print("Взлетаем на 1 метр")
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)
rospy.sleep(5)

print("Летим вперед")
navigate(x=1, y=0, z=1, frame_id='body')
rospy.sleep(3)

print("Летим вправо")
navigate(x=1, y=1, z=1, frame_id='body')
rospy.sleep(3)

print("Летим назад")
navigate(x=0, y=1, z=1, frame_id='body')
rospy.sleep(3)

print("Летим влево")
navigate(x=0, y=0, z=1, frame_id='body')
rospy.sleep(3)

print("Садимся")
land()
rospy.sleep(3)

print("Готово")
