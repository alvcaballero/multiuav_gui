#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix,BatteryState
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Int16,Header
#bloque de generacion de trayectorias dentro de un cuadro limitado o un aplanificacion


def test_publisher():
    uav1_gps_pos_pub = rospy.Publisher('uav_1/mavros/global_position/global', NavSatFix, queue_size=10)
    uav2_gps_pos_pub = rospy.Publisher('uav_2/mavros/global_position/global', NavSatFix, queue_size=10)
    uav3_gps_pos_pub = rospy.Publisher('uav_3/dji_osdk_ros/rtk_position', NavSatFix, queue_size=10)
    uav3_rtk_yaw = rospy.Publisher('uav_3/dji_osdk_ros/rtk_yaw', Int16, queue_size=10)
    uav3_velocity = rospy.Publisher('uav_3/dji_osdk_ros/velocity', Vector3Stamped, queue_size=10)
    uav3_battery_state = rospy.Publisher('uav_3/dji_osdk_ros/battery_state', BatteryState, queue_size=10)


    rospy.init_node('test_publisher')

    while not rospy.is_shutdown():
        gps_position_1 = NavSatFix()
        gps_position_2 = NavSatFix()
        gps_position_3 = NavSatFix()
        batery = BatteryState()
        velocity  = Vector3Stamped()
        rtk_yaw =  16


        gps_position_1.latitude = 38.139503780177535
        gps_position_1.longitude = -3.1725619101288136
        gps_position_2.latitude = 38.13898578773057
        gps_position_2.longitude = -3.1747294071058
        gps_position_3.latitude = 38.138483757290835
        gps_position_3.longitude = -3.1766285043748184
        batery.percentage = 0.8 #charge percentage on 0 to 1 range TwistStamped
        velocity.header.seq = 1
        velocity.header.stamp.secs =30
        velocity.header.stamp.nsecs =12
        velocity.header.frame_id = 'a'
        velocity.vector.x = 3
        velocity.vector.y = 4
        velocity.vector.z = 5


        uav1_gps_pos_pub.publish(gps_position_1)
        uav2_gps_pos_pub.publish(gps_position_2)
        uav3_gps_pos_pub.publish(gps_position_3)
        uav3_rtk_yaw.publish(rtk_yaw)
        uav3_velocity.publish(velocity)
        uav3_battery_state.publish(batery)
        print("working")


        rospy.sleep(1)


if __name__ == '__main__':
    try:
        test_publisher()
    except rospy.ROSInterruptException:
        pass