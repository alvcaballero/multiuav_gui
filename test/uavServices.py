#! /usr/bin/env python
# the main module for ROS-python programs
import rospy
import time
import threading
import os
import argparse
from enum import Enum
# ros services
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from aerialcore_common.srv import ConfigMission, ConfigMissionResponse, finishMission, finishMissionResponse, finishGetFiles, finishGetFilesResponse
from dji_osdk_ros.srv import DownloadMedia, DownloadMediaResponse
# ros messages
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, BatteryState
from geometry_msgs.msg import Vector3Stamped


class Status(Enum):
    READY = 1
    LOAD_MISSION = 2
    DO_MISSION = 3
    FINISH_MISSION = 4
    DONE_MISSION = 5


class SimpleDevice:
    def __init__(self, name):
        # device Variables
        self.name = name
        self.status = Status.READY
        self.mission = {}
        self.position = [0, 0, 0]
        self.yaw = 0
        self.gimbal = 0
        self.battery = 100
        self.speed = 0

        rospy.init_node('SS_'+ns)

        # init services

        self.srvStartMission = rospy.Service(
            '/'+ns+'/dji_control/start_mission', SetBool, self.startMission)

        self.srvSetMission = rospy.Service(
            '/'+ns+'/dji_control/configure_mission', ConfigMission, self.loadMission)

        self.srvDownloadMedia = rospy.Service(
            '/'+ns+'/camera_download_files', DownloadMedia, self.DownloadMedia)

        # init publishers
        self.pubPosition = rospy.Publisher(
            'user_position', NavSatFix, queue_size=10)
        self.pubYaw = rospy.Publisher('user_yaw', Float32, queue_size=10)
        self.pubGimbal = rospy.Publisher(
            'user_gimbal', Vector3Stamped, queue_size=10)
        self.pubBattery = rospy.Publisher(
            'user_battery', BatteryState, queue_size=10)
        self.rate = rospy.Rate(10)  # 1 Hz
        self.lock = threading.Lock()

    # change device variables

    def simulation(self):
        self.position = [self.position[0]+1,
                         self.position[1]+1, self.position[2]+1]
        self.yaw = self.yaw + 1
        self.gimbal = self.gimbal + 1
        self.battery = self.battery - 1

    def publish(self):
        while not rospy.is_shutdown():
            self.simulation()
            with self.lock:
                navsat = NavSatFix()
                navsat.header.stamp = rospy.Time.now()
                navsat.status.service = NavSatStatus.SERVICE_GPS
                navsat.latitude = self.position[0]
                navsat.longitude = self.position[1]
                navsat.altitude = 400 + self.position[2]
                self.pubPosition.publish(navsat)

                self.pubYaw.publish(self.yaw)

                gimbal = Vector3Stamped()
                gimbal.vector.x = 0
                gimbal.vector.y = 0
                gimbal.vector.z = self.gimbal
                self.pubGimbal.publish(self.gimbal)

                statusBattery = BatteryState()
                statusBattery.voltage = 24
                statusBattery.current = 1
                statusBattery.capacity = 100
                statusBattery.percentage = self.battery
                self.pubBattery.publish(self.battery)

                self.rate.sleep()

    def run(self):
        input_thread = threading.Thread(target=self.publish)
        input_thread.daemon = True  # Exit when the main program exits
        input_thread.start()
        rospy.spin()

    def Finish_Download(self, value):
        time.sleep(30)
        rospy.wait_for_service('/GCS/FinishDownload')
        print("Finish download")
        valuex = value.replace(" ", "_").replace("-", "_").replace(":", "_")

        print(valuex)

        os.system(
            " mkdir -p /home/user/uav_media/mission_"+str(valuex)+" && cp /home/user/uav_media/mission/* /home/user/uav_media/mission_"+str(valuex)+"/")

        try:
            print("call GCS finish donwload files service")
            time.sleep(30)
            callservice = rospy.ServiceProxy(
                '/GCS/FinishDownload', finishGetFiles)
            resp1 = callservice(self.name, True)
        except Exception as e:
            print("Service call failed: %s" % e)

    def finish_Mission(self):
        time.sleep(30)
        rospy.wait_for_service('/GCS/FinishMission')
        try:

            print("call GCS finish mission service")
            callservice = rospy.ServiceProxy(
                '/GCS/FinishMission', finishMission)
            resp1 = callservice('14', True)
        except Exception as e:
            print("Service call failed: %s" % e)

    def startMission(self, request):
        thr1 = threading.Thread(target=self.finish_Mission, args=(), kwargs={})
        thr1.start()

        print("Starting mission")
        return SetBoolResponse(
            success=True,
            message="Hey, roger that; we'll be right there!"
        )

    def loadMission(self, request):
        print("Configuring mission")
        print(request.waypoint)
        print(request.yaw)
        print(request.idleVel)
        return ConfigMissionResponse(
            success=True,
        )

    def DownloadMedia(self, request):
        print("Downloading media")
        print(request.downloadCnt)
        print(request.initDate)
        print(request.FinishDate)

        thr = threading.Thread(target=self.Finish_Download,
                               args=(request.initDate,), kwargs={})
        thr.start()

        return DownloadMediaResponse(
            result=True,
        )


if __name__ == "__main__":

    # manage the command line arguments
    parser = argparse.ArgumentParser(description='simple device simulator')
    parser.add_argument('-ns', metavar='Ros NameSpace',
                        help='namespace of the topic  Example: uav_2')
    args, unknown = parser.parse_known_args()
    default_ns = "uav_2"
    if args.ns:
        ns = args.ns
    else:
        ns = default_ns

    try:
        device = SimpleDevice(ns)
        device.run()
    except rospy.ROSInterruptException:
        pass
