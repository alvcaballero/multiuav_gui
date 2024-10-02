#! /usr/bin/env python
# the main module for ROS-python programs
import rospy
import time
import threading
import os
import argparse
from enum import Enum
import math
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
    RUNNING_MISSION = 3
    FINISH_MISSION = 4
    DONE_MISSION = 5


class SimpleDevice:
    def __init__(self, name):
        # device Variables
        self.name = name
        self.status = Status.READY
        self.mission = {}
        # latitude in degrees, longitude in degrees, altitude in meters take zero as home point
        self.homePoint = [37.193646, -6.702930, 0]
        self.position = self.homePoint
        self.yaw = 0
        self.gimbal = 0
        self.battery = 100
        self.speed = 5  # m/s
        self.currentWp = -1
        self.currentAction = 0
        self.rateTime = 2  # 1 Hz
        self.typeMission = "GPS"

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
            '/'+ns+'/dji_osdk_ros/gps_position', NavSatFix, queue_size=10)
        self.pubYaw = rospy.Publisher(
            '/'+ns+'/dji_osdk_ros/compass_heading', Float32, queue_size=10)
        self.pubGimbal = rospy.Publisher(
            '/'+ns+'/dji_osdk_ros/gimbal_angle', Vector3Stamped, queue_size=10)
        self.pubBattery = rospy.Publisher(
            '/'+ns+'/dji_osdk_ros/battery_state', BatteryState, queue_size=10)
        self.rate = rospy.Rate(self.rateTime)  # 1 Hz
        self.lock = threading.Lock()

    # change device variables
    def moveScalar(self, x1, x2, speed):
        dstX = (x2 - x1)
        # print("dstx:", dstX, "x1:", x1, "x2:", x2)
        direction = 1 if dstX > 0 else -1
        newDstX = speed * (1/self.rateTime) * direction
        if abs(newDstX) > abs(dstX) or dstX == 0:
            print("Arrived scalr")
            return {'state': True, 'value': x2}
        return {'state': False, 'value': x1 + newDstX}

    def moveToPoint(self, pos1, pos2, speed):
        dstX = (pos2[0] - pos1[0])
        dstY = (pos2[1] - pos1[1])
        if self.typeMission == "GPS":
            dstX = dstX*(6378137/180)*math.pi
            dstY = dstY*(6378137/180)*math.pi
        dstZ = (pos2[2] - pos1[2])
        dstXY = 0 if dstX == 0 and dstY == 0 else math.sqrt(
            math.pow(dstX, 2) + math.pow(dstY, 2))
        dstXYZ = math.sqrt(math.pow(dstXY, 2) + math.pow(dstZ, 2))
        # print("dstx:", dstX, dstY, dstZ, "dstXY:", dstXY, "dstXYZ:", dstXYZ)

        if dstXYZ == 0:
            print("Arrived wp")
            return {'state': True, 'pos': pos2}

        velZ = (speed * dstZ) / dstXYZ
        velXY = (speed * dstXY) / dstXYZ

        velX = 0 if dstXY == 0 else (velXY * dstX) / dstXY
        velY = 0 if dstXY == 0 else (velXY * dstY) / dstXY
        # print("vel:", velX, velY, velZ)

        newDstX = velX * (1/self.rateTime)
        newDstY = velY * (1/self.rateTime)
        newDstZ = velZ * (1/self.rateTime)

        newPosX = pos1[0] + newDstX
        newPosY = pos1[1] + newDstY
        newPosZ = pos1[2] + newDstZ
        if self.typeMission == "GPS":
            newPosX = pos1[0]+(newDstX*(180/6378137))/math.pi
            newPosY = pos1[1]+(newDstY*(180/6378137))/math.pi

        # print("newDst:", newDstX, newDstY, newDstZ)
        if abs(newDstX) > abs(dstX) or abs(newDstY) > abs(dstY) or abs(newDstZ) > abs(dstZ):
            print("Arrived wp")
            return {'state': True, 'pos': pos2}
        return {'state': False, 'pos': [newPosX, newPosY, newPosZ]}

    def simulation(self):
        if self.status == Status.RUNNING_MISSION:
            if self.currentWp < 0:
                calculate = self.moveToPoint(self.position, [
                    self.homePoint[0], self.homePoint[1], self.homePoint[2]+5], self.speed)
                self.position = calculate['pos']
                if calculate['state']:
                    self.currentWp = 0
            elif self.currentWp < len(self.mission.waypoint):
                calculate = self.moveToPoint(self.position, [
                    self.mission.waypoint[self.currentWp].latitude, self.mission.waypoint[self.currentWp].longitude, self.mission.waypoint[self.currentWp].altitude], self.speed)
                self.position = calculate['pos']

                if calculate['state']:

                    print("action", self.currentAction, self.mission.commandList.data[self.currentWp*10 +
                          self.currentAction], self.mission.commandParameter.data[self.currentWp*10+self.currentAction])

                    if self.mission.commandList.data[self.currentWp*10+self.currentAction] == 4:
                        print(
                            "action yaw", self.mission.commandParameter.data[self.currentWp*10+self.currentAction])
                        actionCalculate = self.moveScalar(
                            self.yaw, self.mission.commandParameter.data[self.currentWp*10+self.currentAction], 10)
                        self.yaw = actionCalculate['value']
                        if actionCalculate['state']:
                            self.currentAction = self.currentAction + 1
                    elif self.mission.commandList.data[self.currentWp*10+self.currentAction] == 5:
                        print(
                            "action gimbal", self.mission.commandParameter.data[self.currentWp*10+self.currentAction])
                        actionCalculate = self.moveScalar(
                            self.gimbal, self.mission.commandParameter.data[self.currentWp*10+self.currentAction], 10)
                        self.gimbal = actionCalculate['value']
                        if actionCalculate['state']:
                            self.currentAction = self.currentAction + 1
                    elif self.mission.commandList.data[self.currentWp*10+self.currentAction] == 1 or self.mission.commandList.data[self.currentWp*10+self.currentAction] == 2 or self.mission.commandList.data[self.currentWp*10+self.currentAction] == 3:
                        print("action photo or video")
                        self.currentAction = self.currentAction + 1
                    elif self.mission.commandList.data[self.currentWp*10+self.currentAction] > 0:
                        print("action other command")
                        self.currentAction = self.currentAction + 1
                    else:
                        for i in range(self.currentAction, 10):
                            print("action-bucle", i, self.mission.commandList.data[self.currentWp*10 +
                                                                                   i], self.mission.commandParameter.data[self.currentWp*10+i])

                            if self.mission.commandList.data[self.currentWp*10+i] != 0:
                                self.currentAction = i
                                break
                            if i == 9:
                                self.currentAction = 10
                            print("action zero")

                    if self.currentAction == 10:
                        self.currentAction = 0
                        self.currentWp = self.currentWp + 1

            else:
                calculate = self.moveToPoint(self.position, [
                    self.position[0], self.position[1], self.homePoint[2]], self.speed)
                self.position = calculate['pos']
                if calculate['state']:
                    self.currentWp = -1
                    self.status = Status.FINISH_MISSION
                    time.sleep(2)
                    self.finish_Mission()

            self.battery = self.battery - 0.5
            if self.battery < 5:
                self.battery = 100

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

                gimbalmsg = Vector3Stamped()
                gimbalmsg.header.stamp = rospy.Time.now()
                gimbalmsg.vector.x = 0
                gimbalmsg.vector.y = 0
                gimbalmsg.vector.z = self.gimbal
                self.pubGimbal.publish(gimbalmsg)

                statusBattery = BatteryState()
                statusBattery.voltage = 24
                statusBattery.current = 1
                statusBattery.capacity = 100
                statusBattery.percentage = self.battery
                self.pubBattery.publish(statusBattery)

                self.rate.sleep()

    def run(self):
        input_thread = threading.Thread(target=self.publish)
        input_thread.daemon = True  # Exit when the main program exits
        input_thread.start()
        rospy.spin()

    def Finish_Download(self, value):
        time.sleep(10)
        rospy.wait_for_service('/GCS/FinishDownload')
        print("Finish download")
        valuex = value.replace(" ", "_").replace("-", "_").replace(":", "_")

        print(valuex)

        os.system(
            " mkdir -p /home/user/uav_media/mission_"+str(valuex)+" && cp /home/user/uav_media/mission/* /home/user/uav_media/mission_"+str(valuex)+"/")

        try:
            print("call GCS finish donwload files service")
            time.sleep(15)
            callservice = rospy.ServiceProxy(
                '/GCS/FinishDownload', finishGetFiles)
            resp1 = callservice(self.name, True)
            self.status = Status.READY
        except Exception as e:
            print("Service call failed: %s" % e)

    def finish_Mission(self):
        # time.sleep(30)
        rospy.wait_for_service('/GCS/FinishMission')
        try:
            print("call GCS finish mission service")
            callservice = rospy.ServiceProxy(
                '/GCS/FinishMission', finishMission)
            resp1 = callservice('14', True)
        except Exception as e:
            print("Service call failed: %s" % e)

    def startMission(self, request):
        print("Starting mission")
        rspSuccess = False
        rspmMessage = "Hey, roger that; we'll be right there!"

        if self.status == Status.LOAD_MISSION:
            self.status = Status.RUNNING_MISSION
            rspSuccess = True

        return SetBoolResponse(
            success=rspSuccess,
            message=rspmMessage
        )

    def loadMission(self, request):
        print("load mission")
        rspSuccess = False
        if self.status == Status.READY or self.status == Status.LOAD_MISSION:
            self.status = Status.LOAD_MISSION
            self.homePoint = self.position
            self.mission = request
            rspSuccess = True
        return ConfigMissionResponse(
            success=rspSuccess,
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
    default_ns = "uav_14"
    if args.ns:
        ns = args.ns
    else:
        ns = default_ns

    try:
        device = SimpleDevice(ns)
        device.run()
    except rospy.ROSInterruptException:
        pass
