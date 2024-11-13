#! /usr/bin/env python
# the main module for ROS-python programs
import rospy
import time
import threading
import os
from enum import Enum
import math
# ros services
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from aerialcore_common.srv import ConfigMission, ConfigMissionResponse, finishMission, finishMissionResponse, finishGetFiles, finishGetFilesResponse
from dji_osdk_ros.srv import DownloadMedia, DownloadMediaResponse
# ros messages
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu, BatteryState
from geometry_msgs.msg import Vector3Stamped, TwistStamped


class Status(Enum):
    READY = 1
    LOAD_MISSION = 2
    RUNNING_MISSION = 3
    FINISH_MISSION = 4
    DONE_MISSION = 5


dictCategory = {
    "dji_M300": {
        "position": "/dji_osdk_ros/gps_position",
                    "yaw": "/dji_osdk_ros/compass_heading",
                    "gimbal": "/dji_osdk_ros/gimbal_angle",
                    "battery": "/dji_osdk_ros/battery_state",
                    "speed": "/dji_osdk_ros/velocity",
    },
    "dji_M600": {
        "position": "/dji_sdk/gps_position",
                    "yaw": "/dji_sdk/compass_heading",
                    "battery": "/dji_sdk/battery_state",
                    "speed": "/dji_sdk/velocity",
    },
    "px4": {
        "position": "/mavros/global_position/global",
                    "yaw": "/mavros/global_position/compass_hdg",
                    "battery": "/mavros/battery",
                    "speed": "/mavros/local_position/velocity_local",
    },
}


class SimpleDevice:
    def __init__(self, name, homePoint=[37.193646, -6.702930, 0], category="dji_M300"):
        print("init-----"+name+'------'+str(homePoint)+"----"+category)
        # device Variables
        self.name = name
        self.status = Status.READY
        self.mission = {}
        # latitude in degrees, longitude in degrees, altitude in meters take zero as home point
        self.homePoint = homePoint
        self.position = self.homePoint
        self.yaw = 0
        self.gimbal = 0
        self.speedX = 0
        self.speedY = 0
        self.speedZ = 0
        self.speedIdle = 5  # m/s
        self.battery = 100
        if category == "px4":
            self.speedIdle = 10  # m/s
            self.battery = 1
        self.currentWp = -1
        self.currentAction = 0
        self.rateTime = 2  # 1 Hz
        self.typeMission = "GPS"
        self.category = category

        # init services
        if self.category == "px4":
            self.srvStartMission = rospy.Service(
                '/'+self.name+'/mission/new', ConfigMission, self.loadMission)
            self.srvSetMission = rospy.Service(
                '/'+self.name+'/mission/start_stop', SetBool,  self.startMission)
        else:
            self.srvStartMission = rospy.Service(
                '/'+self.name+'/dji_control/start_mission', SetBool, self.startMission)

            self.srvSetMission = rospy.Service(
                '/'+self.name+'/dji_control/configure_mission', ConfigMission, self.loadMission)

        self.srvDownloadMedia = rospy.Service(
            '/'+self.name+'/camera_download_files', DownloadMedia, self.DownloadMedia)

        # init publishers
        self.pubPosition = rospy.Publisher(
            '/'+self.name+dictCategory[self.category]["position"], NavSatFix, queue_size=10)

        if self.category == "px4":
            self.pubYaw = rospy.Publisher(
                '/'+self.name+dictCategory[self.category]["yaw"], Float64, queue_size=10)
            self.pubSpeed = rospy.Publisher(
                '/'+self.name+dictCategory[self.category]["speed"], TwistStamped, queue_size=10)
        else:
            self.pubYaw = rospy.Publisher(
                '/'+self.name+dictCategory[self.category]["yaw"], Float32, queue_size=10)
            self.pubSpeed = rospy.Publisher(
                '/'+self.name+dictCategory[self.category]["speed"],  Vector3Stamped, queue_size=10)

        if dictCategory[self.category].get("gimbal"):
            self.pubGimbal = rospy.Publisher(
                '/'+self.name+dictCategory[self.category]["gimbal"], Vector3Stamped, queue_size=10)

        self.pubBattery = rospy.Publisher(
            '/'+self.name+dictCategory[self.category]["battery"], BatteryState, queue_size=10)
        self.rate = rospy.Rate(self.rateTime)  # 1 Hz
        self.lock = threading.Lock()

    # change device variables
    def moveAngle(self, x1, x2, speed):
        dstX = (x2 - x1)
        # print("dstx:", dstX, "x1:", x1, "x2:", x2)
        direction = 1 if dstX > 0 else -1
        direction = direction if abs(dstX) < 180 else -direction
        newDstX = speed * (1/self.rateTime) * direction
        angle = x1 + newDstX
        if angle > 180:
            angle = angle - 360
        if angle < -180:
            angle = angle + 360
        if abs(newDstX) > abs(dstX) or dstX == 0:
            print("Arrived scalr")
            return {'state': True, 'value': x2}
        return {'state': False, 'value': angle}

    def moveToPoint(self, pos1, pos2, speed):
        dstX = (pos2[0] - pos1[0])
        dstY = (pos2[1] - pos1[1])
        dstZ = (pos2[2] - pos1[2])
        print("dstx:", dstX, dstY, dstZ, "speed:", speed)
        if self.typeMission == "GPS":
            dstX = dstX*(6378137/180)*math.pi
            dstY = dstY*(6378137/180)*math.pi
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
        print("vel:", velX, velY, velZ)
        self.speedX = velX
        self.speedY = velY
        self.speedZ = velZ

        newDstX = velX * (1/self.rateTime)
        newDstY = velY * (1/self.rateTime)
        newDstZ = velZ * (1/self.rateTime)

        newPosX = pos1[0] + newDstX
        newPosY = pos1[1] + newDstY
        newPosZ = pos1[2] + newDstZ
        if self.typeMission == "GPS":
            newPosX = pos1[0]+(newDstX*(180/6378137))/math.pi
            newPosY = pos1[1]+(newDstY*(180/6378137))/math.pi

        print("newDst:", newDstX, newDstY, newDstZ)
        if abs(newDstX) > abs(dstX) or abs(newDstY) > abs(dstY) or abs(newDstZ) > abs(dstZ):
            print("Arrived wp")
            return {'state': True, 'pos': pos2}
        return {'state': False, 'pos': [newPosX, newPosY, newPosZ]}

    def calAngle(self, x1, y1, x2, y2):
        xdiff = x2 - x1
        ydiff = y2 - y1
        angle = 0
        if xdiff == 0:
            if ydiff > 0:
                angle = 0
            else:
                angle = math.pi
        else:
            angle = math.atan2(y2-y1, x2-x1)
            print("angle1", math.degrees(angle))
            angle = -angle + math.pi/2
        print("angle", math.degrees(angle))
        return math.degrees(angle)

    def simulation(self):
        if self.status == Status.RUNNING_MISSION:
            print("waypoint", self.currentWp, "action", self.currentAction)
            if self.currentWp < 0:
                calculate = self.moveToPoint(self.position, [
                    self.homePoint[0], self.homePoint[1], self.homePoint[2]+5], self.speedIdle)
                self.position = calculate['pos']
                if calculate['state']:
                    self.currentWp = 0
            elif self.currentWp < len(self.mission.waypoint):
                calculate = self.moveToPoint(self.position, [
                    self.mission.waypoint[self.currentWp].latitude, self.mission.waypoint[self.currentWp].longitude, self.mission.waypoint[self.currentWp].altitude], self.speedIdle)
                print("calculate", calculate)
                if category == "px4":
                    newyaw = self.moveAngle(
                        self.yaw, self.calAngle(self.position[0], self.position[1], self.mission.waypoint[self.currentWp].longitude, self.mission.waypoint[self.currentWp].latitude), 10)['value']

                    print("yaw", self.yaw, newyaw)
                    self.yaw = newyaw

                if calculate['state']:

                    print("action", self.currentAction, self.mission.commandList.data[self.currentWp*10 +
                          self.currentAction], self.mission.commandParameter.data[self.currentWp*10+self.currentAction])

                    if self.mission.commandList.data[self.currentWp*10+self.currentAction] == 4:
                        print(
                            "action yaw", self.mission.commandParameter.data[self.currentWp*10+self.currentAction])
                        actionCalculate = self.moveAngle(
                            self.yaw, self.mission.commandParameter.data[self.currentWp*10+self.currentAction], 10)
                        self.yaw = actionCalculate['value']
                        if actionCalculate['state']:
                            self.currentAction = self.currentAction + 1
                    elif self.mission.commandList.data[self.currentWp*10+self.currentAction] == 5:
                        print(
                            "action gimbal", self.mission.commandParameter.data[self.currentWp*10+self.currentAction])
                        actionCalculate = self.moveAngle(
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
                    self.position[0], self.position[1], self.homePoint[2]], self.speedIdle)
                self.position = calculate['pos']
                if calculate['state']:
                    self.currentWp = -1
                    self.status = Status.FINISH_MISSION
                    time.sleep(2)
                    self.finish_Mission()
            if self.category == "px4":
                self.battery = self.battery - 0.0001
                if self.battery < 0.05:
                    self.battery = 1
            else:
                self.battery = self.battery - 0.01
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

                if dictCategory[self.category].get("gimbal"):
                    gimbalmsg = Vector3Stamped()
                    gimbalmsg.header.stamp = rospy.Time.now()
                    gimbalmsg.vector.x = 0
                    gimbalmsg.vector.y = 0
                    gimbalmsg.vector.z = self.gimbal
                    self.pubGimbal.publish(gimbalmsg)

                if self.category == "px4":
                    speedmsg = TwistStamped()
                    speedmsg.header.stamp = rospy.Time.now()
                    speedmsg.twist.linear.x = self.speedX
                    speedmsg.twist.linear.y = self.speedY
                    speedmsg.twist.linear.z = self.speedZ
                    self.pubSpeed.publish(speedmsg)
                else:
                    speedmsg = Vector3Stamped()
                    speedmsg.header.stamp = rospy.Time.now()
                    speedmsg.vector.x = self.speedX
                    speedmsg.vector.y = self.speedY
                    speedmsg.vector.z = self.speedZ
                    self.pubSpeed.publish(speedmsg)

                statusBattery = BatteryState()
                statusBattery.voltage = 24
                statusBattery.current = 1
                statusBattery.capacity = 100
                statusBattery.percentage = round(self.battery, 2)
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
            resp1 = callservice(self.name, True)
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
            self.speedIdle = self.mission.idleVel

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

    ns = "uav_14"
    homePoint = [37.193646, -6.702930, 0]
    category = "dji_M300"

    rospy.init_node('simulateDevice')

    if rospy.has_param('~ns'):
        ns = rospy.get_param('~ns')
    if rospy.has_param('~lat') and rospy.has_param('~lon'):
        homePoint = [float(rospy.get_param('~lat')),
                     float(rospy.get_param('~lon')), 0]
    if rospy.has_param('~category'):
        category = rospy.get_param('~category')

    try:
        device = SimpleDevice(ns, homePoint, category)
        device.run()
    except rospy.ROSInterruptException:
        pass
