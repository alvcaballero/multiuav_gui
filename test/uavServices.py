#! /usr/bin/env python
# the main module for ROS-python programs
import rospy
import time
import threading
import os

# we are creating a 'Trigger service'...
from std_srvs.srv import Trigger, TriggerResponse, SetBool, SetBoolResponse
from aerialcore_common.srv import ConfigMission, ConfigMissionResponse, finishMission, finishMissionResponse, finishGetFiles, finishGetFilesResponse
from dji_osdk_ros.srv import DownloadMedia, DownloadMediaResponse

# ...Other types are available, and you can create
# custom types


def Finish_Download(value):
    '''
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    time.sleep(30)
    rospy.wait_for_service('/GCS/FinishDownload')
    print("Finish download")
    valuex = value.replace(" ", "_").replace("-", "_").replace(":", "_")

    print(valuex)

    os.system(
        " mkdir -p /home/user/uav_media/mission_"+str(valuex)+" && cp /home/user/uav_media/mission/* /home/user/uav_media/mission_"+str(valuex)+"/")

    try:
        print("call GCS finish donwload files service")
        callservice = rospy.ServiceProxy(
            '/GCS/FinishDownload', finishGetFiles)
        resp1 = callservice('uav_14', True)
    except Exception as e:
        print("Service call failed: %s" % e)


def finish_Mission():
    '''
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    time.sleep(30)
    rospy.wait_for_service('/GCS/FinishMission')

    try:

        print("call GCS finish mission service")
        callservice = rospy.ServiceProxy(
            '/GCS/FinishMission', finishMission)
        resp1 = callservice('14', True)
    except Exception as e:
        print("Service call failed: %s" % e)


def SetBool_Response(request):
    '''
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    thr1 = threading.Thread(target=finish_Mission, args=(), kwargs={})
    thr1.start()

    print("Starting mission")
    return SetBoolResponse(
        success=True,
        message="Hey, roger that; we'll be right there!"
    )


def ConfigMission_Response(request):
    '''
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    print("Configuring mission")
    return ConfigMissionResponse(
        success=True,
    )


def DownloadMedia_Response(request):
    '''
    Callback function used by the service server to process
    requests from clients. It returns a TriggerResponse
    '''
    myInitDate = '2024-10-32 10:22'  # request.initDate
    mystr = ""
    print("Downloading media")
    print(request.downloadCnt)
    print(request.initDate)
    print(request.FinishDate)
    # myInitDate = request.initDate
    # for x in request.initDate:
    #    print(x)
    #    mystr += str(x)
    #    print('value' + mystr)

    thr = threading.Thread(target=Finish_Download,
                           args=(request.initDate,), kwargs={})
    thr.start()

    return DownloadMediaResponse(
        result=True,
    )


rospy.init_node('sos_service')                     # initialize a ROS node
srvConfigureMission = rospy.Service(                        # create a service, specifying its name,
    # type, and callback
    '/uav_14/dji_control/start_mission', SetBool, SetBool_Response
)

srvCommmandMission = rospy.Service(                        # create a service, specifying its name,
    # type, and callback
    '/uav_14/dji_control/configure_mission', ConfigMission, ConfigMission_Response
)

srvDownloadMedia = rospy.Service(                        # create a service, specifying its name,
    # type, and callback
    '/uav_14/camera_download_files', DownloadMedia, DownloadMedia_Response
)

rospy.spin()
