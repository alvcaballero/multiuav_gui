


-----------------------------
SetupCameraStream.srv
#constant for vga image frequency
uint8 FPV_CAM  = 0
uint8 MAIN_CAM = 1

# use above constants to config freq.
uint8 cameraType

# 1 for start camera stream, 0 for stop
uint8 start

---
bool result
--------------------------------
SetupCameraH264.srv

#constant for vga image frequency
uint8 FPV_CAMERA  = 7
uint8 MAIN_CAMERA = 0
uint8 VICE_CAMERA = 1
uint8 TOP_CAMERA  = 2

# use above constants to config freq.
uint8 request_view

# 1 for start camera stream, 0 for stop
uint8 start

---
bool result

dicen que hay que poner asi pero poniendo asi da error

rosservice call /setup_camera_stream "cameraType: 0 start: 1"
