#!/usr/bin/env python3

import rospy
import gphoto2 as gp
from std_msgs.msg import String
from camera_control.msg import CameraControl
from camera_control.srv import request_camera_info, request_camera_infoResponse
from camera_control.srv import capture, captureResponse

#prompt user for deesired settings or use defaults

#List all the cameras connected to the cqameras
def get_connected_cameras():
    context = gp.Context()
    cameras = gp.Camera.autodetect(context)
    return cameras

#Get camera model and intrinscics (Doesn't work)
def get_camera_info(camera):
    camera.init()
    config = camera.get_config()

    #Get the camera model
    model = camera.get_summary()
    # model = config.get_child_by_name('cameramodel').get_value()

    #Get the camera intrinsics
    # intrinsics = config.get_child_by_name('cameraintrinsics').get_value()

    camera.exit()
    return model, intrinsics

def handle_request_camera_info(req):
    #Get the camera model and intrinsics
    cameras = get_connected_cameras
    response = captureResponse()
    for idx, (_, addr) in enumerate(cameras):
        camera = gp.Camera()
        image_path = f"(image_saving_directory)/image_{idx + 1}.jpg"
        capture_image(camera, image_path)
        response.image_paths.append(image_path)``
    return response

def handle_capture(req):
    #Get the camera model and intrinsics
    cameras = get_connected_cameras
    response = captureResponse()
    for idx, (_, addr) in enumerate(cameras):
        camera = gp.Camera()
        image_path = f"(image_saving_directory)/image_{idx + 1}.jpg"
        capture_image(camera, image_path)
        response.image_paths.append(image_path)
    return response

def capture_image(camera, image_path):
    #Capture the image
    camera.init()
    file_path = camera.capture(gp.GP_CAPTURE_IMAGE)
    camera_file = camera.file_get(file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
    camera_file.save(image_path)
    camera.exit()

def camera_control_server():
    rospy.init_node('camera_control_server')
    
    global image_saving_directory
    image_saving_directory = rospy.get_param('~image_saving_directory', '/path/to/default/directory')

    rospy.Service('request_camera_info', request_camera_info, handle_request_camera_info)
    rospy.Service('capture', capture, handle_capture)
    rospy

if __name__ == "__main__":
    camera_control_server()
    
