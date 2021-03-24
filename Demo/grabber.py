import threading
import time
import numpy as np
import abc
import json
import cv2
import os

############################################################
###########          CAMERA FACTORY              ###########
###########################################################

_GRABBER_FACTORY={}

def get_camera(name, frame_queue, config=None, framerate=30):
    """
    factory method to construct a camera wrapper
    """
    if name not in _GRABBER_FACTORY:
        raise Exception('Unrecognized camera type: {}'.format(name))
    else:
        return _GRABBER_FACTORY[name](frame_queue, config=config, framerate=framerate)

def get_available_camera():
    return _GRABBER_FACTORY.keys()

def register_camera_to_factory():
    def decorator(cls):
        _GRABBER_FACTORY[cls._name]=cls
        return cls
    return decorator

#####################################################################
##############           ABSTRACT CAMERA                #############
####################################################################


class ImageGrabber(threading.Thread):
    """
    Thread to grab frames from the camera and load them in frame_queue
    """
    __metaclass__=abc.ABCMeta
    def __init__(self, frame_queue, config=None, framerate=30):
        """
        Args
        -----
        frame_queue: queue
            synchronized queue where left and right frames will be loaded
        config: path
            path to json file for calibration and/or other configuration parameters
        framerate: int
            target frame per second
        """
        threading.Thread.__init__(self)
        self._config = config
        self._connect_to_camera()
        self._buffer = frame_queue
        self._sleeptime = 1/framerate
        self._stop_acquire=False
    
    def stop(self):
        """
        Stop the acquisition of new frames from the camera and kill the thread
        """
        self._stop_acquire=True
    
    def run(self):
        """
        Main body method, grab frames from camera and put them on buffer as a [2,h,w,c] numpy array
        """
        while not self._stop_acquire:
            l,r = self._read_frame()
            self._buffer.put(np.stack([l,r],axis=0))
            time.sleep(self._sleeptime)
        
        self._disconnect_from_camera()

    @abc.abstractmethod
    def _read_frame(self):
        """
        Read left and right rectified frame and return them
        """
    
    @abc.abstractmethod
    def _connect_to_camera(self):
        """
        Connect to external camera
        """
    
    @abc.abstractmethod
    def _disconnect_from_camera(self):
        """
        Disconnect from external camera
        """


#########################################################################
#################           ZED MINI                    #################
#########################################################################

# import pyzed.sl as sl

# @register_camera_to_factory()
# class ZEDMini(ImageGrabber):
#     _name = 'ZED_Mini'
#     _key_to_res = {
#         '2K' : sl.RESOLUTION.RESOLUTION_HD2K,
#         '1080p' : sl.RESOLUTION.RESOLUTION_HD1080,
#         '720p' : sl.RESOLUTION.RESOLUTION_HD720,
#         'VGA' : sl.RESOLUTION.RESOLUTION_VGA
#     }

#     """ Read Stereo frames from a ZED Mini stereo camera. """
#     def _read_frame(self):
#         err = self._cam.grab(self._runtime)
#         if err == sl.ERROR_CODE.SUCCESS:
#             self._cam.retrieve_image(self._left_frame, sl.VIEW.VIEW_LEFT)
#             self._cam.retrieve_image(self._right_frame, sl.VIEW.VIEW_RIGHT)
#             return self._left_frame.get_data()[:,:,:3], self._right_frame.get_data()[:,:,:3]
    
#     def _connect_to_camera(self):
#         # road option from config file
#         with open(self._config) as f_in:
#             self._config = json.load(f_in)

#         self._params = sl.InitParameters()
        
#         if 'resolution' in self._config:
#             self._params.camera_resolution = self._key_to_res[self._config['resolution']]
#         else:
#             self._params.camera_resolution = sl.RESOLUTION.RESOLUTION_HD720
        
#         if 'fps' in self._config:
#             self._params.camera_fps = self._config['fps']
#         else:
#             self._params.camera_fps = 30
        
#         self._cam = sl.Camera()
#         status = self._cam.open(self._params)
#         if status != sl.ERROR_CODE.SUCCESS:
#             print(status)
#             raise Exception('Unable to connect to Stereo Camera')
#         self._runtime = sl.RuntimeParameters()
#         self._left_frame = sl.Mat()
#         self._right_frame = sl.Mat()

#     def _disconnect_from_camera(self):
#         self._cam.close()        


#########################################################################
#################           SMATT CAM                   #################
#########################################################################

#Example of frame grabber for a custom camera

# from stereocam import StereoCamera 

# @register_camera_to_factory()
# class SmattCam(ImageGrabber):
#     _name='SmattCam'
#     """
#     Read frames from smart camera from Mattoccia et al.
#     """
#     def _read_frame(self):
#         left,right =  self._cam.grab_frames()
#         left = np.repeat(left, 3, axis=-1)
#         right = np.repeat(right, 3, axis=-1)
#         return left,right
    
#     def _connect_to_camera(self):
#         self._cam = StereoCamera(self._config)
#         self._cam.calibrate()

    
#     def _disconnect_from_camera(self):
#         pass



### arducam
@register_camera_to_factory()
class Arducam():
    _name = 'Arducam'

    def _read_frame(self):

        ret = False
        while ret == False:
            ret, image = self._cam.read()
        # split
        left = image[0:self.image_size_[1], 0:self.image_size_[0]]
        right = image[0:self.image_size_[1],
                      self.image_size_[0]:2*self.image_size_[0]]
        Uleft = cv2.remap(left, self.left_map_1_,
                          self.left_map_2_, cv2.INTER_LINEAR)
        Uright = cv2.remap(right, self.right_map_1_,
                           self.right_map_2_, cv2.INTER_LINEAR)
        Uleft = Uleft[self.matchedRoi1_[1]: self.matchedRoi1_[
            1] + self.matchedRoi1_[3], self.matchedRoi1_[0]:self.matchedRoi1_[0]+self.matchedRoi1_[2]]
        Uright = Uright[self.matchedRoi2_[1]: self.matchedRoi2_[
            1] + self.matchedRoi2_[3], self.matchedRoi2_[0]:self.matchedRoi2_[0]+self.matchedRoi2_[2]]
        cv2.imshow("testleft", Uleft)
        cv2.imshow("testR", Uright)
        # left = np.repeat(left, 3, axis=-1)
        # right = np.repeat(right, 3, axis=-1)
        return Uleft, Uright

    def _connect_to_camera(self):
        # import params
        self._import_params()
        # calculate rectification matrices and maps for undistorting
        self.R1_, self.R2_, self.P1_, self.P2_, self.Q_, self.validRoi1_, self.validRoi2_ = cv2.stereoRectify(
            self.left_camera_matrix_, self.left_distortion_coefficients_, self.right_camera_matrix_, self.right_distortion_coefficients_, self.image_size_, self.R_, self.T_)
        self.left_map_1_, self.left_map_2_ = cv2.initUndistortRectifyMap(
            self.left_camera_matrix_, self.left_distortion_coefficients_, self.R1_, self.P1_, self.image_size_, cv2.CV_16SC2)
        self.right_map_1_, self.right_map_2_ = cv2.initUndistortRectifyMap(
            self.right_camera_matrix_, self.right_distortion_coefficients_, self.R2_, self.P2_, self.image_size_, cv2.CV_16SC2)

        # connect to cam
        self._cam = cv2.VideoCapture(0, cv2.CAP_GSTREAMER)
        self._cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_size_[0] * 2)
        self._cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_size_[1])
        os.system("v4l2-ctl --set-ctrl=gain=5")

        self._calculate_matched_roi()

    def _import_params(self):
        fs = cv2.FileStorage("CalibParams_Stereo_4cm.yml",
                             cv2.FILE_STORAGE_READ)
        if (fs.isOpened()):
            self.left_camera_matrix_ = fs.getNode("left_camera_matrix").mat()
            self.left_distortion_coefficients_ = fs.getNode(
                "left_distortion_coefficients").mat()

            self.right_camera_matrix_ = fs.getNode("right_camera_matrix").mat()
            self.right_distortion_coefficients_ = fs.getNode(
                "right_distortion_coefficients").mat()
            self.R_ = fs.getNode("R").mat()
            self.T_ = fs.getNode("T").mat()
            self.E_ = fs.getNode("E").mat()
            self.F_ = fs.getNode("F").mat()
            self.image_size_ = (int(fs.getNode("image_width").real()), int(
                fs.getNode("image_height").real()))
            fs.release()
        else:
            print("calibration file could not be opened")

    def _calculate_matched_roi(self):
        new_y_loc = max(self.validRoi1_[1], self.validRoi2_[1])
        new_x_loc_right = max(self.validRoi2_[0], self.image_size_[
                              0] - self.validRoi1_[0] - self.validRoi1_[2])
        new_height = min(self.validRoi1_[3] - (new_y_loc - self.validRoi1_[
                         1]), self.validRoi2_[3] - (new_y_loc - self.validRoi2_[1]))
        new_width = min(min((self.validRoi1_[0] + self.validRoi1_[2]) - new_x_loc_right, self.validRoi2_[
                        2]), self.image_size_[0] - self.validRoi2_[0] - new_x_loc_right)

        self.matchedRoi1_ = (self.image_size_[
                             0] - new_x_loc_right - new_width, new_y_loc, new_width, new_height)
        self.matchedRoi2_ = (new_x_loc_right, new_y_loc, new_width, new_height)

    def _disconnect_from_camera(self):
        self._cam.release()
