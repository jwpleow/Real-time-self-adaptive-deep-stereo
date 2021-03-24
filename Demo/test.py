import numpy as np
import cv2
import os

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



cam= Arducam()
cam._connect_to_camera()
os.system("v4l2-ctl --set-ctrl=gain=5")
while(True):
    cam._read_frame()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
