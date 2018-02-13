#!/usr/bin/env python
import rospy
from tfpose_ros.msg import *
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
from image_geometry import PinholeCameraModel
from sensor_msgs import point_cloud2


HEAD_MARKERS = [0, 1, 2, 5, 15, 16, 17]

class HeadGetter:
    def __init__(self):
        roisSub = rospy.Subscriber('/pose_estimator/pose', Persons, self.persons_callback)
        imageSub = rospy.Subscriber('/kinect2/sd/image_color_rect', Image, self.kinectrgb_callback)
        depthSub = rospy.Subscriber('/kinect2/sd/image_depth_rect', Image, self.kinectdepth_callback)
        camSub = rospy.Subscriber('/kinect2/sd/camera_info', CameraInfo, self.kinectinfcallback)
        self.camInfo = None
        self.img_geo = PinholeCameraModel()
        self.rois = []
        self.image = None
        self.res = [640,320]
        self.box = None


    def get_depth_box(self):
        pass

    def get_roi(self, person):

        part_locs = [part for part in person.body_part if part.part_id in HEAD_MARKERS]
        minx = int(min([part_loc.x*self.res[0] for part_loc in part_locs]))
        maxx = int(max([part_loc.x*self.res[0] for part_loc in part_locs]))
        miny = int(min([part_loc.y*self.res[1] for part_loc in part_locs]))
        maxy = int(max([part_loc.y*self.res[1] for part_loc in part_locs]))
        dx = maxx - minx
        dy = maxy - miny
        box = {'min': (minx, miny - dy), 'max': (maxx, maxy)}
        return box

    def persons_callback(self, msg):
        persons = msg.persons
        try:
            self.res = [msg.image_w, msg.image_h]
            self.rois = [self.get_roi(person) for person in persons]
        except AttributeError:
            return None

    def kinectrgb_callback(self, msg):
        bridge = CvBridge()
        try:
            self.image = cv2.resize(bridge.imgmsg_to_cv2(msg, "bgr8"), (self.res[0], self.res[1]))
            if len(self.rois) > 0:
                for roi in self.rois:
                    cv2.rectangle(self.image, roi['min'], roi['max'], 255)
            cv2.imshow("Image window", self.image)
            cv2.waitKey(3)
        except CvBridgeError or TypeError as e:
            print(e)

    def kinectdepth_callback(self, msg):
        bridge = CvBridge()
        rectImage = bridge.imgmsg_to_cv2(msg)
        print rectImage

    def kinectinfcallback(self, msg):
        self.img_geo.fromCameraInfo(msg)


if __name__ == '__main__':
    # Initialize node
    hg = HeadGetter()
    rospy.init_node('head_getter')
    rospy.logout("head_getter node initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



