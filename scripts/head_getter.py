#!/usr/bin/env python
import rospy
import time
from tfpose_ros.msg import *
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import sensor_msgs
import cv2


HEAD_MARKERS = [0, 1, 2, 5, 15, 16, 17]

class HeadGetter:
    def __init__(self):
        roisSub = rospy.Subscriber('/pose_estimator/pose', Persons, self.persons_callback)
        imageSub = rospy.Subscriber('/kinect2/sd/image_color_rect', Image, self.kinectrgb_callback)
        depthSub = rospy.Subscriber('/kinect2/sd/image_depth_rect', Image, self.kinectdepth_callback)
        caminfoSub = rospy.Subscriber('/kinect2/sd/camera_info', CameraInfo, self.camerainfocallback)
        self.headPub = rospy.Publisher("/head_getter/persons_denorm", Persons)
        self.rois = []
        self.caminfo = None
        self.image = None
        self.res = [640,320]
        self.box = None
        self.part_locs = []
        self.persons = []


    def get_3droi(self, pCloud):
        for person in self.persons.persons:
            for part in person.body_part:
                part.x = int(part.x * self.res[0]) if part.x < 1.0 else part.x
                part.y = int(part.y * self.res[1]) if part.y < 1.0 else part.y
        self.headPub.publish(self.persons)
        print self.persons


    def get_2droi(self, person):
        self.part_locs = [part for part in person.body_part if part.part_id in HEAD_MARKERS]
        minx = int(min([part_loc.x*self.res[0] for part_loc in self.part_locs]))
        maxx = int(max([part_loc.x*self.res[0] for part_loc in self.part_locs]))
        miny = int(min([part_loc.y*self.res[1] for part_loc in self.part_locs]))
        maxy = int(max([part_loc.y*self.res[1] for part_loc in self.part_locs]))
        dx = maxx - minx
        dy = maxy - miny
        box = {'min': (minx, miny - dy), 'max': (maxx, maxy)}
        return box

    def persons_callback(self, msg):
        self.persons = msg
        try:
            self.res = [msg.image_w, msg.image_h]
            self.rois = [self.get_2droi(person) for person in self.persons.persons]
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
        self.get_3droi(msg)

    def camerainfocallback(self, msg):
        self.caminfo = msg





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



