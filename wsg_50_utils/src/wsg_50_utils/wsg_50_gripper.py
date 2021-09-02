import rospy
from wsg_50_common.srv import Move, Conf
from std_srvs.srv import Empty


class WSG50Gripper(object):
    # Wraps the services into methods

    def __init__(self):
        pass

    def set_grasping_force(self, force):
        try:
            force = float(force)
        except (TypeError, ValueError) as e:
            pass
        if type(force) != float or force > 80.0 or force <= 0.0:
            print("Bad grasp force value provided! Not setting grasping force.")
            return None

        rospy.wait_for_service('wsg_50_driver/set_force')
        try:
            force_proxy = rospy.ServiceProxy('wsg_50_driver/set_force', Conf)
            force_resp = force_proxy(force)
            return force_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def move(self, width, speed=50.0):
        rospy.wait_for_service('wsg_50_driver/move')
        try:
            move_proxy = rospy.ServiceProxy('wsg_50_driver/move', Move)
            move_resp = move_proxy(width, speed)
            return move_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def grasp(self, width, speed=50.0):
        rospy.wait_for_service('wsg_50_driver/grasp')
        try:
            close_proxy = rospy.ServiceProxy('wsg_50_driver/grasp', Move)
            grasp_resp = close_proxy(width, speed)
            return grasp_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def open_gripper(self):
        self.move(100.0, 50.0)

    def release(self, width=110.0, speed=50.0):
        rospy.wait_for_service('wsg_50_driver/release')
        try:
            release_proxy = rospy.ServiceProxy('wsg_50_driver/release', Move)
            release_resp = release_proxy(width, speed)
            return release_resp.error
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def ack(self):
        rospy.wait_for_service('wsg_50_driver/ack')
        try:
            ack_proxy = rospy.ServiceProxy('wsg_50_driver/ack', Empty)
            ack_resp = ack_proxy()
            return ack_resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def homing(self):
        rospy.wait_for_service('wsg_50_driver/homing')
        try:
            homing_proxy = rospy.ServiceProxy('wsg_50_driver/homing', Empty)
            homing_resp = homing_proxy()
            return homing_resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
