import copy
import time
import numpy as np

import rospy
from threading import Lock
from typing import Optional, Type
import time

from wsg_50_common.srv import Move, Conf
from std_srvs.srv import Empty
from wsg_50_common.msg import Status


class WSG50Gripper(object):
    # Wraps the services into methods
    def __init__(self, safe=False):
        self.data = None
        self.safe = safe
        self.lock = Lock()
        self.status_topic_name = '/wsg_50_driver/status'
        self.gripper_status_subscriber = rospy.Subscriber(self.status_topic_name, Status, self._gripper_status_callback)

    def _gripper_status_callback(self, msg):
        with self.lock:
            self.data = msg

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

    def move(self, width, speed=50.0, repeat_move_until_ok=False):
        init_width = self.get_width()
        rospy.wait_for_service('wsg_50_driver/move')
        try:
            move_proxy = rospy.ServiceProxy('wsg_50_driver/move', Move)
            if self.safe:
                move_succeed = False
                error_found = False
                num_attempts = 500
                for i in range(num_attempts):
                    move_resp = move_proxy(width, speed)
                    if move_resp.error == 0:
                        # print('No move error')
                        move_succeed = True
                        if error_found:
                            print('', end='\r')
                        break
                    else:
                        error_found = True
                        print(f'Move error detected (Error: {move_resp.error}. Doing ACK and trying again ({i+1}/{num_attempts})', end='\r')
                        self.ack()
                        rospy.sleep(1.)
                        if repeat_move_until_ok:
                            print('repeating motion -- ')
                            self.move(init_width, speed=speed, repeat_move_until_ok=False)
                if not move_succeed:
                    raise AssertionError('The move motion failed')
            else:
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

    def get_status(self, block_until_data=True):
        wait_for(lambda: not (block_until_data and self.data is None), 10, f"WSG50Gripper({self.status_topic_name})")
        with self.lock:
            status = copy.deepcopy(self.data)
        return status

    def get_width(self):
        status = self.get_status()
        width = status.width
        return width

    def get_force(self):
        status = self.get_status()
        force = status.force
        return force

    def get_speed(self):
        status = self.get_status()
        speed = status.speed
        return speed


# From arc_utilities
def wait_for(func, warn_after: Optional[int] = 10, name: Optional[str] = ""):
    """
    Waits for function evaluation to be true. Exits cleanly from ros.

    Introduces sleep delay, not recommended for time critical operations
    """

    start_t = rospy.Time.now()

    while not func() and not rospy.is_shutdown():
        if warn_after is not None and rospy.Time.now() - start_t > rospy.Duration(secs=warn_after):
            warning = f"still waiting after {warn_after}s"
            if name:
                warning += f" for {name}"
            rospy.logwarn_throttle(5, warning)
        time.sleep(0.01)
