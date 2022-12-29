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

from wsg_50_utils.wsg_50_gripper import WSG50Gripper


class FakeWSG50Gripper(WSG50Gripper):
    # Wraps the services into methods
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.data = Status()

    def _get_gripper_status_subscriber(self):
        fake_gripper_status_subscriber = None
        return fake_gripper_status_subscriber

    def set_grasping_force(self, force):
        pass

    def move(self, width, speed=50.0, repeat_move_until_ok=False):
        pass

    def grasp(self, width, speed=50.0):
        pass

    def release(self, width=110.0, speed=50.0):
        pass

    def ack(self):
        pass

    def homing(self):
        pass


