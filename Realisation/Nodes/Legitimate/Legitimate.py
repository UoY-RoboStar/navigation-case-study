# **********************************************************************************
# * Copyright (C) 2024-present Bert Van Acker (B.MKR) <bert.vanacker@uantwerpen.be>
# *
# * This file is part of the roboarch R&D project.
# *
# * RAP R&D concepts can not be copied and/or distributed without the express
# * permission of Bert Van Acker
# **********************************************************************************
import json
import operator
import time
from fractions import Fraction
from typing import Dict, List, Tuple

import yaml
from lidarocclusion.masks import BoolLidarMask
from .messages import *
from rpio.clientLibraries.rpclpy.node import Node
from rpio.clientLibraries.rpclpy.utils import timeit_callback

#<!-- cc_include START--!>

#<!-- cc_include END--!>

#<!-- cc_code START--!>
# user code here
def rotation_to_occlusion_angle(rotation: Dict[str, float]) -> Fraction:
    omega = rotation['omega']
    duration = rotation['duration']

    return Fraction(round(omega) * duration / np.pi)

def rotations_to_occlusion_angles(occlusion_angles: List[Dict[str, float]]) -> List[Fraction]:
    return list(map(rotation_to_occlusion_angle, occlusion_angles))
#<!-- cc_code END--!>

class Legitimate(Node):

    def __init__(self, config='config.yaml',verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "Legitimate"
        self.logger.info("Legitimate instantiated")

        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>
    # -----------------------------AUTO-GEN SKELETON FOR executer-----------------------------
    @timeit_callback
    def legitimate(self,msg):
        self.logger.info("Received new_plan event, starting legitimacy check...")

        raw_lidar_mask: str = self.read_knowledge('lidar_mask')
        lidar_mask: LidarMask = LidarMask.from_json(raw_lidar_mask)

        # raw_directions: str = self.read_knowledge('directions')
        rotations: List[Dict[str, float]] = self.read_knowledge('directions')

        base_angle = lidar_mask.base_angle

        rotation_steps = [
            round(angle/base_angle)
                for angle in rotations_to_occlusion_angles(rotations)
        ]

        rotated_mask = lidar_mask.reduce_rotate(operator.or_, rotation_steps)

        total_mask = BoolLidarMask([True]*lidar_mask.num_values, base_angle)

        plan_legitimate = rotated_mask == total_mask

        # Wait for 100ms as requested
        time.sleep(0.1)

        # Trigger the isLegit event
        if plan_legitimate:
            self.publish_event(PlanIsLegit)
            self.logger.info("Published isLegit event after 100ms delay")
        else:
            self.publish_event(PlanIsNotLegit)
            self.logger.info("Published isLegit event after 100ms delay")

    def register_callbacks(self):
        self.register_event_callback(event_key='new_plan', callback=self.legitimate)        # LINK <inport> new_plan

def main(args=None):
    try:
        with open('config.yaml', 'r') as file:
            config = yaml.safe_load(file)
    except:
        raise Exception("Config file not found")
    node = Legitimate(config=config)
    node.register_callbacks()
    node.start()

if __name__ == '__main__':
    main()
    try:
       while True:
           time.sleep(1)
    except:
       exit()
