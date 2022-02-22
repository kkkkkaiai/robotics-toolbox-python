import sys

import swift
import roboticstoolbox as rtb
import spatialgeometry as sg
import spatialmath as sm
import qpsolvers as qp
import numpy as np
from distutils.dir_util import copy_tree
import math
from os import mkdir, path
from roboticstoolbox import ERobot
import tempfile as tf
from roboticstoolbox.tools.data import rtb_path_to_datafile

env = swift.Swift()
env.launch(realtime=True)


class mr500(ERobot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "mr500_description/urdf/mr500.urdf.xacro",
        )

        super().__init__(
            links,
            name=name,
            manufacturer="shihe",
            gripper_links=links[5],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qdlim = np.array(
            [
                1.6,
                1.6,
                1.6,
                1.6
            ]
        )

        self.addconfiguration("qz", np.array([0, 0, 0, 0]))
        self.addconfiguration("qr", np.array([0, 0, 0, 0]))

        for i in range(self.n):
            if np.any(self.qlim[:, i] != 0) and not np.any(np.isnan(self.qlim[:, i])):
                self._valid_qlim = True
        print(self._valid_qlim)


mr500_dir = "./mr500_description"
if not path.exists(mr500_dir):
    print(mr500_dir, ' is not exist')
    sys.exit()

xacro_path = rtb_path_to_datafile("xacro")
mr500_xacro = xacro_path / "mr500_description"

print('source dir: ', mr500_xacro, '\ndst dir: ', mr500_dir)
copy_tree(mr500_dir, str(mr500_xacro))

dt = 0.025

car_m = mr500()
car_m.q = car_m.qr
print(car_m.q)
env.add(car_m)
env.step(dt)

env.hold()

