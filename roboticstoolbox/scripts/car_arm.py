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


class mr500_kinova(ERobot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "mr500_kinova_description/urdf/mr500_kinova.urdf.xacro",
        )

        super().__init__(
            links,
            name=name,
            manufacturer="mine",
            gripper_links=links[14],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qdlim = np.array(
            [
                1.6,
                1.6,
                1.6,
                1.6,
                2.1750,
                2.1750,
                2.1750,
                2.6100,
                2.6100,
                2.6100
            ]
        )

        print(self.n)

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0]))
        self.addconfiguration("qr", np.array([0, 0, 0, 0,  -0.3, 0, -1.6, 0, -1.0, np.pi / 2]))

        for i in range(self.n):
            if np.any(self.qlim[:, i] != 0) and not np.any(np.isnan(self.qlim[:, i])):
                self._valid_qlim = True
        print(self._valid_qlim)


mr500_kinova_dir = "./mr500_kinova_description"
if not path.exists(mr500_kinova_dir):
    print(mr500_kinova_dir, ' is not exist')
    sys.exit()

xacro_path = rtb_path_to_datafile("xacro")
mr500_kinova_xacro = xacro_path / "mr500_kinova_description"

print('source dir: ', mr500_kinova_xacro, '\ndst dir: ', mr500_kinova_dir)
copy_tree(mr500_kinova_dir, str(mr500_kinova_xacro))

dt = 0.025

env = swift.Swift()
env.launch(realtime=True)

car_m = mr500_kinova()
car_m.q = car_m.qr
print(car_m.q)
env.add(car_m)
env.step(dt)

env.hold()
