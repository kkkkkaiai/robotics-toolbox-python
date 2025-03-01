import sys
import time
import copy
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
from spatialmath import SE3
from roboticstoolbox.tools.data import rtb_path_to_datafile


class mr500_kinova(ERobot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "mr500_kinova_description/urdf/mr500_kinova.urdf.xacro",
        )
        for link in links:
            print(link)
        super().__init__(
            links,
            name=name,
            manufacturer="mine",
            gripper_links=links[12],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.qdlim = np.array(
            [
                1.6,
                1.6,
                2.2300,
                2.2300,
                2.2300,
                2.2300,
                2.2300,
                2.2300,
                # 2.0,
                # 2.0
            ]
        )

        print(self.ee_links)


        # self.grippers[0].tool = SE3(0, 0, 0.1034)

        self.addconfiguration("qz", np.array([0, 0, 0, 0, 0, 0, 0, 0,]))
        self.addconfiguration("qr", np.array([0, 0, 0, -0.3, 1.6, 0, 1.25, -np.pi / 2]))

        for i in range(self.n):
            if np.any(self.qlim[:, i] != 0) and not np.any(np.isnan(self.qlim[:, i])):
                self._valid_qlim = True

        print(self.n)


def init_files():
    mr500_kinova_dir = "./mr500_kinova_description"
    if not path.exists(mr500_kinova_dir):
        print(mr500_kinova_dir, ' is not exist')
        sys.exit()

    xacro_path = rtb_path_to_datafile("xacro")
    mr500_kinova_xacro = xacro_path / "mr500_kinova_description"

    print('source dir: ', mr500_kinova_xacro, '\ndst dir: ', mr500_kinova_dir)
    copy_tree(mr500_kinova_dir, str(mr500_kinova_xacro))


def step_robot(r, Tep):
    wTe = r.fkine(r.q, end='kinova_end_effector_link', fast=True)

    eTep = np.linalg.inv(wTe) @ Tep
    # Spatial error
    coeff = np.array([1,1,1])
    et = np.sum(np.abs(eTep[:3, -1]) * coeff)
    print('空间误差： ', et)

    # Gain term (lambda) for control minimisation
    Y = 0.15

    # Quadratic component of objective function
    Q = np.eye(r.n + 6)

    # Joint velocity component of Q
    Q[: r.n, : r.n] *= Y
    Q[:2, :2] *= 1.0 / et

    # 调节Q和1/et的值，可以让规划器倾向，Q增大，则规划器因为倾向于底盘原因，考虑机械臂末端较少
    # 所以转弯幅度小（因为底盘根据末端方向估计转弯角度）
    print('Q: ', Y, '  |  ', 1/et)

    # Slack component of Q
    Q[r.n:, r.n:] = (1.0 / et) * np.eye(6)

    print(wTe, '\n', Tep)

    v, _ = rtb.p_servo(wTe, Tep, 1.5)
    print("v: ", v)
    # v[3:6] *= 0.8
    # v[6:] *= 1.2
    v[3:] *= 1.3

    # The equality contraints
    Aeq = np.c_[r.jacobe(r.q, end='kinova_end_effector_link', fast=True), np.eye(6)]
    beq = v.reshape((6,))

    # The inequality constraints for joint limit avoidance
    Ain = np.zeros((r.n + 6, r.n + 6))
    bin = np.zeros(r.n + 6)

    # The minimum angle (in radians) in which the joint is allowed to approach
    # to its limit
    ps = 0.1

    # The influence angle (in radians) in which the velocity damper
    # becomes active
    pi = 0.9

    # Form the joint limit velocity damper
    Ain[: r.n, : r.n], bin[: r.n] = r.joint_velocity_damper(ps, pi, r.n)

    # print('Ain: ', Ain)
    # print('Bin: ', bin)

    # Linear component of objective function: the manipulability Jacobian
    # print(r.links[3], r.links[9])

    c = np.concatenate(
        (np.zeros(2), -r.jacobm(start=r.links[4], end='kinova_end_effector_link').reshape((r.n - 2)), np.zeros(6))
    )

    # Get base to face end-effector
    kε = 1
    bTe = r.fkine(r.q, end='kinova_end_effector_link', include_base=False, fast=True)
    θε = math.atan2(bTe[1, -1], bTe[0, -1])
    ε = kε * θε
    c[0] = -ε

    # The lower and upper bounds on the joint velocity and slack variable
    lb = -np.r_[r.qdlim[: r.n], 10 * np.ones(6)]
    ub = np.r_[r.qdlim[: r.n], 10 * np.ones(6)]

    print('c: ', c)
    # Solve for the joint velocities dq
    qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub)
    qd = qd[: r.n]

    if et > 0.5:
        qd *= 0.7 / et
    else:
        qd *= 1.4

    print(qd)

    if et < 0.02:
        return True, qd
    else:
        return False, qd


def main():
    init_files()

    env = swift.Swift()
    env.launch(realtime=True)

    ax_goal = sg.Axes(0.5)
    env.add(ax_goal)
    ax_start = sg.Axes(0.5)
    env.add(ax_start)
    ax_base = sg.Axes(0.5)
    env.add(ax_base)

    mr_k = mr500_kinova()
    mr_k.q = mr_k.qr

    env.add(mr_k)
    # Behind
    env.set_camera_pose([-0, 2, 1.7], [-1, 0.0, 0.5])
    base_ = mr_k.fkine(mr_k.q, end=mr_k.links[2]) * sm.SE3.Rz(np.pi)
    ax_base.base = base_
    start_ = mr_k.fkine(mr_k.q) * sm.SE3.Rz(np.pi)
    ax_start.base = start_
    wTep = mr_k.fkine(mr_k.q, end='kinova_end_effector_link') * sm.SE3.Rz(np.pi)
    wTep.A[:3, :3] = np.array([0, 1, 0, 1, 0, 0, 0, 0, -1]).reshape((3,3))
    wTep.A[0, -1] -= 1.0
    wTep.A[1, -1] += 1.0
    wTep.A[2, -1] -= 0.35
    ax_goal.base = copy.copy(wTep)
    env.step()


    arrived = False
    dt = 0.025

    cnt = 0
    while not arrived:
        cnt += 1
        arrived, mr_k.qd = step_robot(mr_k, wTep.A)

        # update
        base_ = mr_k.fkine(mr_k.q, end=mr_k.links[2]) * sm.SE3.Rz(np.pi)
        ax_base.base = base_


        env.step(dt)

        base_new = mr_k.fkine(mr_k._q, end=mr_k.links[2], fast=True)
        mr_k._base.A[:] = base_new

        mr_k.q[:2] = 0
        # if cnt == 20:
        #     env.hold()

    env.hold()


def calc():
    init_files()

    mr_k = mr500_kinova()
    mr_k.q = mr_k.qr

    for link in mr_k.links:
        print(link)

    b = mr_k.fkine(mr_k.q, end='base_link')
    print(b)


    bto1 = mr_k.fkine(mr_k.q, end='base_link1')
    bto2 = mr_k.fkine(mr_k.q, end='base_link2')
    bto3 = mr_k.fkine(mr_k.q, end='base_link3')
    bto4 = mr_k.fkine(mr_k.q, end='base_link4')
    btom = mr_k.fkine(mr_k.q, end='mount_link')
    print(bto1)
    print(bto2)
    print(bto3)
    print(bto4)
    print(btom)

    b1to2 = bto1.inv() @ bto2
    b2to3 = bto2.inv() @ bto3
    b3to4 = bto3.inv() @ bto4
    mto4 = bto4.inv() @ btom

    # pose3d.SE3

    print(b1to2.rpy('xyz'), b1to2.t)
    print(b2to3.rpy('xyz'), b2to3.t)
    print(b3to4.rpy('xyz'), b3to4.t)
    print(mto4.rpy('xyz'), mto4.t)


def print_info():
    init_files()

    robot = mr500_kinova()
    print(robot)

    robot.q = robot.qr
    print(robot.q)
    print(robot.fkine(robot.q, end='kinova_end_effector_link'))
    print(robot.fkine(robot.q))

    for link in robot.links:
        print(link.name)
        print(link.isjoint)
        print(len(link.collision))

    print()

    for link in robot.grippers[0].links:
        print(link.name)
        print(link.isjoint)
        print(len(link.collision))


if __name__ == '__main__':
    main()
