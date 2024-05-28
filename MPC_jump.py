#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import numpy as np
import cvxpy as cp
import control
from scipy import sparse
import matplotlib.pyplot as plt
from math import sin, cos

import time
from controller_manager_msgs.srv import SwitchController

# MPC parameters and functions
m = 3  # mass
g = 9.8
l = 0.27
dt = 0.25  # Time step
K = True

A = np.array([[0, 1],
              [0, 0]])
B = np.array([[0],
              [1/m]])
omega = np.array([[0], 
                 [-g]])  # constant term representing the gravitational effect

C = np.array([[1, 0],
              [0, 1]])
D = np.array([[0],
              [0]])

# Construct a state-space system
state_space_system = control.StateSpace(A, B, C, D)
state_space_system_discrete = control.c2d(state_space_system, dt, method='zoh')
A_zoh = np.array(state_space_system_discrete.A)
B_zoh = np.array(state_space_system_discrete.B)

# Trajectory generation
angle1_0 = np.pi / 20  # initial angle between link1 and horizon
angle2_0 = np.pi / 10  # initial angle between link2 and horizon
angle1_t = np.pi / 5  # final angle between link1 and horizon
angle2_t = np.pi / 2.5  # final angle between link2 and horizon
x_0 = l * np.sin(angle1_0) + l * np.sin(angle2_0)  # initial position of the upper point
x_t = l * np.sin(angle1_t) + l * np.sin(angle2_t)  # final position of the upper point
v_0 = 0.0  # initial point velocity
v_t = 1.0  # final point velocity

a = v_t ** 2 / (2 * (x_t - x_0))
v = np.arange(v_0, v_t + a * dt, a * dt)
x = [x_0]
for i in range(1, len(v)):
    x.append(x[i - 1] + v[i - 1] * dt + (a * dt ** 2) / 2)

x_states = np.array([x, v]).T[..., np.newaxis]
array2d = np.array([[x_0],
                    [0.]])
array2d_expanded = array2d[np.newaxis, ...]
for i in range(5):
    x_states = np.concatenate((array2d_expanded, x_states), axis=0)

Q = sparse.diags([30., 30.])
R = np.array([[1.]])

def calc_opt_contr_mpc(x, u, x_init, ref_crop):
    cost = 0
    constr = [x[0, :] == x_init[:]]
    for t in range(ref_crop.shape[0]):
        cost += cp.quad_form(ref_crop[t, :] - x[t, :], Q)
        constr += [cp.norm(u[t, :], 'inf') <= 17.]
        constr += [x[t + 1, :] == A_zoh @ x[t, :] + B_zoh @ u[t, :]]

    problem = cp.Problem(cp.Minimize(cost), constr)
    return problem

x_sys = np.array([x_0, v_0])
sim_steps = x_states.shape[0]
x_ref = x_states[:, :, 0]
[nx, nu] = B_zoh.shape
N = 5  # MPC Horizon length

def calculate_jacobian(theta1, theta2, l):
    J = np.array([[-l * np.sin(theta1) - l * np.sin(theta1 + theta2), -l * np.sin(theta1 + theta2)],
                  [l * np.cos(theta1) + l * np.cos(theta1 + theta2), l * np.cos(theta1 + theta2)]])
    return J

def calculate_joint_torques(J, F, theta1, theta2):
    Fx = F * np.cos(theta1 + theta2 - np.pi / 2)  # Force in the x-direction
    Fy = F * np.sin(theta1 + theta2 - np.pi / 2)  # Force in the y-direction
    F = np.array([Fx, Fy])
    joint_torques = np.dot(J.T, F)
    return joint_torques



# ROS node for control
class Controller:
    def __init__(self) -> None:
        self.current_angles = np.array([0, 0, 0])
        self.effort_publishers = [
            rospy.Publisher("/leg/joint_position_controller_4/command", Float64, queue_size=10),
            rospy.Publisher("/leg/joint_effort_controller_6/command", Float64, queue_size=10),
            rospy.Publisher("/leg/joint_effort_controller_8/command", Float64, queue_size=10),
        ]

        self.effort_publishers_position = [
            rospy.Publisher("/leg/joint_position_controller_4/command", Float64, queue_size=10),
            rospy.Publisher("/leg/joint_position_controller_6/command", Float64, queue_size=10),
            rospy.Publisher("/leg/joint_position_controller_8/command", Float64, queue_size=10),
        ]

    def set_current_angles(self, angles):
        self.current_angles = angles

    def apply_effort(self, efforts):
        for pub, effort in zip(self.effort_publishers, efforts):
            pub.publish(Float64(effort))

    def apply_effort_position(self, efforts):
        for pub, effort in zip(self.effort_publishers_position, efforts):
            pub.publish(Float64(effort))


class RosManager:
    def __init__(self) -> None:
        rospy.init_node("listener", anonymous=True)
        self.controller = Controller()
        rospy.Subscriber("/leg/joint_states", JointState, self.listen_joint_state)
        self.current_pos = 0.0
        self.current_vel = 0.0
        self.posistion_mode = True
        self.count = 0

    def run_mpc(self):
        global K  # Объявляем переменную K как глобальную

        x_sys = np.array([self.current_pos, self.current_vel])
        x_traj = []
        v_traj = []
        u_traj = []

        for i in range(sim_steps - N):
            ref_crop = x_ref[i: i + N, :]
            x = cp.Variable((N + 1, nx))  # Create optimization variables.
            u = cp.Variable((N + 1, nu))
            x_init = cp.Parameter(nx)
            x_init.value = x_sys

            prob = calc_opt_contr_mpc(x, u, x_init, ref_crop)
            prob.solve(solver=cp.OSQP, warm_start=True)

            force = u[0, :].value[0]
            x_sys = (A_zoh @ x_sys.reshape(1, -1).T + B_zoh * force).reshape(1, -1)[0]

            x_traj.append(x_sys[0])
            v_traj.append(x_sys[1])
            u_traj.append(force)

        ref_pos = [x_ref[i, 0] for i in range(sim_steps)]
        ref_vel = [x_ref[i, 1] for i in range(sim_steps)]

        if K:
            for i in range(len(u_traj)):
                u_traj[i] += m * g  # add a constant Gravity force
            K = False

        theta1 = np.arcsin(np.array(x_traj) / (2 * l))
        theta2 = np.pi - 2 * theta1

        J = [calculate_jacobian(theta1[i], theta2[i], l) for i in range(len(u_traj))]
        joint_torques = [calculate_joint_torques(J[i], -u_traj[i], theta1[i], theta2[i]) for i in range(len(u_traj))]

        for i in range(len(joint_torques)):
            joint_torques[i][0] = -joint_torques[i][0]
            joint_torques[i][1] = -joint_torques[i][1]

        return joint_torques[0][0], joint_torques[0][1]
        
        # self.controller.apply_effort(joint_torques[0])

    def spin(self):
        rate = rospy.Rate(10)

        # constant_efforts = [0, 0, 0]  # Задаем постоянные моменты для суставов
        
        start_time = time.perf_counter()

        while not rospy.is_shutdown():
            

            # hip_torque, knee_torque = self.run_mpc()
            if (time.perf_counter() - start_time) < 3 or self.count == 4:
                if not self.posistion_mode:
                    self.posistion_mode = True
                rospy.wait_for_service('/leg/controller_manager/switch_controller')
                switch_controller = rospy.ServiceProxy('/leg/controller_manager/switch_controller', SwitchController)
                ret = switch_controller(start_controllers=['joint_position_controller_6', 'joint_position_controller_8'],
                                        stop_controllers=['joint_effort_controller_6', 'joint_effort_controller_8'], strictness=1)
                position_efforts = [0, 0, -0.6]
                self.controller.apply_effort_position(position_efforts)
                rate.sleep()
            else:
                if self.posistion_mode:
                    self.posistion_mode = False

                rospy.wait_for_service('/leg/controller_manager/switch_controller')
                switch_controller = rospy.ServiceProxy('/leg/controller_manager/switch_controller', SwitchController)
                ret = switch_controller(start_controllers=['joint_effort_controller_6', 'joint_effort_controller_8'],
                                        stop_controllers=['joint_position_controller_6', 'joint_position_controller_8'], strictness=1)
                hip_torque, knee_torque = self.run_mpc()
                print(hip_torque, knee_torque)
                self.controller.apply_effort([0, hip_torque*200, knee_torque*200])
                self.count += 1


    def listen_joint_state(self, msg: JointState):
        index = msg.name.index("Slider_2")
        self.current_pos = msg.position[index]
        self.current_vel = msg.velocity[index]


if __name__ == "__main__":
    # rospy.wait_for_service('/leg/controller_manager/switch_controller')
    # switch_controller = rospy.ServiceProxy('/leg/controller_manager/switch_controller', SwitchController)
    # ret = switch_controller(start_controllers=['joint_effort_controller_6', 'joint_effort_controller_8'],
    #                         stop_controllers=['joint_position_controller_6', 'joint_position_controller_8'], strictness=1)
    ros_manager = RosManager()
    ros_manager.spin()

