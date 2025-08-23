#!/usr/bin/env python3
import os
import sys

WEBOTS_HOME = os.environ.get('WEBOTS_HOME', 'D:\Programs\Webots\Webots')
sys.path.append(os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python'))

from controller import Robot

class RobotBase:
    """
    General robot controller base class for any manipulator in Webots.
    """
    
    def __init__(self, robot: Robot, sensor_names: list, motor_names: list, geometric_model, setupby_vel=False):
        """
        Initializes the robot's interfaces.

        Parameters:
        - robot: Webots Robot instance
        - sensor_names: list of joint sensor names
        - motor_names: list of joint motor names
        - geometric_model: instance of GeometricModel
        - setupby_vel: Boolean flag to setup motors for velocity control
        """
        self.robot = robot
        self.timestep = int(robot.getBasicTimeStep())
        self.sensor_names = sensor_names
        self.motor_names = motor_names
        self.geometric_model = geometric_model
        
        self.joint_sensors = []
        self.joint_motors = []

        self._setup_devices(setupby_vel)

    def _setup_devices(self, setupby_vel):
        """
        Connects to joint sensors and motors.
        """
        for name in self.sensor_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.timestep)
            self.joint_sensors.append(sensor)

        for name in self.motor_names:
            motor = self.robot.getDevice(name)
            self.joint_motors.append(motor)

        if setupby_vel:
            for motor in self.joint_motors:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)

    def read_joint_positions(self):
        """
        Reads joint sensor values.
        """
        return [sensor.getValue() for sensor in self.joint_sensors]

    def actuate_joint_positions(self, q_desired):
        """
        Sends desired joint positions to motors.
        """
        for motor, q in zip(self.joint_motors, q_desired):
            motor.setPosition(q)

    def actuate_joint_velocities(self, v_desired):
        """
        Sends desired joint velocities to motors.
        """
        for motor, v in zip(self.joint_motors, v_desired):
            motor.setVelocity(v)

    def step(self):
        """
        Advances simulation by one timestep.
        """
        return self.robot.step(self.timestep)

    def get_tool_pose(self):
        """
        Returns the current end-effector pose from joint sensors.
        """
        q = self.read_joint_positions()
        return self.geometric_model.getToolTransformWrtBase(q)