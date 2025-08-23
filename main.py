#!/usr/bin/env python3
import os
import sys
import numpy as np

# Add Webots path
WEBOTS_HOME = os.environ.get('WEBOTS_HOME', 'D:\Programs\Webots\Webots')
sys.path.append(os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python'))

from controller import Robot

# Import our modules
from geometric_model import GeometricModel
from kinematic_model import KinematicModel
from cartesian_control import CartesianControl
from robot_base import RobotBase
from utils import YPRToRot, KinematicSimulation, parse_expr
from config import *

def main():
    """
    Main function for inverse kinematic control 
    """
    # Initialize robot
    robot = Robot()
    
    # Initialize geometric model
    geometric_model = GeometricModel(iTj_0, JOINT_TYPE, eTt)
    
    # Test initial configuration
    
    iTj = geometric_model.UpdateDirectGeometry(Q0)
    bTe = geometric_model.GetTransformWrtBase(iTj, geometric_model.jointNumber)
    bTt = geometric_model.getToolTransformWrtBase(Q0)
    print(f"Tool wrt base at initial: \n{np.round(bTt, 3)}")
    
    # Initialize kinematic and control models
    kinematic_model = KinematicModel(geometric_model, USE_TOOL)
    cartesian_control = CartesianControl(geometric_model, K_ANGULAR, K_LINEAR, USE_TOOL)
    
    # Initialize robot base
    robot_base = RobotBase(robot, SENSOR_NAMES, MOTOR_NAMES, geometric_model, setupby_vel=True)
    
    
    
    # asl for the input type 
    mode = input("choose input mode (1 for joint angles, 2 for cartesian pose): ")
    
    if mode == '1':
        
        user_input = input("Enter desired joint angles in radians (space-separated, use 'pi' if needed): ")
        q_goal = [parse_expr(angle) for angle in user_input.strip().split()]
        
        if len(q_goal) != 6:
            raise ValueError("You must enter exactly 6 values: q1 q2 q3 q4 q5 q6")
        
    elif mode == '2':
        
        
    
        # Get desired goal pose from user
        user_input = input("Enter desired x, y, z, roll, pitch, yaw (space-separated, use 'pi' if needed): ")
        values = [parse_expr(v) for v in user_input.strip().split()]
        
        if len(values) != 6:
            raise ValueError("You must enter exactly 6 values: x y z roll pitch yaw")
        
        x, y, z, roll, pitch, yaw = values
        
        # Create goal transformation matrix
        bOg = np.array([[x], [y], [z]])
        b_eta_g = YPRToRot(yaw, pitch, roll)
        
        bTg = np.vstack([
            np.hstack([b_eta_g, bOg]),
            np.array([[0, 0, 0, 1]])
        ])
        
        print(f"Goal transformation matrix:\n{bTg}")
    
    # Control loop parameters
    dt = (T_END - T_START) / SAMPLES
    iteration = 0
    
    print("Starting control loop...")
    while robot_base.step() != -1 and iteration < SAMPLES:
        
        if mode == '1':
            
            q_current = robot_base.read_joint_positions()
            J = kinematic_model.updateJacobian(q_current)
            q_dot =  K_ANGULAR * (np.array(q_goal) - np.array(q_current))
            q_dot = KinematicSimulation(q_current, q_dot, dt, QMIN, QMAX, MAX_VEL)
            
            # Send velocities to robot
            robot_base.actuate_joint_velocities(q_dot.flatten())
            
            joint_error = np.linalg.norm(np.array(q_goal) - np.array(q_current))
            if joint_error < 1e-1:
                print(f"Goal reached in {iteration} iterations!")
                final_joint_val = q_current
                print(f"Final joint values: {np.round(final_joint_val, 3)}")
                print(f"Desired joint angles: {np.round(q_goal, 3)}")
                break
            
        elif mode == '2':
                
            # Read current joint positions
            q_current = robot_base.read_joint_positions()
            
            # Compute desired end-effector velocity
            x_dot = cartesian_control.getCartesianreference(bTg, q_current)
            
            # Update Jacobian
            J = kinematic_model.updateJacobian(q_current)
            
            # Compute joint velocities using pseudoinverse
            try:
                q_dot = np.linalg.pinv(J) @ x_dot
            except np.linalg.LinAlgError:
                print("Singular configuration detected, using damped least squares")
                lambda_damping = 0.01
                q_dot = J.T @ np.linalg.inv(J @ J.T + lambda_damping * np.eye(6)) @ x_dot
            
            # Enforce joint limits and velocity limits
            q_dot = KinematicSimulation(q_current, q_dot, dt, QMIN, QMAX, MAX_VEL)
            
            # Send velocities to robot
            robot_base.actuate_joint_velocities(q_dot.flatten())
            
            # Check convergence
            pos_error = np.linalg.norm(x_dot[3:6])
            ori_error = np.linalg.norm(x_dot[0:3])
            
            if iteration % 50 == 0:  # Print every 50 iterations
                current_pose = geometric_model.getToolTransformWrtBase(q_current)
                current_pos = current_pose[0:3, 3]
                print(f"Iteration {iteration}: pos_error={pos_error:.4f}, ori_error={ori_error:.4f}")
                print(f"Current position: {np.round(current_pos, 3)}")
            
            # Check if goal is reached
            if pos_error < POS_THRESHOLD and ori_error < ORI_THRESHOLD:
                print(f"Goal reached in {iteration} iterations!")
                final_pose = geometric_model.getToolTransformWrtBase(q_current)
                print(f"Final position: {np.round(final_pose[0:3, 3], 3)}")
                print(f"Desired position: {bOg.flatten()}")
                break
            
        iteration += 1

    # Stop the robot
    robot_base.actuate_joint_velocities([0.0] * 6)
    
    if iteration >= SAMPLES:
        print("Maximum iterations reached without convergence")
        q_current = robot_base.read_joint_positions()
        final_pose = geometric_model.getToolTransformWrtBase(q_current)
        # final_pos_error = np.linalg.norm(bOg.flatten() - final_pose[0:3, 3].flatten())
        # final_ori_error = np.linalg.norm(b_eta_g - final_pose[0:3, 0:3])
        # print(f"Final position error: {final_pos_error:.4f}, orientation error: {final_ori_error:.4f}")
        raise RuntimeError("Goal is not reachable: final error too large. Motion is impossible with current joint limits or kinematics.")
    
    print("Control loop finished")
    

if __name__ == "__main__":
    main()
    
    
# example for forward kinematics values
# q1 q2 q3 q4 q5 q6
# 0 -pi/2 0 -pi/2 0 0
# 0 -pi/2 0 0 0 0
# pi/3 -pi/2 pi/4 0 pi/5 0
# pi/3 -pi/2 pi/4 0 pi/5 pi/3