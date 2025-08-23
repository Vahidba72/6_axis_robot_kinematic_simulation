#!/usr/bin/env python3
import os
import sys


import numpy as np
import time 

WEBOTS_HOME = os.environ.get('WEBOTS_HOME', 'D:\Prgrams\Webots\Webots')
sys.path.append(os.path.join(WEBOTS_HOME, 'lib', 'controller', 'python'))


from controller import Robot

class GeometricModel:
    
    
    """
    The geometric model for multi arm robotd with different types of join (Rotational or Prismatic)
    
    """
    
    
    def __init__(self, iTj_0,joint_Type, eTt=np.eye(4)):
        
        """
        - iTj_0 --->   is an object containing the trasformations from the frame <i> to <i+1> 
        for the case q = 0
        - joint_Type --->  is an array containing the type of each joint, 0 for rotational and 1 for prismatic
        
        """
        
        self.iTj_0 = np.array(iTj_0)
        self.joint_Type = joint_Type
        self.jointNumber = len(joint_Type)
        self.eTt = eTt
        
        
        
    def UpdateDirectGeometry(self,q):
        """
        calculates the updated transformation matrix from joint i to joint i+1
        after getting the joint value q
        
        Inputs:
        - q: list or array of joint values

        """
        iTj = np.array([np.eye(4)] * self.iTj_0.shape[0])
        
        for i in range(self.jointNumber):
            
            if self.joint_Type[i] == 0:  # Revolute Joint
                
                theta = q[i]
                
                T1 = np.array([
                    [np.cos(theta), -np.sin(theta), 0, 0],
                    [np.sin(theta), np.cos(theta), 0, 0],
                    [0,0,1,0],
                    [0,0,0,1]
                ])
                
            elif self.joint_Type[i] == 1:  # Prismatic Joint
                
                dz = q[i]
                
                T1 = np.array([
                    [1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, dz],
                    [0, 0, 0, 1]
                ]) 
                
            else:
                raise ValueError("Invalid joint type. Use 0 for revolute and 1 for prismatic.")
            
            iTj[i] = iTj_0[i] @ T1
        return iTj
    
    def GetTransformWrtBase(self, iTj , k):
        
        """
        Calculates the transformation matrix from the base to the k-th joint
        
        inputs:
        
        k: the index of the joint for which we want to compute the transformation matrix
        
        Outputs:
        
        bTk: transformation matrix from the manipulator base to the k-th joint in the configuration identified by iTj.
        """
        
        bTk = np.eye(4)
        for i in range(k):
            bTk = bTk @ iTj[i,:,:]
        return bTk
    
    def getToolTransformWrtBase(self,q):
        """
        Returns the transformation from base to the tool (end-effector).
        
        Outputs:
        - bTt: transformation matrix from the manipulator base to the end-effector tool
        """
        iTj = self.UpdateDirectGeometry(q)
        bTe = self.GetTransformWrtBase(iTj, self.jointNumber)
        return bTe @ self.eTt
        

    # def getFrameWrtFrame(self, linkNumber_i, linkNumber_j):
    #     """
    #     Computes the transformation matrix between two joints i and j.
        
    #     Inputs:
    #     - linkNumber_i: Number of the starting link
    #     - linkNumber_j: Number of the target link
        
    #     Outputs:
    #     - iTojTransform: Transformation matrix from link i to link j
    #     """
        
    #     iTojTransform = np.eye(4)

    #     if linkNumber_i < linkNumber_j:
            
    #         for i in range (linkNumber_i, linkNumber_j):
    #             iTojTransform = iTojTransform @ self.iTj_0[i]
                
                
    #     elif linkNumber_i > linkNumber_j:
            
    #         for i in range(linkNumber_j, linkNumber_i):
    #             iTojTransform = np.linalg.inv(self.iTj_0[i]) @ iTojTransform
    
    #     else:
            
    #         # Transformation from a joint to itself
    #         iTojTransform = np.eye(4)
            
    #     return iTojTransform

class KinematicModel:
    """
    Kinematic model for a robot manipulator.
    This class uses the GeometricModel to compute transformations and kinematics.
    """
    def __init__(self, geometric_model, tool = False):
        """
        Initializes the kinematic model with a geometric model.
        
        Inputs:
        - geometric_model: An instance of GeometricModel
        """
        self.gm = geometric_model
        self.J = np.zeros((6,self.gm.jointNumber))
        self.RJ = np.zeros((6,6))
        self.tool = tool
        
    def updateJacobian(self, q):
        """
        Updates the Jacobian matrix based on the current joint values q.
        
        Inputs:
        - q: list or array of joint values
        
        Output:
        - J: Updated Jacobian matrix
        """
        
        # Update transformation for current joint configurations
        iTj = self.gm.UpdateDirectGeometry(q)
        
        bTi = np.zeros((self.gm.jointNumber,4, 4))
        
        
            
        # Compute the jacobian matrix
        for i in range(self.gm.jointNumber):
            
            bTi = self.gm.GetTransformWrtBase(iTj, i+1)
            if self.gm.joint_Type[i] == 0: # Revolute Joint
                
                # angular velocity contribution 
                self.J[0:3,i] = bTi[0:3,2]  
                
                # Linear velocity contribition
                bTe = self.gm.GetTransformWrtBase(iTj, self.gm.jointNumber)
                rei = bTe[0:3,-1] - bTi[0:3,-1]
                self.J[3:,i] = np.cross(bTi[0:3,2], rei)
                
            elif self.gm.joint_Type[i] == 1: # Prismatic Joint
                
                # Angular velocity contribution
                self.J[0:3,i] = np.array([0, 0, 0]).T  # No angular velocity contribution for prismatic joints
                
                # linear velocity contribution
                self.J[3:,i] = bTi[0:3,2]
        
        
        
        if self.tool:
            # Calculate the tool Jacobian
            bTt = self.gm.getToolTransformWrtBase(q)
            b_ert = bTt[0:3, -1] - bTe[0:3, -1]
            
            ert_operator = np.array([
                [0, -b_ert[2], b_ert[1]],
                [b_ert[2], 0, -b_ert[0]],
                [-b_ert[1], b_ert[0], 0]
            ])
            
            # Rigid tool jacobian
            
            self.RJ = np.block([
                [np.eye(3), np.zeros((3, 3))],
                [ert_operator.T, np.eye(3)]
                ])
                    
            return self.RJ @ self.J
        
        else:
            return self.J
class CartesianControl:
    
    def __init__(self, geometric_model ,angular_gain, linear_gain,tool = False):
        
        """
        Cartesian control class for robot manipulators.
        
        Inputs:
        - geometric model: An instance of GeometricModel
        - angular_gain: Gain for angular velocity control
        - linear_gain: Gain for linear velocity control
        """
        
        self.gm = geometric_model
        self.k_a = angular_gain
        self.k_l = linear_gain
        self.tool = tool
    
    
    import numpy as np
    
    def ComputeAngleAxis(self , theta , h):
        """
        Computes the rotation matrix from angle-axis representation.
        
        Inputs:
        - theta: Rotation angle
        - h: Rotation axis (unit vector)
        
        Outputs:
        - R: Rotation matrix (3x3)
        """
        
        # Rodriguez Formula 
        V_operator = np.array([
            [0, -h[2], h[1]],
            [h[2], 0, -h[0]],
            [-h[1], h[0], 0]
        ])
        
        R = np.eye(3) + np.sin(theta) * V_operator + (1 - np.cos(theta)) * (V_operator @ V_operator)
        
        return R
    
    def RottoAngleAxis(self , R):
        
        """
        Converts a rotation matrix to angle-axis representation.
        
        Inputs:
        - R: Rotation matrix (3x3)
        
        Outputs:
        - theta: Rotation angle
        - h: Rotation axis (unit vector)
        """
        
        if R.shape != (3, 3):
            raise ValueError("Input must be a 3x3 rotation matrix.")
        
        # verify orthogonality -> R.T*R = I
        O = R.T @ R - np.eye(3)
        Thres = 1e-6
        O[np.abs(O) < Thres] = 0  # Numerical tolerance
        
        if not np.allclose(O, np.zeros((3, 3)), atol = Thres):
            raise ValueError("Input matrix is not orthogonal.")
        
        
        # Compute if the determinant is one 
        if not np.isclose(np.linalg.det(R), 1, atol=Thres):
            raise ValueError("Input matrix is not a valid rotation matrix (determinant != 1).")
        
        # compute angle of rotation
        theta = np.arccos((np.trace(R) - 1) / 2)
        
        # compute eigenvalues and eigenvectors
        eigVals , eigVecs = np.linalg.eig(R)
        
        # Find the eigenvector corresponding to eigenvector1 
        idx = np.where(np.abs(eigVals - 1) < Thres)[0]
        if idx.size == 0:
            raise ValueError("No valid eigenvector found for rotation axis.")
        
        h = np.real(eigVecs[:, idx[0]]) # ensure it's real
        
        
        # resolve axis sign ambiguity 
        R_pos = self. ComputeAngleAxis(theta , h)
        R_neg = self. ComputeAngleAxis(theta , -h)
        
        if np.allclose(R, R_pos, atol=Thres):
            h = h
            
        elif np.allclose(R, R_neg, atol=Thres):
            h = -h
            
        else:
            raise ValueError("Unable to resolve axis sign ambiguity.")
        
        
        return theta, h
        
            



    
        
    def getCartesianreference(self, bTg , q):
        """
        Computes the desired end-effector velocity based on the desired pose.
        
        Inputs:
        - bTg: Desired end-effector pose (4x4 transformation matrix)
        - q: Current joint values
        Outputs:
        - bVg: Desired end-effector velocity (6x1 vector)
        """
        
        if self.tool:
            # Current end-effector pose
            bTt = self.gm.getToolTransformWrtBase(q)
            
            # linear error in base frame
            error_linear = bTg[0:3, 3] - bTt[0:3, 3]
            
            # Orientation error R_err in the tool frame
            error_angular1 = bTt[0:3 , 0:3].T @ bTg[0:3, 0:3]
            
            # convert oientation error into angle-axis representation
            theta , h = self.RottoAngleAxis(error_angular1)
            
            # map the vector to the base frame 
            error_angular = bTt[0:3 , 0:3] @ (theta * h)
            
        
            x_dot = np.vstack((
                (self.k_a * error_angular).reshape(3, 1),
                (self.k_l * error_linear).reshape(3, 1)
                
                ))
            
            return x_dot
        else:
            # Current end-effector pose
            bTe = self.gm.GetTransformWrtBase(self.gm.UpdateDirectGeometry(q), self.gm.jointNumber)
            
            # linear error in base frame
            error_linear = bTg[0:3, 3] - bTe[0:3, 3]
            
            # Orientation error R_err in the base frame
            error_angular1 = bTg[0:3 , 0:3] @ bTe[0:3, 0:3].T 
            
            # convert oientation error into angle-axis representation
            theta , h = self.RottoAngleAxis(error_angular1)
            
            error_angular = theta * h
            
        
            x_dot = np.vstack((
                (self.k_a * error_angular).reshape(3, 1),
                (self.k_l * error_linear).reshape(3, 1)
                
                ))
            
            return x_dot    
def YPRToRot(psi, theta, phi):
    
    """
    The function computes the rotation matrix using the YPR (Yaw-Pitch_Roll)
    convention, given psi, theta, phi.
    
    Inputs:
    psi angle around z axis (Yaw)
    theta angle around y axis (pitch)
    phi angle around x axis (roll)
    
    outputs:
    R rotation matrix
    
    """      
    
    if abs(theta- np.pi/2) < 1e-6 or abs (theta + np.pi/2) < 1e-6:
        
        Warning('Singular configuration detected: theta = +/- pi/2. Rotation matrix may lose one degree of freedom')
    R_z = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi), np.cos(psi), 0],
        [0, 0, 1]
    ])
    
    R_y = np.array([
        [np.cos(theta), 0, np.sin(theta)],  
        [0, 1, 0],
        [-np.sin(theta), 0, np.cos(theta)]
    ])
    
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi), np.cos(phi)]
    ])
    
    # combine the rotation matrices
    R = R_z @ R_y @ R_x  
    return R   

def KinematicSimulation(q, q_dot, dt, qmin, qmax):
    """
    Simulates the kinematic update of joint positions.
    
    Inputs:
    - q: Current joint positions (list or array)
    - q_dot: Joint velocities (list or array)
    - dt: Time step for the simulation
    - qmin: Minimum joint limits (array)
    - qmax: Maximum joint limits (array)
    
    Outputs:
    - q_new: Updated joint positions after applying velocities and limits
    """
    
    
    q = q + dt * q_dot.flatten()
    
    # saturate the joint positions 
    for i in range(len(q)):
        if q[i] < qmin[i]:
            q[i] = qmin[i]
        elif q[i] > qmax[i]:
            q[i] = qmax[i]
            
    return q

class RobotBase():
    """
    General robot controller base class for any manipulator in Webots.
    """
    def __init__(self, robot: Robot, sensor_names: list, motor_names: list, geometric_model: GeometricModel, setupby_vel=False):
        """
        Initializes the robot's interfaces.

        Parameters:
        - robot: Webots Robot instance
        - sensor_names: list of joint sensor names
        - motor_names: list of joint motor names
        - geometric_model: instance of GeometricModel
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
            # motor.setPosition(0.0)  # default to position control
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


# --- Safe eval for pi expressions ---
def parse_expr(expr):
    return float(eval(expr, {"__builtins__": None}, {"pi": np.pi, "np": np}))   

def Reachability(gm, km, cc, bTg, q, dt, samples, qmin, qmax):
    """
    Checks if the goal pose is reachable from the current configuration using iterative IK.
    Returns True if reachable, raises RuntimeError if not.
    """
    for i in range(samples):
        # Update the position of the goal frame
        iTj = gm.UpdateDirectGeometry(q)
        J = km.updateJacobian(q)
        x_dot = cc.getCartesianreference(bTg, q)

        # Inverse kinematic step
        try:
            q_dot = np.linalg.pinv(J) @ x_dot
        except np.linalg.LinAlgError:
            # Use damped least squares if singular
            lambda_damping = 0.01
            q_dot = J.T @ np.linalg.inv(J @ J.T + lambda_damping * np.eye(6)) @ x_dot

        # Simulate joint update with limits
        q = KinematicSimulation(q, q_dot, dt, qmin, qmax)

        # Check convergence
        pos_error = np.linalg.norm(x_dot[3:6])
        ori_error = np.linalg.norm(x_dot[0:3])
        if pos_error < 0.01 and ori_error < 0.05:
            print("Reachable point found in", i, "iterations.")
            return True

    # If loop completes, goal is not reachable
    print("Goal is NOT reachable: final position/orientation error too large after", samples, "iterations.")
    raise RuntimeError("Goal is not reachable with current joint limits or kinematics.")
        

        
        
if __name__ == "__main__":
    
    robot = Robot()
    
    sensor_names = [
        "shoulder_pan_joint_sensor", 
        "shoulder_lift_joint_sensor",
        "elbow_joint_sensor", 
        "wrist_1_joint_sensor",
        "wrist_2_joint_sensor", 
        "wrist_3_joint_sensor"
        
    ]
    
    motor_names = [
        "shoulder_pan_joint", 
        "shoulder_lift_joint",
        "elbow_joint", 
        "wrist_1_joint",
        "wrist_2_joint", 
        "wrist_3_joint"
        
    ]

    

    # Initialize the geometric model for a UR5e robot
    # UR5e geometric model information
    # This part can be changed for any other manipulator with any number of joints 
    iTj_0 = [
        np.array([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]]),
        np.array([[0,-1,0,0],[0,0,-1,0],[1,0,0,0.1625],[0,0,0,1]]),
        np.array([[0,1,0,0],[-1,0,0,0.425],[0,0,1,0],[0,0,0,1]]),
        np.array([[0,-1,0,-0.3922],[1,0,0,0],[0,0,1,0],[0,0,0,1]]),
        np.array([[0,0,-1,0],[-1,0,0,0],[0,1,0,0.1333],[0,0,0,1]]),
        np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0.0997],[0,0,0,1]]),
        
    ]
    
    # The number of joints here should be euqal to the number of matrices in iTj_0
    joint_type = [0, 0, 0, 0, 0, 0]  # 0 for revolute, 1 for prismatic
    
    # Tool transformation relative to the end-effector frame
    eTt = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.0996],
                    [0, 0, 0, 1]]) 
    
    # initial joint configuration for testing
    q0 = [0,0,0,0,0,0]
    
    
    tool = True

    geometric_model = GeometricModel(iTj_0, joint_type,eTt)
    
    
    iTj = geometric_model.UpdateDirectGeometry(q0)
    print("iTj for 0 is",  iTj)
    
    bTe = geometric_model.GetTransformWrtBase(iTj, geometric_model.jointNumber)
    print("bTi for joint 6 is" , bTe)
    
    bTt = geometric_model.getToolTransformWrtBase(q0)
    print("Tool wrt base at initial is: " , np.round(bTt, 3))
    
    kinematic_model = KinematicModel(geometric_model , tool)
    bTt = geometric_model.getToolTransformWrtBase(q0)
    print(np.round(bTt, 3))
    
    

    robot_base = RobotBase(robot, sensor_names, motor_names, geometric_model, setupby_vel=True)

    
    
    """
        Inverse Kinematic Control
    
    """ 
    # Goal definition 
    
    # --- Get desired goal pose from user as a list ---
    user_input = input("Enter desired x, y, z, roll, pitch, yaw without braces seperated by spaces (example: x y z yaw pitch roll) (use 'pi' if you want, e.g. pi/6): ")

    # Split into tokens
    values = [parse_expr(v) for v in user_input.strip().split()]
    if len(values) != 6:
        raise ValueError("You must enter exactly 6 values: x y z roll pitch yaw")

    # Unpack
    x, y, z, roll, pitch, yaw = values
    
    bOg = np.array([[x], [y], [z]])
    b_eta_g = YPRToRot(yaw, pitch, roll)  # desired orientation of the end-effector in roll-pitch-yaw 
    # Example values:
    # 0.4 0.2 0.8 0 0 pi/2 
    
    
    # bOg = np.array([[0.4], [0.2], [0.8]])  # desired x,y,z position of the end-effector
    # b_eta_g = YPRToRot(0, 0, np.pi/6) # desired orientation of the end-effector in roll-pitch-yaw n
    
    
    bTg = np.vstack([
        
        np.hstack([b_eta_g, bOg]),
        np.array([[0, 0, 0, 1]])
        
        ])
    
   
    
    print(bTg)
    
    # control proportional gain
    k_a = 0.4
    k_l = 0.4
    
    # Cartesian control initialization
    cartesian_control = CartesianControl(geometric_model, k_a, k_l , tool)
    
    
    
    q = [0,0 , 0, 0, 0, 0]
    
    x_dot = cartesian_control.getCartesianreference(bTg, q)
    
    iTj = geometric_model.UpdateDirectGeometry(q)
    
    
    
    
    """
    Initialize control loop 
    
    """
    
    samples  = 2000
    t_start = 0.0
    t_end = 20.0
    dt = (t_end - t_start) / samples
    t = np.arange(t_start, t_end + dt, dt)  # ensures t_end is included
    
    # Joint upper and Lower bounds 
    qmin = -2*np.pi * np.ones((6,1)).flatten()
    qmax = 2*np.pi * np.ones((6,1)).flatten()
    
    
    # Pre allocation variables
    bTi = np.zeros((6,4,4))
    bri = np.zeros((8,3))
    
    iteration = 0
    # reachability_stat = Reachability(geometric_model , kinematic_model , cartesian_control , bTg , q, dt, samples, qmin, qmax)
    if True:
        
        
        while robot_base.step() != -1 and iteration < samples:
            
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
            if pos_error < 0.01 and ori_error < 0.05:
                print(f"Goal reached in {iteration} iterations!")
                final_pose = geometric_model.getToolTransformWrtBase(q_current)
                print(f"Final position: {np.round(final_pose[0:3, 3], 3)}")
                print(f"Desired position: {bOg.flatten()}")
                break
                
            iteration += 1
    
        # Stop the robot
        robot_base.actuate_joint_velocities([0.0] * 6)
        
        if iteration >= samples:
            print("Maximum iterations reached without convergence")
        
        print("Control loop finished")
   
    
    # for i in range(samples):
        
    #     # update the position of the goal frame 
    #     iTj = geometric_model.UpdateDirectGeometry(q)
        
    #     J = kinematic_model.updateJacobian(q)
        
    #     x_dot = cartesian_control.getCartesianreference(bTg, q)
        
    #     #### Inverse kinematic
        
    #     q_dot = np.linalg.pinv(J) @ x_dot
        
    #     q = KinematicSimulation(q , q_dot, dt , qmin, qmax)
    
        
    #     if np.linalg.norm(x_dot[0:3]) < 0.01 and np.linalg.norm(x_dot[3:6]) < 0.01:
            
    #         print ("Reached requested goal in " , t_start + i*dt , " seconds")
    #         break
            
    # print("Final joint configuration: ", np.round(q,3))
    # print ("Final end-effector pose: " , J@q)
    # print ("Final end effector position: " , geometric_model.getToolTransformWrtBase(q)[0:3,3])
    # print ("Desired end effector position: " , bTg[0:3,3])
    
    