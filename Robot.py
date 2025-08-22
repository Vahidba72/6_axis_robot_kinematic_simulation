import numpy as np
import time 

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
    def __init__(self, geometric_model):
        """
        Initializes the kinematic model with a geometric model.
        
        Inputs:
        - geometric_model: An instance of GeometricModel
        """
        self.gm = geometric_model
        self.J = np.zeros((6,self.gm.jointNumber))
        self.RJ = np.zeros((6,6))
        
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
        
        tool = True
        
        if tool:
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
                
                

if __name__ == "__main__":

    

    
    # UR5e geometric model information
    iTj_0 = [
        np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.193],[0,0,0,1]]),
        np.array([[0,0,-1,-0.1805],[0,1,0,0],[1,0,0,0],[0,0,0,1]]),
        np.array([[0,1,0,0.615],[1,0,0,0],[0,0,-1,0],[0,0,0,1]]),
        np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0.141],[0,0,0,1]]),
        np.array([[0,1,0,0],[1,0,0,0.571],[0,0,-1,0],[0,0,0,1]]),
        np.array([[0,0,1,0],[0,1,0,0],[-1,0,0,0.138],[0,0,0,1]]),
        np.array([[0,0,-1,0],[0,1,0,0],[1,0,0,0.118],[0,0,0,1]])
    ]
    joint_type = [0, 0, 0, 1, 0, 0, 0]  # 0 for revolute, 1 for prismatic
    eTt = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0.1103],
                    [0, 0, 0, 1]]) 
    
    
    

    geometric_model = GeometricModel(iTj_0, joint_type,eTt)
    #print(geometric_model.iTj_0[1])
    #print(geometric_model.getFrameWrtFrame(1,2))
    
    iTj = geometric_model.UpdateDirectGeometry([0, -np.pi/6, np.pi/3, 0, np.pi/2, np.pi/2,0])
    #print(iTj[0])
    
    bTt = geometric_model.getToolTransformWrtBase([0, -np.pi/6, np.pi/3, 0, np.pi/2, np.pi/2,0])
    print(np.round(bTt, 3))
    
    kinematic_model = KinematicModel(geometric_model)
    J_total = kinematic_model.updateJacobian([0, -np.pi/6, np.pi/3, 0, np.pi/2, np.pi/2,0])
    
    print(np.round(J_total, 3))
    print(np.round(kinematic_model.RJ, 3))
    bTi = geometric_model.GetTransformWrtBase(iTj, 1)
    print(np.round(bTi, 3))
    
    
        
    