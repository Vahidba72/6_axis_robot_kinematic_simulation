import numpy as np 

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