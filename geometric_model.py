import numpy as np 

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
            
            iTj[i] = self.iTj_0[i] @ T1
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