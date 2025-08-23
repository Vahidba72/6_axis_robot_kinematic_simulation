import numpy as np 

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