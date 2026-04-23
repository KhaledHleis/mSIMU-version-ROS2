import numpy as np
def Absolute_position(parent_absolute_position, parent_heading, relative_position):
    """
    Convert a relative position to an absolute position in 3D space given a parent's 
    absolute position and heading in the NED reference frame.
    
    This function transforms coordinates from a local reference frame (relative to a parent object)
    to a global NED reference frame. It applies rotation about the z-axis (yaw/heading) and then 
    translates by the parent's absolute position. Roll and pitch are assumed to be zero.
    
    Args:
        parent_absolute_position (tuple or list): The (north, east, down) coordinates of the parent 
            in the absolute/global NED reference frame.
        parent_heading (float): The heading/yaw angle of the parent in radians. This is the 
            rotation about the down-axis, measured clockwise from North when looking down.
        relative_position (tuple or list): The (north, east, down) coordinates in the parent's local 
            reference frame that need to be converted to absolute coordinates.
    
    Returns:
        tuple: The (north, east, down) absolute position in the global NED reference frame.
    """
    import math
    
    # Extract coordinates (NED frame)
    pn, pe, pd = parent_absolute_position
    rn, re, rd = relative_position
    
    # Apply rotation transformation based on parent's heading (rotation about down-axis)
    # In NED, heading is measured clockwise from North when looking down
    # Rotation matrix for yaw (heading) in NED:
    # n' = n*cos(ψ) + e*sin(ψ)
    # e' = -n*sin(ψ) + e*cos(ψ)
    # d' = d (unchanged)
    rotated_n = rn * math.cos(parent_heading) + re * math.sin(parent_heading)
    rotated_e = -rn * math.sin(parent_heading) + re * math.cos(parent_heading)
    rotated_d = rd  # down-coordinate unchanged for rotation about down-axis
    
    # Translate by parent's absolute position
    absolute_n = pn + rotated_n
    absolute_e = pe + rotated_e
    absolute_d = pd + rotated_d
    
    absolute_position = np.array([[absolute_n,absolute_e,absolute_d]])
    
    return absolute_position

def convert_field_ned_to_body(field_ned, roll, pitch, yaw):
    """
    Convert a magnetic field vector from NED (world) frame to sensor body frame.
    
    In the body frame:
    - x-axis points forward (along drone nose)
    - y-axis points right (starboard)
    - z-axis points down
    
    Args:
        field_ned (np.ndarray): Magnetic field vector in NED frame [Bn, Be, Bd]
        roll (float): Roll angle in radians (rotation about x-axis, positive right wing down)
        pitch (float): Pitch angle in radians (rotation about y-axis, positive nose up)
        yaw (float): Yaw/heading angle in radians (rotation about z-axis, positive clockwise from North)
    
    Returns:
        np.ndarray: Magnetic field vector in body frame [Bx, By, Bz]
    """
    import numpy as np
    
    # Rotation matrix from NED to Body frame
    # This is the standard aerospace rotation sequence: Yaw -> Pitch -> Roll (Z-Y-X Euler angles)
    
    cos_roll = np.cos(roll)
    sin_roll = np.sin(roll)
    cos_pitch = np.cos(pitch)
    sin_pitch = np.sin(pitch)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    
    # R_NED_to_Body = R_roll * R_pitch * R_yaw
    R = np.array([
        [cos_pitch * cos_yaw,
         cos_pitch * sin_yaw,
         -sin_pitch],
        
        [sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
         sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
         sin_roll * cos_pitch],
        
        [cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
         cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
         cos_roll * cos_pitch]
    ])
    
    # Transform the field vector
    field_body = R @ field_ned.T
    
    return field_body