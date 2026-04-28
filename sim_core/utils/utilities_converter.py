import numpy as np
from pyproj import Transformer

def body_to_ned(parent_absolute_position, roll, pitch, yaw, relative_position):
    """
    Convert a position vector from body frame to absolute NED frame.
    Full Z-Y-X Euler rotation (yaw → pitch → roll) + translation.

    Args:
        parent_absolute_position : (3,) array-like  – [north, east, down] of parent in NED
        roll                     : float            – rotation about x-axis (rad), positive right wing down
        pitch                    : float            – rotation about y-axis (rad), positive nose up
        yaw                      : float            – rotation about z-axis (rad), positive clockwise from North
        relative_position        : (3,) array-like  – [north, east, down] in body frame

    Returns:
        np.ndarray (3,): absolute [north, east, down] in NED frame
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    # R_body_to_NED  (transpose of NED-to-body)
    R = np.array([
        [ cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [ cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,     sr*cp,             cr*cp            ]
    ])

    return np.array(parent_absolute_position) + R @ np.array(relative_position)


def ned_to_body(vector_ned, roll, pitch, yaw):
    """
    Rotate a free vector (field, velocity, …) from NED frame to body frame.
    Full Z-Y-X Euler rotation (yaw → pitch → roll). No translation.

    Args:
        vector_ned : (3,) array-like – vector expressed in NED frame
        roll       : float           – rotation about x-axis (rad), positive right wing down
        pitch      : float           – rotation about y-axis (rad), positive nose up
        yaw        : float           – rotation about z-axis (rad), positive clockwise from North

    Returns:
        np.ndarray (3,): same vector expressed in body frame
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    # R_NED_to_body = R_roll @ R_pitch @ R_yaw
    R = np.array([
        [ cp*cy,          cp*sy,         -sp   ],
        [ sr*sp*cy-cr*sy, sr*sp*sy+cr*cy, sr*cp],
        [ cr*sp*cy+sr*sy, cr*sp*sy-sr*cy, cr*cp]
    ])

    return R @ np.array(vector_ned)



def lld_to_ned_batch(input: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """
    Convert GNSS coordinates (longitude, latitude, depth) to local Cartesian NED coordinates in meters.
    
    Parameters:
    - input: np.ndarray, shape (N, 3), columns = [longitude, latitude, depth]
    - ref: np.ndarray, shape (1, 3), reference point [longitude, latitude, depth]
    
    Returns:
    - np.ndarray of shape (N, 3), local coordinates in meters in NED frame [x_north, y_east, z_down]
    """

    # Validate input
    assert input.ndim == 2 and input.shape[1] == 3, f"Input must be 2D with 3 columns, got {input.shape}"
    assert ref.ndim == 2 and ref.shape == (1, 3), f"Reference must have shape (1,3), got {ref.shape}"

    # Extract reference longitude, latitude, and depth
    lon0, lat0, depth0 = ref[0]

    # Setup a local ENU (East-North-Up) projection using pyproj
    transformer = Transformer.from_crs(
        "epsg:4326",  # WGS84 Lat/Lon
        f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",  # Local tangent plane
        always_xy=True
    )

    # Transform all points
    x_enu = []  # East
    y_enu = []  # North
    for lon, lat, depth in input:
        x, y = transformer.transform(lon, lat)
        x_enu.append(x)
        y_enu.append(y)
    x_enu = np.array(x_enu)
    y_enu = np.array(y_enu)

    # Depth as z (positive down in both frames)
    z_down = input[:, 2] - depth0

    # Convert ENU to NED: NED = [N, E, D] = [y_enu, x_enu, z_down]
    local_coords = np.stack([y_enu, x_enu, z_down], axis=1)

    return local_coords


def lld_to_ned(input: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """
    Convert GNSS coordinates (longitude, latitude, depth) to local Cartesian NED coordinates in meters.
    
    Parameters:
    - input: np.ndarray, shape (1, 3), columns = [longitude, latitude, depth]
    - ref: np.ndarray, shape (1, 3), reference point [longitude, latitude, depth]
    
    Returns:
    - np.ndarray of shape (1, 3), local coordinates in meters in NED frame [x_north, y_east, z_down]
    """

    # Validate input
    assert input.ndim == 2 and input.shape[1] == 3, f"Input must be 2D with 3 columns, got {input.shape}"
    assert ref.ndim == 2 and ref.shape == (1, 3), f"Reference must have shape (1,3), got {ref.shape}"

    # Extract reference longitude, latitude, and depth
    lon0, lat0, depth0 = ref[0]

    # Setup a local ENU (East-North-Up) projection using pyproj
    transformer = Transformer.from_crs(
        "epsg:4326",  # WGS84 Lat/Lon
        f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84",  # Local tangent plane
        always_xy=True
    )
    lon,lat,depth = input.flatten()
    # Transform all points
    x, y = transformer.transform(lon, lat)

    # Depth as z (positive down in both frames)
    z = depth - depth0

    # Convert ENU to NED: NED = [N, E, D] = [y_enu, x_enu, z_down]
    local_coord = np.stack([[y, x, z]], axis=1)

    return local_coord