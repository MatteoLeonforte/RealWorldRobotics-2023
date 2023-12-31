
from typing import Dict

import torch
import numpy as np


JOINT_PARAMETER_NAMES = [
    'palm2plate',
    'finger1_base2pp',
    'finger1_base2pp_virt',
    'finger1_pp2mp_virt',
    'finger1_pp2mp',
    'finger1_mp2dp_virt',
    'finger1_mp2dp',
    'finger2_base2pp_virt',
    'finger2_base2pp',
    'finger2_pp2mp_virt',
    'finger2_pp2mp',
    'finger2_mp2dp_virt',
    'finger2_mp2dp',
    'finger3_base2pp_virt',
    'finger3_base2pp',
    'finger3_pp2mp_virt',
    'finger3_pp2mp',
    'finger3_mp2dp_virt',
    'finger3_mp2dp',
    'finger4_base2pp_virt',
    'finger4_base2pp',
    'finger4_pp2mp_virt',
    'finger4_pp2mp',
    'finger4_mp2dp_virt',
    'finger4_mp2dp',
    'finger5_base2pp_virt',
    'finger5_base2pp',
    'finger5_pp2mp_virt',
    'finger5_pp2mp',
    'finger5_mp2dp_virt',
    'finger5_mp2dp',
]

GC_TENDONS = {
    'palm2plate': {},
    'finger1_base2pp_virt': {'finger1_base2pp': 1},
    'finger1_pp2mp_virt': {'finger1_pp2mp': 1, 'finger1_mp2dp_virt': 1, 'finger1_mp2dp': 1},
    'finger2_base2pp_virt': {'finger2_base2pp': 1},
    'finger2_pp2mp_virt': {'finger2_pp2mp': 1, 'finger2_mp2dp_virt': 1, 'finger2_mp2dp': 1},
    'finger3_base2pp_virt': {'finger3_base2pp': 1},
    'finger3_pp2mp_virt': {'finger3_pp2mp': 1, 'finger3_mp2dp_virt': 1, 'finger3_mp2dp': 1},
    'finger4_base2pp_virt': {'finger4_base2pp': 1},
    'finger4_pp2mp_virt': {'finger4_pp2mp': 1, 'finger4_mp2dp_virt': 1, 'finger4_mp2dp': 1},
    'finger5_base2pp_virt': {'finger5_base2pp': 1},
    'finger5_pp2mp_virt': {'finger5_pp2mp': 1, 'finger5_mp2dp_virt': 1, 'finger5_mp2dp': 1},
}

FINGER_CHAINS = {
    'finger1': [
        'world',
        'world2root_fixed',
        'hand',
        'palm2plate',
        'finger1_base2pp_virt',
        'finger1_base2pp',
        'finger1_pp2mp_virt',
        'finger1_pp2mp',
        'finger1_mp2dp_virt',
        'finger1_mp2dp',
    ],
    'finger2': [
        'world',
        'world2root_fixed',
        'hand',
        'finger2_base2pp_virt',
        'finger2_base2pp',
        'finger2_pp2mp_virt',
        'finger2_pp2mp',
        'finger2_mp2dp_virt',
        'finger2_mp2dp',
    ],
    'finger3': [
        'world',
        'world2root_fixed',
        'hand',
        'finger3_base2pp_virt',
        'finger3_base2pp',
        'finger3_pp2mp_virt',
        'finger3_pp2mp',
        'finger3_mp2dp_virt',
        'finger3_mp2dp',
    ],
    'finger4': [
        'world',
        'world2root_fixed',
        'hand',
        'finger4_base2pp_virt',
        'finger4_base2pp',
        'finger4_pp2mp_virt',
        'finger4_pp2mp',
        'finger4_mp2dp_virt',
        'finger4_mp2dp',
    ],
    'finger5': [
        'world',
        'world2root_fixed',
        'hand',
        'finger5_base2pp_virt',
        'finger5_base2pp',
        'finger5_pp2mp_virt',
        'finger5_pp2mp',
        'finger5_mp2dp_virt',
        'finger5_mp2dp',
    ],
}

FINGER_TO_TIP: Dict[str, str] = {
    "finger1": "fingertip1",
    "finger2": "fingertip2",
    "finger3": "fingertip3",
    "finger4": "fingertip4",
    "finger5": "fingertip5",
}

FINGER_TO_BASE = {
    "finger1": "finger1_pp",
    "finger2": "finger2_pp",
    "finger3": "finger3_pp",
    "finger4": "finger4_pp",
    "finger5": "finger5_pp",
}



def get_mano_joints_dict(joints: torch.Tensor, include_wrist=False, batch_processing=False):
    # joints: 21 x 3
    # For retargeting, we don't need the wrist
    # For visualization, we need the wrist
    if not batch_processing:
        if not include_wrist:
            return {
                "finger1": joints[1:5, :],
                "finger2": joints[5:9, :],
                "finger3": joints[9:13, :],
                "finger4": joints[13:17, :],
                "finger5": joints[17:21, :],
            }
        else:
            return {
                "wrist": joints[0, :],
                "finger1": joints[1:5, :],
                "finger2": joints[5:9, :],
                "finger3": joints[9:13, :],
                "finger4": joints[13:17, :],
                "finger5": joints[17:21, :],
            }
    else:
        if not include_wrist:
            return {
                "finger1": joints[:, 1:5, :],
                "finger2": joints[:, 5:9, :],
                "finger3": joints[:, 9:13, :],
                "finger4": joints[:, 13:17, :],
                "finger5": joints[:, 17:21, :],
            }
        else:
            return {
                "wrist": joints[:, 0, :],
                "finger1": joints[:, 1:5, :],
                "finger2": joints[:, 5:9, :],
                "finger3": joints[:, 9:13, :],
                "finger4": joints[:, 13:17, :],
                "finger5": joints[:, 17:21, :],
            }


def get_mano_fingertips_batch(mano_joints_dict):
    return {
        "finger1": mano_joints_dict["finger1"][:, [3], :],
        "finger2": mano_joints_dict["finger2"][:, [3], :],
        "finger3": mano_joints_dict["finger3"][:, [3], :],
        "finger4": mano_joints_dict["finger4"][:, [3], :],
        "finger5": mano_joints_dict["finger5"][:, [3], :],
    }

def get_mano_pps_batch(mano_joints_dict):
    return {
        "finger1": mano_joints_dict["finger1"][:, [0], :],
        "finger2": mano_joints_dict["finger2"][:, [0], :],
        "finger3": mano_joints_dict["finger3"][:, [0], :],
        "finger4": mano_joints_dict["finger4"][:, [0], :],
        "finger5": mano_joints_dict["finger5"][:, [0], :],
    }

def get_keyvectors(fingertips: Dict[str, torch.Tensor], palm: torch.Tensor):
    return {
        'palm2finger1': fingertips['finger1'] - palm,
        'palm2finger2': fingertips['finger2'] - palm,
        'palm2finger3': fingertips['finger3'] - palm,
        'palm2finger4': fingertips['finger4'] - palm,
        'palm2finger5': fingertips['finger5'] - palm,
        'finger12finger2': fingertips['finger2'] - fingertips['finger1'],
        'finger12finger3': fingertips['finger3'] - fingertips['finger1'],
        'finger12finger4': fingertips['finger4'] - fingertips['finger1'],
        'finger12finger5': fingertips['finger5'] - fingertips['finger1'],
        'finger22finger3': fingertips['finger3'] - fingertips['finger2'],
        'finger22finger4': fingertips['finger4'] - fingertips['finger2'],
        'finger22finger5': fingertips['finger5'] - fingertips['finger2'],
        'finger32finger4': fingertips['finger4'] - fingertips['finger3'],
        'finger32finger5': fingertips['finger5'] - fingertips['finger3'],
        'finger42finger5': fingertips['finger5'] - fingertips['finger4'],
    }


def rotation_matrix_z(angle):
    """
    Returns a 3x3 rotation matrix about the z-axis for the given angle.
    """
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rot_mat = np.array([[cos_theta, -sin_theta, 0],
                        [sin_theta, cos_theta, 0],
                        [0, 0, 1]])
    return rot_mat


def rotation_matrix_y(angle):
    """
    Returns a 3x3 rotation matrix about the y-axis for the given angle.
    """
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rot_mat = np.array([[cos_theta, 0, sin_theta],
                        [0, 1, 0],
                        [-sin_theta, 0, cos_theta]])
    return rot_mat


def rotation_matrix_x(angle):
    """
    Returns a 3x3 rotation matrix about the x-axis for the given angle.
    """
    cos_theta = np.cos(angle)
    sin_theta = np.sin(angle)
    rot_mat = np.array([[1, 0, 0],
                        [0, cos_theta, -sin_theta],
                        [0, sin_theta, cos_theta]])
    return rot_mat

def get_hand_rotation_matrix(joint_pos):
    '''
    param: joint_pos, a numpy array of 3D joint positions (MANO format)

    Returns the rotation matrix that normalizes the joint orientation. 
    '''

    # normalize the translation of the hand: set the wrist point to zero
    wrist_point = joint_pos[0, :]
    joint_pos -= wrist_point

    # construct a plane from wrist, first finger2 finger joint, first finger5 joint
    joint_dict = get_mano_joints_dict(joint_pos, include_wrist=True)
    wrist_point = joint_dict['wrist']
    finger2_point = joint_dict['finger2'][0]
    finger5_point = joint_dict['finger5'][0]

    # find basis vectors for the plane
    base_1 = finger2_point - wrist_point
    base_2 = finger5_point - wrist_point
    normal_vec = np.cross(base_1, base_2)
    base_2 = np.cross(normal_vec, base_1)

    # normalize basis vectors
    normal_vec /= np.linalg.norm(normal_vec)
    base_1 /= np.linalg.norm(base_1)
    base_2 /= np.linalg.norm(base_2)

    # construct the matrix for the base change from the hand frame basis vectors
    base_matrix = np.zeros((3, 3))
    base_matrix[:, 0] = base_1
    base_matrix[:, 1] = base_2
    base_matrix[:, 2] = normal_vec

    return base_matrix @ rotation_matrix_z(-np.pi/2)

def normalize_points(joint_pos, flip_x_axis=True, flip_y_axis=False, add_z_rotation=0):
    '''
    param: joint_pos, a numpy array of 3D joint positions (MANO format)

    Returns the joint positions with normalized translation and rotation. 
    '''

    # normalize the translation of the hand: set the wrist point to zero
    wrist_point = joint_pos[0, :]
    joint_pos -= wrist_point

    # construct a plane from wrist, first finger2 finger joint, first finger5 joint
    joint_dict = get_mano_joints_dict(joint_pos, include_wrist=True)
    wrist_point = joint_dict['wrist']
    finger2_point = joint_dict['finger2'][0]
    finger5_point = joint_dict['finger5'][0]

    # find basis vectors for the plane
    base_1 = finger2_point - wrist_point
    base_2 = finger5_point - wrist_point
    normal_vec = np.cross(base_1, base_2)
    base_2 = np.cross(normal_vec, base_1)

    # normalize basis vectors
    normal_vec /= np.linalg.norm(normal_vec)
    base_1 /= np.linalg.norm(base_1)
    base_2 /= np.linalg.norm(base_2)

    # construct the matrix for the base change from the hand frame basis vectors
    base_matrix = np.zeros((3, 3))
    base_matrix[:, 0] = base_1
    base_matrix[:, 1] = base_2
    base_matrix[:, 2] = normal_vec

    # need to rotate around z axis, order of basis vectors in hand frame might be switched up
    joint_pos = joint_pos @ base_matrix @ rotation_matrix_z(-np.pi/2 + add_z_rotation)

    if flip_y_axis:
        joint_pos = joint_pos @ rotation_matrix_y(np.pi)

    if flip_x_axis:
        # flip the x axis
        joint_pos[:, 0] *= -1

    return joint_pos


def get_unoccluded_hand_joint_idx(joint_pos):
    '''
    param: joint_pos, a numpy array of 3D joint positions (MANO format), not normalized
    Returns the joint that has the least z value and should be visible in the image (y value is in the direction of the camera).
    We can then project this joint into 3D space, and then from there get the 3D position of the wrist (which may be occluded)
    '''

    # get the joint with the lowest z value (closest to camera)
    max_joint_idx = np.argmin(joint_pos[:, 2])
    return max_joint_idx


def get_wrist_translation(joint_idx, joint_pos):
    '''
    param: joint_idx, the finger2 of the joint with the highest y value
    param: joint_pos, a numpy array of 3D joint positions (MANO format), not normalized
    Returns the translation of the wrist in the hand frame relative to the joint_idx joint
    '''

    # get the 3D position of the wrist
    joint = joint_pos[joint_idx, :]
    wrist = joint_pos[0, :]

    return wrist - joint


def rolling_average_filter(positions, new_pos):
    '''
    A rolling average filter for the wrist position.
    param: positions, a numpy array of 3D positions of the wrist
    param: new_pos, a numpy array of the new 3D position of the wrist
    '''

    positions = np.roll(positions, -1, axis=0)
    positions[-1, :] = new_pos

    return positions, np.nanmean(positions, axis=0)


# Actually not used in frankmocap default
def compute_rotation_matrix_from_ortho6d(poses):
    """
    Code from
    https://github.com/papagina/RotationContinuity
    On the Continuity of Rotation Representations in Neural Networks
    Zhou et al. CVPR19
    https://zhouyisjtu.github.io/project_rotation/rotation.html
    """
    x_raw = poses[:, 0:3]  # batch*3
    y_raw = poses[:, 3:6]  # batch*3
        
    x = normalize_vector(x_raw)  # batch*3
    z = cross_product(x, y_raw)  # batch*3
    z = normalize_vector(z)  # batch*3
    y = cross_product(z, x)  # batch*3
        
    x = x.reshape(-1, 3, 1)
    y = y.reshape(-1, 3, 1)
    z = z.reshape(-1, 3, 1)
    matrix = np.concatenate((x, y, z), 2)  # batch*3*3
    return matrix

def normalize_vector(v):
    batch = v.shape[0]
    v_mag = np.sqrt((v**2).sum(1))  # batch
    v_mag = np.maximum(v_mag, np.array([1e-8]))
    v_mag = np.broadcast_to(v_mag.reshape(batch, 1), (batch, v.shape[1]))
    v = v/v_mag
    return v


def cross_product(u, v):
    batch = u.shape[0]
    i = u[:, 1] * v[:, 2] - u[:, 2] * v[:, 1]
    j = u[:, 2] * v[:, 0] - u[:, 0] * v[:, 2]
    k = u[:, 0] * v[:, 1] - u[:, 1] * v[:, 0]
        
    out = np.concatenate((i.reshape(batch, 1), j.reshape(batch, 1), k.reshape(batch, 1)), 1)
        
    return out


def normalize_points_rokoko(joint_pos, mirror_x=False, flip_y_axis=False, flip_x_axis=False):
    # normalize the translation of the hand: set the wrist point to zero
    wrist_point = joint_pos[0, :]
    joint_pos -= wrist_point

    # construct a plane from wrist, first finger2 finger joint, first finger5 joint
    joint_dict = get_mano_joints_dict(joint_pos, include_wrist=True)
    wrist_point = joint_dict['wrist']
    finger3_point = joint_dict['finger3'][0]
    finger5_point = joint_dict['finger5'][0]
    # find basis vectors for the plane
    base_1 = finger3_point - wrist_point
    base_2 = finger5_point - wrist_point
    normal_vec = np.cross(base_1, base_2)
    base_2 = np.cross(normal_vec, base_1)

    # normalize basis vectors
    normal_vec /= np.linalg.norm(normal_vec)
    base_1 /= np.linalg.norm(base_1)
    base_2 /= np.linalg.norm(base_2)

    # construct the matrix for the base change from the hand frame basis vectors
    base_matrix = np.zeros((3, 3))
    base_matrix[:, 0] = base_1
    base_matrix[:, 1] = base_2
    base_matrix[:, 2] = normal_vec

    # need to rotate around z axis, order of basis vectors in hand frame might be switched up
    joint_pos = joint_pos @ base_matrix @ rotation_matrix_z(-np.pi/2)

    if flip_y_axis:
        joint_pos = joint_pos @ rotation_matrix_y(np.pi)

    if flip_x_axis:
        # flip the x axis
        joint_pos = joint_pos @ rotation_matrix_x(-np.pi/2)

    rot_matrix = base_matrix

    z_axis = wrist_point - finger3_point
    
    rot_matrix[:,0] = -base_2
    rot_matrix[:,1] = -normal_vec
    rot_matrix[:,2] = base_1
    return joint_pos, rot_matrix