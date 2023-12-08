#!/usr/bin/env python3
import time
import numpy as np
import torch
from torch.nn.functional import normalize
import os
import pytorch_kinematics as pk
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from utils import retarget_utils, gripper_utils
import numpy as np


class RetargeterNode:
    def __init__(
        self,
        device: str = "cuda",
        lr: float = 2.5,
        hardcoded_keyvector_scaling: bool = True,
        use_scalar_distance_palm: bool = True,
    ) -> None:
        '''
        RetargeterNode
        Requires urdf file of hand (change urdf_path and urdf_filename)
        retarget_utils and gripper_utils contain functions and hardcoded values for the faive hand, will need to be changed for other hands
        '''
        
        self.target_angles = None

        self.device = device
        
        self.base_path = os.path.dirname(os.path.realpath(__file__))

        # self.joint_map = torch.zeros(30, 11).to(device)
        self.joint_map = torch.zeros(31, 11).to(device) # changed to this


        joint_parameter_names = retarget_utils.JOINT_PARAMETER_NAMES
        gc_tendons = retarget_utils.GC_TENDONS

        # CHECK FUNCTION

        for i, (name, tendons) in enumerate(gc_tendons.items()):
            self.joint_map[joint_parameter_names.index(
                name), i] = 1 if len(tendons) == 0 else 0.5
            for tendon, weight in tendons.items():
                self.joint_map[joint_parameter_names.index(
                    tendon), i] = weight * 0.5

        self.urdf_path = self.base_path + "/../../design/"              # Change here
        self.urdf_filename = self.urdf_path + "hand_design.xml"         # Change here

        prev_cwd = os.getcwd()
        os.chdir(self.urdf_path)
        self.chain = pk.build_chain_from_mjcf(                          # changed this to get mjcf and not urdf
            open(self.urdf_filename).read(), 'hand').to(device=self.device) # Changed root --> hand
        os.chdir(prev_cwd)

        self.gc_joints = torch.ones(11).to(self.device) * 30.0
        self.gc_joints.requires_grad_()

        self.lr = lr
        self.opt = torch.optim.RMSprop([self.gc_joints], lr=self.lr)

        self.root = torch.zeros(1, 3).to(self.device)
        self.palm_offset = torch.tensor([0.0, 0.06, 0.03]).to(self.device) # changed this (palm offset of robot hand)

        
        self.scaling_coeffs = torch.tensor([0.7143, 0.8296296296, 0.8214285714, 0.7857142857, 0.7037037037, 0.5897435897, 0.6976744186, 0.6595744681, 0.6274509804,
                                            0.9523809524, 0.7294117647, 0.8130081301, 0.6666666667, 0.7590361446, 1]).to(self.device) # tuned
        
        self.scaling_factors_set = hardcoded_keyvector_scaling
        
        self.loss_coeffs = torch.tensor([5.0, 5.0, 5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0,
                                            1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).to(self.device)

        if use_scalar_distance_palm:
            self.use_scalar_distance = [False, True, True, True, True, False, False, False, False, False, False, False, False, False, False]
        else:
            self.use_scalar_distance = [False, False, False, False, False, False, False, False, False, False, False, False, False, False, False]



        self.sub = rospy.Subscriber(
            '/ingress/mano', Float32MultiArray, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher(
            '/faive/policy_output', Float32MultiArray, queue_size=10)
    

    def retarget_finger_mano_joints(self, joints: np.array, warm: bool = True, opt_steps: int = 2, dynamic_keyvector_scaling: bool = False):
        """
        Process the real MANO (nomalized) joint positions and update the finger joint angles
        joints: (21, 3)
        Over the 21 dims:
        0-4: thumb (from hand base)
        5-8: index
        9-12: middle
        13-16: ring
        17-20: pinky
        --> changed these namings to finger1 - finger5
        """

        #print(f"Retargeting: Warm: {warm} Opt steps: {opt_steps}")  # uncomment
        
        start_time = time.time()

        if not warm:
            self.gc_joints = torch.ones(11).to(self.device) * 30.0
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            21, 3), "The shape of the mano joints array should be (21, 3)"

        joints = torch.from_numpy(joints).to(self.device)

        mano_joints_dict = retarget_utils.get_mano_joints_dict(joints)

        mano_fingertips = {}
        for finger, finger_joints in mano_joints_dict.items():
            mano_fingertips[finger] = finger_joints[[-1], :]

        mano_pps = {}
        for finger, finger_joints in mano_joints_dict.items():
            mano_pps[finger] = finger_joints[[0], :]

        mano_palm = torch.mean(torch.cat([joints[[0], :], mano_pps["finger2"], mano_pps["finger5"]], dim=0).to(     # Changed naming here
            self.device), dim=0, keepdim=True)

        keyvectors_mano = retarget_utils.get_keyvectors(
            mano_fingertips, mano_palm)
        # norms_mano = {k: torch.norm(v) for k, v in keyvectors_mano.items()}
        # print(f"keyvectors_mano: {norms_mano}")

        gc_limits_lower = gripper_utils.GC_LIMITS_LOWER
        gc_limits_upper = gripper_utils.GC_LIMITS_UPPER

        for step in range(opt_steps):
            chain_transforms = self.chain.forward_kinematics(
                self.joint_map @ (self.gc_joints/(180/np.pi)))
            fingertips = {}
            for finger, finger_tip in retarget_utils.FINGER_TO_TIP.items():
                fingertips[finger] = chain_transforms[finger_tip].transform_points(
                    self.root)

            palm = chain_transforms["hand"].transform_points(       # Changed root --> hand
                self.root) + self.palm_offset

            keyvectors_faive = retarget_utils.get_keyvectors(fingertips, palm)
            # norms_faive = {k: torch.norm(v) for k, v in keyvectors_faive.items()}
            # print(f"keyvectors_faive: {norms_faive}")

            # if step == 0:
            #     for i, (keyvector_faive, keyvector_mano) in enumerate(zip(keyvectors_faive.values(), keyvectors_mano.values())):
            #         self.scaling_coeffs[i] = torch.norm(
            #             keyvector_mano, p=2) / torch.norm(keyvector_faive, p=2)
            # print(f'Scaling factors: {self.scaling_coeffs.shape}')

            loss = 0
            
            if dynamic_keyvector_scaling or not self.scaling_factors_set:
                for i, (keyvector_faive, keyvector_mano) in enumerate(zip(keyvectors_faive.values(), keyvectors_mano.values())):
                    with torch.no_grad():
                        scaling_factor = torch.norm(
                            keyvector_mano, p=2) / torch.norm(keyvector_faive, p=2)
                        self.scaling_coeffs[i] = scaling_factor
                    
                    #print(  # uncomment
                     #   f'Keyvector {i} length ratio: {torch.norm(keyvector_mano, p=2) / (self.scaling_coeffs[i] * torch.norm(keyvector_faive, p=2))}')

            for i, (keyvector_faive, keyvector_mano) in enumerate(zip(keyvectors_faive.values(), keyvectors_mano.values())):
                if not self.use_scalar_distance[i]:
                    loss += self.loss_coeffs[i] * torch.norm(keyvector_mano -
                                    keyvector_faive * self.scaling_coeffs[i].detach()) ** 2
                else:
                    loss += self.loss_coeffs[i] * torch.norm(torch.norm(keyvector_mano) -
                                    torch.norm(keyvector_faive * self.scaling_coeffs[i].detach())) ** 2

            self.scaling_factors_set = True
            rospy.loginfo(f"Retargeting: Step: {step} Loss: {loss.item()}")
            self.opt.zero_grad()
            loss.backward()
            self.opt.step()
            with torch.no_grad():
                self.gc_joints[:] = torch.clamp(self.gc_joints, torch.tensor(gc_limits_lower).to(
                    self.device), torch.tensor(gc_limits_upper).to(self.device))

        finger_joint_angles = self.gc_joints.detach().cpu().numpy()

        #print(f'Retarget time: {(time.time() - start_time) * 1000} ms') # uncomment

        # HARDCODING JOINT ANGLES IN DEGREES - MATTEO
        ''' Here I take the normalized joint positions from the MANO and I calculate the joint angles of the fingers'''

        finger_joints_dict = {
            "finger1": [0, 1, 2, 3, 4],
            "finger2": [0, 5, 6, 7],
            "finger3": [0, 9, 10, 11],
            "finger4": [0, 13, 14, 15],
            "finger5": [0, 17, 18, 19],
        }
        
        real_hand_joint_angles = []

        def calculate_angle(point1, point2, point3):
            vector_a = point2 - point1
            vector_b = point3 - point2

            dot_product = np.dot(vector_a, vector_b)
            norm_a = np.linalg.norm(vector_a)
            norm_b = np.linalg.norm(vector_b)

            cosine_angle = dot_product / (norm_a * norm_b)
            angle = np.arccos(cosine_angle)

            return np.degrees(angle)


        for finger, finger_joints in finger_joints_dict.items():
            point1 = joints[finger_joints[0], :]
            point2 = joints[finger_joints[1], :]
            point3 = joints[finger_joints[2], :]
            point4 = joints[finger_joints[3], :]
            point5 = joints[finger_joints[4], :]

            angle_1 = calculate_angle(point1, point2, point3)
            angle_2 = calculate_angle(point2, point3, point4)
            real_hand_joint_angles.append(angle_1)
            real_hand_joint_angles.append(angle_2)

            if finger == "finger1":
                angle_3 = calculate_angle(point3, point4, point5)
                real_hand_joint_angles.append(angle_3)
            
        assert len(real_hand_joint_angles) == 11, "Expected 11 joint angles"


        return real_hand_joint_angles
        #return finger_joint_angles

    def callback(self, msg):
        # Convert the flattened data back to a 2D numpy array
        joints = np.array(msg.data, dtype=np.float32).reshape(
            msg.layout.dim[0].size, msg.layout.dim[1].size)
       
        self.target_angles = self.retarget_finger_mano_joints(joints)
        # DESIRED ANGLES - IMPORTANT
        print(f"Angles: {self.target_angles}")

        time = rospy.Time.now()
        assert self.target_angles.shape == (
            11,), "Expected different output format from retargeter"

        msg = Float32MultiArray

        # Create a Float32MultiArray message and set its 'data' field to the flattened array
        arr = self.target_angles
        msg = Float32MultiArray()
        msg.data = arr.flatten().tolist()

        # Set the 'layout' field of the message to describe the shape of the original array
        rows_dim = MultiArrayDimension()
        rows_dim.label = 'rows'
        rows_dim.size = arr.shape[0]
        rows_dim.stride = 1

        cols_dim = MultiArrayDimension()
        cols_dim.label = 'cols'
        cols_dim.size = 1
        cols_dim.stride = 1

        msg.layout.dim = [rows_dim, cols_dim]

        msg.data = self.target_angles
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('mano_faive_retargeter', anonymous=True)
    retargeter = RetargeterNode(device="cpu")
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
