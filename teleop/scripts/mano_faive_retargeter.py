#!/usr/bin/env python3
import time
import numpy as np
import torch
from torch.nn.functional import normalize
import os
import pytorch_kinematics as pk
import rospy
import tf2_ros
from std_msgs.msg import Float32MultiArray, MultiArrayDimension 
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped

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

        def hide_code_MATTEO():
            '''
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

            '''
            pass

        self.sub = rospy.Subscriber(
            '/ingress/mano', Float32MultiArray, self.callback, queue_size=1, buff_size=2**24)
        self.pub = rospy.Publisher(
            '/faive/policy_output', Float32MultiArray, queue_size=10)
        
        # RVIZ
        tf_buffer = tf2_ros.Buffer()
        tf_broadcaster = tf2_ros.TransformBroadcaster()
        print("AAAAAAAAAAAAAAAAAA")
        # Set up map fram in RVIZ
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = "world"
        transform_stamped.child_frame_id = "map"  # Replace with your desired child frame
        transform_stamped.transform.translation.x = 0.0  # Adjust translation values
        transform_stamped.transform.translation.y = 0.0
        transform_stamped.transform.translation.z = 0.0
        transform_stamped.transform.rotation.x = 0.0  # Adjust rotation values
        transform_stamped.transform.rotation.y = 0.0
        transform_stamped.transform.rotation.z = 0.0
        transform_stamped.transform.rotation.w = 1.0
        print("BBBBBBBBBBBBBBBBBBB")

        tf_broadcaster.sendTransform(transform_stamped)

        print("CCCCCCCCCCCCCCCCCCCCC")
        print("DDDDDDDDDDDDDDDDDDDD")
        
        # Set up hand visualization publisher
        self.pub_marker = rospy.Publisher(
            '/mano_viz', MarkerArray, queue_size=10)



    def convert_to_point(self, joint)->Point:
        point = Point()
        point.x, point.y, point.z = joint
        return point

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

        if not warm: # CHECK
            self.gc_joints = torch.ones(11).to(self.device) * 30.0
            self.gc_joints.requires_grad_()

        assert joints.shape == (
            21, 3), "The shape of the mano joints array should be (21, 3)"

        joints = torch.from_numpy(joints).to(self.device)

        # The following commented code is the original remapping of the MANO joints to the faive joints
        def hide_code_ORIGINAL_MATTEO():
            '''

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
            '''
            #print(f'Retarget time: {(time.time() - start_time) * 1000} ms') # uncomment 
            pass

        # HARDCODING JOINT ANGLES IN DEGREES - MATTEO
        ''' Here I take the normalized joint positions from the MANO and I calculate the joint angles of the fingers'''

        real_hand_joint_angles = np.zeros(11)

        def calculate_angle(vector_a, vector_b):

            dot_product = np.dot(vector_a, vector_b)
            norm_a = np.linalg.norm(vector_a)
            norm_b = np.linalg.norm(vector_b)

            cosine_angle = dot_product / (norm_a * norm_b)
            angle = np.arccos(cosine_angle)

            return angle#np.rad2deg(angle)
        def vector(a, b):
            return b-a
        def map_angle(angle, from_range: list, to_range: list): # INPUT IN DEGREES
            mapped_angle = (angle - from_range[0]) * (to_range[1] - to_range[0]) / (from_range[1] - from_range[0]) + to_range[0]
            # TO-DO: CAP MAX AND MIN VALUES
            return np.deg2rad(mapped_angle) # OUTPUT IN RADIANS

        # MAPPING
        wrist = joints[0, :]
        wrist_point = self.convert_to_point(wrist)
        thumb_0 = joints[1, :]
        thumb_0_point = self.convert_to_point(thumb_0)
        thumb_1 = joints[2, :]
        thumb_1_point = self.convert_to_point(thumb_1)
        thumb_2 = joints[3, :]
        thumb_2_point = self.convert_to_point(thumb_2)
        thumb_tip = joints[4, :]
        thumb_tip_point = self.convert_to_point(thumb_tip)
        index_0 = joints[5, :]
        index_0_point = self.convert_to_point(index_0)
        index_1 = joints[6, :]
        index_1_point = self.convert_to_point(index_1)
        index_2 = joints[7, :]
        index_2_point = self.convert_to_point(index_2)
        index_tip = joints[8, :]
        index_tip_point = self.convert_to_point(index_tip)
        middle_0 = joints[9, :]
        middle_0_point = self.convert_to_point(middle_0)
        middle_1 = joints[10, :]
        middle_1_point = self.convert_to_point(middle_1)
        middle_2 = joints[11, :]
        middle_2_point = self.convert_to_point(middle_2)
        middle_tip = joints[12, :]
        middle_tip_point = self.convert_to_point(middle_tip)
        ring_0 = joints[13, :]
        ring_0_point = self.convert_to_point(ring_0)
        ring_1 = joints[14, :]
        ring_1_point = self.convert_to_point(ring_1)
        ring_2 = joints[15, :]
        ring_2_point = self.convert_to_point(ring_2)
        ring_tip = joints[16, :]
        ring_tip_point = self.convert_to_point(ring_tip)
        pinky_0 = joints[17, :]
        pinky_0_point = self.convert_to_point(pinky_0)
        pinky_1 = joints[18, :]
        pinky_1_point = self.convert_to_point(pinky_1)
        pinky_2 = joints[19, :]
        pinky_2_point = self.convert_to_point(pinky_2)
        pinky_tip = joints[20, :]
        pinky_tip_point = self.convert_to_point(pinky_tip)


        # Plate
        angle_plate = calculate_angle(vector(thumb_0, thumb_2), vector(thumb_0, index_0)) # RADIANS
        angle_plate = angle_plate*1.5                                        # RADIANS
        #angle_plate = map_angle(angle_plate, from_range=range, to_range=[50,-50])

        # Thumb
        angle_low_thumb = calculate_angle(vector(thumb_0, thumb_1), vector(thumb_1, thumb_2))
        #angle_low_thumb = map_angle(angle_low_thumb, from_range=[5,30], to_range=[0,90])
        angle_high_thumb = calculate_angle(vector(thumb_1, thumb_2), vector(thumb_2, thumb_tip))
        #angle_high_thumb = map_angle(angle_high_thumb, from_range=[5,80], to_range=[0,90])
        
        # Index
        angle_low_index = calculate_angle(vector(wrist, index_0), vector(index_0, index_1))
        #angle_low_index = map_angle(angle_low_index, from_range=range, to_range=[0,90])
        angle_high_index = calculate_angle(vector(index_0, index_1), vector(index_1, index_2))
        #angle_high_index = map_angle(angle_high_index, from_range=range, to_range=[0,90])

        # Middle
        angle_low_middle = calculate_angle(vector(wrist, middle_0), vector(middle_0, middle_1))
        angle_low_middle = angle_low_middle + np.deg2rad(-20) # RADIANS
        #angle_low_middle = map_angle(angle_low_middle, from_range=[0,90], to_range=[0,90])
        angle_high_middle = calculate_angle(vector(middle_0, middle_1), vector(middle_1, middle_2))
        angle_high_middle = angle_high_middle + np.deg2rad(-20) # RADIANS
        #angle_high_middle = map_angle(angle_high_middle, from_range=[00,90], to_range=[0,90])

        # Ring
        angle_low_ring = calculate_angle(vector(wrist, ring_0), vector(ring_0, ring_1))
        angle_low_ring = angle_low_ring + np.deg2rad(-20) # RADIANS
        #angle_low_ring = map_angle(angle_low_ring, from_range=[0,90], to_range=[0,90])
        angle_high_ring = calculate_angle(vector(ring_0, ring_1), vector(ring_1, ring_2))
        angle_high_ring = angle_high_ring + np.deg2rad(-20) # RADIANS
        #angle_high_ring = map_angle(angle_high_ring, from_range=[0,90], to_range=[0,90])
        # Pinky
        angle_low_pinky = calculate_angle(vector(wrist, pinky_0), vector(pinky_0, pinky_1))
        angle_low_pinky = angle_low_pinky + np.deg2rad(-20) # RADIANS

        #angle_low_pinky = map_angle(angle_low_pinky, from_range=[0,90], to_range=[0,90])
        angle_high_pinky = calculate_angle(vector(pinky_0, pinky_1), vector(pinky_1, pinky_2))
        angle_high_pinky = angle_high_pinky + np.deg2rad(-20) # RADIANS

        #angle_high_pinky = map_angle(angle_high_pinky, from_range=[0,90], to_range=[0,90])
        


        # Mapping
        real_hand_joint_angles[0] = angle_plate
        real_hand_joint_angles[1] = angle_low_thumb
        real_hand_joint_angles[2] = angle_high_thumb
        real_hand_joint_angles[3] = angle_low_index
        real_hand_joint_angles[4] = angle_high_index
        real_hand_joint_angles[5] = angle_low_middle
        real_hand_joint_angles[6] = angle_high_middle
        real_hand_joint_angles[7] = angle_low_ring
        real_hand_joint_angles[8] = angle_high_ring
        real_hand_joint_angles[9] = angle_low_pinky
        real_hand_joint_angles[10] = angle_high_pinky
            
        assert len(real_hand_joint_angles) == 11, "Expected 11 joint angles"

        
        # RVIZ

        # Generate lines for RVIZ
        lines = MarkerArray()

        line_palm  = Marker()
        line_palm.points = [wrist_point, index_0_point, middle_0_point, ring_0_point, pinky_0_point, wrist_point]
        line_palm.type = Marker.LINE_STRIP
        line_palm.header.frame_id = "map"

        line_thumb = Marker()
        line_thumb.points = [wrist_point, thumb_0_point, thumb_1_point, thumb_2_point, thumb_tip_point]
        line_thumb.type = Marker.LINE_STRIP
        line_thumb.header.frame_id = "map"

        line_index = Marker()
        line_index.points = [index_0_point, index_1_point, index_2_point, index_tip_point]
        line_index.type = Marker.LINE_STRIP
        line_index.header.frame_id = "map"

        line_middle = Marker()
        line_middle.points = [middle_0_point, middle_1_point, middle_2_point, middle_tip_point]
        line_middle.type = Marker.LINE_STRIP
        line_middle.header.frame_id = "map"

        line_ring = Marker()
        line_ring.points = [ring_0_point, ring_1_point, ring_2_point, ring_tip_point]
        line_ring.type = Marker.LINE_STRIP
        line_ring.header.frame_id = "map"

        line_pinky = Marker()
        line_pinky.points = [pinky_0_point, pinky_1_point, pinky_2_point, pinky_tip_point]
        line_pinky.type = Marker.LINE_STRIP
        line_pinky.header.frame_id = "map"

        lines.markers.append(line_palm)
        lines.markers.append(line_thumb)
        lines.markers.append(line_index)
        lines.markers.append(line_middle)
        lines.markers.append(line_ring)
        lines.markers.append(line_pinky)

        self.pub_marker.publish(lines)
        print ("Published marker array")

        #time.sleep(0.5)
        return torch.Tensor(real_hand_joint_angles)
        #return finger_joint_angles

    def callback(self, msg):
        # Convert the flattened data back to a 2D numpy array (normalized)
        joints = np.array(msg.data, dtype=np.float32).reshape(
            msg.layout.dim[0].size, msg.layout.dim[1].size)
    
       
        self.target_angles = self.retarget_finger_mano_joints(joints)
        

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
