'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for differnt joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        # YOUR CODE HERE
        self.chains = { 'Head': ['HeadYaw', 'HeadPitch'],
						'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
						'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
						'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
						'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       }
                       # , 'LWristYaw', 'LHand'
                       # , 'RWristYaw', 'RHand'
        # Länge der Abstände der einzelnen Gelenke in den chains
        self.jointLengths = {'HeadYaw' : (0, 0, 126.50), 'HeadPitch' : (0, 0, 0),
                             # left arm and leg
                            'LShoulderPitch' : (0, 98.00, 0), 'LShoulderRoll' : (0, 0, 0), 'LElbowYaw' : (105, 15, 0),
                            'LElbowRoll' : (0, 0, 0), 'LWristYaw' : (55.95, 0, 0), 'LHand' : (57.75, 0, 12.31),
                            'LHipYawPitch' : (0, 50.00, -85.00), 'LHipRoll' : (0, 0, 0), 'LHipPitch' : (0, 0, 0), 
                            'LKneePitch' : (0, 0, -100.00), 'LAnklePitch' : (0, 0, -102.90), 'LAnkleRoll' : (0, 0, 45.19),
                             # right arm and leg
                            'RShoulderPitch' : (0, -98.00, 0), 'RShoulderRoll' : (0, 0, 0), 'RElbowYaw' : (105, -15, 0),
                            'RElbowRoll' : (0, 0, 0), 'RWristYaw' : (55.95, 0, 0), 'RHand' : (57.75, 0, 12.31),
                            'RHipYawPitch' : (0, -50.00, -85.00), 'RHipRoll' : (0, 0, 0), 'RHipPitch' : (0, 0, 0), 
                             'RKneePitch' : (0, 0, -100.00), 'RAnklePitch' : (0, 0, -102.90), 'RAnkleRoll' : (0, 0, 45.19)}

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''

        T = np.zeros((4,4))
        # YOUR CODE HERE
        s = np.sin(joint_angle)
        c = np.cos(joint_angle)

        # nach foliensatz 3 Kinematics folie 35
        Rx = np.array([[1,0,0,0], [0,c,-s,0], [0,s,c,0], [0,0,0,1]])
        Ry = np.array([[c,0,s,0], [0,1,0,0], [-s,0,c,0], [0,0,0,1]])
        Rz = np.array([[c,-s,0,0], [s,c,0,0], [0,0,1,0], [0,0,0,1]])

        T = np.dot(np.dot(Rx,Ry),Rz)
        #print(joint_name)
        #print(T)

        # Verschiebung durch die Länge der einzelnen Gelenkabstände
        T[3][0] = self.jointLengths[joint_name][0]
        T[3][1] = self.jointLengths[joint_name][1]
        T[3][2] = self.jointLengths[joint_name][2]


        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''

        #print("joints dic : ", joints)
        #print (len(self.chains.values()))
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T,Tl)
                self.transforms[joint] = T
        #print(self.transforms)

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
