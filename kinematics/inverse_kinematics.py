'''In this exercise you need to implemente inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinemtatics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h25/joints_h25.html
       http://doc.aldebaran.com/2-1/family/nao_h25/links_h25.html
    2. use the results of inverse kinemtatics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinemtatics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity

import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''

        jointss = {'HeadYaw': 0.0, 'RHipPitch': -0.0, 'RElbowYaw': -0.0, 'RShoulderPitch': 0.0, 'LShoulderPitch': 0.0, 'LKneePitch': -0.0, 'RAnkleRoll': 0.0, 'LShoulderRoll': 0.008726646259971648, 'LHipPitch': -0.0, 'LElbowYaw': 0.0, 'LAnklePitch': -0.0, 'RHipYawPitch': -0.0, 'HeadPitch': 0.0, 'LElbowRoll': -0.0066322511575784525, 'RShoulderRoll': -0.0066322511575784525, 'LAnkleRoll': -0.0, 'LHipYawPitch': -0.0, 'RAnklePitch': -0.0, 'LHipRoll': -0.0, 'RHipRoll': 0.0, 'RElbowRoll': 0.008726646259971648, 'RKneePitch': -0.0}


        self.forward_kinematics(jointss)

        joints = self.chains[effector_name]
        #print(transform)
        #Goal
        x_desired = np.array(transform)[3][0]
        y_desired = np.array(transform)[3][1]
        z_desired= np.array(transform)[3][2]
        #print (xE,yE,zE)
        J = np.zeros((4,len(joints)))
        #print(self.transforms)
        for (i, joint) in enumerate(joints):
            xc = np.array(self.transforms.get(joint))[3][0]
            yc = np.array(self.transforms.get(joint))[3][1]
            zc = np.array(self.transforms.get(joint))[3][2]
            J[0, i] = x_desired - xc
            J[1, i] = y_desired - yc
            J[2, i] = z_desired - zc
            J[3, i] = 1
            #print(J, yE , yc)
            #print(joint , xc, yc, zc)
            
        Jplus = np.dot(np.linalg.pinv(np.dot(J.T,J)),J.T)
        #print (len(Jplus),len(Jplus[0]))
        
        d = np.ones((4,1))
        #print(joints)
        d[0] = x_desired - np.array(self.transforms.get(joints[-1]))[3][0]
        d[1] = y_desired - np.array(self.transforms.get(joints[-1]))[3][1]
        d[2] = z_desired - np.array(self.transforms.get(joints[-1]))[3][2]

        djoints = np.dot(Jplus,d)
        joint_angles = []
        #print(djoints)
        for i in djoints:
            joint_angles.append(i[0])
        #print(joint_angles)
        # YOUR CODE HERE
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        v = self.inverse_kinematics(effector_name,transform)
        transform = 
        
        
        
        #name, times, keys
        name = []
        times = []
        keys = []
        self.keyframes = (name, times, keys)  # the result joint angles have to fill in

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
