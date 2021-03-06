'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import rightBackToStand


class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.startTime = -1

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        
        #print self.keyframes
        if(self.keyframes == ([],[],[])):
            #print "return"
            return target_joints
        
        # adjust time value
        if(self.startTime == -1):
            self.startTime = perception.time
            #print "start time"
            #print self.startTime
        adjTime = perception.time - self.startTime
        #print "adj time"
        #print adjTime
        
        (names, times, keys) = keyframes
        
        # iterate over all joints in the keyframes
        skippedJoints = 0
        for (i, name) in enumerate(names):
            
            timeLow = 0     # represents the lower threshold of the keyframe
            timeHigh = 0    # represents the upper threshold of the keyframe
            kfNum = 0       # the upper index of the keyframe which has to be used for interpolation
            jointTimes = times[i]    # just for easier reading/writing
            lenJointTimes = len(jointTimes)
            
            # interpolation is finished for this joint, dont do any more steps
            if (adjTime > jointTimes[-1]):
                skippedJoints += 1
                # if we skipped all joints interpolation is done -> reset timer and keyframes
                if(skippedJoints == len(names)):
                    self.startTime = -1
                    self.keyframes = ([],[],[])
                continue
            
            
            
            
            # iterate over all times of the current joint to find the right time span
            for j in xrange(lenJointTimes):
                timeHigh = jointTimes[j]
                
                # we found the right interval -> break
                if ((adjTime >= timeLow and adjTime <= timeHigh)): 
                    kfNum = j
                    break
                timeLow = timeHigh
            
            # calculate t-value
            t = (adjTime - timeLow) / (timeHigh - timeLow)
            
            # set p-values
            # if kfNum == 0, we are before the first time in jointTimes -> no values for p0 and p1
            if (kfNum == 0):
                p3 = keys[i][kfNum][0]
                p2 = p3 + keys[i][kfNum][1][2]
                p0 = 0
                p1 = 0
            else:
                p0 = keys[i][kfNum-1][0]
                p3 = keys[i][kfNum][0]
                p1 = p0 + keys[i][kfNum-1][2][2]
                p2 = p3 + keys[i][kfNum][1][2]
                
            
            # calculate joint angle and append to dictionary
            angle = ((1-t)**3)*p0 + 3*t*((1-t)**2)*p1 + 3*(t**2)*(1-t)*p2 + (t**3)*p3
            #if(name in INVERSED_JOINTS):
            #    target_joints[name] = -1*angle
            #else:
            target_joints[name] = angle
            if(name == "LHipYawPitch"):
                target_joints["RHipYawPitch"] = angle
            #print degrees(angle)
            

        #print "return"
        #print target_joints
        #self.realStartTime = -1
        #self.keyframes = ({},{},{})
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
