
from mapUtilities import *
from utilities import *
from numpy import cos, sin
import numpy as np


class particle:

    def __init__(self, pose, weight):
        self.pose = pose
        self.weight = weight

    def motion_model(self, v, w, dt):
        #TODO: Implement the motion model for the particle
        """
        v: linear velocity
        w: angular velocity
        dt: time step
        """
        # Generate samples of x,y,theta, according to motion (prediction) - Week 8 slide 70
        # pose = [x y theta]
        # Unicycle (differential-drive) motion model
        theta = self.pose[2]

        if abs(w) < 1e-6:
            # Straight line motion
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt
        else:
            # Integration for constant v and w over time
            dx = (v / w) * (np.sin(theta + w * dt) - np.sin(theta))
            dy = (v / w) * (-np.cos(theta + w * dt) + np.cos(theta))

        # Update pose
        self.pose[0] += dx
        self.pose[1] += dy
        self.pose[2] = normalize_angle(theta + w * dt)

    # TODO: You need to explain the following function to TA
    def calculateParticleWeight(self, scanOutput: LaserScan, mapManipulatorInstance: mapManipulator, laser_to_ego_transformation: np.array):
        # 1. build laser to map transform from particle pose and sensor data
        T = np.matmul(self.__poseToTranslationMatrix(), laser_to_ego_transformation)

        # 2. laser scan to homogenous cartesian points in laser frame then map frame
        _, scanCartesianHomo = convertScanToCartesian(scanOutput)
        scanInMap = np.dot(T, scanCartesianHomo.T).T

        # 3. Map positions to grid cells
        likelihoodField = mapManipulatorInstance.getLikelihoodField()
        cellPositions = mapManipulatorInstance.position_2_cell(
            scanInMap[:, 0:2])

        # 4. In-bounds mask (handle image row - down vs. map y - up)
        lm_x, lm_y = likelihoodField.shape

        cellPositions = cellPositions[np.logical_and.reduce(
                (cellPositions[:, 0] > 0, -cellPositions[:, 1] > 0, cellPositions[:, 0] < lm_y,  -cellPositions[:, 1] < lm_x))]

        # 5. Log likelihood accumulation
        log_weights = np.log(
            likelihoodField[-cellPositions[:, 1], cellPositions[:, 0]])
        log_weight = np.sum(log_weights)
        weight = np.exp(log_weight)
        weight += 1e-10

        self.setWeight(weight)

    def setWeight(self, weight):
        self.weight = weight

    def getWeight(self):
        return self.weight

    def setPose(self, pose):
        self.pose = pose

    def getPose(self):
        return self.pose[0], self.pose[1], self.pose[2]

    def __poseToTranslationMatrix(self):
        x, y, th = self.getPose()

        translation = np.array([[cos(th), -sin(th), x],
                                [sin(th), cos(th), y],
                                [0, 0, 1]])

        return translation
