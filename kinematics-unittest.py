from coordinate_tools import Transformation
from coordinate_tools import RotationMatrix
from kinematics import Kinematics

import numpy as np
import math
import random

import unittest
from test import support

class DifferentialTest(unittest.TestCase):
    """Performs random differential tests on the forward() and reverse function
    of the 'Kinematics' class.


    """
    def setUp(self):
        random.seed(1)
        np.random.seed(1)

    def test_RC_representation(self):
        # Tests forward and inverse kinematics in RC coordinates
        k = Kinematics.from_origin()
        k.set_joint2_offset(30)
        k.set_joint2_height(30)
        k.set_joint4_offset(10)
        k.set_arm23_length(150)
        k.set_arm35_length(150)
        k.set_wrist_length(10)
        k.set_endeffector(Transformation.from_identity())

        target_orientation = np.random.rand(3)
        orientation_mat = RotationMatrix(target_orientation).matrix_at_angle(\
            random.randrange(-10,10)*random.random())
        #orientation_mat = RotationMatrix.from_axis('z').matrix_at_angle(math.pi/4)
        target_location = np.array([100,0,100])
        #target_location = np.ones(3)*100+np.random.rand(3)*20

        target = Transformation(orientation_mat, target_location)
        angles = k.inverse(target)
        target = k.forward(angles)

        print(target.get_translation())

        #assert np.allclose(target.get_rotation(), orientation_mat)
        assert np.allclose(target.get_translation(), target_location)

    def test_feature_two(self):
        # Test feature two.
        pass


