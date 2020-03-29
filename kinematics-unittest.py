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

    def test_RC_lockJ4(self):
        """Tests forward and inverse kinematics in RC coordinates with joint 4
        being locked to zero degrees. Test without end-effector.

        """
       
        k = Kinematics.from_origin() # test robot coordinates only
        k.set_joint2_offset(30)
        k.set_joint2_height(30)
        k.set_joint4_offset(10)
        k.set_arm23_length(150)
        k.set_arm35_length(150)
        k.set_wrist_length(10)
        k.set_endeffector(Transformation.from_identity())

        rot_mat = RotationMatrix.from_axis('z') # implicitly locks joint 4
        for i in range(1000):  
            orientation_mat = rot_mat.matrix_at_angle(random.random())
            target_location = np.ones(3)*100+np.random.rand(3)*40
            target = Transformation(orientation_mat, target_location, \
                calc_inverse=False)
    
            angles = k.inverse(target)
            assert math.isclose(angles[3], 0) # joint 4 is locked to zero 
            target = k.forward(angles)

            assert np.allclose(target.get_rotation(), orientation_mat)
            assert np.allclose(target.get_translation(), target_location)

    def test_RC_6DOF(self):
        """Tests forward and inverse kinematics with 6 degrees of freedom in
        robot coordinates.

        """

        k = Kinematics.from_origin() # test robot coordinates only
        k.set_joint2_offset(30)
        k.set_joint2_height(30)
        k.set_joint4_offset(10)
        k.set_arm23_length(150)
        k.set_arm35_length(150)
        k.set_wrist_length(10)
        k.set_endeffector(Transformation.from_identity())

        for i in range(1000):
            rot_mat = RotationMatrix(np.random.rand(3))
            orientation_mat = rot_mat.matrix_at_angle(random.random())
            target_location = np.ones(3)*100+np.random.rand(3)*40
            expected = Transformation(orientation_mat, target_location, \
                calc_inverse=False)
    
            angles = k.inverse(expected)
            actual = k.forward(angles)

            # numerical errors might occur
            #if not np.allclose(target.get_rotation(), orientation_mat):
            #    print(target.get_rotation() - orientation_mat)
            assert np.allclose(actual.get_rotation(), orientation_mat) 
            assert np.allclose(actual.get_translation(), target_location)

