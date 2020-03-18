from coordinate_tools import Transformation
import numpy as np

class Kinematics():
    """Represents a robotic arm's kinematics

    """

    def __init__(self, inertial_transformation):
        """Initializes the robots coordinate system.

        Args:
            * inertial_transformation (Transformation): coordinate
            transformation object, which transfers robot coordinates into
            world (inertial) coordinates

        """
        self._inert_transform = inertial_transformation

    @classmethod
    def from_origin(cls):
        """Returns a new Kinematics object which has the same coordinate
        system as the inertial system.

        """

        transformation = Transformation(np.eye(3), np.zeros((3)))
        return cls(transformation)

    @classmethod
    def from_translation(cls, translation_vector):
        """Returns a new Kinematics object which is translated but not rotated
        with respect to the intertial system.

        Args:
            * translation_vector (np.array): real 3 by 1 numpy vector which
            describes the translation of the robots COS with respect to the
            intertial system.

        Raises:
            * ValueError if translation_vector is not a 3x1 numpy vector
        
        """
        if not Transformation.valid_vector(translation_vector):
            raise ValueError('Translation vector must be 3x1 numpy vector')

        transformation = Transformation(np.eye(3), translation_vector)
        return cls(transformation)

    def forward(self, angles):
        pass

    def inverse(self, to, end_effecotor_COS):
        pass

    def set_end_effector(self):
        pass

