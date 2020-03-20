from coordinate_tools import Transformation
from coordinate_tools import Coordinate
import numpy as np

class Kinematics():
    """Represents kinematics of a robotic arm's with an in-line wrist.

    """

    def __init__(self, inertial_transformation):
        """Initializes the robots coordinate system.

        Args:
            * inertial_transformation (coordinate_tools.Transformation): 
            coordinate transformation object, which transfers robot coordinates
            into world (inertial) coordinates

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
        if not Coordinate.valid_vector(translation_vector):
            raise ValueError('Translation vector must be 3x1 numpy vector')

        transformation = Transformation(np.eye(3), translation_vector)
        return cls(transformation)

    def set_joint2_height(self, joint2_height):
        """Sets the verticle distance between this robot's COS origin and the
        second joint.

        Args:
            * joint2_height (double): joint2_height >= 0
        
        Raises:
            * ValueError: if joint2_height < 0

        """
        
        if not joint2_height >= 0:
            raise ValueError('The verticle distance between this robots COS\
                origin and the second joint must not be negative.')

        self._joint2_height = joint2_height 

    def set_joint2_offset(self, joint2_offset):
        """Sets the horizontal offset between the first and the second joint.

        Args:
            * joint2_offset (double)

        """
        self._joint2_offset = joint2_offset
    
    def set_arm23_length(self, arm23_length):
        """Sets the distance between the second and third joint.

        Args:
            * arm23_length (double): arm23_length > 0

        Raises:
            * ValueError: If arm23_length <= 0
        
        """

        if not arm23_length > 0:
            raise ValueError('The arm-length must not be negative or zero.')

        self._arm23_length = arm23_length

    def set_joint4_offset(self, joint4_offset):
         """Sets the x3 offset between the third and the forth joint.

        Args:
            * joint4_offset (double)

        """

        self._joint4_offset = joint4_offset

    def set_arm35_length(self, arm35_length):
        """Sets the distance between the third joint and the robot's wrist.

        Args:
            * arm35_length (double): arm35_length > 0

        Raises:
            * ValueError: If arm35_length <= 0
        
        """

        if not arm35_length > 0:
            raise ValueError('The arm-length must not be negative or zero.')

        self._arm35_length = arm35_length

    def set_wrist_length(self, wrist_length):
        """Sets the distance between the fifth and the sixth joint.

        Args:
            * wrist_length (double): wrist_length >= 0

        Raises:
            * ValueError: If wrist_length < 0
        
        """

        if not wrist_length >= 0:
            raise ValueError('The distance between the fifth and the sixth\
                joint must not be negative.')

        self._wrist_length = wrist_length

    def set_endeffector_transform(self, transform):
        """Defines a coordinate transformation for the endeffector's COS with
        respect to the tool mount COS

        Args:
            * transform (coordinate_tools.Transformation): converts from
            endeffector to tool mount coordinates

        """
        
        self.end_effector_transform = transform

    def forward(self, angles):
        pass

    def inverse(self, to, end_effecotor_COS):
        pass

    def set_end_effector(self):
        pass

