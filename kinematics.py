from coordinate_tools import Transformation as Transform
from coordinate_tools import RotationMatrix
from coordinate_tools import Coordinate
from coordinate_tools import Trigonometry as trig
import math
import numpy as np

class Kinematics():
    """Represents kinematics of a robotic arm's with an in-line wrist.

    """

    def __init__(self, inertial_transformation):
        """Sets the interial coordinate system and initializes coordinate
        systems for every joint.

        Args:
            * inertial_transformation (coordinate_tools.Transformation): 
            coordinate transformation object, which transfers robot coordinates
            into world (inertial) coordinates

        """
        self._inert_transform = inertial_transformation

        self._initialize_rotation_matrices()
        self._initialize_translation_vectors()
        self._default_geometry(reset=True)

    @classmethod
    def from_origin(cls):
        """Returns a new Kinematics object which has the same coordinate
        system as the inertial system.

        """

        return cls(Transform.from_identity())

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

        transformation = Transform.from_translation(translation_vector)
        return cls(transformation)

    def set_joint2_height(self, joint2_height):
        """Sets the verticle distance between this robot's COS origin and the
        second joint. Updates the corresponding translation vector.

        Args:
            * joint2_height (double): joint2_height >= 0
        
        Raises:
            * ValueError: if joint2_height < 0

        """
        
        if not joint2_height >= 0:
            raise ValueError('The verticle distance between this robots COS\
                origin and the second joint must not be negative.')

        self._joint2_height = joint2_height
        self._translation_vectors[1][2] = joint2_height

    def set_joint2_offset(self, joint2_offset):
        """Sets the horizontal offset between the first and the second joint.
        Updates the corresponding translation vector.

        Args:
            * joint2_offset (double)

        """
        self._joint2_offset = joint2_offset
        self._translation_vectors[1][0] = joint2_offset
    
    def set_arm23_length(self, arm23_length):
        """Sets the distance between the second and third joint.
        Updates the corresponding translation vector.

        Args:
            * arm23_length (double): arm23_length > 0

        Raises:
            * ValueError: If arm23_length <= 0
        
        """

        if not arm23_length > 0:
            raise ValueError('The arm-length must not be negative or zero.')

        self._arm23_length = arm23_length
        self._translation_vectors[2][2] = arm23_length

    def set_joint4_offset(self, joint4_offset):
        """Sets the x3 offset between the third and the forth joint. Updates
        the corresponding translation vector.

        Args:
            * joint4_offset (double)

        """
        self._joint4_offset = joint4_offset
        self._translation_vectors[3][0] = joint4_offset

    def set_arm35_length(self, arm35_length):
        """Sets the distance between the third joint and the robot's wrist.
        Updates the corresponding translation vector.

        Args:
            * arm35_length (double): arm35_length > 0

        Raises:
            * ValueError: If arm35_length <= 0
        
        """

        if not arm35_length > 0:
            raise ValueError('The arm-length must not be negative or zero.')

        self._arm35_length = arm35_length
        self._translation_vectors[4][2] = arm35_length

    def set_wrist_length(self, wrist_length):
        """Sets the distance between the fifth and the sixth joint. Updates the
        corresponding translation vector.

        Args:
            * wrist_length (double): wrist_length >= 0

        Raises:
            * ValueError: If wrist_length < 0
        
        """

        if not wrist_length >= 0:
            raise ValueError('The distance between the fifth and the sixth\
                joint must not be negative.')

        self._wrist_length = wrist_length
        self._translation_vectors[5][2] = wrist_length

    def set_endeffector(self, transform):
        """Defines a coordinate transformation for the endeffector's COS with
        respect to the tool mount COS.

        Args:
            * transform (coordinate_tools.Transformation): converts from
            endeffector to tool mount coordinates

        """

        if transform is None:
            self._end_effector_transform = Transform.from_identity()
        
        self._end_effector_transform = transform

    def forward(self, angles):
        """Return a coordinate transformation object, which transfers
        end-effector coordinate into (inertial) world coordinates for the given
        setting of angles.

        Sets up a coordinate transformation pipeline. Calculates the desired
        transformation as a composition of eight transformations.

        Args:
            * angles (np.array): 6x1 vector containing angles for each joint

        Raises:
            * ValueError: if angles is not a 6x1 numpy vector

        """
        
        if not (angles.shape == (6,)) or (angles.shape == (6,1)):
            raise ValueError('Angles must be a 6x1 numpy vector.')
        
        pipeline = list()
        pipeline.append(self._inert_transform)

        for angle, rot_matrix, trans_vec in zip(angles, \
            self._rotation_matrices, self._translation_vectors):
            
            transform = Transform(rot_matrix.matrix_at_angle(angle), trans_vec)
            pipeline.append(transform)

        pipeline.append(self._end_effector_transform)

        return Transform.from_pipeline(pipeline)

    def inverse(self, transform):
        """Returns the angle of every joint so that the robotic arm's end-
        effector reaches the desired point and has the desired orientation.

        Args:
            * transform (Transformation): COS transformation from world to
            end-effector coordinates (contains origin + orientation)

        Raises:
            * KinematicError: If the desired setting of the robotic arm is
            kinematically not possible.

        """

        if not self.is_initialized():
            raise KinematicError('inverse kinematics', \
                'Information missing - complete initialization before calling.')

        angles = np.zeros(6) # Initialize the output vector

        # Tool-center-point (tcp) in world and robot coordinates
        tcp_WC = transform.get_translation()
        tcp_RC = self._inert_transform.retransform(tcp_WC)

        # transformation from tool-center (=end-effector) to robot coordinates
        RC_trans_TC = Transform.from_composition(\
            self._inert_transform.get_inverse(), transform)

        # transformation from joint 6 (=tool-mount) to robot coordinates
        RC_trans_J6 = Transform.from_composition(RC_trans_TC, \
            self._end_effector_transform.get_inverse())

        j6_RC = RC_trans_J6.get_translation()
        j5_RC = j6_RC - self._translation_vectors[5]

        # Calculate the angle of the first joint
        angles[0] = math.atan2(j5_RC[1], j5_RC[0])

        RC_rot_J1 = self._rotation_matrices[0].matrix_at_angle(angles[0])
        RC_trans_J1 = Transform(RC_rot_J1, np.zeros(3))
        j2_RC = RC_trans_J1.transform(self._translation_vectors[1])

        # Using cosine sentence, calculate the angle[2] (third joint)
        j2j5_RC = j5_RC - j2_RC
        j2j5_norm = np.linalg.norm(j2j5_RC)
        j3j5_norm = np.hypot(self._joint4_offset, self._arm35_length)
        try:
            gamma = trig.cosine_sentence(j3j5_norm, self._arm23_length, \
                j2j5_norm)
        except:
            raise KinematicError('inverse kinematics', \
                'Kinematic failed: robot cannot reach the desired position.')
        gamma_dot = math.atan(self._joint4_offset / self._arm35_length)
        angles[2] = -math.pi + gamma - gamma_dot

        # Similarly, using cosine sentence, calculate angle[1] (second joint)
        j2j5_J2 = RC_trans_J1.retransform(j2j5_RC)
        alpha_dot = math.atan2(j2j5_J2[2], j2j5_J2[0])
        alpha = trig.cosine_sentence(self._arm23_length, j2j5_norm, j3j5_norm)
        angles[1] = alpha + alpha_dot - math.pi / 2

        pipeline = list()
        for angle, rot_matrix, trans_vec in zip(angles[0:3], \
            self._rotation_matrices, self._translation_vectors):
            
            transform = Transform(rot_matrix.matrix_at_angle(angle), trans_vec)
            pipeline.append(transform)
        RC_trans_J3 = Transform.from_pipeline(pipeline)

        z3 = RC_trans_J3.get_rotation()[:,2] # extract z-column
        z6 = RC_trans_J6.get_rotation()[:,2] # extract z-column
        angles[4] = math.acos(np.dot(z3, z6))

        return angles

    def is_initialized(self):
        """Returns true if the all the required geometry information has been 
        provided. Does NEITHER assess correctness NOR completeness of this 
        robot's geometric representation.

        """

        if self._arm23_length <= 0:
            return False
        if self._arm35_length <= 0:
            return False
        if self._wrist_length < 0:
            return False
        if self._end_effector_transform is None:
            return False
        return True

    def _initialize_rotation_matrices(self):
        """Initializes rotation matrices for every of the six joints and stores
        them in a list.

        """
        self._rotation_matrices = list()

        # Every rotation is described with respect to the previous rotation.
        self._rotation_matrices.append(RotationMatrix.from_axis('z')) 
        self._rotation_matrices.append(RotationMatrix.from_axis('y'))
        self._rotation_matrices.append(RotationMatrix.from_axis('y'))
        self._rotation_matrices.append(RotationMatrix.from_axis('z'))
        self._rotation_matrices.append(RotationMatrix.from_axis('y'))
        self._rotation_matrices.append(RotationMatrix.from_axis('z'))

    def _initialize_translation_vectors(self):
        """Initializes translation vectors for every of the six joints and
        stores them in a list.

        """

        self._translation_vectors = [np.zeros(3) for i in range(6)]

    def _default_geometry(self, reset = False):
        """Sets the default geometry. Resets all the geometry settings (opt).

        Warning:
            * reset = True ==> self.is_initialized == False

        """

        self.set_joint2_height(0)
        self.set_joint2_offset(0)
        self.set_joint4_offset(0)
        
        if reset:
            self._arm35_length = -1
            self._arm23_length = -1
            self._wrist_length = -1
            self._end_effector_transform = None

class KinematicError(Exception):
    """Represents errors of illegal states or calls on Kinematics objects.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, sender, message):
        self.sender = sender
        self.message = message

    def __str__(self):
        err_msg = ""
        err_msg += '\033[93m'+'Kinematic error \033[0m@'+self.sender+'\n'
        err_msg += '\033[93m'+self.message+'\033[0m'
        return err_msg
    


