import numpy as np

class Transformation():
    """Handles cartesian coordinate transformations.

    """

    def __init__(self, rotation_matrix, translation_vector, \
        calc_inverse = True):
        """Initializes a coordinate transformation object.

        Accepts a rotation matrix and a translation vector, by which a linear
        coordinate transformation is defined. Calculates the the inverse
        transformation function and stores it as a CoordinateTransformation
        object.

        Args:
            * rotation_matrix (numpy.matrix): a real 3 by 3 orthogonal matrix
            * translation_vector (numpy.array): a real 3 by 1 column vector
            * calc_inverse (bool): (optional) automatically calculate an
            inverse transformation for this transformation

        Raises:
            * ValueError: If either translation_vector or rotation_matrix are
            not 3-dimensional
            * LinearAlgebraError: If rotation_matrix is not a orthogonal 3 by 3
            matrix
        """

        if not Coordinate.valid_vector(translation_vector):
            raise ValueError('Translation vector must be a 3x1 numpy \
                                array')
        if not Coordinate.valid_matrix(rotation_matrix):
            raise ValueError('Rotation matrix must be a 3x3 numpy matrix')

        if not Coordinate.orthogonal_matrix(rotation_matrix):
            raise LinearAlgebraError('Rotation matrix is not orthogonal \
                and can thus not represent a cartesian coordinate system')

        self.__rotation_matrix = rotation_matrix
        self.__translation_vector = translation_vector
        if calc_inverse:
            self.__inverse_transformation = self._calc_inverse_transformation()
        else:
            self.__inverse_transformation = None

    @classmethod
    def from_translation(cls, translation_vector):
        """Returns a translation coordinate transformation. 

        Args:
            * translation_vector (np.array): 3x1 column vector which represents
            the translation

        Raises:
            * ValueError: if translation_vector is not a valid 3x1 numpy vector

        """

        if not Coordinate.valid_vector(translation_vector):
            raise ValueError('Translation vector must be a 3x1 numpy array')

        return cls(np.eye(3), translation_vector)

    @classmethod
    def from_composition(cls, outer, inner):
        """Returns a transformation object, which represent the compostion g∘f,
        where f is the inner and g is the outer transformation function.

        Args:
            * outer (Transformation)
            * inner (Transformation)

        """

        rotation_matrix = np.dot(outer.get_rotation(), \
            inner.get_rotation())
        translation_vector = np.dot(outer.get_rotation(), \
            inner.get_translation()) + outer.get_translation()

        return cls(rotation_matrix, translation_vector)

    def get_rotation(self):
        """Returns the rotation matrix.

        """
        
        return self.__rotation_matrix

    def get_translation(self):
        """Returns the translation vector.

        """

        return self.__translation_vector

    def get_inverse(self):
        """Returns the inverse transformation. Calculates this function if
        necessary.

        """

        if self.__inverse_transformation is None:
            self.__inverse_transformation = self._calc_inverse_transformation()
        
        return self.__inverse_transformation

    def transform(self, affine_vector):
        """Transforms an affine vector to base coordinates.

        Args:
            * affine_vecotr (numpy.array): a real 3 by 1 column vector

        Raises:
            * ValueError: If affine_vector is not 3-dimensional

        """
        
        if not Coordinate.valid_vector(affine_vector):
            raise ValueError('Affine vector must be a 3x1 dimensonal numpy \
                array')
        
        return np.dot(self.__rotation_matrix, affine_vector) \
            + self.__translation_vector

    def retransform(self, base_vector):
        """Transforms an base vector to affine coordinates.

        Args:
            * base_vector (numpy.array): a real 3 by 1 column vector

        Raises:
            * ValueError: If base_vector is not 3-dimensional

        """

        if not Coordinate.valid_vector(base_vector):
            raise ValueError('Base vector must be a 3x1 dimensonal numpy \
                array')
     
        return self.__inverse_transformation.transform(base_vector) 

    def _calc_inverse_transformation(self):
        """Calculates the inverse transformation function of this object.

        """

        inverse_matrix = np.linalg.inv(self.__rotation_matrix)
        inverse_vector = -np.dot(inverse_matrix, self.__translation_vector)

        return Transformation(inverse_matrix, inverse_vector,
            calc_inverse=False)

class Coordinate():
    """Provides useful tools to use numpy vectors as coordinate vectors.

    This class is considered to be a toolbox for handling numpy vectors. To
    ensure maximum compatibility accross standard python libraries, numpy
    arrays are preferred over proprietary Coordinate objects. 

    """
    
    @staticmethod
    def valid_vector(vector):
        """Returns true if vector is 3 by 1 dimensional and false otherwise.

        Args:
            * vector (numpy.array)

        """
        return (vector.shape == (3,)) or (vector.shape == (3,1))

    @staticmethod
    def valid_matrix(matrix):
        """Returns true if matrix is 3 by 3 dimensional and false otherwise.

        Args:
            * matrix (numpy.matrix)

        """
        return matrix.shape == (3,3)
    
    @staticmethod
    def orthogonal_matrix(matrix):
        """Returns true if matrix orthogonal and false otherwise.

        A matrix A is orthogonal if and only if A multiplied by its transpose
        is an identity matrix.

        Args:
            * matrix (numpy.matrix)

        """

        return np.allclose(np.dot(matrix, np.transpose(matrix)), np.eye(3))

class RotationMatrix():

    def __init__(self, rotation_vector):
        """Accepts the rotation vector and prepares future calculations.

        Args:
            * rotation_vector (np.array): must be a 3x1 numpy vector

        Raises_
            * ValueError: if rotation_vector is not a valid vector

        """
        
        if not Coordinate.valid_vector(rotation_vector):
            raise ValueError('Rotation vector must be a 3x1 numpy array')

        unit_vector = rotation_vector / np.linalg.norm(rotation_vector)
        unit_vector = np.reshape(unit_vector, (3,1))
        self._rotation_vector = unit_vector

        # outer product of two vectors is a matrix
        self._outer_product = np.dot(unit_vector, np.transpose(unit_vector))
        self._cosine_matrix = np.eye(3) - self._outer_product
        
        uv1 = np.asscalar(unit_vector[0])
        uv2 = np.asscalar(unit_vector[1])
        uv3 = np.asscalar(unit_vector[2])
        self._cross_matrix = np.array([[0,-uv3, uv2],[uv3,0,-uv1], \
            [-uv2, uv1,0]])

    @classmethod
    def from_axis(cls, axis = 'z'):
        """Returns a rotation matrix object for a given coordinate axis.

        Args:
            * axis (str): textual axis description
        
        Raises:
            * ValueError: if textual description does not match any coordinate
            axis

        """
        if axis == 'x' or axis == 'X':
            return cls(np.array([1,0,0]))
        if axis == 'y' or axis == 'Y':
            return cls(np.array([0,1,0]))
        if axis == 'z' or axis == 'Z':
            return cls(np.array([0,0,1]))
        else:
            raise ValueError('Axis is not a coordinate axis.')

    def matrix_at_angle(self, angle):
        """Returns a rotation matrix for this instances axis for a given angle.

        See:
            * Formula "Rotation matrix from axis and angle": https://w.wiki/Knf

        Args:
            * angle (double): angle of the affine COS's rotation with respect 
            to the reference COS
        
        """

        return np.cos(angle) * self._cosine_matrix + self._outer_product + \
            np.sin(angle) * self._cross_matrix

class LinearAlgebraError(Exception):
    """Exception raised for invalid linear algebra operations related to 
    transformation process of cartesian cooridinate systems.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, message):
        self.message = message
