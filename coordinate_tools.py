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
        return (vector.shape == (3,)) | (vector.shape == (3,1))

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

class LinearAlgebraError(Exception):
    """Exception raised for invalid linear algebra operations related to 
    transformation process of cartesian cooridinate systems.

    Attributes:
        message -- explanation of the error
    """

    def __init__(self, message):
        self.message = message
