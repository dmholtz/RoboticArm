class Robot():
    """Virtual representation of a robotic arm with up to six-axis and up to six
    degrees of freedom. Manipulates a real robot 

    """
    pass

class Axis():
    """Represents a robot axis by defining the range of motion. Provides methods
    to convert geometric / kinematic data into step-position data for the
    motors.

    """

    def __init__(self, min, max, map_function):
        self.min = min
        self.max = max
        self.map_function = map_function

        assert self.map_function is not None or self.max == self.min

    @classmethod
    def rigid(cls, locked_to = 0):
        """Returns a rigid axis which is locked to a optional setting.
        """

        return cls(min = locked_to, max = locked_to, map_function = None)

    def within_range(self, value):
        """Return true if and only if the given value (angle or metric position)
        is within the boundaries of min and max.

        """
        
        return self.min <= value and value <= self.max

    def to_machine(self, value):
        """Transforms
        """
        return self.map_function.to_steps(value)


class MapFunction():
    """Maps geometric positions (angles / translations) to robot positions 
    (steps).

    """

    def __init__(self, ratio, zero_at = 0):
        """Initializes the map function.

        Args:
            * ration (double): [steps per rad] or [steps per mm]. Sign includes
            direction change.
            * zero_at (int): step count, where the geometric quantity is zero
        
        """
        
        self.ratio = ratio
        self.zero_at = zero_at

    def to_steps(self, value):
        """Returns the robot position [steps] for a given geometric position.
        """
        return value * self.ratio + self.zero_at