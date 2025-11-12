# vector.py

import math

class Vector2D:
    """A utility class for 2D vector mathematics."""
    
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other):
        """Adds another vector to this one."""
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        """Subtracts another vector from this one."""
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar):
        """Multiplies the vector by a scalar value."""
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        """Divides the vector by a scalar value."""
        if scalar == 0:
            return Vector2D(0, 0)
        return Vector2D(self.x / scalar, self.y / scalar)
        
    def magnitude(self):
        """Calculates the length (magnitude) of the vector."""
        return math.sqrt(self.x**2 + self.y**2)

    def normalize(self):
        """Returns a unit vector (length 1) in the same direction."""
        mag = self.magnitude()
        if mag == 0:
            return Vector2D(0, 0)
        return self / mag

    def __repr__(self):
        return f"Vector2D({self.x:.2f}, {self.y:.2f})"