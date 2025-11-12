# grid.py

import math

class Grid:
    """A spatial partitioning grid to optimize neighbor searches."""
    def __init__(self, bounds, cell_size):
        self.bounds = bounds
        self.cell_size = float(cell_size)
        self.width = int(math.ceil(bounds[0] / self.cell_size))
        self.height = int(math.ceil(bounds[1] / self.cell_size))
        self.cells = {} # Using a dictionary for sparse storage

    def _get_cell_coords(self, position):
        """Get the (x, y) integer coordinates of the cell for a given position."""
        x = int(position.x / self.cell_size)
        y = int(position.y / self.cell_size)
        return (x, y)

    def clear(self):
        """Clears all particles from the grid for the new frame."""
        self.cells = {}

    def insert(self, particle):
        """Inserts a particle into the correct grid cell."""
        coords = self._get_cell_coords(particle.position)
        if coords not in self.cells:
            self.cells[coords] = []
        self.cells[coords].append(particle)

    def get_neighbors(self, particle):
        """
        Returns a list of all particles in the same cell as the given
        particle and in the 8 neighboring cells.
        """
        center_coords = self._get_cell_coords(particle.position)
        neighbor_particles = []
        for dy in range(-1, 2):
            for dx in range(-1, 2):
                check_coords = (center_coords[0] + dx, center_coords[1] + dy)
                if check_coords in self.cells:
                    neighbor_particles.extend(self.cells[check_coords])
        return neighbor_particles