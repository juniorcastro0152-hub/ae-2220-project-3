import numpy as np
from typing import Optional


class Trajectory:
    def __init__(self, points: np.ndarray, threshold: float = 0.1):
        pts = np.asarray(points)
        if pts.ndim != 2 or pts.shape[1] != 4:
            raise ValueError("Points array must have shape (N, 4) for 3D trajectory + end rotation.")
        self.points = pts
        self.current_target_index = 0
        self.threshold = threshold  # Distance threshold to consider a point reached

    def get_point(self, index: int) -> np.ndarray:
        """Return the full (x,y,z,rot) point at `index`."""
        return self.points[index]
    
    def get_current_target_pos(self) -> np.ndarray:
        return self.points[self.current_target_index][:3]
    
    def get_current_target_rot(self) -> float:
        # return a native Python float for convenience
        return float(self.points[self.current_target_index][3])
    
    def advance_target(self) -> None:
        if self.current_target_index < len(self.points) - 1:
            self.current_target_index += 1

    def reached_target(self, position: np.ndarray, threshold: Optional[float] = None) -> bool:
        """Return True when `position` is within `threshold` of the current target.

        If `threshold` is None, the instance's `self.threshold` is used.
        """
        if threshold is None:
            threshold = self.threshold
        target = self.get_current_target_pos()
        return np.linalg.norm(target - position[:3]) < float(threshold)
    
    def vector_to_target(self, position: np.ndarray) -> np.ndarray:
        target = self.get_current_target_pos()
        return target - position[:3]