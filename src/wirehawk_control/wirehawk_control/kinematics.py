import numpy as np


class CDPRKinematics:
    """
    Pure geometric inverse kinematics for an N-cable CDPR.
    Commands nominal Euclidean lengths without artificial slack or cable dropping.
    """
    def __init__(self, anchors: np.ndarray):
        self.anchors = np.asarray(anchors, dtype=float)
        self.num_cables = len(self.anchors)

    def inverse_kinematics(self, pos: np.ndarray) -> np.ndarray:
        """
        Nominal Euclidean distance from each anchor to payload:
        L_i = ||a_i - p||
        """
        diffs = self.anchors - pos
        return np.linalg.norm(diffs, axis=1)

    def compute_commanded_lengths(self, pos: np.ndarray) -> np.ndarray:
        """Direct 1:1 mapping to geometric lengths."""
        return self.inverse_kinematics(pos)
