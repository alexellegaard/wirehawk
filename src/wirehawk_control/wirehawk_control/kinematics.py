import numpy as np

class CDPRKinematics:
    """
    Kinematics engine for a 4-cable CDPR with smooth geometric slack allocation.
    """
    def __init__(self, anchors: np.ndarray, slack_offset: float = 0.05):
        self.anchors = np.asarray(anchors, dtype=float)
        self.num_cables = len(self.anchors)
        self.slack_offset = float(slack_offset)

    def inverse_kinematics(self, pos: np.ndarray) -> np.ndarray:
        """Nominal Euclidean distance from each anchor to payload: L_i = ||a_i - p||"""
        diffs = self.anchors - pos
        return np.linalg.norm(diffs, axis=1)

    def compute_commanded_lengths(self, pos: np.ndarray) -> np.ndarray:
        """
        Computes commanded lengths. Projects position onto diagonal directions
        to smoothly blend slack onto the non-supporting trailing cable.
        """
        geom_lengths = self.inverse_kinematics(pos)
        cmd_lengths = np.copy(geom_lengths)

        if self.num_cables == 4:
            # Anchor horizontal signs relative to origin:
            # Anchor 0: (+X, +Y) -> opposes (-X, -Y)
            # Anchor 1: (+X, -Y) -> opposes (-X, +Y)
            # Anchor 2: (-X, -Y) -> opposes (+X, +Y)
            # Anchor 3: (-X, +Y) -> opposes (+X, -Y)
            px, py = pos[0], pos[1]

            # Measure how much the payload is moving AWAY from each anchor
            # Dot product of position with anchor horizontal direction:
            anchor_dir = self.anchors[:, :2]  # (4, 2)
            projections = anchor_dir @ np.array([px, py])  # (4,)

            # The anchor with the most negative projection is the most "opposing/trailing"
            trailing_idx = int(np.argmin(projections))

            # Apply slack offset to the trailing line
            cmd_lengths[trailing_idx] += self.slack_offset

        return cmd_lengths