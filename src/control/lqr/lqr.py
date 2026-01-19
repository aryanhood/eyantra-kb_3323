import numpy as np

class LQRController:
    """
    Runtime LQR controller.
    Gain K is precomputed offline and loaded at runtime.
    """
    def __init__(self, K):
        self.K = np.asarray(K).reshape(1, -1)

    def compute_control(self, state):
        """
        state: np.array [theta, theta_dot] or [x1, x2]
        """
        state = np.asarray(state).reshape(-1, 1)
        u = -self.K @ state
        return float(u)
