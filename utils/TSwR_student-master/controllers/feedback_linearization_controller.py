import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        M = self.model.M(x)
        C = self.model.C(x)
        e = q_r - x[:2]
        e_dot = q_r_dot - x[2:]
        Kd = np.array([[25, 0], [0, 15]])
        Kp = np.array([[25, 0], [0, 60]])
        v = q_r_ddot + Kd@e_dot + Kp@e
        tau = M@v + C@q_r_dot
        return tau
