import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)


    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        q1, q2, q1_dot, q2_dot = x

        Kd = -0.5
        Kp = -0.5
        q1_dot_delta = q1_dot - q_r_dot[0]
        q2_dot_delta = q2_dot - q_r_dot[1]
        q1_delta = q1 - q_r[0]
        q2_delta = q2 - q_r[1]
        # v = q_r_ddot
        v = np.array([q_r_ddot[0] + Kd * q1_dot_delta + Kp * q1_delta,
                      q_r_ddot[1] + Kd * q2_dot_delta + Kp * q2_delta])



        Tau = self.model.M(x) @ v + self.model.C(x) @ np.array([q1_dot, q2_dot])
        # print('type',type(q_r_dot))


        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        return Tau
