import manifpy as manif
import numpy as np
import toml


class SE3EKF:
    def __init__(self, dt, init_state, config_path):
        self.read_config(config_path)

        self.T = init_state[0]  # list [T, v]
        self.v = init_state[1]
        self.P = np.diag(np.array(self.config["EKF"]["init_state_std"]) ** 2)  # concentrated gaussian tangent space gaussian
        self.process_cov = np.diag(np.array(self.config["EKF"]["process_noise"]) ** 2)
        self.N = np.diag(np.array(self.config["EKF"]["measurement_noise"]) ** 2)
        self.dt = dt

        self.sigma = np.array([1.0, 1.0, 1.0, np.pi, np.pi, np.pi])

        J_T = np.zeros((6, 6))
        J_v = np.zeros((6, 6))
        self.H = np.block([np.eye(6), np.zeros((6, 6))])

    def read_config(self, config_path):
        with open(config_path) as f:
            self.config = toml.loads(f.read())

    def _predict(self):
        # compute x_bar P_bar (prior of mean and cov)
        J_T = np.zeros((6, 6))
        J_v = np.zeros((6, 6))

        self.T = self.T.rplus(self.v * self.dt, J_T, J_v)  # T <- T*Exp(v)
        self.v = self.v  # v <- v

        F = np.block([[J_T, J_v * self.dt], [np.zeros((6, 6)), np.eye(6)]])
        Q = np.block(
            [
                [J_v @ self.process_cov @ J_v.T * self.dt ** 2, self.process_cov @ J_v.T * self.dt],
                [J_v @ self.process_cov * self.dt, self.process_cov],
            ]
        )  # 12 x 12

        self.P = F @ self.P @ F.T + Q

    def _ekf_update(self, measurement):
        z = (measurement - self.T).coeffs()  # innovation
        Z = self.H @ self.P @ self.H.T + self.N
        K = self.P @ self.H.T @ np.linalg.inv(Z)  # near-optimal kalman gain

        # update x, P (posterior)
        dx = K @ z

        self.T = self.T + manif.SE3Tangent(dx[:6])
        self.v = manif.SE3Tangent(self.v.coeffs() + dx[6:])

        self.v = manif.SE3Tangent(np.clip(self.v.coeffs(), -np.sqrt(self.sigma), np.sqrt(self.sigma)))

        self.P = self.P - K @ Z @ K.T

        return (self.T, self.v), self.P

    def step(self, measurement):
        self._predict()
        return self._ekf_update(measurement)
