"""
Fang, Huazhen, Mulugeta A. Haile, and Yebin Wang. 
"Robust extended kalman filtering for systems with measurement outliers."
IEEE Transactions on Control Systems Technology (2021).
"""

from motion_filters import SE3EKF
import numpy as np
import manifpy as manif

class SE3ISEKF(SE3EKF):
    def __init__(self, dt, init_state, config_path):
        super().__init__(dt, init_state, config_path)
        
        self.P = np.diag(np.array(self.config["ISEKF"]["init_state_std"])**2) # concentrated gaussian tangent space gaussian
        self.process_cov = np.diag(np.array(self.config["ISEKF"]["process_noise"])**2)
        self.N = np.diag(np.array(self.config["ISEKF"]["measurement_noise"])**2)

        self.sigma = np.array(self.config["ISEKF"]["sigma_init"])**2 * self.dt**2
        self.epsilon = np.array(self.config["ISEKF"]["epsilon_init"])

        self.lambda1 = self.config["ISEKF"]["lambda1"]
        self.lambda2 = self.config["ISEKF"]["lambda2"]

        self.gamma1 = self.config["ISEKF"]["gamma2"]
        self.gamma2 = self.config["ISEKF"]["gamma2"]

        print("sigma init ",self.sigma)
        print("epsilon init ",self.epsilon)
        print("lambda1 init ",self.lambda1)
        print("lambda2 init ",self.lambda2)
        print("gamma2 init ",self.gamma2)

    def _isekf_update(self, measurement):
        z = (measurement - self.T).coeffs() # innovation
        z_clip = np.clip(z, -np.sqrt(self.sigma), np.sqrt(self.sigma))
        
        Z = self.H @ self.P @ self.H.T + self.N
        K = self.P @ self.H.T @ np.linalg.inv(Z) # near-optimal kalman gain
        
        #update x, P (posterior)
        dx = K @ z_clip
        
        self.T = self.T + manif.SE3Tangent(dx[:6])
        self.v = manif.SE3Tangent(self.v.coeffs() + dx[6:])
        
        self.P = self.P - K @ Z @ K.T

        self.dynamic_clip_update(z)

        return (self.T, self.v), self.P

    def step(self, measurement):
        self._predict()
        return self._isekf_update(measurement)

    def dynamic_clip_update(self, z):        
        self.sigma = self.lambda1 * self.sigma + self.gamma1 * self.epsilon * np.exp(-self.epsilon)
        self.epsilon = self.lambda2 * self.epsilon + self.gamma2 * (z)**2