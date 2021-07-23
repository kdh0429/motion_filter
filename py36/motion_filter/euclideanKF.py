import numpy as np
import scipy
from motion_filter.distutils import *
import cvxpy as cp

#ukf
class MerweScaledSigmaPoints:
    def __init__(self, dim, alpha, beta, kappa):
        self.dim = dim
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lamb = alpha **2 * (dim + kappa) - dim
        
        self.compute_weights()
        
        
    def compute_weights(self):
        self.Wn = []
        self.Wc = []
        for i in range(2*self.dim + 1):
            if i==0:
                self.Wn.append(self.lamb/(self.dim + self.lamb))
                self.Wc.append(self.lamb/(self.dim + self.lamb) + 1 - self.alpha**2 + self.beta)

            else:
                self.Wn.append(1/2/(self.dim + self.lamb))
                self.Wc.append(1/2/(self.dim + self.lamb))

    def compute_points(self, mean, cov):
        M = scipy.linalg.sqrtm((self.dim + self.lamb) * cov)
        M = np.real(M)
        X = []
        for i in range(2 * self.dim + 1):
            if i==0:
                X.append(mean)
            elif i>0 and i<=self.dim:
                X.append(mean + M[i-1])
            else:
                X.append(mean - M[i - 2-1])
        X = np.vstack(X)
        return X

class UKF:
    def __init__(self, dim, dynM, obsM, sigmas, Q, R, x0, P0):
        self.dim = dim
        self.dynM = dynM
        self.obsM = obsM
        self.sigmas = sigmas
        
        self.Q = Q
        self.R = R
        
        self.x = x0
        self.P = P0
        
    def predict(self):
        X = self.sigmas.compute_points(self.x, self.P)
        self.Y = []

        for i in range(2*self.dim + 1):
            self.Y.append(self.dynM(X[i]))
        self.Y = np.vstack(self.Y)
        
        self.x = np.sum(np.diag(self.sigmas.Wn)@self.Y, axis=0)
        self.P = (self.Y - self.x).T @ np.diag(self.sigmas.Wc) @ (self.Y - self.x) + self.Q

    def update(self, y):
        Z = []

        for i in range(2*self.dim + 1):
            Z.append(self.obsM(self.Y[i]))
        Z = np.vstack(Z)
        
        
        mu_z = np.sum(np.diag(self.sigmas.Wn) @ Z, axis=0)
        P_z = (Z - mu_z).T @ np.diag(self.sigmas.Wc) @ (Z - mu_z) + self.R
        
        K = (self.Y - self.x).T @ np.diag(self.sigmas.Wc) @ (Z - mu_z) @ np.linalg.inv(P_z)
        
        dz = y - mu_z #innovation

        self.x = self.x + K @ dz
        
        self.P = self.P - K @ P_z @ K.T
        
class RUKF:
    def __init__(self, dim, dynM, obsM, sigmas, Q, R, x0, P0, w, cauchy_P):
        self.dim = dim
        self.dynM = dynM
        self.obsM = obsM
        self.sigmas = sigmas
        
        self.w = w
        
        self.Q = Q
        self.R = R
        
        self.x = x0
        self.P = P0
        
        self.cauchy_P = cauchy_P
        
    def predict(self):
        X = self.sigmas.compute_points(self.x, self.P)
        self.Y = []

        for i in range(2*self.dim + 1):
            self.Y.append(self.dynM(X[i]))
        self.Y = np.vstack(self.Y)
        
        self.x = np.sum(np.diag(self.sigmas.Wn)@self.Y, axis=0)
        self.P = (self.Y - self.x).T @ np.diag(self.sigmas.Wc) @ (self.Y - self.x) + self.Q

    def bodyupdate(self):
        Z = []

        for i in range(2*self.dim + 1):
            Z.append(self.obsM(self.Y[i]))
        Z = np.vstack(Z)  
        
        mu_z = np.sum(np.diag(self.sigmas.Wn) @ Z, axis=0)
        P_z = (Z - mu_z).T @ np.diag(self.sigmas.Wc) @ (Z - mu_z) + self.R

        return mu_z, P_z, Z
    
    def update(self, y):
        mu_z, P_z, Z = self.bodyupdate() #y gaussian mean, cov, sigma points
        
        PSI = []

        for i in range(2*self.dim + 1):
            PSI.append(self.feature_y(Z[i], mu_z, P_z))
        PSI = np.vstack(PSI)
        
        PSI_mu = np.sum(np.diag(self.sigmas.Wn) @ PSI, axis=0)
        
        f_dim = PSI_mu.shape[0]
        
        PSI_P = (PSI - PSI_mu).T @ np.diag(self.sigmas.Wc) @ (PSI - PSI_mu)
        YPSI_P = (self.Y - self.x).T @ np.diag(self.sigmas.Wc) @ (PSI - PSI_mu)
        
        A = np.block([[np.eye(2), -YPSI_P],[np.zeros((f_dim,2)), PSI_P]]) + 0.001 * np.eye(2+f_dim)
        b = np.hstack([self.x, self.feature_y(y, mu_z, P_z) - PSI_mu])
        self.x = np.linalg.solve(A ,b)[:2]

        A = np.block([[np.eye(2), + YPSI_P],[np.zeros((f_dim,2)), PSI_P]]) + 0.001 * np.eye(2+f_dim)
        B = np.block([[self.P],[YPSI_P.T]]) 
        self.P = np.linalg.solve(A ,B)[:2, :]     
        
        
    def feature_y(self, z, mu_z, P_z):
        gauss_ll = norm_pdf_multivariate(z, mu_z, P_z)
        cauchy_ll = multivariate_t_distribution(z, mu_z, self.cauchy_P, df=1)
#         print(z, mu_z, gauss_ll, cauchy_ll)
        features = []
        
        features.append(np.array((1 - self.w) * gauss_ll))
        features.append(z * (1 - self.w) * gauss_ll)
        features.append(np.array(self.w * cauchy_ll))
#         features.append(z)
#         features.append(z**2)
#         features.append(z**3)
        
#         features.append(np.ones(1))

        features = np.hstack(features)
        
        denum = (1 - self.w) * gauss_ll + self.w * cauchy_ll
#         denum = 1.0
        
        return features / denum

class KF:
    def __init__(self, x_dim, y_dim, F, H, Q, R, x0, P0, alpha = 1.0):
        self.x_dim = x_dim
        self.y_dim = y_dim
        
        self.F = F
        self.H = H
                
        self.Q = Q
        self.R = R
        
        self.x = x0
        self.P = P0
        
        self.alpha = alpha
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.alpha * (self.F @ self.P @ self.F.T + self.Q)

    def update(self, y):
        dz = y - self.H @ self.x
        Z = self.H @ self.P @ self.H.T + self.R

        K = self.P @ self.H.T @ np.linalg.inv(Z)
        
        self.x = self.x + K @ dz
        self.P = self.P - K @ Z @ K.T

    def update2(self, y):
        A = np.block([[np.eye(self.x_dim), - self.P@self.H.T],
                      [np.zeros((self.y_dim, self.x_dim)), self.H @ self.P @ self.H.T + self.R]])
        b = np.hstack([self.x, y- self.H @ self.x])
        mean = np.linalg.solve(A ,b)

        A = np.block([[np.eye(self.x_dim), + self.P@self.H.T],[np.zeros((self.y_dim, self.x_dim)), self.H @ self.P @ self.H.T + self.R]])
        B = np.block([[self.P],[self.H@self.P]])

        cov = np.linalg.solve(A ,B)
        
        inov = y- self.H @ self.x
        inov_p = self.H @ self.P @ self.H.T + self.R
        
        self.x = mean[:self.x_dim]
        self.P = cov[:self.x_dim, :]
        
        return inov, inov_p
    
class RKF:
    def __init__(self, x_dim, y_dim, F, H, Q, R, x0, P0, alpha = 1.0):
        self.x_dim = x_dim
        self.y_dim = y_dim
        
        self.F = F
        self.H = H
                
        self.Q = Q
        self.R = R
        
        self.x = x0
        self.P = P0
        
        self.alpha = alpha
    def predict(self):
        self.x = self.F @ self.x
        self.P = self.alpha * (self.F @ self.P @ self.F.T + self.Q)

    def update(self, y):
        inov = y- self.H @ self.x
        inov_p = self.H @ self.P @ self.H.T + self.R
        
        cp_x = cp.Variable(self.x_dim)
        x_error = 0.5 * cp.quad_form(cp_x - self.x, np.linalg.inv(self.P))
#         y_error = 0.5 * cp.quad_form(y- self.H @ cp_x, np.linalg.inv(self.R))
        y_error = cp.huber((y- self.H @ cp_x) @ scipy.linalg.sqrtm(np.linalg.inv(self.R)).real ,0.5)
        
#         bound = (y- self.H @ cp_x) @ scipy.linalg.sqrtm(np.linalg.inv(self.R)).real
#         constraints = [bound <= 1.0, bound >= -1.0]
        cost = x_error + y_error
        prob = cp.Problem(cp.Minimize(cost))
        prob.solve()
        
        A = np.block([[np.eye(self.x_dim), + self.P@self.H.T],[np.zeros((self.y_dim, self.x_dim)), self.H @ self.P @ self.H.T + self.R]])
        B = np.block([[self.P],[self.H@self.P]])

        cov = np.linalg.solve(A ,B)
        

        
        self.x = cp_x.value
        self.P = cov[:self.x_dim, :]
        
        return inov, inov_p