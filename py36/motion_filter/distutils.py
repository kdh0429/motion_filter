import numpy as np
from math import *

#distribution
def multivariate_t_distribution(x,mu,Sigma,df):
    '''
    Multivariate t-student density:
    output:
        the density of the given element
    input:
        x = parameter (d dimensional numpy array or scalar)
        mu = mean (d dimensional numpy array or scalar)
        Sigma = scale matrix (dxd numpy array)
        df = degrees of freedom
    '''
    d = len(x)
    Num = gamma(1. * (d+df)/2)
    Denom = ( gamma(1.*df/2) * pow(df*np.pi,1.*d/2) * pow(np.linalg.det(Sigma),1./2) \
             * pow(1 + (1./df)*np.dot(np.dot((x - mu),np.linalg.inv(Sigma)), (x - mu)),1.* (d+df)/2))
    return 1. * Num / Denom 

def norm_pdf_multivariate(x, mu, sigma):
    size = len(x)
    if size == len(mu) and (size, size) == sigma.shape:
        det = np.linalg.det(sigma)
        if det == 0:
            raise NameError("The covariance matrix can't be singular")

        norm_const = 1.0/ ( pow((2*np.pi),float(size)/2) * pow(det,1.0/2) )
        x_mu = (x - mu).reshape(1, -1)
        inv = np.linalg.inv(sigma)
        result = np.exp(-0.5 * (x_mu @ inv @ x_mu.T)).item()
        return norm_const * result
    else:
        raise NameError("The dimensions of the input don't match")