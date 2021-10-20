import numpy as np

def jacobian_cd_(theta,
                 L,
                 alpha: float = 1e-8,
                 forKin_func = None):
    """Get Jacobian from central difference.
    """
    J = np.asmatrix(np.zeros((3, len(theta)), dtype=np.float32))
    J[0,:] = 0

    theta = np.asmatrix(theta)
    for i in range(J.shape[0]):
        alpha_ = np.asmatrix(np.zeros((len(theta), 1), dtype=np.float32))
        alpha_[i, :] = alpha

        # Central difference, w.r.t theta_i
        # Remove homogeneous term
        j = (forKin_func(theta + alpha_, L)[:-1,-1] - forKin_func(theta - alpha_, L)[:-1,-1]) / (2 * alpha)
        j = np.asmatrix(j).T
        J[:,i] = j  # column

    return J

def jacobian_a_(theta,
                L, 
                p0 = np.array([0.0, 0.0, 0.0, 1])):
    """Get the analytical Jacobian.
    """
    # Prep jacobian
    J = np.asmatrix(np.zeros((3, len(theta)), dtype=np.float32))
    return J

def invKin_v1(theta,
              L,
              pos1,
              n_iters: int = 1, 
              threshold: float = 1e-2,
              forKin_func = None,
              jacobian_func = jacobian_a_):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 1: Newton's method.
    """
    theta = np.asmatrix(theta)
    for _ in range(n_iters):
        if "jacobian_cd_" in str(jacobian_func):
            J = jacobian_func(theta, L, forKin_func=forKin_func)
        else:
            J = jacobian_func(theta, L)

        pos0 = forKin_func(theta, L)[:-1,-1]
        delta_p = np.asmatrix(pos0 - pos1).T
        
        s = np.linalg.solve(-J, delta_p)
        theta += s
        if s.all() < threshold:
            break
    return theta

def invKin_v2(theta,
              L,
              pos1,
              n_iters: int = 5, 
              threshold: float = 1e-2,
              forKin_func = None,
              jacobian_func = jacobian_a_):
    """Inverse Kinematics
    Given desired pos, get joint angles.
    Version 2: Broyden's method.
    """
    theta = np.asmatrix(theta)
    # Initial estimation of the jacobian
    if "jacobian_cd_" in str(jacobian_func):
        B = jacobian_func(theta, L, forKin_func=forKin_func)
    else:
        B = jacobian_func(theta, L)

    print('#'*30)
    for _ in range(n_iters):
        pos0 = forKin_func(theta, L)[:-1,-1]
        delta_p = np.asmatrix(pos0 - pos1).T
        s = np.linalg.pinv(-B) * delta_p

        # TODO: Explore thresholding condition***
        if np.linalg.norm(s) < threshold:
            break

        # Update
        theta += s

        # Update approx jacobian
        y = np.asmatrix(forKin_func(theta, L)[:-1,-1] - pos0).T   # delta_p current iter
        B += ((y - B * s) * s.T) / (s.T * s)

        print('Cond:', np.linalg.cond(B), np.linalg.cond(s), 'Err:', np.linalg.norm(B), np.linalg.norm(s))
    print('#'*30)

    return theta