import numpy as np

def jacobian_cd_(theta,
                 L,
                 n_poses,
                 forKin_func,
                 pose_tracker,
                 alpha: float = 1e-8):
    """Get Jacobian from central difference.
    """
    J = np.asmatrix(np.zeros((n_poses, len(theta)), dtype=np.float32))

    theta = np.asmatrix(theta)
    for i in range(J.shape[1]):
        alpha_ = np.asmatrix(np.zeros((len(theta), 1), dtype=np.float32))
        alpha_[i, :] = alpha

        # Central difference, w.r.t theta_i
        # Get 3D pose 1st
        # Track the 2D pose from camera
        P_plus = forKin_func(theta + alpha_, L)[:,-1]
        p_plus = pose_tracker.track(P_plus)
        P_minus = forKin_func(theta - alpha_, L)[:,-1]
        p_minus = pose_tracker.track(P_minus)

        # C. D.
        j = (p_plus - p_minus) / (2 * alpha)
        j = np.asmatrix(j).T
        J[:,i] = j  # column

    return J

def invKin_vs(theta,
              L,
              pos1,
              forKin_func,
              pose_tracker,
              step_size: float = 1e-1,
              n_iters: int = 5, 
              threshold: float = 10.0,
              f_report=None):
    # Ensure matrix calculation
    theta = np.asmatrix(theta)

    n_poses = pos1.shape[0]
    
    # Initial estimation of the jacobian
    B = jacobian_cd_(theta, L, n_poses, 
                     forKin_func=forKin_func,
                     pose_tracker=pose_tracker)

    # NOTE: Debug msg
    if f_report is not None:
        f_report.write('####################\n')

    for iter_i in range(n_iters):
        pos0 = pose_tracker.track(forKin_func(theta, L)[:,-1])

        delta_p = np.asmatrix(pos0 - pos1).T
        s = np.linalg.pinv(-B) * delta_p

        # TODO: Explore thresholding condition***
        #if np.linalg.norm(s) < threshold:
        #    break
        # NOTE: Debug msg
        if f_report is not None:
            f_report.write("VS Iter: {}\n".format(iter_i + 1))
            f_report.write("B - Cond: {}, Norm: {}\n".format(np.linalg.cond(B), 
                                                             np.linalg.norm(B)))
            f_report.write("s - Cond: {}, Norm: {}\n".format(np.linalg.cond(s), 
                                                             np.linalg.norm(s)))
            f_report.write('#####\n')

        # Update
        theta += s

        # Update approx jacobian
        y = np.asmatrix(pose_tracker.track(forKin_func(theta, L)[:,-1]) - pos0).T   # delta_p current iter
        B += ((y - B * s) * s.T) / (s.T * s)

    return theta