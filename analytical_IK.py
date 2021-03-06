import numpy as np
import modern_robotics as mr

np.set_printoptions(precision=4, suppress=False)


def analytical_IK(t, S, E, W):
    '''
    param:
        S: 1 = Sholder left, -1 = sholder right
        E: 1 = Elbow up, -1 = elbow down
        W: 1 = Wrist not flipped, -1 = wrist flipped
    '''
    
    # Resulting theta values. Filled inn when a value is found
    theta_res = np.zeros(6)     # joint 1 has index 0 and joint 6 has index 5

    n_e = t[:3, 0]
    s_e = t[:3, 1]
    a_e = t[:3, 2]
    p_e = t[:3, 3]

    p_5 = p_e - d5*a_e
    angle_1 = np.arctan2(S * p_5[1], S * p_5[0])   # angle = atan2(y5, x5)
    delta_1 = np.arctan2(d4, np.sqrt(p_5[0]**2 + p_5[1]**2 - d4**2)) # NOTE: Only defined when x5^2+y5^2 >= d4^2
    theta_res[0] = angle_1 + S * delta_1

    # Given theta_1 the unit vectors of frame 1 are computed as
    x_1 = np.array([np.cos(theta_res[0]), np.sin(theta_res[0]), 0]).T
    y_1 = np.array([0, 0, 1]).T
    z_1 = np.array([np.sin(theta_res[0]), -np.cos(theta_res[0]), 0]).T

    # Determine unitvectors of frame {4}
    z_4 = W * S * (np.cross(z_1, a_e) / np.linalg.norm(np.cross(z_1, a_e)))   # NOTE: Assumes |z_1 x a_e| != 0
    x_4 = np.cross(z_1, z_4)
    y_4 = np.cross(z_4, x_4)

    theta_res[4] = np.arctan2(np.dot(-a_e, x_4), np.dot(a_e, y_4))
    theta_res[5] = np.arctan2(np.dot(-z_4, n_e), np.dot(-z_4, s_e))

    # t2 and t3 are found from the position p3 
    p_3 = p_5 - d5*z_4 - d4*z_1

    # t2 and t3 is found from the law of cosines for the triangle with 
    # endpoint with horisontal coordinates p_h and vertical coordinate p_v
    p_h = S * np.sqrt(p_3[0]**2 + p_3[1]**2)
    p_v = p_3[2] - d1

    # The law of cosines then gives
    c3 = (p_h**2 + p_v**2 - a2**2 - a3**2) / (2 * a2 * a3)
    s3 = E * np.sqrt(1 - c3**2)
    theta_res[2] = np.arctan2(s3, c3)

    c2 = (p_h * (a2 + a3 * c3) + p_v * a3 * s3) / (p_h**2 + p_v**2)
    s2 = (p_v * (a2 + a3 * c3) - p_h * a3 * s3) / (p_h**2 + p_v**2)
    theta_res[1] = np.arctan2(s2, c2)

    # t4 is found by computing t234
    t234 = np.arctan2(np.dot(z_4, x_1), np.dot(z_4, y_1))
    theta_res[3] = t234 - (theta_res[1] + theta_res[2])
    
    return theta_res


if __name__ == "__main__":    
    d1 = 0.089    # Robot dimentions. These are set by 
    a2 = -0.425   # the robots physical properties.
    a3 = -0.392
    d4 = 0.109
    d5 = 0.095
    d6 = 0.082

    a = np.array([0, a2, a3, 0, 0, 0])
    d = np.array([d1, 0, 0, d4, d5, d6])
    
    T = np.array([[0, -1, 0, -a[2]+d[4]],      # The DH representation of 
                  [-1, 0, 0, -d[3]],           # the wanted configuration
                  [0, 0, -1, d[0]-a[1]-d[5]],
                  [0, 0, 0, 1]])
    
    for i in range(1, -2, -2):
        for j in range(1, -2, -2):
            for k in range(1, -2, -2):
                print(analytical_IK(T, i, j, k))
    