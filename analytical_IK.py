import numpy as np

np.set_printoptions(precision=4, suppress=False)

'''
Origin of DH frame i, denoted as p_i
Frame {e} is the same as frame {6}
''' 

def analytical_IK_simple():
    d1 = 0.089
    a2 = -0.425
    a3 = -0.392
    d4 = 0.109
    d5 = 0.095
    d6 = 0.082

    # Resulting theta values. Filled inn when a value is found
    theta_res = np.zeros(6)     # joint 1 has index 0 and joint 6 has index 5

    # Denavit-Hartenberg parameters for the UR5 robot
    a = np.array([0, a2, a3, 0, 0, 0])
    alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    d = np.array([d1, 0, 0, d4, d5, d6])

    # Theta values for home-position. NB: Not zero pos
    theta_home = np.array([0, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0]).T

    # Transformation matrix from 0 to end effector e.
    T_0e = np.array([[0, -1, 0, -a3+d5],        # also written as
                    [-1, 0, 0, -d4],            # T_0e = [[n_e, s_e, a_e, p_e],
                    [0, 0, -1, d1-a2-d6],       #         [0,   0,   0,   1]]
                    [0, 0, 0, 1]])

    n_e = T_0e[:3, 0]
    s_e = T_0e[:3, 1]
    a_e = T_0e[:3, 2]
    p_e = T_0e[:3, 3]

    p_5 = p_e - d5*a_e
    angle_1 = np.arctan2(p_5[1], p_5[0])   # angle = atan2(y5, x5)
    delta_1 = np.arctan2(d4, np.sqrt(p_5[0]**2 + p_5[1]**2 - d4**2)) # NOTE: Only defined when x5^2+y5^2 >= d4^2
    theta_res[0] = angle_1 + delta_1

    # Given theta_1 the unit vectors of frame 1 are computed as
    x_1 = np.array([np.cos(theta_res[0]), np.sin(theta_res[0]), 0]).T
    y_1 = np.array([0, 0, 1]).T
    z_1 = np.array([np.sin(theta_res[0]), -np.cos(theta_res[0]), 0]).T

    # Determine unitvectors of frame {4}
    z_4 = np.cross(z_1, a_e) / np.linalg.norm(np.cross(z_1, a_e))   # NOTE: Assumes |z_1 x a_e| != 0
    x_4 = np.cross(z_1, z_4)
    y_4 = np.cross(z_4, x_4)

    theta_res[4] = np.arctan2(np.dot(-a_e, x_4), np.dot(a_e, y_4))
    theta_res[5] = np.arctan2(np.dot(-z_4, n_e), np.dot(-z_4, s_e))

    # t2 and t3 are found from the position p3 
    p_3 = p_5 - d5*z_4 - d4*z_1

    # t2 and t3 is found from the law of cosines for the triangle with 
    # endpoint with horisontal coordinates p_h and vertical coordinate p_v
    p_h = np.sqrt(p_3[0]**2 + p_3[1]**2)
    p_v = p_3[2] - d1

    # The law of cosines then gives
    c3 = (p_h**2 + p_v**2 - a2**2 - a3**2) / (2 * a2 * a3)
    s3 = np.sqrt(1 - c3**2)
    theta_res[2] = np.arctan2(s3, c3)

    c2 = (p_h * (a2 + a3 * c3) + p_v * a3 * s3) / (p_h**2 + p_v**2)
    s2 = (p_v * (a2 + a3 * c3) - p_h * a3 * s3) / (p_h**2 + p_v**2)
    theta_res[1] = np.arctan2(s2, c2)


    # t4 is found by computing t234
    t234 = np.arctan2(np.dot(z_4, x_1), np.dot(z_4, y_1))
    theta_res[3] = t234 - (theta_res[1] + theta_res[2])
    
    print(np.rad2deg(theta_res))
    return theta_res


def analytical_IK(S, E, W):
    '''
    param:
        S: 1 = Sholder left, -1 = sholder right
        E: 1 = Elbow up, -1 = elbow down
        W: 1 = Wrist not flipped, -1 = wrist flipped
    '''
    
    d1 = 0.089
    a2 = -0.425
    a3 = -0.392
    d4 = 0.109
    d5 = 0.095
    d6 = 0.082

    # Resulting theta values. Filled inn when a value is found
    theta_res = np.zeros(6)     # joint 1 has index 0 and joint 6 has index 5

    # Denavit-Hartenberg parameters for the UR5 robot
    a = np.array([0, a2, a3, 0, 0, 0])
    alpha = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
    d = np.array([d1, 0, 0, d4, d5, d6])

    # Theta values for home-position. NB: Not zero pos
    theta_home = np.array([0, -np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, 0]).T

    # Transformation matrix from 0 to end effector e.
    T_0e = np.array([[0, -1, 0, -a[2]+d[4]],        # also written as
                    [-1, 0, 0, -d[3]],              # T_0e = [[n_e, s_e, a_e, p_e],
                    [0, 0, -1, d[0]-a[1]-d[5]],     #         [0,   0,   0,   1]]
                    [0, 0, 0, 1]])

    n_e = T_0e[:3, 0]
    s_e = T_0e[:3, 1]
    a_e = T_0e[:3, 2]
    p_e = T_0e[:3, 3]

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
    
    print(np.rad2deg(theta_res))
    return theta_res


if __name__ == "__main__":
    for i in range(1, -2, -2):
        for j in range(1, -2, -2):
            for k in range(1, -2, -2):
                analytical_IK(i, j, k)
   