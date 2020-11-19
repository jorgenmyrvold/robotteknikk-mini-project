import numpy as np
import modern_robotics as mr

np.set_printoptions(precision = 4, suppress = True)


#Values are based on the UR5 robot in example 4.5 in Modern Robotics

#Values needed to calculate the Forward Kinematics with the DH convention
W1 = 109 #[mm]
W2 = 82
L1 = 425
L2 = 392
H1 = 89
H2 = 95
d1 = H1
a2 = -L1
a3 = -L2
d4 = W1
d5 = H2
d6 = W2
alist = np.array([0, a2, a3, 0, 0, 0])
alphalist = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
dlist = np.array([d1, 0, 0, d4, d5, d6])
thetalist = np.array([0, 0, 0, 0, 0, 0])

#Values needed to calculate the Forward Kinematics with the PoE computations
Slist = np.array([[0, 0, 1, 0, 0, 0],
                [0, 1, 0, -H1, 0, 0],
                [0, 1, 0, -H1, 0, L1],
                [0, 1, 0, -H1, 0, L1+L2],
                [0, 0, -1, -W1, L1+L2, 0],
                [0, 1, 1, H2-H1, 0, L1+L2]]).T

tlist = np.array([0, 0, 0, 0, 0, 0])

M = np.array([[-1, 0, 0, L1+L2],
              [0, 0, 1, W1+W2],
              [0, 1, 0, H1-H2],
              [0, 0, 0, 1]])

def forward_kinematic_PoE(M, Slist, tlist):
    T = np.eye(4)

    #Creates a list over the [S_i] matrices based on the Slist
    S_bracket_list = np.array([mr.VecTose3(Slist[0]),
                            mr.VecTose3(Slist[1]),
                            mr.VecTose3(Slist[2]),
                            mr.VecTose3(Slist[3]),
                            mr.VecTose3(Slist[4]),
                            mr.VecTose3(Slist[5])])

    #Creates a list over the exp{[S_i]omega_i} based on the [S_i] matrices
    Elist = np.array([mr.MatrixExp6(S_bracket_list[0]),
                    mr.MatrixExp6(S_bracket_list[1]),
                    mr.MatrixExp6(S_bracket_list[2]),
                    mr.MatrixExp6(S_bracket_list[3]),
                    mr.MatrixExp6(S_bracket_list[4]),
                    mr.MatrixExp6(S_bracket_list[5])])

    for i in Elist:
        T = T @ i
    T = T @ M
    return T

def forward_kinematic_DH(alist, alphalist, dlist, thetalist):
    #Solves the forward kinetmatics of a UR5 robot
    A01 = np.eye(4)
    A12 = np.eye(4)
    A23 = np.eye(4)
    A34 = np.eye(4)
    A45 = np.eye(4)
    A56 = np.eye(4)
    A_matrices = np.array([A01, A12, A23, A34, A45, A56])
    
    #Setting up the A-matrices
    n = 0
    for i in A_matrices:
        r = alist[n]
        a = alphalist[n]
        d = dlist[n]
        t = thetalist[n]

        A_matrices[n] = np.array([[np.cos(t), -np.sin(t)*np.cos(a), np.sin(t)*np.sin(a), r*np.cos(t)],
                    [np.sin(t), np.cos(t)*np.cos(a), -np.cos(t)*np.sin(a), r*np.sin(t)],
                    [0, np.sin(a), np.cos(a), d],
                    [0, 0, 0, 1]])
        n += 1

    #Multiplying A matrices to find T06
    T06 = A01
    n = 1
    for i in A_matrices:
        T06 = T06 @ i
    return T06

if __name__ == "__main__":
    T06_PoE = forward_kinematic_PoE(M, Slist, tlist)
    T06_DH = forward_kinematic_DH(alist, alphalist, dlist, thetalist)

    print('The matrix T_06 found by the function FKinSpace is:')
    print(mr.FKinSpace(M, Slist, tlist))
    print('\nThe matrix T_06 found by the Power of Exponentials computations is:')
    print(T06_PoE)
    print('\nThe matrix T_06 found by the Denavit Hartenberg convention is:')
    print(T06_DH)