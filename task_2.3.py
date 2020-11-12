import numpy as np
import modern_robotics as mr

#Values are based on the UR5 robot in example 4.5 in Modern Robotics

d1 = 89     #H1
a2 = -425   #-L1
a3 = 392    #-L2
d4 = 109    #W1
d5 = 95     #H2
d6 = 82     #W2


alist = np.array([0, a2, a3, 0, 0, 0])
alphalist = np.array([np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0])
dlist = np.array([d1, 0, 0, d4, d5, d6])
thetalist = np.array([0, 0, 0, 0, 0, 0])

def forward_kinematic_PoE():
    

    return True

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
    #T06_PoE =
    T06_DH = np.matrix.round(forward_kinematic_DH(alist, alphalist, dlist, thetalist), 0, None)

    print('The matrix T_06 found by the Power of Exponentials computations is:\n')
    print('The matrix T_06 found by the Denavit Hartenberg convention is:\n')
    print(T06_DH)