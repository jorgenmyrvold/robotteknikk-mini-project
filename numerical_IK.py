import numpy as np
import modern_robotics as mr

l_1 = 0.425
l_2 = 0.392
w_1 = 0.109
w_2 = 0.082
h_1 = 0.089
h_2 = 0.095




M = np.array([[-1,0,0,l_1+l_2],
              [0,0,1,w_1+w_2],
              [0,1,0,h_1-h_2],
              [0,0,0,1]])

Slist = np.array([[0,0,1,0,0,0],
                  [0,1,0,-h_1,0,0],
                  [0,1,0,-h_1,0,l_1],
                  [0,1,0,-h_1,0,l_1+l_2],
                 [0,0,-1,-w_1,l_1+l_2,0],
                  [0,1,0,h_2-h_1,0,l_1+l_2]]).T
d1 = 0.089
a2 = -0.425
a3 = -0.392
d4 = 0.109
d5 = 0.095
d6 = 0.082

a = np.array([0, a2, a3, 0, 0, 0])
d = np.array([d1, 0, 0, d4, d5, d6])

T = np.array([[0, -1, 0, -a[2]+d[4]],        
                    [-1, 0, 0, -d[3]],              
                    [0, 0, -1, d[0]-a[1]-d[5]],    
                    [0, 0, 0, 1]])
eomg = 0.01
ev = 0.001
[numerical_solution, success] = mr.IKinSpace(Slist,M,T,analytical_solution1,eomg,ev)