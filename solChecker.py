from ikbtbasics.ik_classes import *
from ikbtfunctions.ik_robots import * 
from ikbtbasics.kin_cl import *
from math import *
import os

testing = False

robot = 'Puma'


((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')

#   Get the robot model 
[dh, vv, params, pvals, unknowns] = robot_params(robot)  # see ik_robots.py 

[M, R, unknowns] = kinematics_pickle(robot, dh, params, pvals, vv, unknowns, testing)

# need to type in the poses
pose = {th_1: 30*deg, th_2: 50*deg, th_3: 40*deg, th_4: 45*deg, th_5: 120*deg, \
        th_6: 60*deg, th_23: 90*deg}


params_num = {a_2: 5, a_3: 1, d_3: 2, d_4:4}
#params_num = pvals

a_2 = 5
a_3 = 1
d_3 = 2
d_4 = 4



#T = forward_kinematics_N(M, pose, pvals)
T = forward_kinematics_N(M, pose, params_num)

def convert_to_degrees(solution_list):
    solution_in_degree = []
    for a_set in solution_list:
        curr_set = []
        for curr_rad in a_set:
            if curr_rad > 2*pi:
                curr_rad = curr_rad % (2*pi)
            curr_set.append(degrees(curr_rad))
        solution_in_degree.append(curr_set)
    return solution_in_degree

def verify_T_matrices(pose_list, variable_template, M, params_num):
    T_mat_list = []

    for one_pose in pose_list:
        pose = {}
        for i in range(len(one_pose)):
            pose[variable_template[i]] = one_pose[i]*deg

        T = forward_kinematics_N(M, pose, params_num)
        T_mat_list.append(T)
    return T_mat_list






#define the input vars
r_11 = T[0,0]
r_12 = T[0,1]
r_13 = T[0,2]
r_21 = T[1,0]
r_22 = T[1,1]
r_23 = T[1,2]
r_31 = T[2,0]
r_32 = T[2,1]
r_33 = T[2,2]
Px = T[0,3]
Py = T[1,3]
Pz = T[2,3]

    

#
# Caution:    Generated code is not yet validated
#


#

   #Variable:  th_1
th_1s1 = atan2(Px, -Py) + atan2(sqrt(Px**2 + Py**2 - d_3**2), -d_3)
th_1s2 = atan2(Px, -Py) + atan2(-sqrt(Px**2 + Py**2 - d_3**2), -d_3)


#Variable:  th_3
th_3s3 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)
th_3s2 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(-sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)
th_3s1 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s1) + Py*sin(th_1s1))**2)
th_3s4 = atan2(-2*a_2*d_4, 2*a_2*a_3) + atan2(-sqrt(4*a_2**2*a_3**2 + 4*a_2**2*d_4**2 - (Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)**2), Pz**2 - a_2**2 - a_3**2 - d_4**2 + (Px*cos(th_1s2) + Py*sin(th_1s2))**2)


#Variable:  th_23
th_23s2 = atan2(Pz*(-a_2*cos(th_3s2) - a_3) - (-Px*cos(th_1s1) - Py*sin(th_1s1))*(a_2*sin(th_3s2) - d_4), Pz*(a_2*sin(th_3s2) - d_4) + (-Px*cos(th_1s1) - Py*sin(th_1s1))*(-a_2*cos(th_3s2) - a_3))
th_23s4 = atan2(Pz*(-a_2*cos(th_3s4) - a_3) - (-Px*cos(th_1s2) - Py*sin(th_1s2))*(a_2*sin(th_3s4) - d_4), Pz*(a_2*sin(th_3s4) - d_4) + (-Px*cos(th_1s2) - Py*sin(th_1s2))*(-a_2*cos(th_3s4) - a_3))
th_23s1 = atan2(Pz*(-a_2*cos(th_3s3) - a_3) - (-Px*cos(th_1s2) - Py*sin(th_1s2))*(a_2*sin(th_3s3) - d_4), Pz*(a_2*sin(th_3s3) - d_4) + (-Px*cos(th_1s2) - Py*sin(th_1s2))*(-a_2*cos(th_3s3) - a_3))
th_23s3 = atan2(Pz*(-a_2*cos(th_3s1) - a_3) - (-Px*cos(th_1s1) - Py*sin(th_1s1))*(a_2*sin(th_3s1) - d_4), Pz*(a_2*sin(th_3s1) - d_4) + (-Px*cos(th_1s1) - Py*sin(th_1s1))*(-a_2*cos(th_3s1) - a_3))


#Variable:  th_2
th_2s2 = th_23s4 - th_3s4
th_2s4 = th_23s3 - th_3s1
th_2s1 = th_23s2 - th_3s2
th_2s3 = th_23s1 - th_3s3


#Variable:  th_4
th_4s3 = atan2(r_13*sin(th_1s2) - r_23*cos(th_1s2), r_13*cos(th_1s2)*cos(th_23s4) + r_23*sin(th_1s2)*cos(th_23s4) - r_33*sin(th_23s4))
th_4s7 = atan2(r_13*sin(th_1s1) - r_23*cos(th_1s1), r_13*cos(th_1s1)*cos(th_23s3) + r_23*sin(th_1s1)*cos(th_23s3) - r_33*sin(th_23s3))
th_4s2 = atan2(-r_13*sin(th_1s1) + r_23*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_23s2) - r_23*sin(th_1s1)*cos(th_23s2) + r_33*sin(th_23s2))
th_4s6 = atan2(-r_13*sin(th_1s2) + r_23*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_23s1) - r_23*sin(th_1s2)*cos(th_23s1) + r_33*sin(th_23s1))
th_4s4 = atan2(-r_13*sin(th_1s2) + r_23*cos(th_1s2), -r_13*cos(th_1s2)*cos(th_23s4) - r_23*sin(th_1s2)*cos(th_23s4) + r_33*sin(th_23s4))
th_4s1 = atan2(r_13*sin(th_1s1) - r_23*cos(th_1s1), r_13*cos(th_1s1)*cos(th_23s2) + r_23*sin(th_1s1)*cos(th_23s2) - r_33*sin(th_23s2))
th_4s8 = atan2(-r_13*sin(th_1s1) + r_23*cos(th_1s1), -r_13*cos(th_1s1)*cos(th_23s3) - r_23*sin(th_1s1)*cos(th_23s3) + r_33*sin(th_23s3))
th_4s5 = atan2(r_13*sin(th_1s2) - r_23*cos(th_1s2), r_13*cos(th_1s2)*cos(th_23s1) + r_23*sin(th_1s2)*cos(th_23s1) - r_33*sin(th_23s1))


#Variable:  th_5
th_5s4 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s6), -r_13*sin(th_23s1)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s1) - r_33*cos(th_23s1))
th_5s8 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s5), -r_13*sin(th_23s1)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s1) - r_33*cos(th_23s1))
th_5s1 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s3), -r_13*sin(th_23s4)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s4) - r_33*cos(th_23s4))
th_5s2 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s7), -r_13*sin(th_23s3)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s3) - r_33*cos(th_23s3))
th_5s6 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s1), -r_13*sin(th_23s2)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s2) - r_33*cos(th_23s2))
th_5s3 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s2), -r_13*sin(th_23s2)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s2) - r_33*cos(th_23s2))
th_5s7 = atan2((-r_13*sin(th_1s1) + r_23*cos(th_1s1))/sin(th_4s8), -r_13*sin(th_23s3)*cos(th_1s1) - r_23*sin(th_1s1)*sin(th_23s3) - r_33*cos(th_23s3))
th_5s5 = atan2((-r_13*sin(th_1s2) + r_23*cos(th_1s2))/sin(th_4s4), -r_13*sin(th_23s4)*cos(th_1s2) - r_23*sin(th_1s2)*sin(th_23s4) - r_33*cos(th_23s4))


#Variable:  th_6
th_6s6 = atan2(-(-r_12*sin(th_23s2)*cos(th_1s1) - r_22*sin(th_1s1)*sin(th_23s2) - r_32*cos(th_23s2))/sin(th_5s3), (-r_11*sin(th_23s2)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s2) - r_31*cos(th_23s2))/sin(th_5s3))
th_6s5 = atan2(-(-r_12*sin(th_23s2)*cos(th_1s1) - r_22*sin(th_1s1)*sin(th_23s2) - r_32*cos(th_23s2))/sin(th_5s6), (-r_11*sin(th_23s2)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s2) - r_31*cos(th_23s2))/sin(th_5s6))
th_6s1 = atan2(-(-r_12*sin(th_23s1)*cos(th_1s2) - r_22*sin(th_1s2)*sin(th_23s1) - r_32*cos(th_23s1))/sin(th_5s4), (-r_11*sin(th_23s1)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s1) - r_31*cos(th_23s1))/sin(th_5s4))
th_6s8 = atan2(-(-r_12*sin(th_23s4)*cos(th_1s2) - r_22*sin(th_1s2)*sin(th_23s4) - r_32*cos(th_23s4))/sin(th_5s5), (-r_11*sin(th_23s4)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s4) - r_31*cos(th_23s4))/sin(th_5s5))
th_6s4 = atan2(-(-r_12*sin(th_23s3)*cos(th_1s1) - r_22*sin(th_1s1)*sin(th_23s3) - r_32*cos(th_23s3))/sin(th_5s2), (-r_11*sin(th_23s3)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s3) - r_31*cos(th_23s3))/sin(th_5s2))
th_6s2 = atan2(-(-r_12*sin(th_23s1)*cos(th_1s2) - r_22*sin(th_1s2)*sin(th_23s1) - r_32*cos(th_23s1))/sin(th_5s8), (-r_11*sin(th_23s1)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s1) - r_31*cos(th_23s1))/sin(th_5s8))
th_6s7 = atan2(-(-r_12*sin(th_23s3)*cos(th_1s1) - r_22*sin(th_1s1)*sin(th_23s3) - r_32*cos(th_23s3))/sin(th_5s7), (-r_11*sin(th_23s3)*cos(th_1s1) - r_21*sin(th_1s1)*sin(th_23s3) - r_31*cos(th_23s3))/sin(th_5s7))
th_6s3 = atan2(-(-r_12*sin(th_23s4)*cos(th_1s2) - r_22*sin(th_1s2)*sin(th_23s4) - r_32*cos(th_23s4))/sin(th_5s1), (-r_11*sin(th_23s4)*cos(th_1s2) - r_21*sin(th_1s2)*sin(th_23s4) - r_31*cos(th_23s4))/sin(th_5s1))

##################################
#
#package the solutions into a list for each set
#
###################################
solution_list = []
#(note trailing commas allowed in python
solution_list.append( [  th_1s2,  th_23s4,  th_2s2,  th_3s4,  th_4s3,  th_5s1,  th_6s3,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s1,  th_23s3,  th_2s4,  th_3s1,  th_4s7,  th_5s2,  th_6s4,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s1,  th_23s2,  th_2s1,  th_3s2,  th_4s2,  th_5s3,  th_6s6,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s2,  th_23s1,  th_2s3,  th_3s3,  th_4s6,  th_5s4,  th_6s1,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s2,  th_23s4,  th_2s2,  th_3s4,  th_4s4,  th_5s5,  th_6s8,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s1,  th_23s2,  th_2s1,  th_3s2,  th_4s1,  th_5s6,  th_6s5,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s1,  th_23s3,  th_2s4,  th_3s1,  th_4s8,  th_5s7,  th_6s7,  ] )
#(note trailing commas allowed in python
solution_list.append( [  th_1s2,  th_23s1,  th_2s3,  th_3s3,  th_4s5,  th_5s8,  th_6s2,  ] )




# convert to degree
solution_list = convert_to_degrees(solution_list)

# create a template
# write down the symbol for each variable in order (solution_list)
variable_temp = [th_1, th_23, th_2, th_3, th_4, th_5, th_6]

# test if each pose can generate the same T matrices

T_matrices = verify_T_matrices(solution_list, variable_temp, M, params_num)


save_dir = 'solution_num/'
if not os.path.isdir(save_dir):
    os.mkdir(save_dir)

# save the original version
np.save('solution_num/Puma_Solutions.npy', solution_list)
np.save('solution_num/Puma_T_matrices.npy', T_matrices)

# print the truncated/round up version

# print poses/solutions
# to print float up to 5 decimal places
class prettyfloat(float):
    def __repr__(self):
        #return "%0.5f" % self
        return "{:10.5f}".format(self)
        
for a_set in solution_list:
    truncated_pose = map(prettyfloat, a_set)
    print truncated_pose

# this indexing is only for Numpy Array
def printNumArray(a):
    for row in range(a.shape[0]):
        row_ls = a[row].tolist()
        truncate = map(prettyfloat, row_ls[0])
        print truncate
    print '\n'

    
# print the original T matrix
print "the original T matrix"
printNumArray(T)

print "T matrices calculated from solution poses"
for one_T in T_matrices:
    printNumArray(one_T)




