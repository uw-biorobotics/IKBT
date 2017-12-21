#!/usr/bin/python
#

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp
import numpy as np
import sys
######################################################################
#
#   Symbolic Python symbols and utilties
#      Revision 1  4-Feb-2016
#      Revision 2  1-Apr-2016
#

#  Constants
deg = 6.28317531/360.0
rad2deg = 1.0/deg


# Special Types used for validating numerical evaluations etc.
#    These need to be converted before using numerical matrices but
#     tend to hang around some times.
stype_zero  = type(sp.S(sp.cos(sp.pi/2)))
stype_one   = type(sp.S(sp.sin(sp.pi/2)))
stype_float = type(sp.S(0.12345))
stype_int   = type(sp.S(5))
stype_m_one = type(sp.S(sp.sin(-sp.pi/2)))


# define a place for manipulator parameters
params = {}


###############################################################################################
#
#                   useful functions
#
# test for approximate equality
def approx_N(a,b,verbose):
  thr = max(0.001*max(abs(a),abs(b)), 1.0E-5)
  if(np.abs(a-b) < thr):
    return True
  if(verbose):
    print 'approx = False [',a,b,']',
  return False

# move angles into the range -pi to +pi
def wrap_angle_N(t):
  if(t < -np.pi):
    t = t + 2*np.pi
  elif(t >= np.pi):
    t = t - 2*np.pi
  return(t)

#  check a matrix to make sure all its elements were properly converted to numerical form
#   (if they were not, very cryptic error messages result)

def Num_check(x):
  r,c = np.shape(x)
  for i in range(0,r):
    for j in range(0,c):
      if ((type(x[i,j]) != stype_zero) &
          (type(x[i,j]) != stype_one)  &
          (type(x[i,j]) != stype_m_one)  &
          (type(x[i,j]) != stype_int )  &
          (type(x[i,j]) != stype_float)):
               print 'Symbolic to Numeric Conversion Error: at least one of your symbols not converted'
               print ' >> ', x[i,j]
               sys.exit()

#################################################################################################
#
#             Kinematics
#

# Homogeneous transform for link based on DH parameters
def Link_S(al, a, d, th):
  t = sp.Matrix ([
    [sp.cos(th),                -sp.sin(th),               0,                a],
    [sp.sin(th)*sp.cos(al),  sp.cos(th)*sp.cos(al), -sp.sin(al), -sp.sin(al)*d],
    [sp.sin(th)*sp.sin(al),  sp.cos(th)*sp.sin(al),  sp.cos(al),  sp.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t

# Homogeneous transform for link based on DH parameters
def Link_N(al, a, d, th):
  t = np.matrix ([
    [np.cos(th),                -np.sin(th),               0,                a],
    [np.sin(th)*np.cos(al),  np.cos(th)*np.cos(al), -np.sin(al), -np.sin(al)*d],
    [np.sin(th)*np.sin(al),  np.cos(th)*np.sin(al),  np.cos(al),  np.cos(al)*d],
    [0  ,0  ,0  ,1]
    ])
  return t

########################################################################
#   Declare some symbols
#
# abbreviated symbols for the trig functions

#(s_1, s_2, s_3, s_4, s_5, s_6) = sp.symbols(('s_1', 's_2', 's_3', 's_4', 's_5', 's_6'))
sp.var('s_:7')
sp.var('c_:7')
del s_0 , c_0    # there is no joint 0!

#(c_1, c_2, c_3, c_4, c_5, c_6) = sp.symbols(('c_1', 'c_2', 'c_3', 'c_4', 'c_5', 'c_6'))

#DH params for six Links
sp.var('al_:7')  # \alpha_N-1
sp.var('a_:7')   # a_N-1
sp.var('d_:7')   # d_N
sp.var('th_:7')  #\theta_N 


## symbols for sum-of-angles identities  
sp.var('c_12 s_12 c_23 s_23 c_34 s_34 c_45 s_45 c_56 s_56 c_13 s_13 ')

def notation_squeeze(T):
  # substitute compact names for cos(theta) etc.
  T = T.subs(sp.sin(th_1), s_1)
  T = T.subs(sp.sin(th_2), s_2)
  T = T.subs(sp.sin(th_3), s_3)
  T = T.subs(sp.sin(th_4), s_4)
  T = T.subs(sp.sin(th_5), s_5)
  T = T.subs(sp.sin(th_6), s_6)

  T = T.subs(sp.cos(th_1), c_1)
  T = T.subs(sp.cos(th_2), c_2)
  T = T.subs(sp.cos(th_3), c_3)
  T = T.subs(sp.cos(th_4), c_4)
  T = T.subs(sp.cos(th_5), c_5)
  T = T.subs(sp.cos(th_6), c_6)
  
  # Substitute terms of form cos(th_i + th_j) (for the major cases)
  
  pairs = [(sp.cos(th_1+th_2), c_12) ,
	   (sp.sin(th_1+th_2), s_12) , 
	   (sp.cos(th_2+th_3), c_23) ,
	   (sp.sin(th_2+th_3), s_23) ,
	   (sp.cos(th_3+th_4), c_34) ,
	   (sp.sin(th_3+th_4), s_34) ,
	   (sp.cos(th_4+th_5), c_45) ,
	   (sp.sin(th_4+th_5), s_45) ,
	   (sp.cos(th_5+th_6), c_56) ,
	   (sp.sin(th_5+th_6), s_56) ,
	   (sp.cos(th_1+th_3), c_13) ,  #might happen with prismatic in between??
	   (sp.sin(th_1+th_3), s_13) 
	   ]
  T = T.subs(pairs)

  return T



  


###################### Symbolic functions

##  The cannonical rotation matrices


def RotX_S(t):
  return(sp.Matrix([
    [1,         0,           0],
    [0, sp.cos(t),  -sp.sin(t)],
    [0, sp.sin(t),   sp.cos(t)]
    ]))

def RotY_S(t):
  return(sp.Matrix([
    [ sp.cos(t),   0,      sp.sin(t)],
    [0,            1,          0    ],
    [-sp.sin(t),   0,      sp.cos(t)]
    ]))

def RotZ_S(t):
  return(sp.Matrix([
    [ sp.cos(t),  -sp.sin(t),       0],
    [ sp.sin(t),   sp.cos(t),       0],
    [ 0,              0,            1]
    ]))

#  4x4 transforms which are pure rotations

def RotX4_S(t):
  return(sp.Matrix([
    [1,         0,           0,      0],
    [0, sp.cos(t),  -sp.sin(t),      0],
    [0, sp.sin(t),   sp.cos(t),      0],
    [0,0,0,1]
    ]))

def RotY4_S(t):
  return(sp.Matrix([
    [ sp.cos(t),   0,      sp.sin(t),    0],
    [0,            1,          0    ,    0],
    [-sp.sin(t),   0,      sp.cos(t),    0],
    [0,0,0,1]
    ]))

def RotZ4_S(t):
  return(sp.Matrix([
    [ sp.cos(t),  -sp.sin(t),       0,    0],
    [ sp.sin(t),   sp.cos(t),       0,    0],
    [ 0,              0,            1,    0],
    [0,0,0,1]
    ]))
 
#  Roll, Pitch, Yaw to rotation matrix (symbolic)

def RPY3_S(r,p,y):
   R = RotZ_S(y)*RotY_S(p)*RotX_S(r)
   return R

def RPY4_S(r,p,y):
   R = RPY3_S(r,p,y)
   T = sp.Matrix(sp.zeros(4))
   for i in range(0,3):
     for j in range(0,3):
       T[i,j] = R[i,j]
   T[3,3] = 1.0
   return T
 
#  translate along a symbolic vector
def Trans4_S(v):
  T = sp.eye(4)
  T[0,3] = v[0]
  T[1,3] = v[1]
  T[2,3] = v[2]
  return T


#   Get the inverse of a symbolic 4x4 Homogeneous transform
def H_inv_S(T):
  R = T[0:3,0:3].T
  P = -R * T[0:3,3]
  T1 = R.row_join(P)
  T1 = T1.col_join(sp.Matrix([0,0,0,1]).T)
  return T1
  
  
  

###################### Numerical functions

##  The cannonical rotation matrices

def RotX_N(t):
  return(np.matrix([
    [1,         0,           0],
    [0, np.cos(t),  -np.sin(t)],
    [0, np.sin(t),   np.cos(t)]
    ]))

def RotY_N(t):
  return(np.matrix([
    [ np.cos(t),   0,      np.sin(t)],
    [0,            1,          0    ],
    [-np.sin(t),   0,      np.cos(t)]
    ]))

def RotZ_N(t):
  return(np.matrix([
    [ np.cos(t),  -np.sin(t),       0],
    [ np.sin(t),   np.cos(t),       0],
    [ 0,              0,            1]
    ]))

#  4x4 transforms which are pure rotations

def RotX4_N(t):
  return(np.matrix([
    [1,         0,           0,      0],
    [0, np.cos(t),  -np.sin(t),      0],
    [0, np.sin(t),   np.cos(t),      0],
    [0,0,0,1.0]
    ]))

def RotY4_N(t):
  return(np.matrix([
    [ np.cos(t),   0,      np.sin(t),    0],
    [0,            1,          0    ,    0],
    [-np.sin(t),   0,      np.cos(t),    0],
    [0,0,0,1]
    ]))

def RotZ4_N(t):
  return(np.matrix([
    [ np.cos(t),  -np.sin(t),       0,    0],
    [ np.sin(t),   np.cos(t),       0,    0],
    [ 0,              0,            1,    0],
    [0,0,0,1]
    ]))
 

#  Roll, Pitch, Yaw to rotation matrix (numerical)

def RPY3_N(r,p,y):
   R = RotZ_N(y)*RotY_N(p)*RotX_N(r)
   return R

def RPY4_N(r,p,y):
   R = RPY3_N(r,p,y)
   #T = np.matrix(np.zeros(16).reshape(4,4))
   T = np.matrix(np.zeros(4))
   for i in range(0,3):
     for j in range(0,3):
       T[i,j] = R[i,j]
   T[3,3] = 1.0
   return T

#   Get the inverse of a symbolic 4x4 Homogeneous transform
def H_inv_N(T):
  R = T[0:3,0:3].T
  P = -R*T[0:3,3]
  T1 = R.row_join(P)
  T1 = T1.col_join(sp.Matrix([0,0,0,1]).T)
  return T1


###############################################################################
#
#    Symbolic Jacobian Matrix
#

def ManipJacobian_S(v, w, x):
  # compute two halves of J
  A = sp.zeros(3,6)
  B = sp.zeros(3,6)
  a = v.jacobian(x)
  b = w.jacobian(x)
  for i in range(0,3):
    for j in range(0,6):
      A[i,j] = sp.expand(a[i,j])
      B[i,j] = sp.expand(b[i,j])
  # put the halves together
  J = sp.Matrix([
    A[0,:],
    A[1,:],
    A[2,:],
    B[0,:],
    B[1,:],
    B[2,:]
    ])
  return J

