#!/usr/bin/python
#
#     Test for full IK solutions of know robot(s)

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford 
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sympy as sp
from sys import exit, argv
import pickle     # for storing pre-computed FK eqns
import unittest

# modified by BH, local version in current dir
import b3 as b3          # behavior trees

# local modules
import ikbtfunctions.helperfunctions as hf
from ikbtfunctions.ik_robots import * 
from ikbtbasics import *


TEST_DATA_GENERATION = False

sp.init_printing()

if not TEST_DATA_GENERATION:
    print ""
    print "          Running IK solution "
    print ""
    print ""
else:
    print '-'*50
    print ""
    print "          Generating IKBT TEST DATA only "
    print ""
    print "          (for production: line 32: TEST_DATA_GENERATION = False)"
    print ""
    print '-'*50

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')

# move definition of Sum-of-Angles variables to ik-classes.py so they are available everywhere

########################################################
#
#     Robot Parameters


########################################################    NEW Style robot param setups

# robot example

#
#    UW BRL Mini Direct Drive Robot, 5-DOF
#
robot = 'MiniDD'
dh = sp.Matrix([
    [    0     ,     0  , d_1   ,  0    ],
    [ -sp.pi/2 ,     0  ,   0   , th_2  ],
    [ -sp.pi/2 ,   l_3  ,   0   , th_3  ],
    [ -sp.pi/2 ,     0  , l_4   , th_4  ],
    [ -sp.pi/2 ,     0  ,   0   , th_5  ],
    [   0      ,     0  ,   0   ,   0   ]     
    ])
vv = [0,1,1,1,1,1]

variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5) ]

params = [l_3, l_4]
pvals = {l_3: 5, l_4:2}


m = kc.mechanism(dh, params, vv)
m.pvals = pvals  # store numerical values of parameters
m.forward_kinematics()
print "Completed Forward Kinematics"
print m.T_06

print '\n\n\n\n'
sp.pprint(notation_squeeze(m.T_06))

print ' w33'
sp.pprint(notation_squeeze(m.w_33))
print '-------------'

print ' w55'
sp.pprint(notation_squeeze(m.w_55))
print '-------------'

print ' v55'
sp.pprint(notation_squeeze(m.v_55)) 


print ManipJacobian_S(m.v_33, m.w_33, [th_1,th_2, th_3])