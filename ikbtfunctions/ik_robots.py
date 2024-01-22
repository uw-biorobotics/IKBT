#!/usr/bin/python
#
#     Inverse Kinematics Classes
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
#import numpy as np
from ikbtbasics.kin_cl import *
from ikbtfunctions.helperfunctions import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy
#

####
#
#   NOTE: due to an obscure sympy bug, you cannot use numerical values in any DH position.   Use a symbolic constant
#         instead (like a_3 below), declare it in params, and give it your value in pvals
#
#####
def robot_params(name):
    pvals = {}   # null for most robots
    List = ['ICP5p5_A21','KR16', 'Issue4',
            'UR5', 
            'Puma', 'Pumaoffset',
            'Chair_Helper', 
            'Bartell',
            'Brad', 
            'Sims11',
            'ArmRobo', 
            'Wrist', 
            'Arm_3', 
            'MiniDD', 
            'Olson13',
            'Stanford', 
            'Chair6DOF',  # below not solvable yet
            'Khat6DOF',    
            'Craig417', 
            'KawasakiRS05L',
            'KawasakiRS007L',
            'Khat6DOF',
            'Raven-II',
            'Wachtveitl', 'Frei13', 'Parkman13', 'Palm13', 'Minder13', 'Mackler13',
            'Axtman13', 'Srisuan11', 'MiniDD', 'ICP5p5_A21', 'KR16', 'Issue4'
            'DZhang']
    
    if not (name in List):
        print('robot_params(): Unknown robot, ' + name )
        print('Here are the defined robots: ')
        for n in List:
            print('   ', n)
        quit()




    if(name == 'Raven-II'):

        sp.var('ld_1 ld_2 ld_3 ld_4')

        dh = sp.Matrix([
        [  0,          0,     0,      th_1  ],
        [ al_1,   0 ,     0,      th_2  ],
        [ al_2,      0,    d_3,   sp.pi/2 ],         # testing
        [ 0,           0,     ld_2,     th_4  ],
        [ sp.pi/2 ,    0,        0,     th_5  ],
        [ sp.pi/2 ,    ld_3,    0,      th_6  ]
        ])
        vv = [1,1,0,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(d_3), unknown(th_4), unknown(th_5), unknown(th_6)]

        params = [  ld_2, ld_3 , al_1, al_2]
        #pvals = {ld_2: -470.0, ld_3: 13.0, al_1 : sp.pi*75/180, al_2: (180-52)*sp.pi/180}
        # use np.pi because these are fixed numerical params.
        pvals = {ld_2: -470.0, ld_3: 13.0, al_1 : np.pi*75/180, al_2: (180-52)*np.pi/180}

############################################################33    Issues 3 and 4 tests
    if(name == 'Issue4'):

        sp.var('ld_1 ld_2 ld_3 ld_4')

        dh = sp.Matrix([
        [  0,          0,     ld_1,     th_1  ],
        [-sp.pi/2,    a_1,     0,      th_2  ],
    #   [  sp.pi,      a_2,     0,      th_3  ],
        [  0,      a_2,     0,      th_3  ],         # testing
        [ sp.pi/2 ,   a_3,    ld_2,     th_4  ],
        [ sp.pi/2 ,    0,     ld_3,     th_5  ],
        [-sp.pi/2 ,    0,     ld_4,     th_6  ]
        ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]

        params = [ld_1, a_1, a_2, a_3, ld_2, ld_3, ld_4]
        pvals = {ld_1:0.112891 , a_1:0.12893 , a_2:0.8382, a_3:0.064 , ld_2:0.4615 , ld_3:0.029, ld_4:0.418}


    if(name == 'KR16'):         # two link planar test (Issue #12) 
        #
        dh = sp.Matrix([
        [       0  ,   0 , l_1 , th_1 ],
        [  -sp.pi/2, a_1 ,   0 , th_2 ],
        [       0  , a_2 ,   0 , th_3 ],
        [  sp.pi/2 , a_3 , l_3 , th_4 ],
        [ -sp.pi/2 ,   0 ,   0 , th_5],
        [  sp.pi/2 ,   0 ,   0 , th_6 ]  # can't solve with d_6 ~= 0!
        ])

        vv = [1,1,1,1,1,1]
        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [a_1, a_2, a_3, l_1, l_2, l_3, l_4]
        pvals = {a_1:1, a_2:1, a_3:1, l_1:1,l_2:1,l_3:1,l_4:1}
        
############################################################
         

        
    if(name == 'ICP5p5_A21'):
        
         dh = sp.Matrix([
             [0,         0,      0,  th_1 ],
             [0,       l_1,    d_2,  0    ],
             [sp.pi/2,   0,    l_2,  th_3 ],
             [-sp.pi/2, l_3,     0,  th_4 ],
             [0,         0,      0,  0 ],
             [0,         0,      0,  0] 
             ] )
         vv = [1, 0, 1, 1, 1, 1]
         variables = [unknown(th_1), unknown(d_2), unknown(th_3), unknown(th_4)]
         params = [ l_1, l_2, l_3]
         pvals = {l_1: 295 , l_2: 105 , l_3: 330}
        
        
    if(name == 'KawasakiRS05L'):
        
         dh = sp.Matrix([
             [0,        0,      0,  th_1 ],
             [sp.pi/2,  l_2,    0,  th_2 ],
             [0,        l_3,    0,  th_3 ],
             [-sp.pi/2, l_4,    0,  th_4 ],
             [-sp.pi/2, 0,    l_5,  th_5 ],
             [sp.pi/2,  0,      0,  th_6 ] ])
         vv = [1, 1, 1, 1, 1, 1]
         variables = [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
         params = [ l_1, l_2, l_3, l_4, l_5]
         pvals = {l_1: 295 , l_2: 105 , l_3: 330, l_4: 80, l_5: 410}
         
    if(name == 'KawasakiRS007L'):
        
         dh = sp.Matrix([
             [0,        0,     l_1,  th_1 ],
             [sp.pi/2,  0,      0,  th_2 ],
             [0,        l_2,    0,  th_3 ],
             [ sp.pi/2, 0,    l_3,  th_4 ],
             [-sp.pi/2, 0,      0,  th_5 ],
             [ sp.pi/2, 0,      0,  th_6 ] ])
         vv = [1, 1, 1, 1, 1, 1]
         variables = [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
         params = [ l_1,l_2, l_3]
         pvals = {l_1: 360 ,l_2:355,  l_3: 375}
         
    if(name == 'ArmRobo'):    # submitted by issue #15

        # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
        
        # Olson 2013
        # DOF: 6
        # methods to test: m5, m3, 
        # Yb = d_1, Xb = d_2, L1 = l3, L2 = l4, L3 = l5
        dh = sp.Matrix([
            [sp.pi/2,    0.,       l_2,              th_1],
            [0,         l_1,        0.,              th_2],
            [sp.pi/2,    0.,        0.,              th_3],
            [-sp.pi/2,   0.,       l_3,              th_4],
            [sp.pi/2,    0.,        0.,              th_5],
            [0,          0.,       d_6,                0.]
            ])
            
        vv = [1, 1, 1, 1, 1, 0]
        variables = [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(d_6)]
        params = [l_1, l_2, l_3]
        pvals = {l_1: 0.19681, l_2: 0.251, l_3:0.145423}
#Big thanks for your help, really appreciate it.      

    if(name == 'Arm_3'):         # two link planar test (Issue #12) 
        #
        dh = sp.Matrix([
        [ 0 , a_1 , 0 , th_1 ],
        [ 0 , a_2 , 0 , th_2 ],
        [ 0 , a_3 , 0 , th_3 ],
        [ 0 , 0 , 0 , 0 ],
        [ 0 , 0 , 0 , 0 ],
        [ 0 , 0 , 0 , 0 ]
        ])

        vv = [1,1,1,0,0,0]
        variables =  [unknown(th_1), unknown(th_2), unknown(th_3)]
        params = [a_1, a_2, a_3]
        pvals = {a_1:1, a_2:1, a_3:1}
        
    if(name == 'Brad'):    # 3-RRR articulated manipulator
        
        # Brad 2019
        # DOF: 3
        dh = sp.Matrix([
            [0, 0., d_1, th_1],
            [sp.pi/2., l_2, 0., th_2],
            [0., l_3, 0., th_3],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.],
            [0., 0., 0., 0.]
            ])
            
        vv = [1, 1, 1, 0, 0, 0]
        variables = [unknown(th_1), unknown(th_2), unknown(th_3)]
        params = [d_1, l_2, l_3]
        pvals = {d_1: 0.06, l_2: 0.15, l_3: 0.15}

    if(name == 'UR5'):  #Universal Robots - test case for 3 parallel axes
        dh = sp.Matrix([
            [    0    ,    0 ,  d_1 ,     th_1  ],
            [ sp.pi/2 ,    0 ,   0  ,     th_2  ],
            [    0    ,   a_2,   0  ,     th_3  ],   
            [    0    ,   a_3,  d_4 ,     th_4  ],      
            [ sp.pi/2 ,   0 ,   d_5 ,     th_5  ],      
            [ -sp.pi/2 ,   0 ,   d_6 ,     th_6  ]
            ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [a_2,a_3,d_1,d_4,d_5,d_6]
        pvals = {a_2: .425, a_3: .392, d_1:.089, d_4:.109, d_5:.095, d_6:.082}  # meters
   
     
    if(name == 'Craig417'):
        dh = sp.Matrix([
            [    0   ,    0 ,   0 ,     th_1  ],
            [-sp.pi/2,    0 ,   0   ,     th_2  ],
            [ sp.pi/4 ,   0,    d_2 ,     th_3  ],   
            [    0    ,   a_3,  d_3 ,     th_4  ],      
            [    0    ,   0 ,   0,         0    ],      
            [    0    ,   0 ,   0,         0    ]
            ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4)]
        params = [d_2, d_3, a_3]
        pvals = {d_2:1, d_3:1,  a_3:1}  # meters
   

#   The famous Puma 560  (solved in Craig)
#        
    if(name == 'Puma'):
        dh = sp.Matrix([
            [  0      ,    0 ,  d_1 ,     th_1  ],   #  Note: Puma is used for tests so mods to this table
            [-sp.pi/2 ,    0 ,   0 ,      th_2  ],   #  may break ikbtleaves.updateL.TestSolver007
            [      0  ,   a_2, d_3 ,      th_3  ],   
            [-sp.pi/2 ,   a_3, d_4,       th_4  ],      
            [-sp.pi/2 ,   0,  0 ,       th_5  ],
            [ sp.pi/2 ,   0,    0,       th_6  ]   # d_6= 0 simplifies the math
            ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [d_1, a_2, a_3, d_3, d_4]
        pvals = {d_1:0.6,a_2:0.432, a_3:0.0203, d_3:0.1245, d_4:0.432}  # meters
        

#   The famous Puma 560  (solved in Craig)
#        
    if(name == 'Pumaoffset'):
        dh = sp.Matrix([
            [  0      ,    0 ,  d_1 ,     th_1  ],   #  Note: Puma is used for tests so mods to this table
            [-sp.pi/2 ,    0 ,   0 ,      th_2  ],   #  may break ikbtleaves.updateL.TestSolver007
            [      0  ,   a_2, d_3 ,      th_3  ],   
            [-sp.pi/2 ,   a_3, d_4,       th_4  ],      
            [-sp.pi/2 ,   0,  0 ,       th_5  ],
            [ sp.pi/2 ,   0,  d_6,       th_6  ]
            ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [d_1, a_2, a_3, d_3, d_4,d_6]
        pvals = {d_1:0.6,a_2:0.432, a_3:0.0203, d_3:0.1245, d_4:0.432,d_6:0.25}  # meters
        

    if(name == 'Chair_Helper'):                
            vv = [0,1,1,1,1,1]   # must be length 5 since 5dof and 5 unks

            dh = sp.Matrix([
            [  0,    0,  d_1 ,   0  ],
            [ 0  ,     l_1,  0 ,   th_2  ],
            [ sp.pi/2,    0, l_2 ,   th_3  ],   
            [ sp.pi/2 ,     0,   0,      th_4  ],     # must fill remaining rows with zeros
            [-sp.pi/2 ,   0,   l_4,      th_5  ],
            [      0 ,     0,   0,      0  ]
            ])

            variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5)]
            
            params = [l_1, l_2, l_4]
            pvals = {l_1: 2, l_2: 1, l_4: 3} # can change to values
        
    if(name == 'Wrist'):
        sp.var('A B C')
        
        ###   These somewhat wierd DH params give you the ZYX Euler Angles
        #       matrix of example 4.7  (don't ask how I got this!)
        dh = sp.Matrix([
        [        0,    0,   0,     A  ], 
        [ -sp.pi/2,    0,   0,    (sp.pi/2 + B)  ],   
        [  sp.pi/2 ,   0,   0,    (sp.pi/2 + C)  ],     # must fill remaining rows with zeros
        [ -sp.pi/2,     0,   0,   -sp.pi/2  ],
        [      0 ,     0,   0,   0  ],
        [      0 ,     0,   0,   0  ]
        ])

        vv = [1,1,1,1,1,1]
        variables = [unknown(A), unknown(B), unknown(C)]
        params = []
        pvals = {}


    if(name == 'MiniDD'):
        #
        #    UW BRL Mini Direct Drive Robot, 5-DOF
        #
        dh = sp.Matrix([
            [    0     ,     0  , d_1   ,  0    ],
            [ -sp.pi/2 ,     0  ,   0   , th_2  ],
            [ -sp.pi/2 ,   l_3  ,   0   , th_3  ],
            [ -sp.pi/2 ,     0  , l_4   , th_4  ],
            [ -sp.pi/2 ,     0  ,   0   , th_5  ],
            [   0      ,     0  ,   0   ,   0   ]     
            ])
        vv = [0,1,1,1,1]

        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5) ]

        params = [l_3, l_4]
        pvals = {l_3: 5, l_4:2}

        
    if(name == 'Olson13'):
        # standardize on the order "alpha N-1, a N-1, d N, theta N' for the DH table columns.
        
        # Olson 2013
        # DOF: 6
        # methods to test: m5, m3, 
        # Yb = d_1, Xb = d_2, L1 = l3, L2 = l4, L3 = l5
        dh = sp.Matrix([
            [-sp.pi/2,  0.,         d_1,        sp.pi/2],
            [sp.pi/2,   0.,         d_2,        -sp.pi/2],
            [sp.pi/2,   0.,         l_3,        th_3],
            [sp.pi/2,   0.,         0.,         th_4],
            [0.,        l_4,        0.,         th_5],
            [sp.pi/2,   0.,         l_5,        th_6]
            ])
            
        vv = [0, 0, 1, 1, 1, 1]
        variables = [unknown(d_1), unknown(d_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [l_3, l_4, l_5]
        pvals = {l_3: 1, l_4: 4, l_5:2}
                


    if(name == 'Stanford'):  
        sp.var('l_4 l_6')
        dh = sp.Matrix([   # Alpha's Corrected  10-Nov-21(!)
            [0 ,          0.,          l_1,    th_1],
            [sp.pi/2,     0.,          l_2,    th_2],
            [sp.pi/2,     0.,          d_3,       0 ],
            [0 ,          0.,           0,    th_4],
            [-sp.pi/2,    0.,           0.,    th_5],
            [sp.pi/2.,    0.,           0,    th_6]
            ])
                    
        vv = [1, 1, 0, 1, 1, 1]
        variables = [unknown(th_1), unknown(th_2), unknown(d_3), unknown(th_4), unknown(th_5), unknown(th_6)]
        params = [l_1, l_2, ]
        pvals = {l_1:1, l_2: 2}
        
    if(name=='Sims11'):
        # Sims 2011,
        # DOF: 5
        # methods to test: m5, m3, m4, m6
        print("looking at Sims11")
        sp.var('l_1 l_2 l_3')
        dh = sp.Matrix([
            [0.,        0.,         d_1,        0.],
            [sp.pi/2,   0.,         d_2,        0.],
            [sp.pi/2,   l_1,        0.,         th_3],
            [sp.pi/2,   0.,         l_2,        th_4],
            [sp.pi/2,   l_3,        0.,         th_5],
            [0.,        0.,         0.,         0.]
            ])
            
        vv = [0, 0, 1, 1, 1, 1, 1]
        variables = [unknown(d_1), unknown(d_2), unknown(th_3), unknown(th_4), unknown(th_5)]
        params = [l_1, l_2, l_3]
        pvals = {l_1: 5, l_2:2, l_3:4}

    if(name == 'Srisuan11'):
            # Srisuan 2011,
        # DOF: 6
        dh = sp.Matrix([
            [0.,        0.,         d_1,        0.],
            [0.,        0.,         0.,         th_2],
            [sp.pi/2,   0.,         l_1,        th_3],
            [sp.pi/2,   0.,         d_4,        sp.pi/2],
            [0.,        0.,         0.,         th_5],
            [sp.pi/2,   0.,         0.,         th_6]
            ])
            
        vv = [0, 1, 1, 0, 1, 1]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(d_4), unknown(th_5), unknown(th_6)]
        params = [l_1]
        pvals = {l_1:2}

    if(name == 'Axtman13'):
        # Axtman 2013sp,
        # DOF: 4
        dh = sp.Matrix([
            [0.,        0.,         d_1,        0.],
            [sp.pi/2,   0.,         l_2,        th_2],
            [sp.pi/2,   0.,         0.,         th_3],
            [0.,        l_3,        0.,         th_4],
            [0.,        l_4,        0.,         0.],
            [0.,        0.,         0.,         0.],
        ])  
        sp.var('l_3 l_4')
        vv = [0, 1, 1, 1, 0, 0]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4)]
        params = [l_2, l_3, l_4]
        pvals = {l_2: 1, l_3:2, l_4:3}

    if(name == 'Mackler13'):
        # Mackler 2013sp
        # DOF: 5
        dh = sp.Matrix([
            [-sp.pi/2,  h,          d_1,        0.],
            [sp.pi/2,   0.,         l_1,        th_2],
            [sp.pi/2,   l_2,        0.,         th_3],
            [sp.pi/2,   0.,         l_3,        th_4],
            [-sp.pi/2,  0.,         0.,         th_5],
            [0, 0, 0, 0]
            ])
        sp.var('l_3')
        vv = [0, 1, 1, 1, 1, 0]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5)]
        params = [l_1, l_2, l_3]
        pvals = {l_1:2, l_2:2, l_3: 4}

    if(name == 'Minder13'):
        # Minder 2013sp
        # DOF: 4
    
        dh = sp.Matrix([
            [0.,        0.,         d_1,        0.],
            [sp.pi/2,   0.,         l_2,         th_2],
            [sp.pi/2,   0.,         0.,          th_3],
            [sp.pi/2,   0.,         -l_3,       th_4],
            [0.,        0.,         0.,         0.],
            [0.,        0.,         0.,         0.]
        ])
        sp.var('l_3')
        vv = [0, 1, 1, 1, 0, 0]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4)]
        params = [l_2,l_3]
        pvals = {l_2: 1, l_3:2}

    if(name == 'Palm13'):
        # Palm 2013sp
        # DOF: 4
        dh = sp.Matrix([
            [sp.pi/2,   0.,         l_1,        th_1],
            [sp.pi/2,   0.,         d_2,        -sp.pi/2],
            [0.,        0.,         0.,         th_3],
            [-sp.pi/2,  l_3,        h,         th_4],
            [0.,        0,        0,        0.],
            [0.,        0.,         0.,         0.]
            ])
        sp.var('l_1 l_3 h')
        vv = [1, 0, 1, 1, 0, 0]
        variables = [unknown(th_1), unknown(d_2), unknown(th_3), unknown(th_4)]
        params = [l_1, l_3, h]
        pvals = {l_1:3, l_3: 1, h:2}

    if(name == 'Parkman13'):
        # Parkman 2013sp
        # DOF: 5
        dh = sp.Matrix([
            [0.,        0.,         h,          th_1],
            [sp.pi/2,   0.,         0.,         th_2   ],
            [0.,        l_2,        0.,         th_3],
            [sp.pi,     l_3,        d_4,        0.],
            [sp.pi/2,   0.,         l_5,        th_5],
            [0.,        0.,         0.,         0.]
            ])
        sp.var('h l_2 l_3 l_5')
            
        vv = [1, 1, 1, 0, 1, 0]
        variables = [unknown(th_1), unknown(th_2), unknown(th_3), unknown(d_4), unknown(th_5)]
        params = [h, l_2, l_3, l_5]
        pvals = {h: 1, l_2: 2, l_3:3, l_5:5}

    if(name == 'Frei13'):
        # Frei 13sp
        # DOF 5
        dh = sp.Matrix([
            [0.,        0.,         d_1,        0.],
            [sp.pi/2,   0.,         0.,         th_2],
            [sp.pi/2,   0.,         l_3,        th_3],
            [sp.pi/2,   0.,         0.,         th_4],
            [sp.pi/2,   0.,         l_5,        th_5],
            [0.,        0.,         0.,         0.]
            ])
        sp.var('l_3 l_5')
        vv = [0, 1, 1, 1, 1, 0]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5)]
        params = [l_3, l_5]
        pvals = {l_3: 6, l_5:3}

    if(name == 'Wachtveitl'):
        # Wachtveitl 2013sp
        # DOF: 5

        dh = sp.Matrix([
            [-sp.pi/2,  0.,         d_1,        0.],
            [sp.pi/2,   0.,         h,          th_2],
            [sp.pi/2,   0.,         0.,         th_3],
            [0.,        l_2,        l_3,        th_4],
            [sp.pi/2,   0.,         l_4,        th_5],
            [0.,        0.,         0,        0.]
            ]) 
        sp.var('h l_3 l_4')
        vv = [0, 1, 1, 1, 1, 0]
        variables = [unknown(d_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5)]
        params = [h, l_2, l_3, l_4]
        pvals = {h:1, l_2:2, l_3:3, l_4:2}


    if(name == 'Bartell'):
        # Bartell 2013
        # DOF: 5
        dh = sp.Matrix([
            [0.,        0.,         l_1,        th_1],
            [sp.pi/2,   0.,         d_2,        0.],
            [0.,        0.,         0.,         th_3],
            [sp.pi/2,   0.,         d_4,        0.],
            [0.,        0.,         0.,         th_5],
            [sp.pi/2,   0.,         0.,         0.]
            ])

        vv = [1, 0, 1, 0, 1, 1]
        variables = [unknown(th_1), unknown(d_2), unknown(th_3), unknown(d_4), unknown(th_5)]

        params = [l_1]
        pvals = {l_1:2}

    if(name == 'DZhang'):
        # Dianmu Zhang
        # DOF 5
        dh = sp.Matrix([
            [0.,        0.,         h,         th_1],
            [sp.pi/2,   l_1,         0,        th_2],
            [0,        l_3,    0,        th_3],
            [sp.pi/2,        0.,    l_4,         th_4],
            [sp.pi/2,     0.,       0,         th_5],
            [0,     0.,        0.,        0]
            ])
            
        sp.var('h l_3 l_4')
        vv = [1, 1, 1, 1, 1, 1]
        variables = [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5)]
        params = [h, l_1, l_3, l_4]
        pvals = {h:1, l_1:1, l_3:3, l_4:4}

    if(name == 'Khat6DOF'):                
        #
        #   This is Kuka Model KR60
        #    ( as analyzed in Khatamian6DOF_IK2015.pdf)
        # unsolved, 6DOF

        dh = sp.Matrix([                  ##  This one requires sum-of-angles.
        [  0,       a_1 , l_1 ,     th_1  ],
        [ sp.pi/2,    0,  0  ,      th_2  ],
        [      0 ,  a_2,  0  ,      th_3  ],   
        [ sp.pi/2 , a_3, l_4,       th_4  ],      
        [-sp.pi/2 ,   0,  0 ,       th_5  ],
        [ sp.pi/2 ,   0, 0  ,       th_6  ]
        ])
        vv = [1,1,1,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(th_3), unknown(th_4), unknown(th_5), unknown(th_6)]

        params = [a_1, l_1, a_2, a_3, l_4]
        pvals = {a_1: 1, l_1:2, a_2:4, a_3:2, l_4:5}
        
        
        
        
    ################## (all robots) ######################
    ##  make sure each unknown knows its position (index)
    i = 1
    for v in variables:   
        v.n = i
        i+=1
        
    return [dh, vv, params, pvals, variables]
   
