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
import sys
#
import yaml

####
#
#    This is a test of the code to serialize a robot parameter set,
#  dump it to a yaml file, and then read it back in.    Asserts are given
#  so normal completion == pass.
#####



class RobotParms:
    def __init__(self):
        pass

    def setup(self, name, dh, vv, params, pvals, variables ):
        #print(name)
        #print(dh)
        #print(pvals)
        #print(variables)
        #print(' ')
        self.name = name
        self.vv = vv
        self.params = params
        self.pvals = pvals
        self.variables = variables
        self.dh = dh

    def from_dict(self,d):
        self.name = d['name']
        print('variable check:')
        self.variables = []
        for  v in d['variables']:
            print(str(v), v, eval(v))
            self.variables.append(eval(v))
        print('')

        self.params = []
        for p in d['params']:
            self.params.append(eval(p))
        self.dh = eval(d2['dh'])
        self.vv = d['vv']
        self.pvals = eval(d['pvals'])

    def to_dict(self):
        d = {}
        #d = { 'a': 5, 'b' : 'B', 'c': [ 1, 5, 7, 13 ]}
        d['name'] = self.name
        svars = []
        for v in self.variables:
            svars.append(str(v))
        d['variables'] = svars
        d['vv'] = self.vv
        spar = []
        for p in self.params:
            spar.append(str(p))
        d['params'] = spar
        spvs = {}  # pvals is a dict.
        for pvk in self.pvals.keys():
            spvs[pvk] = self.pvals[pvk]
        d['pvals'] = str(spvs)
        dhs = str(self.dh)
        dhs = dhs.replace('Matrix(','').replace('])',']')
        d['dh'] = dhs
        return d


        #d2 = getRobotInfo(name).to_dict()
        #ydata = yaml.dump(d2,indent=2, sort_keys=False)
        #fp.write(ydata)
        #fp.close()




def testSympyYaml():
    pvals = {}   # null for many robots

    dh = []
    variables = []
    params = []
    pvals = []

    name = 'Raven-II-to-yaml'

    if True:

        sp.var('ld_1 ld_2 ld_3 ld_4')

        dh = sp.Matrix([
        [  0,          0,     0,      th_1  ],
        [ sp.pi*75/180,   0 ,     0,      th_2  ],
        [ (180-52)*sp.pi/180,      0,    d_3,   sp.pi/2 ],         # testing
        [ 0,           0,     ld_2,     th_4  ],
        [ sp.pi/2 ,    0,        0,     th_5  ],
        [ sp.pi/2 ,    ld_3,    0,      th_6  ]
        ])
        vv = [1,1,0,1,1,1]

        variables =  [unknown(th_1), unknown(th_2), unknown(d_3), unknown(th_4), unknown(th_5), unknown(th_6)]

        params = [  ld_2, ld_3 ]
        pvals = {ld_2: -470.0, ld_3: 13.0 }

    RP = RobotParms()
    RP.setup(name, dh, vv, params, pvals, variables )
    return RP


if __name__=='__main__':

    yfname = 'SympyTest.yml'

    if len(sys.argv)==1:
        WRITE=True
        READ =True
    else:  # use an arg to force test of reading only
        WRITE=False
        READ=True

    print('test of Yaml serialization with sympy')

    if WRITE:
        print('TESTING: writing out parameters.')
        #set up the params
        RP = testSympyYaml()
        d = RP.to_dict()

        fp = open(yfname,'w')

        ydata = yaml.dump(d,indent=2, sort_keys=False)
        fp.write(ydata)
        fp.close()

        print('Yaml file is written: {yfname}')

    if READ:
##########################################    Test load yaml

        print('TESTING: reading in parameters.')
        fp = open(yfname, 'r')
        d2 = yaml.safe_load(fp)
        print('Result: dict: ')
        for k in d2.keys():
            print('    ',f'{k:12} {d2[k]}')


        # convert dh parameters from string to sympy variables/expressions
        pi = sp.pi  # sympy takes of 'sp.' from string rep.
        print('\n eval() result:')
        print(type(d2['dh']))
        mat = eval(f"vars({d2['dh']})")
        x = sp.var('x')
        print('type: dh element: ', type(mat[2][0]) )
        assert 'sympy' in str(type(mat[2][0])), 'failed to convert dh elements from str to sympy  '

        print('sympy expression from matrix')
        syexp = mat[2][0]*(mat[0][3] + mat[1][3])/mat[2][0]
        assert syexp == mat[0][3] + mat[1][3], 'sympy conversion problem'

        print(syexp)

        print('Check from_dict(to_dict(R)) == identity')

        RP2 = RobotParms()
        RP2.from_dict(RP.to_dict())
        assert RP2.name==RP.name, 'to-from dictionary error: name'
        assert RP2.vv==RP.vv, 'to-from dictionary error: vv'
        assert RP2.pvals==RP.pvals, 'to-from dictionary error: pvals'
        assert RP2.params==RP.params, 'to-from dictionary error: params'
        print('RP2.variables: ',RP2.variables, type(RP2.variables))
        print('RP.variables: ', RP.variables,  type(RP.variables))
        for i in range(6):
            RP2.variables[i] =  unknown(RP2.variables[i])  #ikbtbasics.kin_cl.unknown(()
            print(RP.variables[i], type(RP.variables[i]), type(RP2.variables[i]))
        assert RP2.variables==RP.variables, 'to-from dictionary error: variables'
        d3 = RP.to_dict()
        d4 = RP2.to_dict()
        assert d3==d4, 'from-to dictionary error'

    print('\n\n              ALL TESTS PASS\n\n')
