#!/usr/bin/python
#
#     Inverse Kinematics Classes
#

# Copyright 2017-2024 University of Washington

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
import yaml
import ast     # string to list

####
#
#   NOTE: due to an obscure sympy bug, you cannot use numerical values in any DH position.   Use a symbolic constant
#         instead (like a_3 below), declare it in params, and give it your value in pvals
#
#####



class RobotParms:
    def __init__(self, name, dh, vv, params, pvals, variables ):
        self.name = name              # string
        self.dh = dh                  # sp.Matrix of variables and constants
        self.vv = vv                  # list of 1s and 0s
        self.params = params          # list of sympy vars
        self.pvals = pvals            # list of floats (corresponding to params)
        self.variables = variables    # list of sympy vars

    def __init__(self):
        pass

    def to_dict(self):
        d = {}
        #d = { 'a': 5, 'b' : 'B', 'c': [ 1, 5, 7, 13 ]}
        d['name'] = self.name
        svars = []
        for v in self.variables:
            svars.append(str(v))
        d['variables'] = svars
        d['vv'] = self.vv
        for p in self.params:
            spar.append(str(p))
        d['params'] = spar
        spvs = []
        for v in self.pvals:
            spvs.append(str(v))
        d['pvals'] = spvs
        d['dh'] = str(self.dh)
        return d

    def fm_dict(self,d):
        dhp = d['dh'].replace('Matrix(','').replace(')','')
        #dhM = eval(d['dh']).replace('Mat','sp.Mat')
        print('DEBUG fm-dict:',dhp)
        self.name = d['name']
        #self.dh    = ast.literal_eval(d['dh'])
        self.dh = eval(dhp)
        self.vv    = ast.literal_eval(d['vv'])
        self.params = symbols(d['params'])
        self.pvals = ast.literal_eval(d['pvals'])
        self.variables = symbols(d['variables'])

    def expand(self):
        # return the params as IKBT expects them:
        #    return [dh, vv, params, pvals, variables]
        return [self.dh, self.vv, self.params, self.pvals, self.variables]

def load_robot(name):
    filename = 'IKBTrobotData.yaml'
    with open(filename, 'r') as fp:
        robot_docs = yaml.safe_load_all(fp)

        for doc in robot_docs:
            if doc['name'] == name:
                return doc

RPardict = load_robot('Raven-II')
RPsympy = RobotParms()
RPsympy.fm_dict(RPardict)






