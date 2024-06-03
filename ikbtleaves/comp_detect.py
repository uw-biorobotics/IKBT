#!/usr/bin/python
#
#     BT Nodes for specific symbolic steps
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
from sys import exit

from ikbtfunctions.helperfunctions import *
from ikbtbasics.kin_cl import *
from ikbtbasics.ik_classes import *     # special classes for Inverse kinematics in sympy

import b3 as b3          # behavior trees
import time       

       
#   Detect when all unknowns are solved
#   
 
class comp_det(b3.Action):
    
    def __init__(self):
        super(b3.Action, self).__init__()
        self.FailAllDone = False   # we can set up to succeed when all are done or succeed when more to do. 
        self.Name = '*completion_detect*'
        
    def tick(self,tick):
        unks = tick.blackboard.get('unknowns')
        Tm = tick.blackboard.get('Tm')
        R  = tick.blackboard.get('Robot')
        
        SOA_DEBUG = False  # just turn this on for debugging
        if SOA_DEBUG:
            L1 = tick.blackboard.get('eqns_1u')
            L2 = tick.blackboard.get('eqns_2u')
            L3 = tick.blackboard.get('eqns_3pu')
            print('\n')
            print('L1: ', L1)   ##  for debugging sum of angles
            print('L2: ', L2)
            print('L3: ', L3)
            
        n = 0
        ns = 0
        print('\n\n')
        for u in unks:
            n += 1
            status = 'Unsolved'
            if(u.solved):
                ns += 1
                status = 'SOLVED'
            sname = str(u.symbol)
            print(f'{sname:10}: {status}')

        print('           Completion Detector: ', n, ' variables, ', ns, ' are solved.')
        print('             solved: ',)
        for u in unks:
            if(u.solved):
                outstr = '{} ({});  '.format(u.symbol, u.solvemethod)
                print(outstr,                )
        print('\n\n\n')
        time.sleep(10)  # for easier reading/ stopping
            
        #
        #   Look for sum-of-angle equations which can now be solved
        #      for example: th2 = th23-th3 (where th23 and th3 are known)
        #     
        #   this should be algebric solver's work, 
        #   also the set solve here causes problem
        
                
        if(self.FailAllDone):               # we can set up to succeed when all are done or succeed when more to do. 
            DONEComplete   = b3.FAILURE
            DONEIncomplete = b3.SUCCESS
        else:
            DONEComplete   = b3.SUCCESS
            DONEIncomplete = b3.FAILURE
        if(n == ns):
            print("")
            print(" Solution Complete!!")
            print("")
            return DONEComplete  # we have solved all vars
        else:
            return DONEIncomplete # we still have unsolved vars
        
        
##   Self Tests
if __name__ == "__main__":

    bb = b3.Blackboard()
    up_tester = b3.BehaviorTree() 
      
