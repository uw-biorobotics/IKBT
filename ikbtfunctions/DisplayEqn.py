#!/usr/bin/python
#
#   TEMPLATE
#     BT Nodes for Testing, ID, Solving
#

#  Replace TEMPLATE below with your solution method
#

#

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import unittest
import re   # regular expressions for output conversions
import sympy as sp
import numpy as np
from pykinsym import *
#import helperfunctions as hf

## 
#   Pretty Print a matrix (but elements should be "small")
#

def matrix_pp(M, indeces=False):
    if(not indeces):
        sp.pprint(notation_squeeze(M))
    else:        
       r = 0
       for r in [0,1,2]:
            print '\n Row ',r+1
            for c in [0,1,2,3]:
                print ' [',r+1,',',c+1,']'
                #sp.pprint(notation_squeeze(sp.simplify(M[r,c])))
                sp.pprint(notation_squeeze(sp.simplify(M[r,c])))   # lets move simplfy to updateT
       
       print '\n Row 4'
       print '[  0,  0,  0,  1]'
    
    
class display_eqn(b3.Action):    # action leaf for  
    
    def __init__(self):
        super(b3.Action, self).__init__()
        self.Name = '* display eqn *'
        
    def tick(self, tick):
       Tm = tick.blackboard.get('Tm')   # the current matrix equation 
       unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
       print ' ------------- start                  Display Matrix Equation      ------------------'
       print '\n'
       matrix_pp(Tm.Td,True)
       
       print '\n     = '
       matrix_pp(Tm.Ts, True)
       print '\n\n     Variable list: '
       vars = tick.blackboard.get('unknowns')
       print 'Unknowns: '
       for v in vars:
           if(not v.solved):
               print v.symbol, 
       print '\nKnowns'
       for v in vars:
           if(v.solved):
               print v.symbol,
       print''
       print ' -------------   end                  Display Matrix Equation      ------------------'
       print '\n\n '
       
       raw_input (' <enter> to continue ')
       return b3.SUCCESS 
       

class display_eqn_lists(b3.Action):    # action leaf for  
    
    def __init__(self):
        super(b3.Action, self).__init__()
        self.Name = '* display eqn *'
        
    def tick(self, tick):
        unknowns = tick.blackboard.get('unknowns')   # the current list of unknowns
        R = tick.blackboard.get('Robot')
        
        one_unk = tick.blackboard.get('eqns_1u')
        two_unk = tick.blackboard.get('eqns_2u')
                
        print ' ------------- start                  Display Equation Lists     ------------------'
        print '\n One Unkowns:' 
        for e in one_unk:
            print e
        print '\n Two Unkowns:' 
        for e in two_unk:
            print e
        print '\n\n     Variable list: '
        vars = tick.blackboard.get('unknowns')
        print 'Unknowns: '
        for v in vars:
            if(not v.solved):
                print v.symbol, 
        print '\nKnowns'
        for v in vars:
            if(v.solved):
                print v.symbol,
        print''
        print ' -------------   end                  Display Matrix Equation      ------------------'
        print '\n\n '
        
        raw_input (' <enter> to continue ')
        return b3.SUCCESS 
        

    
       
##  put in test code here.  See sinANDcos.py for example
##
#if __name__ == '__main__':
    
