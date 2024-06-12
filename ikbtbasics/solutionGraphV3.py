# Solution graph Version 3

# Blake Hannaford   Summer 2024

# Copyright 2024 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import unittest
import sympy as sp
from ikbtbasics.ik_classes import *
import ikbtbasics.kin_cl as kc
from ikbtbasics.matching import *
import itertools as itt



class Node:
    '''  a node in the solution graph - primarily contains an unknown '''
    def __init__(self, unk):
        self.unknown = unk
        self.symbol = unk.symbol
        self.name = str(unk.symbol)
        self.argument = unk.argument
        self.solvemethod = unk.solvemethod
        # nversions is in the unknown class self.unk.nversions
        #  aren't these redundant since we have them in the unk??
        self.eqnlist = []
        self.nsolutions = 0
        self.solutions = []

        self.arguments = {}  # argument needing domain testing
        self.solveorder = -1

    def details(self):
        return('Node details: \n'+self.unknown.details())

    def __lt__(self, other):   # for sorting Nodes
        return self.solveorder < other.solveorder

    def __eq__(self, other): #equal judgement, also hashing in python 3.x
        if other != None:
            return self.symbol == other.symbol
        return False

    def __hash__(self): #hash function "inherits" from symbol
        return self.symbol.__hash__()

    def __repr__(self): # string representation
        return self.symbol.__repr__()



class Edge: # from unk --->  dependency
    def __init__(self, N1, N2, multiplicity=1):   # N1 depends on N2
        '''child and parent are notation with subscript type'''
        self.dependsOn = N2  # depends on  (arrow pts .startNode --> .dependsOn)
        self.StartNode = N1   #  the solution node
        self.mult = multiplicity

    def __repr__(self):
        return   'Edge:' +  str(self.StartNode) + " depends on: " + str(self.dependsOn) + f'  ({self.StartNode}-->{self.dependsOn})'

    def __eq__(self, other):
        return self.StartNode == other.StartNode and self.dependsOn == other.dependsOn and self.mult == other.mult

    def __hash__(self):
        return self.StartNode.__hash__() * self.dependsOn.__hash__() + self.mult.__hash__()












class SolutionGraphV2Tests(unittest.TestCase):
    # the tests were designed for v1 (independent module), not suitable for v2
    def test_mock(self):
        print("place holder for real test: solution_graph_v2")

if __name__ == '__main__':
    #notation_graph = set()
    #unittest.main()

    print('\n\n===============  Test solutionGraphV3 =====================')
    #testsuite = unittest.TestLoader().loadTestsFromTestCase(SolutionGraphV2Tests)  # replace TEMPLATE
    #unittest.TextTestRunner(verbosity=2).run(testsuite)
    unittest.main()


