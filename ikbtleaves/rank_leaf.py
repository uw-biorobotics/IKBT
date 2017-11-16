# major refactor 2: control logic change, move the looping logic outside of the solver nodes
# 21-Jun-2017 DZ
# ranking node compares the solutions from sin/cos and tan solvers
# and chooses the better one as final solution

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

# helper function: count veriables (regardless of solved status)
def count_variables(unknowns, expr):
    n = 0
    for unk in unknowns:
        if expr.has(unk.symbol):
            n += 1
    return n

class rank(b3.Action):
    def tick(self, tick):
        u = tick.blackboard.get("curr_unk")
        unknowns = tick.blackboard.get("unknowns")
        R = tick.blackboard.get("Robot")
        choosen = None
        # only invokes comparisions when solved by both solvers
        if u.solvable_sincos and u.solvable_tan:
            # less number of solutions is better
            if (len(u.sincos_solutions) < len(u.tan_solutions)) and (len(u.sincos_solutions) > 0):
                choosen = "sincos"
            elif (len(u.sincos_solutions) > len(u.tan_solutions)) and (len(u.tan_solutions) > 0):
                choosen = "tan"
            else:
                # if the number of solutions are the same, countinue
                # less dependency
                sol_sin = u.sincos_solutions[0]
                sol_tan = u.tan_solutions[0]
                if count_variables(unknowns, sol_sin) < count_variables(unknowns, sol_tan):
                    choosen = "sincos"
                else:
                    choosen = "tan"
        
            # reset soltusions
            u.solutions = None
            #u.solveorder = u.solveorder - 1
            if choosen == "sincos":
                u.solutions = u.sincos_solutions
                u.nsolutions = len(u.sincos_solutions)
                u.eqntosolve = u.sincos_eqnlist[0]
                u.solvemethod = "sin or cos"
            elif choosen == "tan":
                u.solutions = u.tan_solutions
                u.nsolutions = len(u.tan_solutions)
                u.eqntosolve = u.tan_eqnlist[0]
                u.secondeqn = u.tan_eqnlist[1]
                u.solvemethod = "atan2(y,x)"

            u.set_solved(R, unknowns)

        elif u.solvable_sincos or u.solvable_tan:
            u.set_solved(R, unknowns)
                
        tick.blackboard.set("curr_unk", u)
        tick.blackboard.set("unknowns", unknowns)
        tick.blackboard.set("Robot", R)
        return b3.SUCCESS
        
        
