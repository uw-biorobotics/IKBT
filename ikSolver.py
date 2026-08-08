#!/usr/bin/python
#
#     Test for full IK solutions of know robot(s)
#
#   This is now a thin command-line front end.  The pipeline lives in
#   ikbtfunctions/ik_driver.py and the behavior tree in ikbtfunctions/bt_assembly.py,
#   so that tests and alternative front ends can import them without triggering
#   a full solve.
#
#        python3 ikSolver.py <RobotName>       ( no argument -> Wrist )

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

from ikbtfunctions.ik_driver   import (load_robot, run_solver, emit_outputs,
                                       print_solved_equations, ensure_logdir)
from ikbtfunctions.bt_assembly import build_default_bt

TEST_DATA_GENERATION = False

sp.init_printing()

# generic variables for any maniplator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('Px Py Pz')


def banner():
    if not TEST_DATA_GENERATION:
        print("")
        print("          Running IK solution ")
        print("")
        print("")
    else:
        print('-'*50)
        print("")
        print("          Generating IKBT TEST DATA only ")
        print("")
        print("          (for production: line 32: TEST_DATA_GENERATION = False)")
        print("")
        print('-'*50)


def main(argv):
    banner()

    ########################################################
    #
    #     Robot Parameters
    #
    ######################################################

    if len(argv) == 1:  # no argument - use default
        #robot = 'Gomez'
        #robot = 'Puma'
        #robot = 'Chair_Helper'
        #robot = 'Khat6DOF'
        robot = 'Wrist'

    elif len(argv) == 2:
        robot = str(argv[1])

    print('')
    print('')
    print('             Working on '+robot)
    print('')
    print('')

    #
    #     Set up robot equations for further solution by BT
    #
    M, R, unknowns = load_robot(robot)

    ####################################################################################
    ##
    #                                   Set up the BT
    #
    ikbt, nodes = build_default_bt(leaf_debug=False, solver_debug=False)

    ensure_logdir()

    #
    #     Logging and per-robot debug setup
    #
    #   Every node is reachable through the `nodes` dict, e.g.
    #
    #       ikbt.log_flag = 2       # log exits: 1=SUCCESS only, 2=BOTH S,F
    #       ikbt.log_file = open('logs/BT_'+robot+'_node_log.txt', 'w')
    #       ikbt.log_file.write(robot+' Solution Node Log\n')
    #
    #       nodes['tanID'].BHdebug        = True
    #       nodes['x2z2_Solver'].BHdebug  = True
    #       nodes['sumOfAnglesID'].BHdebug = True
    #       nodes['compDetect'].FailAllDone = False  # SUCCEED when work remains (not default)
    #
    #   (this replaces the ~200 lines of commented-out per-robot blocks that
    #    used to live here)

    ################################################################################
    #
    #           Perform the Computation via ticking the BT
    #
    R, unks, bb = run_solver(R, unknowns, ikbt,
                             create_solutions=not TEST_DATA_GENERATION)

    if TEST_DATA_GENERATION:
        # Now we're going to save some results for use in tests.
        print(' Storing results for test use')
        test_pickle_dir = 'Test_pickles/'
        name = test_pickle_dir + R.name + 'test_pickle.p'
        with open(name, 'wb') as pf:
            pickle.dump([R, unks], pf)
        return

    #
    #  Version 3 of solution set finding (non tree).  Version 2 was tree-based and
    #  used R.notation_collections / matching.matching_func(); that path is gone.
    #
    emit_outputs(R, unks)

    #################################################
    # print out all equations that used to solve variables
    print_solved_equations(unks)

    print('\n\n\n                  End of solution job \n                  (normal exit) \n\n')


if __name__ == "__main__":
    main(argv)
