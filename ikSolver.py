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

import ikbtfunctions.ik_driver as ik_driver
from ikbtfunctions.ik_driver   import (load_robot, run_solver,
                                       print_solved_equations, ensure_logdir,
                                       solved_anything)
from ikbtfunctions.bt_assembly import build_default_bt
from ikbtfunctions.progress    import enable_sympy_meter

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


#  Performance and layout commentary:  the per-call sympy meter (slow simplify /
#  trigsimp lines) and the report-fitting messages.  OFF by default -- they are
#  diagnostics for someone chasing a slow solve, and in an ordinary run they bury
#  the per-pass progress they sit between.  Set True to get them back.
PERFORMANCE_OUTPUT = False


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
    #  codegen=True: the TREE writes LaTex/ and CodeGen/, through the
    #  output_gen_full leaf on the end of the symbolic branch.  This front end is
    #  the one caller that wants file side effects -- library and test callers
    #  get the default, codegen=False, and leave the generated artifacts alone.
    ikbt, nodes = build_default_bt(leaf_debug=False, solver_debug=False,
                                   codegen=not TEST_DATA_GENERATION)

    #  A human is waiting on this front end, so meter the sympy calls:  37-85 %
    #  of a solve's wall clock is inside sp.simplify(), and a call slower than
    #  the threshold prints as it happens.  That is the only output that can
    #  appear DURING a blocking simplify -- symbolic_loop's per-pass line cannot
    #  print until the pass returns.  Batch callers (robot_baseline, tests) do
    #  not enable it;  it wraps a third-party class, so it stays opt-in.
    if PERFORMANCE_OUTPUT:
        enable_sympy_meter()
        ik_driver.REPORT_PROGRESS = True

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
    #  create_solutions=False always:  with codegen in the tree, the
    #  output_gen_full leaf owns create_solution_set().  It must be called
    #  exactly once -- it appends to unknown.LHSversionNames -- so run_solver()
    #  must not also do it.  On the TEST_DATA_GENERATION path nobody calls it,
    #  which is what that path wants (it pickles the raw post-solve state).
    R, unks, bb = run_solver(R, unknowns, ikbt, create_solutions=False)

    if TEST_DATA_GENERATION:
        # Now we're going to save some results for use in tests.
        print(' Storing results for test use')
        test_pickle_dir = 'Test_pickles/'
        name = test_pickle_dir + R.name + 'test_pickle.p'
        with open(name, 'wb') as pf:
            pickle.dump([R, unks], pf)
        return

    if not solved_anything(bb):
        #  The tree gave up having solved nothing -- see comp_det.  The symbolic
        #  branch FAILed, so the codegen leaf never ticked and nothing was
        #  written;  this is just the report to the user.
        print('\n\n           No solution generated for ' + robot + '.')
        print('           Nothing written to LaTex/ or CodeGen/.\n')
        print('\n                  End of solution job \n                  (no solution) \n\n')
        return

    #
    #  The solution set and all three output formats were produced inside the
    #  tick, by the output_gen_full leaf at the end of the symbolic branch.
    #  (Version 3 of solution-set finding.  Version 2 was tree-based and used
    #   R.notation_collections / matching.matching_func(); that path is gone.)
    #

    #################################################
    # print out all equations that used to solve variables
    print_solved_equations(unks)

    print('\n\n\n                  End of solution job \n                  (normal exit) \n\n')


if __name__ == "__main__":
    main(argv)
