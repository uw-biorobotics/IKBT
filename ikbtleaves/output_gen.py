#!/usr/bin/python
#
#   output_gen.py --  report and code generation, as a BT leaf
#
#   Moving codegen into the tree is what lets a SECOND solution strategy exist:
#   each branch can finish with its own output generator, emitting artifacts that
#   describe the way that branch actually solved the robot.  While codegen sat in
#   the caller, there was exactly one way to finish.
#
#   OFF BY DEFAULT, and that is a promise being kept rather than caution.
#   tests/test_chair_helper.py runs a complete solve and documents that it leaves
#   LaTex/ and CodeGen/ alone;  every unit test that builds a tree would start
#   overwriting the repo's generated artifacts the moment this leaf fired on its
#   own.  So library and test callers get an inert node, and the ikSolver.py
#   command line opts in --
#
#       bt, nodes = build_default_bt(codegen=True)
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees

from ikbtfunctions.ik_driver import emit_outputs


class output_gen_full(b3.Action):
    '''Build the solution set and write the LaTeX report plus generated Python
       and C++, for a robot solved entirely in closed form.

           SUCCESS  artifacts written, or the leaf is disabled
           FAILURE  the solution set could not be built

       When disabled this leaf does nothing at all -- it does not even build the
       solution set.  That is on purpose:  ik_driver.run_solver() still owns
       create_solution_set() on the ordinary path, and create_solution_set() is
       NOT idempotent (it appends to unknown.LHSversionNames), so exactly one of
       the two must call it.  Enabling this leaf means the tree owns the whole
       tail end, and the caller passes create_solutions=False.'''

    def __init__(self):
        super(output_gen_full, self).__init__()
        self.Name = 'Output Generator (closed form)'
        self.BHdebug = False

        #  Follows the invariant_gen precedent:  a documented, default-off node
        #  that is present in the tree and inert until switched on.
        self.enabled = False

    def tick(self, tick):
        if not self.enabled:
            #  run_solver() builds the solution set and the caller writes the
            #  files, exactly as before.  Succeed so the branch completes.
            return b3.SUCCESS

        bb = tick.blackboard
        R = bb.get('Robot')
        unks = bb.get('unknowns')

        if R is None or not unks:
            print(self.Name, ': no Robot or no unknowns on the blackboard.')
            return b3.FAILURE

        try:
            R.create_solution_set()
        except Exception as e:
            #  make_LHS_versions() under create_solution_set() raises IndexError
            #  on an empty solution matrix.  Report it and fail the branch
            #  rather than letting it escape the tick and kill the run:  a
            #  fallback strategy downstream may still have something to offer.
            print(self.Name, ': could not build the solution set --',
                  '%s: %s' % (type(e).__name__, e))
            return b3.FAILURE

        if self.BHdebug:
            print('\n', self.Name, ': ', len(R.solutionSet), ' solutions for ',
                  R.name)

        emit_outputs(R, unks)
        return b3.SUCCESS


#####################################################################
#
#   Test code
#

import unittest


class TestSolver017(unittest.TestCase):
    '''The codegen leaf -- specifically, that it stays inert until asked.

       No solve here, and no files:  the disabled path is checked by observing
       that it touches NOTHING, which is the property the rest of the test
       suite depends on.'''

    def setUp(self):
        print('\n\n===============  Test output generator leaf  =====================')
        return

    def runTest(self):
        self.test_outA_disabled_is_inert()
        self.test_outB_disabled_needs_no_blackboard()
        self.test_outC_enabled_reports_a_bad_solution_set()

    def tick(self, node, bb):
        t = b3.BehaviorTree()
        t.root = node
        return t.tick('testing output_gen', bb)

    def test_outA_disabled_is_inert(self):
        '''Disabled: SUCCESS, and create_solution_set() is NOT called.

           Double-calling create_solution_set() corrupts the solution set --
           it appends to unknown.LHSversionNames -- so "does not call it" is a
           real requirement, not a detail.'''
        fs = ' output_gen disabled FAIL'

        class spy(object):
            name = 'Spy'
            calls = 0
            solutionSet = set()
            def create_solution_set(self):
                self.calls += 1

        R = spy()
        bb = b3.Blackboard()
        bb.set('Robot', R)
        bb.set('unknowns', ['not really an unknown'])

        node = output_gen_full()
        self.assertFalse(node.enabled, fs + ' (must default to OFF)')
        st = self.tick(node, bb)
        self.assertEqual(st, b3.SUCCESS, fs)
        self.assertEqual(R.calls, 0,
                         fs + ' (disabled leaf called create_solution_set)')

    def test_outB_disabled_needs_no_blackboard(self):
        '''A disabled leaf must succeed even on an empty blackboard -- the BT
           structural tests tick trees that were never given a robot.'''
        fs = ' output_gen empty-blackboard FAIL'
        st = self.tick(output_gen_full(), b3.Blackboard())
        self.assertEqual(st, b3.SUCCESS, fs)

    def test_outC_enabled_reports_a_bad_solution_set(self):
        '''Enabled, but the solution set will not build:  FAILURE, and the
           exception does not escape.  IndexError out of make_LHS_versions() on
           an empty solution matrix is the real case.'''
        fs = ' output_gen error-path FAIL'

        class boom(object):
            name = 'Boom'
            def create_solution_set(self):
                raise IndexError('list index out of range')

        bb = b3.Blackboard()
        bb.set('Robot', boom())
        bb.set('unknowns', ['not really an unknown'])

        node = output_gen_full()
        node.enabled = True
        st = self.tick(node, bb)
        self.assertEqual(st, b3.FAILURE, fs)

        #  ... and with nothing on the blackboard at all
        node2 = output_gen_full()
        node2.enabled = True
        self.assertEqual(self.tick(node2, b3.Blackboard()), b3.FAILURE, fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver017)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
