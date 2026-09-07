#!/usr/bin/python
#
#   output_gen.py --  report and code generation, as a BT leaf
#
#   ONE report generator, ticked after whichever branch produced the solution:
#
#       Sequence[ analysis, report_gen ]
#       analysis = Priority[ symbolic_branch, hybrid_branch ]
#
#   Per BH:  the report is a property of the finished solve, not of the branch
#   that produced it, so it is generated once at the end rather than duplicated
#   inside each branch.  b3.Sequence aborts on FAILURE, so a solve that got
#   nowhere never reaches this leaf and no empty report is written.
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

from ikbtfunctions.ik_driver import emit_outputs, emit_hybrid_outputs, load_robot

import ikbtbasics.dh_analysis as da


class report_gen(b3.Action):
    '''Build the solution set and write the LaTeX report plus generated Python
       and C++.

           SUCCESS  artifacts written, or the leaf is disabled
           FAILURE  the solution set could not be built

       When disabled this leaf does nothing at all -- it does not even build the
       solution set.  That is on purpose:  ik_driver.run_solver() still owns
       create_solution_set() on the ordinary path, and create_solution_set() is
       NOT idempotent (it appends to unknown.LHSversionNames), so exactly one of
       the two must call it.  Enabling this leaf means the tree owns the whole
       tail end, and the caller passes create_solutions=False.'''

    def __init__(self):
        super(report_gen, self).__init__()
        self.Name = 'Report Generator'
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

        #  The joint-axis geometry statement.
        #
        #  PREFER the snapshot pieper_geom_report put on the blackboard.  On
        #  the hybrid path the Robot here is a DERIVED arm -- install_simplified
        #  swapped it -- so generating the statement from R would describe the
        #  simplified robot and silently mis-report the real one.  The snapshot
        #  was taken before the swap.
        #
        #  Falling back to R is correct on the symbolic path, where
        #  pieper_geom_report never ticked (it sits in the hybrid branch, which
        #  a completed symbolic solve never reaches) and R *is* the true robot.
        #  Either way a failure here is a warning: no statement must never cost
        #  us the report.
        hybrid = bb.get('hybrid_source')

        stashed = bb.get('pieper_latex')
        if stashed:
            R.pieper_latex = stashed
        else:
            try:
                R.pieper_latex = da.pieper_latex(
                    R.Mech.DH, R.Mech.pvals,
                    da.ndof_from_unknowns(unks), R.name)
            except Exception as e:
                print(self.Name, ': no joint-axis geometry statement --',
                      '%s: %s' % (type(e).__name__, e))

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

        if not hybrid:
            emit_outputs(R, unks)
            return b3.SUCCESS

        #  HYBRID.  R is the DERIVED arm;  Phase II has to refine against the
        #  TRUE one, so load it fresh.  It is not on the blackboard --
        #  install_simplified replaced it -- and re-loading is cheap because
        #  kinematics_pickle() serves the true robot's FK from fk_eqns/.
        true_name = hybrid.get('true_robot')
        R_true = None
        try:
            M_true, R_true, _ = load_robot(true_name)
            #  The report describes the TRUE arm's joint-axis geometry in its
            #  own section, and that statement was snapshotted before the swap.
            if stashed:
                R_true.pieper_latex = stashed
        except Exception as e:
            #  DEGRADE, do not fail.  Without the true arm there is no Phase II
            #  and no numeric correction -- but the closed form for the
            #  simplified arm is real work and the report still says, in its own
            #  section, which arm it describes.  Losing the whole solve here
            #  would be the worse trade.
            print(self.Name, ': could not load the true robot %r -- %s: %s'
                  % (true_name, type(e).__name__, e))
            print(self.Name, ': writing the report WITHOUT the numeric'
                  ' correction step.')

        emit_hybrid_outputs(R, unks, hybrid, R_true)
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

        node = report_gen()
        self.assertFalse(node.enabled, fs + ' (must default to OFF)')
        st = self.tick(node, bb)
        self.assertEqual(st, b3.SUCCESS, fs)
        self.assertEqual(R.calls, 0,
                         fs + ' (disabled leaf called create_solution_set)')

    def test_outB_disabled_needs_no_blackboard(self):
        '''A disabled leaf must succeed even on an empty blackboard -- the BT
           structural tests tick trees that were never given a robot.'''
        fs = ' output_gen empty-blackboard FAIL'
        st = self.tick(report_gen(), b3.Blackboard())
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

        node = report_gen()
        node.enabled = True
        st = self.tick(node, bb)
        self.assertEqual(st, b3.FAILURE, fs)

        #  ... and with nothing on the blackboard at all
        node2 = report_gen()
        node2.enabled = True
        self.assertEqual(self.tick(node2, b3.Blackboard()), b3.FAILURE, fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver017)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
