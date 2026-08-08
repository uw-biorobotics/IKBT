#!/usr/bin/python
#
#   Structural tests for ikbtfunctions/bt_assembly.py
#
#   Every other test in this repo hand-assembles a 2-leaf tree with a bespoke
#   setup node.  This is the first one that inspects the REAL composite the
#   solver runs, which is where a whole class of defects lives:  a leaf wired
#   into the wrong parent, a Priority child order silently changed, an OrNode
#   downgraded to a Priority.  No solving happens here, so it is fast enough for
#   the main suite.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import unittest

import b3 as b3

from ikbtfunctions.bt_assembly import make_leaves, build_worktools, build_default_bt


class TestSolver013(unittest.TestCase):
    '''bt_assembly:  the tree ikSolver.py actually builds.'''

    def setUp(self):
        print('\n\n===============  Test bt_assembly  =====================')
        return

    def runTest(self):
        self.test_btaA_every_node_is_named()
        self.test_btaB_worktools_child_order()
        self.test_btaC_tan_and_sincos_both_run()
        self.test_btaD_tree_shape()
        self.test_btaE_nodes_are_shared_not_copied()
        self.test_btaF_leaf_debug_flags_propagate()

    def test_btaA_every_node_is_named(self):
        '''An unnamed node prints as a bare class in the BT log, which makes
           debugging a 60-tick solve considerably worse.'''
        fs = ' bt_assembly FAIL'
        bt, nodes = build_default_bt()
        for key, node in nodes.items():
            self.assertTrue(getattr(node, 'Name', ''),
                            fs + ' (node %s has no Name)' % key)

    def test_btaB_worktools_child_order(self):
        '''b3.Priority returns on its first non-FAILURE child, so this order is
           the solver's whole preference policy -- cheap pattern matchers first,
           transforms last.  invariantGen must stay last:  that placement is
           what makes it behavior-preserving for robots that already solve.'''
        fs = ' bt_assembly worktools order FAIL'
        nodes = make_leaves()
        wt = build_worktools(nodes)

        self.assertIsInstance(wt, b3.Priority,
                              fs + ' (worktools is not a Priority)')
        expected = ['algSol', 'sc_tan', 'Simu_Eqn_Sol', 'sacSol',
                    'x2z2_Solver', 'invariantGen']
        self.assertEqual(wt.children, [nodes[k] for k in expected],
                         fs + ' (expected order ' + str(expected) + ')')

    def test_btaC_tan_and_sincos_both_run(self):
        '''rank picks the nicer of two solutions, so it needs BOTH candidates.
           That requires b3.OrNode, which runs all its children -- a Priority
           here would short-circuit after tan and rank would have nothing to
           choose between.  This is deliberate, not a bug.'''
        fs = ' bt_assembly sc_tan FAIL'
        nodes = make_leaves()
        sc_tan = nodes['sc_tan']

        self.assertIsInstance(sc_tan, b3.Sequence, fs + ' (sc_tan not a Sequence)')
        ornode, ranknode = sc_tan.children
        self.assertIsInstance(ornode, b3.OrNode,
                              fs + ' (tan/sincos parent is not an OrNode)')
        self.assertEqual(ornode.children, [nodes['tanSol'], nodes['scSol']], fs)
        self.assertIs(ranknode, nodes['rankNode'], fs + ' (rank is not last)')

    def test_btaD_tree_shape(self):
        '''    RepeatUntilSuccess(x10)
                 Sequence[ sub_transform,
                           RepeatUntilSuccess(x6, Sequence[assigner, sum_id, worktools]),
                           updateL,
                           comp_det ]                                            '''
        fs = ' bt_assembly tree shape FAIL'
        bt, nodes = build_default_bt()

        #  b3 decorators hold a single .child; composites hold .children
        top = bt.root
        self.assertIsInstance(top, b3.RepeatUntilSuccess, fs + ' (root)')
        self.assertIs(top, nodes['topnode'], fs)
        self.assertEqual(top.max_loop, 10, fs + ' (outer loop budget)')
        self.assertIs(top.child, nodes['solveRoutine'], fs + ' (root child)')

        routine = nodes['solveRoutine']
        self.assertEqual(routine.children,
                         [nodes['sub_trans'], nodes['subtree'],
                          nodes['updateLNode'], nodes['compDetect']],
                         fs + ' (solveRoutine children)')

        inner = nodes['subtree']
        self.assertIsInstance(inner, b3.RepeatUntilSuccess, fs + ' (subtree)')
        self.assertEqual(inner.max_loop, 6, fs + ' (inner loop budget)')
        self.assertEqual(inner.child.children,
                         [nodes['asgn'], nodes['sumOfAnglesID'], nodes['worktools']],
                         fs + ' (subtree sequence children)')

    def test_btaE_nodes_are_shared_not_copied(self):
        '''build_default_bt() returns the node dict so a caller can set BHdebug
           on the instances the tree really holds.  If it handed back copies,
           every debug flag would silently do nothing.'''
        fs = ' bt_assembly node sharing FAIL'
        bt, nodes = build_default_bt()

        nodes['tanID'].BHdebug = True
        tan_seq = nodes['sc_tan'].children[0].children[0]
        self.assertTrue(tan_seq.children[0].BHdebug,
                        fs + ' (flag set via nodes dict did not reach the tree)')
        nodes['tanID'].BHdebug = False

    def test_btaF_leaf_debug_flags_propagate(self):
        '''The leaf_debug / solver_debug arguments must actually reach leaves.'''
        fs = ' bt_assembly debug flag FAIL'
        nodes = make_leaves(leaf_debug=True, solver_debug=False)
        self.assertTrue(nodes['tanID'].BHdebug, fs + ' (leaf_debug ignored)')

        nodes = make_leaves(leaf_debug=False, solver_debug=True)
        self.assertTrue(nodes['tanSolver'].BHdebug, fs + ' (solver_debug ignored)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver013)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    run_test()
