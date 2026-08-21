#!/usr/bin/python
#
#   hybrid_ik.py --  the hybrid symbolic-numeric branch
#
#   futurework.md item 1:  when IKBT cannot solve a robot in closed form,
#   simplify its DH parameters until it can, solve the SIMPLIFIED robot
#   symbolically, and correct the result numerically using the symbolic
#   Jacobian.  Six robots have no Pieper triple of any kind, and in each one a
#   single DH parameter is responsible -- KinovaLite with d_5 -> 0 goes from
#   0-of-6 variables solved to all 6.
#
#   RIGHT NOW THIS FILE IS A PLACEHOLDER.  hybrid_stub always FAILs, so the
#   branch is inert and the tree behaves exactly as it did before the branch
#   existed.  That is the point:  it makes the tree RESTRUCTURE separable from
#   the new behavior, so a baseline diff over every robot can show that
#   rearranging the tree changed nothing, before any new solver is added.
#
#   The stub is replaced one leaf at a time, each proven by its own baseline
#   diff:
#
#       pieper_id         does this robot have a Pieper triple?  The branch is
#                         gated on the INVERSE -- a robot that has a triple and
#                         still failed symbolically is a solver defect, not a
#                         geometry problem, and must not be simplified.
#       simplified_arm    which single DH parameter costs the least task-space
#                         displacement to zero or snap?
#       solve_simplified  solve the derived robot with a nested tree
#       numeric_ik +      damped least squares from the closed-form seed, and
#       output_gen_hybrid artifacts labelled with the parameter that was changed
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import b3 as b3          # behavior trees


class hybrid_stub(b3.Action):
    '''Placeholder for the hybrid branch.  Always FAILURE, so the branch is a
       no-op and the enclosing Priority falls through as if it were not there.

       FAILURE rather than SUCCESS on purpose:  a SUCCESS here would tell the
       tree a robot had been solved by a strategy that does not exist yet, and
       the first thing downstream would be codegen over an empty solution set.

       Left deliberately noisy under BHdebug so that a robot reaching this leaf
       is visible -- reaching it means the symbolic solver came up empty, which
       is exactly the population the hybrid method is for.'''

    def __init__(self):
        super(hybrid_stub, self).__init__()
        self.Name = 'Hybrid Branch (not implemented)'
        self.BHdebug = False

    def tick(self, tick):
        if self.BHdebug:
            R = tick.blackboard.get('Robot')
            print('\n', self.Name, ':', getattr(R, 'name', '?'),
                  'was not solved symbolically, and the hybrid branch is a stub.')
        return b3.FAILURE


#####################################################################
#
#   Test code
#

import unittest


class TestSolver018(unittest.TestCase):
    '''The hybrid placeholder.  One property matters: it is inert.'''

    def setUp(self):
        print('\n\n===============  Test hybrid branch stub  =====================')
        return

    def runTest(self):
        self.test_hybA_always_fails()
        self.test_hybB_inverted_stub_would_not_gate()

    def test_hybA_always_fails(self):
        '''FAILURE on an empty blackboard and on a populated one alike.  The
           whole Phase-B claim -- that restructuring the tree changed nothing --
           rests on this leaf never returning anything else.'''
        fs = ' hybrid_stub FAIL'
        for bb in (b3.Blackboard(), b3.Blackboard()):
            t = b3.BehaviorTree()
            t.root = hybrid_stub()
            self.assertEqual(t.tick('testing hybrid_stub', bb), b3.FAILURE, fs)

    def test_hybB_inverted_stub_would_not_gate(self):
        '''Sanity check on the shape the real branch will have: the gate is
           Inverter(pieper_id), so a leaf that FAILs becomes a SUCCESS under
           the inverter.  Recorded here because it is the easy thing to get
           backwards -- the hybrid runs when there is NO triple.'''
        fs = ' hybrid_stub inverter FAIL'
        t = b3.BehaviorTree()
        t.root = b3.Inverter(hybrid_stub())
        self.assertEqual(t.tick('testing inverter', b3.Blackboard()),
                         b3.SUCCESS, fs)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver018)
    unittest.TextTestRunner(verbosity=2).run(suite)


##   Self Tests
if __name__ == "__main__":
    run_test()
