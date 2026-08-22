#!/usr/bin/python
#
#   Structural tests for ikbtfunctions/bt_assembly.py
#
#   The BT is meant to be an EXPERIMENTAL SURFACE:  a user should be able to
#   re-order worktools, wrap a subtree differently, drop a solver, or add a new
#   solution strategy without a test suite telling them they are wrong.  So this
#   file does not compare the tree to a stored shape.  Instead it validates any
#   candidate tree against invariants that hold for EVERY working IKBT tree:
#
#       * there is a root, and it is a real b3 node
#       * no childless Priority / Sequence / OrNode  (a childless Priority always
#         FAILs, a childless Sequence always SUCCEEDs -- both are silent no-ops)
#       * no decorator with an empty .child  (b3 returns b3.ERROR)
#       * every child slot holds a node INSTANCE, not a class  (the b3.Sequence(
#         [tan_id, tan_solve]) missing-parens mistake)
#       * no node instance appears twice, and no cycles
#       * every solver we ship is present SOMEWHERE in the tree
#       * every ID leaf is sequenced ahead of its solver leaf
#       * loop decorators have a finite, non-zero budget
#       * the custom leaves all carry a Name (they appear in the tick log)
#
#   bt_problems() below is the whole test:  it is a reusable linter, so a user
#   experimenting with a new strategy can call it on their own tree --
#
#       from tests.bt_assembly_test import bt_problems
#       print(bt_problems(my_tree))     #  [] means structurally sound
#
#   No solving happens here, so it is fast enough for the main suite.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington


#
#   AI Statement by BH Aug 2026:   This is the second version of a
#   test suite developed by Claude.
#   It tests the Behavior Tree assembled in ikSolver. The test includes
#   a BT "linter" written by Claude which syntacticly checks any BT.  Then
#   it checks for some invalid cases such as a Priority or Sequence node with
#   no children etc.   Finally, it assumes that all id/solver nodes that we now
#   employ must be present in any BT used for robot arm IK.  This MIGHT cause
#   a test failure for some experimental code.
#

import importlib
import pkgutil
import unittest

import b3 as b3

import ikbtleaves
from ikbtfunctions.bt_assembly import make_leaves, build_worktools, build_default_bt

#  Solver / support leaves referred to by class below.  Imported as classes (not
#  instances) on purpose:  the checks ask "is this ALGORITHM in the tree", never
#  "is it in this position".
from ikbtleaves.assigner_leaf    import assigner
from ikbtleaves.rank_leaf        import rank
from ikbtleaves.algebra_solver   import algebra_id, algebra_solve
from ikbtleaves.tan_solver       import tan_id, tan_solve
from ikbtleaves.sincos_solver    import sincos_id, sincos_solve
from ikbtleaves.sinANDcos_solver import sinandcos_id, sinandcos_solve
from ikbtleaves.two_eqn_m7       import simu_id, simu_solver
from ikbtleaves.invariant_gen    import invariant_gen
from ikbtleaves.x2y2_transform   import x2z2_transform
from ikbtleaves.sub_transform    import sub_transform
from ikbtleaves.sum_id           import sum_id, sum_solve
from ikbtleaves.updateL          import updateL
from ikbtleaves.comp_detect      import comp_det
from ikbtleaves.symbolic_loop    import symbolic_loop
from ikbtleaves.output_gen       import output_gen_full
from ikbtleaves.hybrid_ik        import hybrid_stub, pieper_id


###############################################################################
#
#    What a tree has to contain (NOT where it has to be)
#

#  Drop one of these and IKBT loses a solution method outright:  some robot that
#  used to solve now stops.  This is the "does the tree contain all our valid
#  solvers" list.
REQUIRED_SOLVERS = [algebra_id, algebra_solve,
                    tan_id, tan_solve,
                    sincos_id, sincos_solve,
                    sinandcos_id, sinandcos_solve,
                    simu_id, simu_solver,
                    x2z2_transform]

#  Not solvers, but nothing solves without them:  assigner advances curr_unk,
#  rank is what actually calls set_solved() for tan/sincos, sum_id makes the
#  sum-of-angles equations visible to the algebra leaf, sub_transform and
#  updateL maintain the equation lists, comp_det terminates the tree.
REQUIRED_SUPPORT = [assigner, rank, sum_id, sub_transform, updateL, comp_det]

#  Legal in a tree, not required in one.  invariant_gen is a documented,
#  off-by-default extension point;  sum_solve is superseded by the algebra leaf.
#
#  The three top-of-tree leaves are OPTIONAL on purpose, and it is worth saying
#  why, because they are all in the shipped tree.  This file lints ANY candidate
#  tree, and none of the three is needed to solve a robot:  the outer loop can be
#  a plain b3.RepeatUntilSuccess (it was, until Aug 2026), codegen can stay with
#  the caller (it did), and the hybrid branch does not exist yet.  Requiring them
#  would tell somebody experimenting with a simpler tree that they are wrong.
#  What the shipped tree actually guarantees is asserted directly instead -- see
#  test_btaR / test_btaS below.
OPTIONAL_LEAVES = [invariant_gen, sum_solve,
                   symbolic_loop, output_gen_full, hybrid_stub,
                   pieper_id]

#  An ID leaf stashes state on the blackboard that its solver leaf then consumes,
#  so the ID must be sequenced AHEAD of the solver.  sum_id is deliberately
#  absent:  its partner solving is done by the algebra leaf, not sum_solve.
ID_SOLVER_PAIRS = [(algebra_id, algebra_solve),
                   (tan_id, tan_solve),
                   (sincos_id, sincos_solve),
                   (sinandcos_id, sinandcos_solve),
                   (simu_id, simu_solver)]

#  tan_solve and sincos_solve deliberately do NOT call set_solved() -- see the
#  comment at tan_solver.py:338.  rank does it for both.  A tree with either
#  solver and no rank leaf discards every solution they find.
NEEDS_RANK = [tan_solve, sincos_solve]

#  b3 stock helper actions have no meaningful Name and need none.
STOCK_ACTIONS = (b3.Succeeder, b3.Failer, b3.Runner, b3.Wait, b3.Error)


def is_placeholder_name(name):
    '''True for a Name nobody chose:  BaseNode's '--unnamed--' default and the
       '*Priority*' / '*Sequence*' / '*OrNode*' defaults the stock composites
       set on themselves.  Naming a plumbing composite is good practice but not
       required, so placeholders are not counted as duplicates.'''
    return (not name) or name == '--unnamed--' or (name.startswith('*')
                                                   and name.endswith('*'))

#  Sequence-like composites:  children run in order, so "ID before solver" is
#  only meaningful under one of these.
SEQUENCE_TYPES = (b3.Sequence, b3.MemSequence)


###############################################################################
#
#    Tree walking
#

def child_slots(node):
    '''Every child slot of a b3 node, INCLUDING junk.

       Deliberately tolerant:  it must not raise on the malformed trees this
       suite is here to detect, so it returns whatever is in the slot rather
       than assuming a BaseNode.  b3.Decorator initializes .child to [] when it
       is given none, hence the list case.'''
    if isinstance(node, b3.Composite):
        return list(getattr(node, 'children', None) or [])
    if isinstance(node, b3.Decorator):
        kid = getattr(node, 'child', None)
        if isinstance(kid, (list, tuple)):
            return list(kid)              # empty .child default
        return [] if kid is None else [kid]
    return []                             # actions and conditions are leaves


def walk_bt(root):
    '''Yield (node, path) for every node reachable from root, depth first.

       path is the tuple of ancestors.  Cycle safe:  a node that is its own
       ancestor is yielded but not descended into a second time.  Non-node junk
       in a child slot is skipped (bt_problems reports it separately).'''
    if not isinstance(root, b3.BaseNode):
        return
    stack = [(root, ())]
    while stack:
        node, path = stack.pop()
        yield node, path
        if any(kid is node for kid in path):
            continue                      # cycle -- do not go around again
        for kid in child_slots(node):
            if isinstance(kid, b3.BaseNode):
                stack.append((kid, path + (node,)))


def bt_nodes(bt_or_root):
    '''Every distinct node instance in a tree (or subtree), in walk order.'''
    root = bt_or_root.root if isinstance(bt_or_root, b3.BehaviorTree) else bt_or_root
    out = []
    for node, _ in walk_bt(root):
        if not any(n is node for n in out):
            out.append(node)
    return out


def _subtree_classes(node, memo):
    '''Set of node classes appearing at or below node.'''
    key = id(node)
    if key in memo:
        return memo[key]
    memo[key] = set()                     # guard against cycles
    found = {node.__class__}
    for kid in child_slots(node):
        if isinstance(kid, b3.BaseNode):
            found |= _subtree_classes(kid, memo)
    memo[key] = found
    return found


###############################################################################
#
#    The linter
#

def bt_problems(bt, required_solvers=None, required_support=None):
    '''Return a list of human readable structural problems with a tree.

       [] means the tree is sound.  Accepts a b3.BehaviorTree or a bare root
       node.  Pass required_solvers=[] / required_support=[] to lint a partial
       tree (a subtree under development, say) without demanding full coverage.

       Nothing here depends on the tree's SHAPE -- only on properties that any
       tree must have in order to solve anything at all.'''

    if required_solvers is None:
        required_solvers = REQUIRED_SOLVERS
    if required_support is None:
        required_support = REQUIRED_SUPPORT

    p = []

    #  ---- root
    if isinstance(bt, b3.BehaviorTree):
        root = bt.root
        if root is None:
            return ['no root:  BehaviorTree.root is None (nothing will tick)']
    else:
        root = bt
    if not isinstance(root, b3.BaseNode):
        return ['no root:  root is %r, not a b3 node' % (root,)]

    #  ---- one pass over the tree
    seen = []                             # distinct instances, walk order
    for node, path in walk_bt(root):
        nm = getattr(node, 'Name', '')
        where = node.__class__.__name__ if is_placeholder_name(nm) else nm

        if any(n is node for n in seen):
            if any(a is node for a in path):
                p.append('cycle:  %s is its own ancestor (tick would recurse '
                         'forever)' % where)
            else:
                p.append('shared node:  %s appears at more than one place in the '
                         'tree -- b3 keeps per-node state on the blackboard keyed '
                         'by node id, so the two positions would collide'
                         % where)
            continue
        seen.append(node)

        slots = child_slots(node)

        #  childless composites are silent no-ops, not errors b3 reports
        if isinstance(node, b3.Composite) and not slots:
            verdict = ('always SUCCEEDs' if isinstance(node, SEQUENCE_TYPES)
                       else 'always FAILs')
            p.append('empty %s:  %s has no children (%s, silently)'
                     % (node.__class__.__name__, where, verdict))

        if isinstance(node, b3.Decorator) and not slots:
            p.append('empty %s:  %s has no child (b3 returns ERROR when ticked)'
                     % (node.__class__.__name__, where))

        #  a class instead of an instance:  b3.Sequence([tan_id, tan_solve])
        for kid in slots:
            if not isinstance(kid, b3.BaseNode):
                extra = ''
                if isinstance(kid, type) and issubclass(kid, b3.BaseNode):
                    extra = ' -- looks like a missing () (class, not instance)'
                p.append('bad child of %s:  %r is not a b3 node%s'
                         % (where, kid, extra))

        #  loop budgets
        if hasattr(node, 'max_loop'):
            if node.max_loop == 0:
                p.append('loop budget:  %s has max_loop == 0, its child never '
                         'runs' % where)
            elif node.max_loop < 0:
                p.append('loop budget:  %s has max_loop %d (unbounded) -- a '
                         'solver that stops making progress would hang'
                         % (where, node.max_loop))

        #  Names show up in the BT tick log and in logs/*.  Stock b3 helpers are
        #  exempt;  our own leaves are not.  (A stock composite carrying its own
        #  '*Sequence*' default is untidy but legible, so it is left alone.)
        if not isinstance(node, STOCK_ACTIONS):
            if not nm or nm == '--unnamed--':
                p.append('unnamed node:  %s has no Name, so the tick log will '
                         'show a bare class' % node.__class__.__name__)

    #  ---- duplicate Names (distinct leaves logging under one label)
    names = {}
    for node in seen:
        if isinstance(node, STOCK_ACTIONS):
            continue
        nm = getattr(node, 'Name', '')
        if not is_placeholder_name(nm):
            names.setdefault(nm, []).append(node)
    for nm, group in sorted(names.items()):
        if len(group) > 1:
            p.append('duplicate Name:  %d nodes are all called "%s" (%s) -- the '
                     'tick log cannot tell them apart'
                     % (len(group), nm,
                        ', '.join(sorted(n.__class__.__name__ for n in group))))

    #  ---- capability coverage:  is every algorithm we ship in here somewhere?
    present = set(node.__class__ for node in seen)
    for cls in required_solvers:
        if cls not in present:
            p.append('missing solver:  %s is not in the tree, so IKBT loses that '
                     'solution method' % cls.__name__)
    for cls in required_support:
        if cls not in present:
            p.append('missing support leaf:  %s is not in the tree' % cls.__name__)

    #  ---- ID leaf must be sequenced ahead of its solver leaf
    memo = {}
    for id_cls, solve_cls in ID_SOLVER_PAIRS:
        if id_cls in present and solve_cls not in present:
            p.append('unpaired leaf:  %s is in the tree but %s is not, so the '
                     'equations it identifies are never solved'
                     % (id_cls.__name__, solve_cls.__name__))
        if solve_cls in present and id_cls not in present:
            p.append('unpaired leaf:  %s is in the tree but %s is not, so it '
                     'runs with no equations identified for it'
                     % (solve_cls.__name__, id_cls.__name__))
        if id_cls not in present or solve_cls not in present:
            continue
        ordered = False
        for node in seen:
            if not isinstance(node, SEQUENCE_TYPES):
                continue
            kids = [k for k in child_slots(node) if isinstance(k, b3.BaseNode)]
            sets = [_subtree_classes(k, memo) for k in kids]
            for i, si in enumerate(sets):
                if id_cls in si and any(solve_cls in sj for sj in sets[i + 1:]):
                    ordered = True
        if not ordered:
            p.append('ordering:  no Sequence puts %s ahead of %s (the solver '
                     'would run before its ID leaf sets up the blackboard)'
                     % (id_cls.__name__, solve_cls.__name__))

    #  ---- rank has to be there for the leaves that rely on it
    #  (skipped when the caller has waived rank, e.g. linting a fragment)
    if rank in required_support and rank not in present:
        for cls in NEEDS_RANK:
            if cls in present:
                p.append('missing rank:  %s does not call set_solved() itself, so '
                         'without a rank leaf its solutions are thrown away'
                         % cls.__name__)

    return p


def discovered_leaf_classes():
    '''Every b3 node class defined in ikbtleaves/, by import scan.

       Used by the inventory advisory below, so that a NEW leaf file shows up in
       the test output instead of going unnoticed.'''
    skip = ('testData', 'TEMPLATE')
    found = {}
    for m in pkgutil.iter_modules(ikbtleaves.__path__):
        if m.name.startswith(skip):
            continue
        mod = importlib.import_module('ikbtleaves.' + m.name)
        for nm, obj in vars(mod).items():
            if (isinstance(obj, type) and issubclass(obj, b3.BaseNode)
                    and obj.__module__ == mod.__name__
                    and not nm.startswith('test_')):
                found[obj] = m.name
    return found


###############################################################################
#
#    Tests
#

def alt_tree(nodes, worktools):
    '''A tree with a DIFFERENT shape from build_default_bt(), built from the same
       leaves:  one loop level instead of two, the failure-swallowing Priority
       moved inside the loop, different loop budgets.  Used to prove this suite
       accepts trees other than today's.'''
    body = b3.Sequence([nodes['asgn'], nodes['sumOfAnglesID'], worktools])
    body.Name = 'alt: assign + SOA + solve'

    loop = b3.RepeatUntilSuccess(b3.Priority([body, b3.Succeeder()]), 25)
    loop.Name = 'alt: solve loop'

    onepass = b3.Sequence([b3.Priority([nodes['sub_trans'], b3.Succeeder()]),
                           loop, nodes['updateLNode'], nodes['compDetect']])
    onepass.Name = 'alt: one pass'

    top = b3.RepeatUntilSuccess(onepass, 4)
    top.Name = 'alt: top'

    bt = b3.BehaviorTree()
    bt.root = top
    return bt


class TestSolver013(unittest.TestCase):
    '''bt_assembly:  structural soundness of a BT, whatever its shape.'''

    def setUp(self):
        print('\n\n===============  Test bt_assembly  =====================')
        return

    def runTest(self):
        self.test_btaA_default_tree_is_sound()
        self.test_btaB_default_tree_uses_every_leaf_it_builds()
        self.test_btaC_reordered_worktools_is_sound()
        self.test_btaD_alternative_shape_is_sound()
        self.test_btaE_no_root_detected()
        self.test_btaF_empty_composite_detected()
        self.test_btaG_empty_decorator_detected()
        self.test_btaH_class_instead_of_instance_detected()
        self.test_btaI_missing_solver_detected()
        self.test_btaJ_unpaired_and_misordered_leaves_detected()
        self.test_btaK_missing_rank_detected()
        self.test_btaL_bad_loop_budget_detected()
        self.test_btaM_shared_node_and_cycle_detected()
        self.test_btaN_unnamed_and_duplicate_names_detected()
        self.test_btaO_debug_flags_reach_the_leaves()
        self.test_btaP_nodes_dict_is_the_tree()
        self.test_btaQ_leaf_inventory_advisory()
        self.test_btaR_codegen_is_off_unless_asked()
        self.test_btaS_hybrid_branch_is_inert()
        self.test_btaT_pieper_id_ticks_ahead_of_the_branches()

    #  ------------------------------------------------  the shipped tree

    def test_btaA_default_tree_is_sound(self):
        '''The tree ikSolver.py runs must pass every check.  This is the only
           test that looks at the default tree as a whole, and it asserts
           soundness, not shape -- re-ordering worktools keeps it passing.'''
        fs = ' bt_assembly FAIL: default tree has structural problems:\n   '
        bt, nodes = build_default_bt()
        probs = bt_problems(bt)
        self.assertEqual(probs, [], fs + '\n   '.join(probs))

    def test_btaB_default_tree_uses_every_leaf_it_builds(self):
        '''make_leaves() exists to serve the default tree, so a node in the dict
           that is not IN the tree means someone built a leaf and forgot to wire
           it up.  (Checks reachability, not position.)'''
        fs = ' bt_assembly FAIL'
        bt, nodes = build_default_bt()
        in_tree = bt_nodes(bt)
        for key, node in sorted(nodes.items()):
            self.assertTrue(any(n is node for n in in_tree),
                            fs + ' (node "%s" (%s) is in the make_leaves dict but '
                            'is not reachable from the root -- never ticks)'
                            % (key, node.__class__.__name__))

    def test_btaC_reordered_worktools_is_sound(self):
        '''worktools order IS the solver preference policy, and changing it is a
           legitimate experiment (see build_worktools.__doc__ on promoting
           invariantGen).  A re-ordered worktools must still lint clean.'''
        fs = ' bt_assembly FAIL: reordered worktools rejected:\n   '
        nodes = make_leaves()
        wt = build_worktools(nodes)
        self.assertTrue(isinstance(wt, b3.Composite),
                        ' bt_assembly FAIL (worktools is not a composite)')

        reversed_wt = b3.Priority(list(reversed(wt.children)))
        reversed_wt.Name = 'Work Tools (reversed)'
        probs = bt_problems(alt_tree(nodes, reversed_wt))
        self.assertEqual(probs, [], fs + '\n   '.join(probs))

    def test_btaD_alternative_shape_is_sound(self):
        '''Same leaves, different topology -- the case the old version of this
           test failed by design.'''
        fs = ' bt_assembly FAIL: alternative tree shape rejected:\n   '
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        probs = bt_problems(alt_tree(nodes, wt))
        self.assertEqual(probs, [], fs + '\n   '.join(probs))

    #  ------------------------------------------------  the error cases
    #
    #  Each of these breaks a tree one way and asserts the linter says so.  A
    #  broken tree usually trips several checks at once, so they assert that the
    #  RELEVANT complaint is present, not that it is the only one.

    def has(self, probs, *needles):
        for needle in needles:
            self.assertTrue(any(needle in x for x in probs),
                            ' bt_assembly FAIL: expected a problem mentioning '
                            '"%s", got:\n   %s' % (needle, '\n   '.join(probs)))

    def test_btaE_no_root_detected(self):
        '''A BehaviorTree with no root ticks nothing at all.'''
        bt = b3.BehaviorTree()
        self.has(bt_problems(bt), 'no root')

    def test_btaF_empty_composite_detected(self):
        '''A childless Priority always FAILs and a childless Sequence always
           SUCCEEDs -- both without a word of complaint from b3, which is the
           worst way for a mis-assembled tree to behave.'''
        nodes = make_leaves()

        empty_pri = b3.Priority([])
        empty_pri.Name = 'empty worktools'
        self.has(bt_problems(alt_tree(nodes, empty_pri)),
                 'empty Priority', 'always FAILs')

        nodes = make_leaves()
        empty_seq = b3.Sequence([])
        empty_seq.Name = 'empty sequence'
        wt = b3.Priority(build_worktools(nodes).children + [empty_seq])
        wt.Name = 'Work Tools'
        self.has(bt_problems(alt_tree(nodes, wt)),
                 'empty Sequence', 'always SUCCEEDs')

        #  and an empty OrNode, which is the same bug in the tan/sincos slot
        nodes = make_leaves()
        empty_or = b3.OrNode()
        empty_or.Name = 'empty ornode'
        wt = b3.Priority(build_worktools(nodes).children + [empty_or])
        wt.Name = 'Work Tools'
        self.has(bt_problems(alt_tree(nodes, wt)), 'empty OrNode')

    def test_btaG_empty_decorator_detected(self):
        '''b3.Decorator defaults .child to [] and RepeatUntilSuccess returns
           b3.ERROR on a missing child -- ERROR is not FAILURE, so the parent
           composite treats it as success and the pass looks fine.'''
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)

        orphan = b3.RepeatUntilSuccess(None, 3)
        orphan.Name = 'loop with no child'
        bt.root = b3.Sequence([orphan, bt.root])
        bt.root.Name = 'root sequence'
        self.has(bt_problems(bt), 'empty RepeatUntilSuccess', 'no child')

    def test_btaH_class_instead_of_instance_detected(self):
        '''b3.Sequence([tan_id, tan_solve]) -- forgetting the parens -- builds a
           tree that only blows up later, deep in a tick, with an unhelpful
           AttributeError.'''
        nodes = make_leaves()
        bad = b3.Sequence([tan_id, tan_solve])       # classes, not instances!
        bad.Name = 'tan (mis-wired)'
        wt = b3.Priority(build_worktools(nodes).children + [bad])
        wt.Name = 'Work Tools'
        self.has(bt_problems(alt_tree(nodes, wt)),
                 'bad child', 'class, not instance')

        #  and outright junk in a child slot
        nodes = make_leaves()
        junk = b3.Sequence(['solve it please'])
        junk.Name = 'junk'
        wt = b3.Priority(build_worktools(nodes).children + [junk])
        wt.Name = 'Work Tools'
        self.has(bt_problems(alt_tree(nodes, wt)), 'is not a b3 node')

    def test_btaI_missing_solver_detected(self):
        '''Dropping a solver silently costs IKBT whole robots.  Every entry in
           REQUIRED_SOLVERS / REQUIRED_SUPPORT is checked this way.'''
        for cls in REQUIRED_SOLVERS + REQUIRED_SUPPORT:
            nodes = make_leaves()
            wt = build_worktools(nodes)
            wt.Name = 'Work Tools'
            bt = alt_tree(nodes, wt)

            #  prune every node of this class out of the tree
            for node in bt_nodes(bt):
                if isinstance(node, b3.Composite):
                    node.children = [k for k in node.children
                                     if not isinstance(k, cls)]
            probs = bt_problems(bt, required_support=REQUIRED_SUPPORT)
            self.assertTrue(any(cls.__name__ in x and 'missing' in x
                                for x in probs),
                            ' bt_assembly FAIL: removing %s was not reported as '
                            'missing.  Got:\n   %s'
                            % (cls.__name__, '\n   '.join(probs)))

    def test_btaJ_unpaired_and_misordered_leaves_detected(self):
        '''An ID leaf stashes blackboard state that its solver consumes, so a
           solver without its ID, or ahead of its ID, is a real defect.'''
        #  solver present, ID gone
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        nodes['algSol'].children = [nodes['algSolver']]      # ID dropped
        self.has(bt_problems(bt), 'unpaired leaf', 'algebra_id')

        #  ID present, solver gone
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        nodes['Simu_Eqn_Sol'].children = [nodes['SimuEqnID']]
        self.has(bt_problems(bt), 'unpaired leaf', 'simu_solver')

        #  both present, wrong order
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        nodes['tanSol'].children = [nodes['tanSolver'], nodes['tanID']]
        self.has(bt_problems(bt), 'ordering', 'tan_id', 'tan_solve')

        #  both present but never sequenced together (siblings under a Priority,
        #  so the solver can be reached on a tick where the ID did not run)
        nodes = make_leaves()
        wt = b3.Priority([nodes['tanID'], nodes['tanSolver'],
                          nodes['algSol'], nodes['sc_tan'],
                          nodes['Simu_Eqn_Sol'], nodes['sacSol'],
                          nodes['x2z2_Solver']])
        wt.Name = 'Work Tools'
        nodes['tanSol'].children = []              # move them out of the Sequence
        self.has(bt_problems(alt_tree(nodes, wt)), 'ordering')

    def test_btaK_missing_rank_detected(self):
        '''tan_solve and sincos_solve leave set_solved() to the rank leaf
           (tan_solver.py:338).  Without rank they solve and discard.'''
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        nodes['sc_tan'].children = [nodes['sc_tan'].children[0]]   # rank dropped
        self.has(bt_problems(bt), 'missing rank', 'set_solved')

    def test_btaL_bad_loop_budget_detected(self):
        '''max_loop == 0 never runs the child;  max_loop < 0 is unbounded, which
           turns "no progress" into a hang instead of a report.'''
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        bt.root.max_loop = 0
        self.has(bt_problems(bt), 'max_loop == 0')

        bt.root.max_loop = -1
        self.has(bt_problems(bt), 'unbounded')

    def test_btaM_shared_node_and_cycle_detected(self):
        '''b3 keeps per-node state on the blackboard keyed by node id, so the
           same INSTANCE in two places is not two nodes -- it is one node whose
           open/close state the two positions fight over.  A cycle is the same
           mistake taken further:  the tick recurses forever.'''
        nodes = make_leaves()
        wt = b3.Priority(build_worktools(nodes).children + [nodes['algSol']])
        wt.Name = 'Work Tools'                     # algSol wired in twice
        self.has(bt_problems(alt_tree(nodes, wt)), 'shared node')

        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        wt.children = list(wt.children) + [bt.root]      # root under itself
        self.has(bt_problems(bt), 'cycle')

    def test_btaN_unnamed_and_duplicate_names_detected(self):
        '''Name is what the tick log and logs/* show.  An unnamed leaf prints as
           a bare class, and two leaves sharing a Name are indistinguishable in
           a 60-tick trace -- which is exactly when you are reading the trace.'''
        nodes = make_leaves()
        wt = build_worktools(nodes)
        wt.Name = 'Work Tools'
        bt = alt_tree(nodes, wt)
        nodes['tanID'].Name = ''
        self.has(bt_problems(bt), 'unnamed node', 'tan_id')
        nodes['tanID'].Name = 'Tangent ID'

        nodes['scID'].Name = nodes['sacID'].Name = 'Sin Cos ID'
        self.has(bt_problems(bt), 'duplicate Name')

    #  ------------------------------------------------  the make_leaves contract

    def test_btaO_debug_flags_reach_the_leaves(self):
        '''make_leaves(leaf_debug=, solver_debug=) replaced ~200 lines of
           commented-out debug blocks in ikSolver.py.  If the arguments stop
           arriving, debugging quietly does nothing.'''
        fs = ' bt_assembly debug flag FAIL'
        nodes = make_leaves(leaf_debug=True, solver_debug=False)
        self.assertTrue(nodes['tanID'].BHdebug, fs + ' (leaf_debug ignored)')

        nodes = make_leaves(leaf_debug=False, solver_debug=True)
        self.assertTrue(nodes['tanSolver'].BHdebug, fs + ' (solver_debug ignored)')

    def test_btaP_nodes_dict_is_the_tree(self):
        '''build_default_bt() hands back the node dict so a caller can set
           BHdebug (or invariantGen.enabled) on the instances the tree really
           holds.  If it ever handed back copies, every such flag would silently
           do nothing -- so check by identity, not by shape.'''
        fs = ' bt_assembly node sharing FAIL'
        bt, nodes = build_default_bt()
        in_tree = bt_nodes(bt)

        target = nodes['tanID']
        found = [n for n in in_tree if n is target]
        self.assertEqual(len(found), 1,
                         fs + ' (nodes["tanID"] is not the instance in the tree)')

        target.BHdebug = True
        self.assertTrue(found[0].BHdebug,
                        fs + ' (flag set via the nodes dict did not reach the tree)')
        target.BHdebug = False

        #  the documented nodes= path must reuse the caller's instances too
        pre = make_leaves()
        pre['invariantGen'].enabled = True
        bt2, nodes2 = build_default_bt(nodes=pre)
        self.assertIs(nodes2['invariantGen'], pre['invariantGen'], fs)
        self.assertTrue([n for n in bt_nodes(bt2)
                         if n is pre['invariantGen']][0].enabled,
                        fs + ' (build_default_bt(nodes=...) did not use the '
                        'caller\'s nodes)')

    def test_btaQ_leaf_inventory_advisory(self):
        '''ADVISORY, does not fail:  list any b3 node class in ikbtleaves/ that
           this file does not classify as required or optional.  A new leaf
           showing up here is the prompt to decide whether the default tree
           should require it -- not a reason to break the suite for someone
           mid-experiment.'''
        known = set(REQUIRED_SOLVERS) | set(REQUIRED_SUPPORT) | set(OPTIONAL_LEAVES)
        unclassified = {cls: mod for cls, mod in discovered_leaf_classes().items()
                        if cls not in known}
        if unclassified:
            print('\n  bt_assembly ADVISORY: leaf classes not classified in '
                  'tests/bt_assembly_test.py:')
            for cls, mod in sorted(unclassified.items(), key=lambda kv: kv[0].__name__):
                print('     %-22s (ikbtleaves/%s.py)' % (cls.__name__, mod))
            print('  Add each to REQUIRED_SOLVERS, REQUIRED_SUPPORT, or '
                  'OPTIONAL_LEAVES.')
        else:
            print('  bt_assembly: all ikbtleaves node classes are classified.')

    #  ------------------------------------  what the shipped tree promises

    def test_btaR_codegen_is_off_unless_asked(self):
        '''Building a tree must have NO file side effects unless the caller asks.

           tests/test_chair_helper.py runs a complete solve and documents that
           it leaves LaTex/ and CodeGen/ alone;  so does every structural test
           in this file.  A codegen leaf that fired by default would silently
           overwrite the repo's generated artifacts from inside the unit
           suite.'''
        fs = ' bt_assembly codegen FAIL'

        bt, nodes = build_default_bt()
        gens = [n for n in bt_nodes(bt) if isinstance(n, output_gen_full)]
        self.assertEqual(len(gens), 1, fs + ' (expected exactly one codegen leaf)')
        self.assertFalse(gens[0].enabled,
                         fs + ' (codegen is ON by default -- it must not be)')

        bt, nodes = build_default_bt(codegen=True)
        gens = [n for n in bt_nodes(bt) if isinstance(n, output_gen_full)]
        self.assertTrue(gens[0].enabled, fs + ' (codegen=True did not enable it)')

        #  ... and the opt-in must survive the documented nodes= path, which is
        #  where it would be easy to drop it.
        pre = make_leaves()
        bt, nodes = build_default_bt(nodes=pre, codegen=True)
        self.assertTrue(pre['outputGen'].enabled,
                        fs + ' (codegen=True lost through nodes=)')

    def test_btaS_hybrid_branch_is_inert(self):
        '''The hybrid branch must not change any outcome while it is a stub.

           The whole point of building the branch before the behavior is that
           the restructure can be proven to move nothing (see
           scripts/robot_baseline.py --diff).  That rests on the stub always
           FAILing, so the enclosing Priority falls through as though the branch
           were not there.'''
        fs = ' bt_assembly hybrid FAIL'

        bt, nodes = build_default_bt()
        stubs = [n for n in bt_nodes(bt) if isinstance(n, hybrid_stub)]
        self.assertEqual(len(stubs), 1, fs + ' (expected exactly one hybrid stub)')

        t = b3.BehaviorTree()
        t.root = stubs[0]
        self.assertEqual(t.tick('hybrid stub', b3.Blackboard()), b3.FAILURE,
                         fs + ' (the stub must always FAIL)')

        #  The gate is the symbolic loop's status, so the loop must be the thing
        #  the Priority chooses on -- exactly one, with a finite budget.
        loops = [n for n in bt_nodes(bt) if isinstance(n, symbolic_loop)]
        self.assertEqual(len(loops), 1, fs + ' (expected exactly one solve loop)')
        self.assertTrue(loops[0].max_loop > 0,
                        fs + ' (solve loop has no usable budget)')
        self.assertFalse(loops[0].require_complete,
                         fs + ' (require_complete ON would discard partial '
                         'solves, which IKBT has always reported)')


    def test_btaT_pieper_id_ticks_ahead_of_the_branches(self):
        '''pieper_id must sit OUTSIDE the branch Priority, ahead of it.

           Its LaTeX statement goes into the report whichever branch produced
           the solution.  Placed inside either branch it would only tick when
           that branch ran -- so the 18 robots that solve symbolically would
           silently get no geometry section.  Position is normally not this
           file\'s business, but here position IS the behaviour.'''
        fs = ' bt_assembly pieper_id placement FAIL'

        bt, nodes = build_default_bt()
        root = bt.root
        self.assertTrue(isinstance(root, SEQUENCE_TYPES),
                        fs + ' (root must be a Sequence: pieper_id then branches)')

        kids = [k for k in child_slots(root) if isinstance(k, b3.BaseNode)]
        self.assertTrue(kids, fs + ' (empty root)')
        self.assertTrue(isinstance(kids[0], pieper_id),
                        fs + ' (pieper_id must be the FIRST thing ticked, got %s)'
                        % kids[0].__class__.__name__)

        #  ... and it must NOT also live inside a branch
        memo = {}
        for kid in kids[1:]:
            self.assertNotIn(pieper_id, _subtree_classes(kid, memo),
                             fs + ' (pieper_id also appears inside the branches)')

        #  exactly one instance, and it must always SUCCEED or the Sequence
        #  aborts before any solving happens
        found = [n for n in bt_nodes(bt) if isinstance(n, pieper_id)]
        self.assertEqual(len(found), 1, fs + ' (expected exactly one pieper_id)')
        t = b3.BehaviorTree()
        t.root = found[0]
        self.assertEqual(t.tick('pieper_id on an empty blackboard', b3.Blackboard()),
                         b3.SUCCESS,
                         fs + ' (must SUCCEED even with no Robot -- a FAILURE '
                         'here would abort the whole solve)')


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver013)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    run_test()
