#!/usr/bin/python
#
#   expected.py --  ONE table of what each robot is known to deliver, and one
#                   rule for which files a finished solve owes the user.
#
#   It replaces three hand-maintained tables over overlapping sets of the same
#   robots, which had to agree and had no mechanism to.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os


###############################################################################
#
#    What each robot is known to deliver
#
#  name -> (good, total)
#
#    good   how many returned solution branches must reproduce the probe pose
#    total  how many branches the robot is known to return
#
#  BOTH NUMBERS, not just `good`.  `good` alone catches a solution going wrong;
#  `total` catches a SPURIOUS BRANCH COMING BACK, which is a real defect that
#  leaves `good` untouched -- Chair_Helper went from 2-of-4 to 2-of-2 when the
#  branch that was wrong at every pose stopped being generated, and a
#  `good >= 2` test passes either way.
#
#  100% IS NOT A RULE.  IKBT enumerates version combinations without filtering
#  spurious ones, so a robot legitimately scoring less than 100% is recorded
#  exactly as it measures -- (2, 4) for Craig417.  See KNOWN INCOMPLETE below.
EXPECT = {
    'Puma':         (8, 8),
    'Stanford':     (8, 8),
    'Khat6DOF':     (8, 8),
    'Olson13':      (4, 4),
    'Brad':         (1, 1),
    'Wrist':        (2, 2),
    'Chair_Helper': (2, 2),

    #  Measured by check_solution_sets (the symbolic version matrix), not yet
    #  by the generated-code checker.  The two counts agree by construction --
    #  the generator emits one branch per row of solListMatrix -- so this is
    #  expected to hold, and the first full sweep will say so either way.
    'ICP5p5_A21':   (1, 1),

    #  HYBRID.  `total` is the number of Phase I seeds, `good` the number that
    #  reach the TRUE arm after Phase II.  All eight converge, from a raw seed
    #  error of 43-57 mm down to 1e-12..1e-9 in 4 or 5 iterations.
    'KinovaLite':   (8, 8),

    #  --------------------------------------------------------------------
    #  Added 2026-09-05, from the first full 32-robot closed-loop sweep.
    #  Nothing had ever measured these:  the old table covered 8 robots and
    #  was written by hand.
    'Bartell':      (2, 2),
    'Minder13':     (2, 2),
    'Frei13':       (4, 4),
    'Srisuan11':    (4, 4),
    'KawasakiRS007L': (8, 8),

    #  HYBRID, and new.  Panda is one of the three arms the old Pieper gate
    #  turned away;  it now solves 6/6 on the derived arm and all eight refined
    #  poses reach the true arm.
    'Panda':        (8, 8),

    #  --------------------------------------------------------------------
    #  KNOWN INCOMPLETE.  These are floors, not targets:  IKBT does not filter
    #  spurious version combinations, so some branches legitimately do not
    #  reproduce the pose.  When one is FIXED the check fires and the entry is
    #  updated -- that is the intended workflow.
    #
    #  The diagnosis to reach for:  look for an arcsin or arccos in the robot's
    #  solvemethods, and check whether a sin+cos pair for that variable was
    #  available -- see the ordering note in bt_assembly.build_worktools().
    'Sims11':       (1, 2),
    'Wachtveitl':   (1, 2),
    'Palm13':       (1, 2),
    'Axtman13':     (1, 2),
    'Craig417':     (2, 4),
    'MiniDD':       (1, 4),

    'Mackler13':    (1, 4),

    #  Parkman13 is mostly wrong and unexplained.
    'Parkman13':    (1, 4),
}

#  FIVE ROBOTS SOLVE COMPLETELY AND THEIR GENERATED CODE CANNOT BE RUN.  They are
#  deliberately NOT in EXPECT:  an entry would make the gate fail forever on a
#  defect that is already recorded.  Measured 2026-09-05, all with tex+py+cpp
#  written and status 'solved':
#
#      Arm_3          UnboundLocalError: th_23v1  -- the solution CONTAINS the
#      JennyGuoSp24   UnboundLocalError: th_3v1      variable it solves for
#      UR5            asin/acos out of range, AND the same self-reference
#      KR16           sqrt(): expected a nonnegative input, got -202.2
#      DZhang         asin/acos: expected a number in range -1..1
#
#  DZhang's `h` is a REAL dh parameter (declared in sp.var, in params, pvals[h]=1)
#  and is not the problem;  its failure is the asin/acos range error alone.
#  Mackler13 was on this list for a stray `h` of its own -- that one was a typo
#  in the DH table, is fixed, and it now checks 1 of 4.
#
#  Move a robot into EXPECT the moment its generated code runs.
UNCHECKABLE = ['Arm_3', 'JennyGuoSp24', 'KR16', 'UR5', 'DZhang']


def judge_counts(name, good, total):
    '''Complaints about one robot's closed-loop result.  Empty list = fine.

       A robot with no entry here is unjudged -- it is measured and reported,
       and nothing is asserted.  That is deliberate: most of ROBOT_LIST does not
       solve completely, and "does not solve" is a legitimate outcome, not a
       failure.'''

    want = EXPECT.get(name)
    if want is None:
        return []

    want_good, want_total = want
    bad = []
    if good < want_good:
        bad.append('%d of %d branches reproduce the pose, expected %d'
                   % (good, total, want_good))
    if total != want_total:
        #  Named separately from the count: more branches is not "better", it
        #  means the solution set changed shape and the extras are unvalidated.
        bad.append('returned %d branches, expected %d' % (total, want_total))
    return bad


###############################################################################
#
#    What a finished solve owes the user, and where it lands
#

#  Five kinds, because the two paths write different sets and WHICH set appeared
#  is the thing worth asserting.
#
#      tex     the report                    both paths
#      py      closed-form IK                symbolic path, and the DERIVED arm
#                                            on the hybrid one
#      cpp     closed-form IK, C++           symbolic path only
#      hybrid  the two-phase top level       hybrid path, TRUE name only
#      fk      FK (+ Jacobian) callables     hybrid path
#
#  A name here is an ARM, not a robot:  on the hybrid path the true robot and
#  the derived arm each own some of these, and both are checked.
def artifact_paths(name):
    return {
        'tex':    os.path.join('LaTex', 'ik_solution_%s.tex' % name),
        'py':     os.path.join('CodeGen', 'Python', 'IK_equations%s.py' % name),
        'cpp':    os.path.join('CodeGen', 'Cpp', 'IK_equations%s.cpp' % name),
        'hybrid': os.path.join('CodeGen', 'Python', 'IK_hybrid_%s.py' % name),
        'fk':     os.path.join('CodeGen', 'Python', 'FK_numeric%s.py' % name),
    }


NOTHING = frozenset()


def artifacts_owed(branch, complete):
    """Which artifacts a run owes, as (under the TRUE name, under the DERIVED name).

       branch    'symbolic' or 'hybrid', read from the blackboard's
                 hybrid_source -- NOT inferred from what is on disk, which
                 would make this check compare the files against themselves.
       complete  did the arm that was actually solved solve every unknown?

       A RULE, NOT A PER-ROBOT TABLE.  This used to be five hand-written entries
       in bt_path_gate.EXPECTED, so it asserted nothing about the other 27
       robots.  The contract is not per-robot -- it follows from the path and
       from whether the solve finished -- so writing it once applies it to
       every robot in the sweep.

       NOTHING when the solve did not complete.  symbolic_loop carries
       require_complete = True on both instances:  a closed form for SOME of the
       joints is not inverse kinematics, so the branch FAILs, the outer Sequence
       aborts, and report_gen never ticks.

       NO 'py' AND NO 'cpp' UNDER THE TRUE NAME ON THE HYBRID PATH.  There is no
       closed form for that robot -- that is why the path was taken -- so a file
       claiming to be one would be a simplified arm's equations shipped under
       the real robot's name.  That is the one thing this method must never do,
       and it is why the sets are compared EXACTLY: an unexpected artifact is a
       failure, not just a missing one."""

    if not complete:
        return NOTHING, NOTHING
    if branch == 'hybrid':
        return frozenset({'tex', 'hybrid', 'fk'}), frozenset({'py', 'fk'})
    return frozenset({'tex', 'py', 'cpp'}), NOTHING


def fresh_since(paths, t0):
    """Which of `paths` this run actually wrote, as a sorted list of keys.

       FRESHNESS, not existence.  CodeGen/ and LaTex/ carry artifacts from
       earlier sessions for most of these robots, so "the file is there" proves
       nothing.

       Timed against the solve's own start rather than a before/after snapshot,
       because the DERIVED arm's name is not known until install_simplified has
       run and its files therefore cannot be snapshotted in advance.  One rule
       for both names beats two rules that could disagree."""

    got = []
    for k in sorted(paths):
        try:
            if os.path.getmtime(paths[k]) >= t0:
                got.append(k)
        except OSError:
            pass                     # not there at all: not written
    return got


def artifact_complaints(got, want, where):
    '''What is wrong with one name-space's artifact set.'''

    out = []
    missing = sorted(set(want) - set(got))
    extra = sorted(set(got) - set(want))
    if missing:
        out.append('did not write %s %s' % (', '.join(missing), where))
    if extra:
        #  The loud one.  Named separately from "missing" because an unexpected
        #  artifact is not an omission, it is a claim:  a file that says it is
        #  the IK of a robot IKBT could not solve.
        out.append('WROTE UNEXPECTED %s %s' % (', '.join(extra), where))
    return out
