#!/usr/bin/python
#
#   expected.py --  ONE table of what each robot is known to deliver, and one
#                   rule for which files a finished solve owes the user.
#
#   It replaces three hand-maintained tables over overlapping sets of the same
#   robots, which had to agree and had no mechanism to.
#
#   STILL LIVE (asked and answered, BH 2026-09-27).  Two importers, and they are
#   what --gate and --full judge against:
#
#       scripts/numerical_closed_loop_sol_check.py   EXPECT, judge_counts
#       scripts/robot_baseline.py                    EXPECT, artifact_paths,
#                                                    artifacts_owed
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

    #  ONE VARIABLE since 2026-09-21, HYBRID before that, and the counts happen
    #  to be the same 8 of 8 -- so the entry did not move and the reason it
    #  passes did.  It is now `total` solutions returned by the 1-D search over
    #  th_1 and `good` that reach the pose, on the TRUE arm with no DH parameter
    #  changed.  The hybrid figures it replaced: eight Phase I seeds, all
    #  converging from a raw seed error of 43-57 mm to 1e-12..1e-9 in 4 or 5
    #  iterations, on a derived arm with d_5 zeroed.
    'KinovaLite':   (8, 8),

    #  --------------------------------------------------------------------
    #  Added 2026-09-05, from the first full 32-robot closed-loop sweep.
    'Bartell':      (2, 2),
    'Minder13':     (2, 2),
    'Frei13':       (4, 4),
    'Srisuan11':    (4, 4),
    'KawasakiRS007L': (8, 8),

    #  HYBRID.  Panda solves 6/6 on the derived arm, and all eight refined
    #  poses reach the true arm.
    'Panda':        (8, 8),

    #  ONE VARIABLE.  `total` is how many solutions the 1-D search RETURNED and
    #  `good` how many of them reach the pose -- and a third thing is checked
    #  that no count can carry: whether the joint vector the probe pose was
    #  built from is among them.  C-Arm is Friedman et al.'s own arm, solved
    #  with th_2 assumed known;  all eight reproduce the pose to 1e-14 or
    #  better, and the probe is recovered.
    'C-Arm':        (8, 8),

    #  ONE VARIABLE, and both were previously beyond IKBT:  KawasakiRS05L
    #  solved nothing by either earlier path, ArmRobo reached 2 of 7 on a
    #  derived arm.  Every returned solution reaches the pose and the probe is
    #  recovered in both.  The counts are low for a 6-DOF arm and they are
    #  MEASURED, not settled:  raising the sweep from 128 samples to 2048 finds
    #  no more, so they are not a sampling artifact at this pose, but whether
    #  the arm has others elsewhere is a question about the arm.
    'ArmRobo':      (4, 4),
    'KawasakiRS05L': (2, 2),

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

    #  A DOMAIN ERROR IS DATA, NOT A FAULT.  A negative discriminant or an
    #  out-of-range arccosine means that BRANCH has no solution at that pose,
    #  so sqrt_dc() and acos_dc() give NaN and only the postures that exist
    #  come back.
    #
    #  KR16's discriminant goes negative at the probe pose (-202.2).  The four
    #  postures that do exist reproduce it to 5.6e-16, and the generated C++
    #  agrees bit for bit.  4 of 4, not 4 of 8:  the other four do not exist
    #  there, so they are not branches.
    'KR16':         (4, 4),

    #  DZhang hits an out-of-range arccosine, which acos_dc() gives as NaN.
    #  A KNOWN INCOMPLETE like the entries above, not a clean solve.  Its `h`
    #  is a REAL dh parameter and is not the problem.
    'DZhang':       (1, 2),

    'Mackler13':    (1, 4),

    #  Parkman13 is mostly wrong and unexplained.
    'Parkman13':    (1, 4),
}

#  THREE ROBOTS SOLVE COMPLETELY AND THEIR GENERATED CODE CANNOT BE RUN.  They
#  are deliberately NOT in EXPECT:  an entry would make the gate fail forever
#  on a defect that is already recorded.
#
#      Arm_3          UnboundLocalError: th_23v1  -- the solution CONTAINS the
#      JennyGuoSp24   UnboundLocalError: th_3v1      variable it solves for
#      UR5            UnboundLocalError: th_2v1      ... and so does this one
#
#  ONE DEFECT, NOT THREE, and it is the SOLVER's, not the code generator's:
#  an equation of the form th_23v1 = atan2(..., ... + a_3*sin(th_23v1 - th_2v1))
#  is not a solution for th_23v1.  No amount of domain checking helps.
#
#  This list was five (2026-09-05).  KR16 and DZhang left it on 2026-09-29 --
#  their failures were domain errors, which are now reported rather than
#  raised;  see their EXPECT entries.  UR5's recorded failure was "asin/acos
#  out of range, AND the same self-reference" -- the first half is fixed, so
#  what is left is the self-reference alone.
#
#  WHAT THE GENERATED C++ DOES WITH ALL THREE:  it compiles and runs, and
#  returns NO branches.  Every solved variable is declared up front and
#  initialised to NaN, so a self-reference reads NaN, the row is non-finite,
#  and it is dropped -- where python raises.  Quieter, and no better:  an
#  empty answer here is a wrong answer wearing "unreachable" as a disguise,
#  which is why these stay out of EXPECT in both languages.
#
#  Mackler13 was on this list for a stray `h` of its own -- that one was a
#  typo in the DH table, is fixed, and it now checks 1 of 4.
#
#  Move a robot into EXPECT the moment its generated code runs.
UNCHECKABLE = ['Arm_3', 'JennyGuoSp24', 'UR5']


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

#  Seven kinds.  The three paths write different sets, and WHICH set appeared
#  is the thing worth asserting:  a path that emits some of what it owes and
#  not the rest is a half-finished path.
#
#      tex          the report                    every path
#      py           closed-form IK                symbolic path, and the
#                                                 DERIVED arm on the hybrid one
#      hybrid       the two-phase top level       hybrid path, TRUE name only
#      onevar       the 1-D search                one-variable path
#      cond         closed form given one value   one-variable path
#      fk           FK (+ Jacobian) callables     every path
#      cpp_robot    ALL of the above, in C++      every path, TRUE name only
#
#  ONE C++ KEY, WHERE PYTHON HAS FIVE.  The C++ is one translation unit per
#  robot -- <Robot>.cpp, holding FK, Jacobian and whichever IK was found -- so
#  there is one artifact to owe however the robot was solved.  The python side
#  keeps a module per artifact because a python module is the unit of import.
#  A DERIVED arm owes no C++ of its own:  its sections live inside the true
#  robot's file.
#
#  A name here is an ARM, not a robot:  on the hybrid path the true robot and
#  the derived arm each own some of these, and both are checked.
def artifact_paths(name, robot=None):
    #  robot is the ROBOT THE USER ASKED ABOUT;  name is the arm the file
    #  describes.  They differ only on the hybrid path, and only the C++
    #  cares:  python files are flat in CodeGen/Python/, but the C++ is one
    #  directory per robot, and the derived arm has no C++ file of its own.
    robot = robot or name
    cppdir = os.path.join('CodeGen', 'Cpp', '%sCppCode' % robot)
    return {
        'tex':    os.path.join('LaTex', 'ik_solution_%s.tex' % name),
        'py':     os.path.join('CodeGen', 'Python', 'IK_equations%s.py' % name),
        'hybrid': os.path.join('CodeGen', 'Python', 'IK_hybrid_%s.py' % name),
        'onevar': os.path.join('CodeGen', 'Python', 'IK_onevar%s.py' % name),
        'cond':   os.path.join('CodeGen', 'Python', 'IK_conditional%s.py' % name),
        'fk':     os.path.join('CodeGen', 'Python', 'FK_numeric%s.py' % name),

        #  The C++, all of it.  Named for the ROBOT, not for this arm:  on the
        #  hybrid path the derived arm has no file of its own, it is sections
        #  inside the true robot's.  See ikbtfunctions/output_cpp_robot.py.
        'cpp_robot': os.path.join(cppdir, '%s.cpp' % robot),
    }


NOTHING = frozenset()


def artifacts_owed(branch, complete):
    """Which artifacts a run owes, as (under the TRUE name, under the DERIVED name).

       branch    'symbolic', 'onevar' or 'hybrid', read from the blackboard's
                 onevar_source / hybrid_source -- NOT inferred from what is on
                 disk, which would make this check compare the files against
                 themselves.
       complete  did the arm that was actually solved solve every unknown?

       A RULE, NOT A PER-ROBOT TABLE.  What to expect is not per-robot -- it
       follows from the path and from whether the solve finished -- so
       writing it once applies it to every robot in the sweep.

       NOTHING when the solve did not complete.  symbolic_loop carries
       require_complete = True on both instances:  a closed form for SOME of the
       joints is not inverse kinematics, so the branch FAILs, the outer Sequence
       aborts, and report_gen never ticks.

       NO 'py' UNDER THE TRUE NAME ON THE HYBRID PATH.  There is no closed form
       for that robot -- that is why the path was taken -- so a file claiming to
       be one would be a simplified arm's equations shipped under the real
       robot's name.  C++ says the same thing with a FUNCTION name: inside
       <True>.cpp the derived arm's closed form is ikin_<Derived>(), and there
       is no plain ikin().  That is the one thing this method must never do,
       and it is why the sets are compared EXACTLY: an unexpected artifact is a
       failure, not just a missing one.

       THE SAME ON THE ONE-VARIABLE PATH, for a different reason.  There the
       equations ARE this robot's -- nothing was approximated -- but they hold
       only where the assumed value is right, so they ship as 'cond'
       (IK_conditional<name>), never as 'py'.  'onevar' is the search that
       makes them usable and is the entry point;  'fk' is what the search
       measures against.

       'fk' AND 'cpp_robot' ARE OWED ON EVERY PATH.  The report has a Forward
       Kinematics section and a Jacobian section whichever path wrote it, so
       the generated code has them on every path too.

       'cpp_robot' IS THE WHOLE C++ DELIVERABLE:  one file per robot, holding
       FK, Jacobian and whichever IK was found.  Which path produced it is in
       the file's own SOLUTION_PATH constant, which detect_path() reads."""

    if not complete:
        return NOTHING, NOTHING
    if branch == 'hybrid':
        #  The derived arm owes python only:  its C++ is inside cpp_robot.
        return (frozenset({'tex', 'hybrid', 'fk', 'cpp_robot'}),
                frozenset({'py', 'fk'}))
    if branch == 'onevar':
        return (frozenset({'tex', 'onevar', 'cond', 'fk', 'cpp_robot'}),
                NOTHING)
    return frozenset({'tex', 'py', 'fk', 'cpp_robot'}), NOTHING


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
