#!/usr/bin/python
#
#   output_cpp_robot.py --  ONE C++ FILE PER ROBOT
#
#   CodeGen/Cpp/<Robot>CppCode/<Robot>.cpp holds everything specific to that
#   robot -- forward kinematics, Jacobian, and whichever inverse kinematics the
#   behavior tree managed to produce -- inside `namespace ikbt::<Robot>`.
#   Everything generic stays in Cpp_src/, reached by a relative #include.  So
#   an application asks one question and gets one answer:
#
#       #include "CodeGen/Cpp/PumaCppCode/Puma.cpp"
#       ikbt::Puma::ikin(T);
#
#   THE NAMESPACE IS THE QUALIFICATION.  Inside `namespace ikbt::Panda` the
#   true arm's functions drop the `_Panda` a caller would otherwise have to
#   type twice:  fk(), jacobian(), ikin(), ikin_given(), solve(),
#   refine_seed().  A DERIVED arm sharing that namespace keeps its suffix --
#   fk_Panda_a_3_0_a_4_0() -- because it is a different robot and a reader
#   should not have to look up which is which.
#
#   The namespace is also what lets two robots be linked into one program.  A
#   plain `ikin()` at global scope would collide as soon as two arms of equal
#   DOF met in the same executable.
#
#   BODY FIRST, FILE SECOND.  Every section is rendered into a buffer before
#   the file is opened, because open(path, 'w') truncates immediately:  a
#   generator that raises half way through -- an unresolved pval, an
#   expression C++ cannot say -- would otherwise leave a broken file where a
#   good one was.
#
#   WHAT STAYS OUT:  fkOnly.py writes FK_numeric<Robot>.h and not this file.
#   It knows only the forward kinematics, so letting it write <Robot>.cpp
#   would mean that running it after a solve quietly threw that solve's
#   inverse kinematics away.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import glob
import os
from io import StringIO

import ikbtfunctions.output_cpp as oc
import ikbtfunctions.output_cpp_hybrid as och
import ikbtfunctions.output_cpp_onevar as oco
from ikbtfunctions.output_cpp_common import (cpp_identifier, fk_body_cpp,
                                             file_header, robot_dir,
                                             src_includes)


#  The Cpp_src/ headers each path needs, in dependency order -- types,
#  linalg, pose_error, dls, search.
#
#  The symbolic path includes ikbt_pose_error.h although it does not use it.
#  It is 120 lines, and it is what the first thing anyone does with a
#  closed-form answer needs:  round-trip it through fk() and ask how far off
#  it is.  A generated file should be able to check its own work.
SYMBOLIC_HEADERS = ['ikbt_types.h', 'ikbt_pose_error.h']


def _assemble(name, dirname, what, headers, sections, main_text, solved_by,
              std_includes=('cstdio',)):
    '''Render the sections, then write <name>.cpp around them.

       sections   (emit, required) pairs, emitted in order.  Since this is one
                  translation unit, that order is declaration order:  a
                  section may only call what an earlier one defined.
       main_text  the file's one self-test, which sits OUTSIDE the namespaces
                  because main() has to be at global scope

       A SECTION THAT IS NOT REQUIRED MAY FAIL AND BE SKIPPED, with a warning.
       There is one:  the symbolic path's forward kinematics, which needs
       every parameter to have a numeric value.  A robot with incomplete pvals
       is still a perfectly good symbolic solve, and losing its closed form
       over an extra section would be the wrong trade.  Everywhere else the
       sections call each other -- the 1-D search measures the closed form
       against fk() -- so a file missing one would not compile;  there the
       exception is allowed out and no file is written at all.

       Returns the path written.'''

    dirname = robot_dir(name, dirname)
    ident = cpp_identifier(name)

    #  Everything into a buffer first -- see the module header.
    body = StringIO()
    for emit, required in sections:
        part = StringIO()
        try:
            emit(part)
        except Exception as e:
            if required:
                raise
            print('   %s: section skipped -- %s: %s'
                  % (name, type(e).__name__, e))
            continue
        body.write(part.getvalue())

    _sweep_superseded(dirname)

    filename = '%s.cpp' % name
    path = os.path.join(dirname, filename)
    with open(path, 'w') as f:
        print(file_header(what, name, filename), file=f)
        print(src_includes(headers, dirname), file=f)
        print('', file=f)
        for inc in std_includes:
            print('#include <%s>' % inc, file=f)
        print('', file=f)
        print('//  ONE NAMESPACE PER ROBOT, so two robots can be linked into', file=f)
        print('//  one program:  ikbt::%s::ikin() and ikbt::Other::ikin()' % ident,
              file=f)
        print('//  are different functions.  Nested inside ikbt, so Mat4,', file=f)
        print('//  JointVec and pose_error() need no qualification here.', file=f)
        print('namespace ikbt {', file=f)
        print('namespace %s {' % ident, file=f)
        print('', file=f)
        print('//  WHICH OF IKBT\'s THREE SOLUTION PATHS ANSWERED THIS', file=f)
        print('//  ROBOT.  What the file contains follows from it:', file=f)
        print('//  "symbolic" has ikin();  "onevar" has ikin_given() and', file=f)
        print('//  solve();  "hybrid" has ikin_approx() and refine_seed().', file=f)
        print('const char* const SOLUTION_PATH = "%s";' % solved_by, file=f)
        f.write(body.getvalue())
        print('', file=f)
        print('}   // namespace %s' % ident, file=f)
        print('}   // namespace ikbt', file=f)
        print(main_text.replace('**IDENT**', ident), file=f)

    return path


#  Generated file names this package does not write.  A robot directory may
#  still hold some, and they are the worst kind of leftover:  they compile,
#  they look current, and nothing will ever refresh them.  Reading a stale
#  generated file is a real trap, so writing <Robot>.cpp clears them.
#
#  FK_numeric*.h is NOT on this list:  fkOnly.py writes it, and it is a live
#  artifact rather than a leftover.
SUPERSEDED = ('IK_equations*.cpp', 'IK_conditional*.cpp', 'IK_onevar*.cpp',
              'IK_hybrid_*.cpp')


def _sweep_superseded(dirname):
    """Delete any superseded generated files from this robot's directory."""

    for pattern in SUPERSEDED:
        for old in glob.glob(os.path.join(dirname, pattern)):
            try:
                os.remove(old)
                print('   removed superseded %s' % old)
            except OSError:
                pass            # read-only, or already gone;  not worth failing over


def write_symbolic_cpp(R, solution_groups, dirname=None):
    '''<Robot>.cpp for a robot solved symbolically:  FK, Jacobian, ikin(T).'''

    name = R.name.replace('test: ', '')

    def fk(f):
        print(oc.section_banner('forward kinematics and Jacobian'), file=f)
        fk_body_cpp(R.Mech, name, f, jacobian=True, owner=name)

    def ik(f):
        oc.output_cpp_code(R, solution_groups, f, owner=name)

    return _assemble(name, dirname,
                     'C++ kinematics for %s -- FK, Jacobian and a closed-form IK'
                     % name,
                     SYMBOLIC_HEADERS, [(fk, False), (ik, True)],
                     oc.MAIN_BLOCK
                     .replace('**FUNC**', 'ikin')
                     .replace('**ROBOT**', name)
                     .replace('**EXTRA_ARG**', '')
                     .replace('**KNOWN_NOTE**', ''),
                     'symbolic')


def write_onevar_cpp(R, solution_groups, known, dirname=None, n_samples=128,
                     max_samples=4096):
    '''<Robot>.cpp for a ONE-VARIABLE solve:  FK, Jacobian, the conditional
       closed form ikin_given(T, <known>), and solve(T) -- the 1-D search over
       <known>, which is the entry point.

       Nothing about the arm is approximated, so there is only one arm here
       and it owns every name in the namespace.'''

    name = R.name.replace('test: ', '')

    def fk(f):
        print(oc.section_banner('forward kinematics and Jacobian'), file=f)
        fk_body_cpp(R.Mech, name, f, jacobian=True, owner=name)

    def ik(f):
        oc.output_cpp_code(R, solution_groups, f, known=known, owner=name)

    def search(f):
        oco.write_onevar_top_cpp(R.Mech, name, known, f, n_samples=n_samples,
                                 max_samples=max_samples)

    return _assemble(name, dirname,
                     'ONE-VARIABLE inverse kinematics for %s (%s assumed known)'
                     % (name, known),
                     oco.CORE_HEADERS,
                     [(fk, True), (ik, True), (search, True)],
                     oco.MAIN, 'onevar',
                     std_includes=('cstdio', 'cstdlib'))


def write_hybrid_cpp(R, solution_groups, R_true, true_name, derived_name,
                     edits_text, cost_text, dirname=None):
    '''<Robot>.cpp for a HYBRID solve, under the TRUE arm's name.

       Two arms in one namespace:  the derived arm's FK and closed form keep
       their _<derived> suffix, the true arm's FK and Jacobian take the plain
       names, and the two phases on top are the entry point.

       R        the DERIVED robot, solved in closed form
       R_true   the TRUE robot.  Required:  without its FK and Jacobian there
                is no Phase II, and a Phase I seed on its own is not an
                answer.'''

    def derived_fk(f):
        print(oc.section_banner('the APPROXIMATE arm:  forward kinematics   (%s)'
                                % derived_name), file=f)
        fk_body_cpp(R.Mech, derived_name, f, jacobian=False, owner=true_name)

    def derived_ik(f):
        oc.output_cpp_code(R, solution_groups, f, owner=true_name)

    def true_fk(f):
        print(oc.section_banner('the TRUE arm:  forward kinematics and Jacobian'),
              file=f)
        fk_body_cpp(R_true.Mech, true_name, f, jacobian=True, owner=true_name)

    def phases(f):
        och.write_hybrid_top_cpp(R_true.Mech, true_name, derived_name,
                                 edits_text, cost_text, f)

    return _assemble(true_name, dirname,
                     'HYBRID inverse kinematics for %s (via %s)'
                     % (true_name, derived_name),
                     och.CORE_HEADERS,
                     [(derived_fk, True), (derived_ik, True),
                      (true_fk, True), (phases, True)],
                     och.MAIN, 'hybrid')
