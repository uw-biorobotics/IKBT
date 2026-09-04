#!/usr/bin/python
#
#     Classes to generate LaTex outputs
#

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import sympy as sp
import shutil as sh
import os as os
import sys as sys
import pickle
import re
import ikbtbasics.pykinsym as pks
import ikbtbasics.kin_cl as kc
from   ikbtbasics.solutionGraphV3 import *
import ikbtbasics.matching as mtch
import ikbtbasics.numeric_ik as nik   # dof_of():  the arm's REAL joint count

import b3 as b3          # behavior trees

from   ikbtfunctions.helperfunctions import *
import ikbtfunctions.graph2latex as gl
from   ikbtfunctions.subexpressions import SubexprPool
#from kin_cl import *


class LatexFile():
    def __init__(self,fname):
        self.filename = fname + '.tex'
        if self.filename.endswith('.tex.tex'):
            self.filename = self.filename[:-4]
        print('Working with Latex file: ',self.filename)
        self.preamble = [] # list of line strings
        self.sections = [] # list of lists of line strings
        self.close =    []  # list of line strings

        f = open('LaTex/IK_preamble.tex','r')
        self.preamble = f.readlines()
        f.close()
        f = open('LaTex/IK_close.tex','r')
        self.close = f.readlines()
        f.close()

    #  output the final latex file
    def output(self):
        f = open(self.filename,'w')
        plines(self.preamble,f)
        for s in self.sections:
            print('\n\n',file=f)
            plines(s,f)
        plines(self.close,f)
        f.close()


def plines(sl,f):
    for s in sl:
        print(s,end='\n',file=f)
    #print(sl, end='\n', file=f)


#
#      Generate a complete report in latex
#

#  Per BH:  every report the HYBRID method produces carries this, immediately
#  after the Introduction.  Kept as one constant rather than inlined so that
#  the wording is edited in one place and cannot drift between report kinds --
#  and so that turning it on for the symbolic reports too is a one-line change
#  at the call site, not a copy of the sentence.
AI_STATEMENT = (
    r'\paragraph{Statement of AI contributions} The hybrid solution method '
    r'used here and IKBT2 in general were produced by Blake Hannaford working '
    r'with support of Claude Code.')


#  Symbols kin_cl.forward_kinematics() INVENTS -- 'ca2', 'sa2' -- when a DH
#  twist angle is not a multiple of 90 degrees.
_GEN_ALPHA = re.compile(r'^(c|s)a(\d+)$')


def generated_alpha_defs(Robot):
    r"""The ca_i / sa_i this robot's FK invented, with what each one means.

       Returns [(name, 'cos'|'sin', alpha_expression), ...] in DH row order, or
       [] for the 30 of 32 robots that have none.

       WHY THEY EXIST (BH).  When a twist angle is a constant -- which it always
       is -- so are its sine and cosine.  Replacing sin(alpha)/cos(alpha) with a
       named constant therefore costs NO accuracy, and it buys two things:  the
       equations get shorter, and sympy has one atom to carry instead of a
       trig call to re-derive at every simplification.  That is why
       forward_kinematics() makes the substitution at all, and it only bothers
       when alpha is not a multiple of 90 degrees -- otherwise sin and cos
       already reduce to 0 or +-1 and there is nothing to gain.

       WHY THE REPORT MUST DECLARE THEM.  They are real parameters, in
       Mech.params and Mech.pvals, but they are created deep inside the FK
       computation and nothing has ever declared them to the reader.  Worse,
       sympy's LaTeX printer renders Symbol('sa2') as sa_{2}, which sits in the
       report looking like a member of the a_2 / d_2 family of LINK LENGTHS.
       On Craig417 that is actively misleading:  sa_2 is sin(pi/4) = 0.707,
       while a_2 is not a parameter of that robot at all.  (Spotted by BH in
       ik_solution_Craig417.pdf, 2026-09-03.)

       Only Craig417 (alpha = pi/4) and Raven-II generate any, which is why it
       went unnoticed:  every other robot's twists are 0 or +-90 degrees."""

    mech = getattr(Robot, 'Mech', None)
    if mech is None:
        return []

    out = []
    for prm in (getattr(mech, 'params', None) or []):
        m = _GEN_ALPHA.match(str(prm))
        if not m:
            continue
        try:
            alpha = mech.DH[int(m.group(2)), 0]      # the alpha_{i-1} column
        except Exception:
            continue
        out.append((str(prm), 'cos' if m.group(1) == 'c' else 'sin', alpha))

    #  Row order, cos before sin within a row -- the order a reader scanning
    #  the DH table would meet them.
    out.sort(key=lambda t: (int(_GEN_ALPHA.match(t[0]).group(2)), t[1]))
    return out


def alpha_definition_section(Robot, eol='\n'):
    r"""Declare the ca_i / sa_i.  '' when the robot generated none."""

    defs = generated_alpha_defs(Robot)
    if not defs:
        return ''

    sec = eol + r'\subsection{Twist-angle constants}' + eol
    sec += (r'This robot has a joint twist $\alpha$ that is not a multiple of'
            r' $90^{\circ}$, so $\sin\alpha$ and $\cos\alpha$ do not reduce to'
            r' $0$ or $\pm 1$.  Since a twist angle is a constant, so are its'
            r' sine and cosine, and the forward kinematics carries each as a'
            r' named constant instead:  this shortens the equations and helps'
            r' them simplify, with no loss of accuracy.' + eol + eol)
    sec += (r'\textbf{These are trigonometric functions of a twist ANGLE, not'
            r' link lengths.}  In particular $sa_{i}$ means $\sin\alpha_{i}$'
            r' and is unrelated to the link length $a_{i}$.' + eol)

    sec += r'\begin{center}\begin{tabular}{lll}' + eol
    sec += r'Symbol & Meaning & Value (4 d.p.) \\ \hline' + eol
    pvals = getattr(getattr(Robot, 'Mech', None), 'pvals', None) or {}
    for name, kind, alpha in defs:
        row = int(_GEN_ALPHA.match(name).group(2))
        fn = r'\cos' if kind == 'cos' else r'\sin'

        #  The VALUE THAT IS ACTUALLY USED, read back from pvals rather than
        #  recomputed here -- a table that recomputed could disagree with the
        #  model it claims to describe.  Rounded to 4 decimals for the page
        #  only;  full precision is what the kinematics carries.
        val = None
        for k, v in pvals.items():
            if str(k) == name:
                try:
                    val = '%.4f' % float(v)
                except (TypeError, ValueError):
                    val = r'\mathrm{%s}' % sp.latex(sp.sympify(str(v)))
                break
        if val is None:
            try:
                val = sp.latex(sp.simplify(getattr(sp, kind)(alpha)))
            except Exception:
                val = r'\mathrm{(symbolic)}'

        sec += ('$' + sp.latex(sp.Symbol(name)) + r'$ & $'
                + fn + r'\alpha_{' + str(row) + r'} = '
                + fn + r'\left(' + sp.latex(alpha) + r'\right)$ & $'
                + val + r'$ \\' + eol)
    sec += r'\end{tabular}\end{center}' + eol
    return sec


def definition_block(defs, eol):
    r"""The `K_i = ...` definitions that go IMMEDIATELY ABOVE an equation.

       Each is its own dmath, so a definition that is still wide can break --
       and unlike the equation it was lifted out of, a definition is a plain
       sum or quotient rather than a function argument, so breqn can get at it.

       Printed above rather than below (BH):  the reader meets each name before
       the equation that uses it, and never has to scan forward to find out
       what a symbol means."""

    if not defs:
        return ''

    out = (r'Writing, for brevity,' + eol)
    for sym, e in defs:
        out += (r'\begin{dmath} '
                + str(kc.kequation(sym, e).LaTexOutput(False))
                + r' \end{dmath}' + eol)
    return out


def version_tex(vname):
    r"""One version name, typeset:  'th_23v12' -> '\theta_{23}v12'.

       THE BRACES MATTER.  A LaTeX subscript takes ONE token, so $th_23v12$
       sets only the 2 as a subscript and drops 3v12 onto the baseline --
       th_23 comes out looking like th_2 followed by a stray 3.  Every
       sum-of-angle variable has a multi-digit subscript, so this was wrong for
       exactly the variables whose identity is hardest to guess from context.

       The rest of the report runs names through the same two steps (see the
       unknowns and solutions sections);  the solution-set tables were built by
       string concatenation and never got either."""

    return theta_expand(re.sub(r'_(\d+)', r'_{\1}', vname))


def pose_columns(Robot):
    """Which solutionSet columns are real JOINTS.

       Returns (keep, joint_names, soa_names):  the column indices to show, the
       unknown names those columns hold, and the sum-of-angle names dropped.

       WHY DROP THEM.  A solution set is a set of POSES, and a pose is a value
       per joint.  th_23 is not a joint -- it is an intermediate the solver
       introduced, and it is fully determined by th_2 and th_3, which are both
       already in the row.  Showing it makes a 6-DOF arm's pose table 7 columns
       wide and invites the reader to count seven joints.

       This is the same rule the code generator already follows:  ikin_*()
       returns JOINT_NAMES and lists the sum-of-angle variables separately as
       AUX_NAMES, precisely because returning them made a 6-DOF arm come back 7
       wide.  The report and the generated code now answer "what is a pose"
       identically, which they did not before.

       Column i of a row belongs to solution_nodes[i]:  create_solution_set()
       adds one column per node, in that order.  The joints come from the DH
       table via numeric_ik.joint_symbols(), NOT from the unknown list, which
       is exactly the list that has been extended with the SOA variables."""

    order = [nd.unknown.name for nd in Robot.solution_nodes]
    jnames = [str(s) for s in nik.joint_symbols(Robot.Mech)]
    keep = [i for i, n in enumerate(order) if n in jnames]
    soa = [n for n in order if n not in jnames]
    return keep, [order[i] for i in keep], soa


def solution_rows(Robot, groups):
    """The solution set as an ORDERED list of rows.

       Robot.solListMatrix is a list and preserves the order create_solution_set()
       built;  solutionSet (which is what `groups` is) is a SET of tuples, so its
       iteration order moves with string hashing and the report stops being
       diffable from one run to the next.  The code generator was fixed to read
       solListMatrix for this reason;  the report was not, until now.

       Falls back to `groups` for a Robot that has no solListMatrix -- the V2
       callers, and any test that passes a solution set directly."""

    rows = getattr(Robot, 'solListMatrix', None)
    if rows:
        return [list(r) for r in rows]
    return [list(g) for g in groups]


def tex_name(n):
    """A robot name safe to typeset:  underscores are subscripts in math mode
       and an error in text mode, and every derived arm has several."""
    return n.replace('test: ', '').replace('_', r'\_')


def hybrid_true_arm_section(hybrid, R_true):
    r"""The part of a hybrid report that describes the arm the user ASKED for.

       hybrid   the blackboard's hybrid_source dict
       R_true   the true Robot, or None if it could not be loaded

       Returns a list of lines, or [] when there is nothing to say.

       WHY THIS SECTION EXISTS.  Everything else in the report is generated
       from Robot.Mech, and on the hybrid path that is the DERIVED arm --
       install_simplified swapped it before the second solver ran.  Without a
       section like this one the report would be a complete, correct,
       confident document about a robot nobody asked about, carrying the real
       robot's name on the title page.  That is the worst thing this method
       could produce, so the true arm is stated FIRST and the substitution is
       stated before any closed form appears."""

    if not hybrid:
        return []

    eol = '\n'
    true_name = tex_name(hybrid.get('true_robot') or '?')
    derived = tex_name(hybrid.get('derived_robot') or '?')

    sec = r'\section{The Arm This Report Solves}' + eol
    sec += r'\textbf{IKBT could not find a closed-form inverse kinematic' + eol
    sec += r'solution for ' + true_name + r'.}  What follows is therefore a' + eol
    sec += r'\emph{hybrid} result, in two parts:' + eol
    sec += r'\begin{enumerate}' + eol
    sec += (r'\item a closed form solved exactly for a \emph{simplified} arm, '
            + derived + r', which differs from the real robot in the' + eol)
    sec += r'Denavit-Hartenberg parameters listed below;  and' + eol
    sec += (r'\item a numerical correction that refines those joint values'
            r' against the real robot' + "'" + r's own forward kinematics.' + eol)
    sec += r'\end{enumerate}' + eol
    sec += (r'The closed form alone does \textbf{not} place ' + true_name +
            r' at the requested pose.  Only the corrected values do.' + eol)

    ####  what was changed, and what it cost
    sec += r'\subsection{The simplification}' + eol
    edits = hybrid.get('edits') or []
    if edits:
        sec += r'\begin{center}\begin{tabular}{lrr}' + eol
        sec += r'Parameter & Real value & Simplified value \\ \hline' + eol
        for e in edits:
            sec += (r'$' + sp.latex(sp.Symbol(str(e['symbol']))) + r'$ & ' +
                    str(e['from']) + ' & ' + str(e['to']) + r' \\' + eol)
        sec += r'\end{tabular}\end{center}' + eol

    axes = hybrid.get('axes')
    kind = hybrid.get('kind')
    if axes and kind:
        sec += (r'These changes make joint axes $' +
                ', '.join(str(a) for a in axes) + r'$ ' +
                ('intersect at a common point' if kind == 'intersect'
                 else 'mutually parallel') +
                r", which satisfies Pieper's sufficient condition for a closed"
                r' form to exist.' + eol)

    cost = hybrid.get('cost')
    if cost is not None:
        sec += (r'The simplification was chosen as the \emph{cheapest}'
                r' available, measured in task-space displacement:  the mean'
                r' of $\|\Delta p\| + w_{rot}\Delta\theta$ over sampled joint'
                r' vectors, comparing the real arm' + "'" + r's end-effector'
                r' pose with the simplified arm' + "'" + r's at the same joint'
                r' values.  Its cost is $' + ('%.4g' % float(cost)) +
                r'$, in the length units of the DH table.' + eol)

    ####  the true arm's own parameters, so the reader can compare
    if R_true is not None and getattr(R_true, 'Mech', None) is not None:
        sec += r'\subsection{Kinematic parameters of ' + true_name + r'}' + eol
        sec += (r'\[ \left [ \alpha_{i-1}, \quad a_{i-1}, \quad d_i,'
                r' \quad \theta_i \right  ] \]' + eol)
        sec += r'\begin{dmath}' + sp.latex(R_true.Mech.DH) + r'\end{dmath}' + eol

        pl = getattr(R_true, 'pieper_latex', None)
        if pl:
            sec += pl + eol

    return sec.splitlines()


def hybrid_numeric_section(hybrid, true_name):
    r"""The Phase II half of a hybrid report:  how the closed form is
       corrected, and what the generated code calls it."""

    if not hybrid:
        return []

    eol = '\n'
    ident = tex_name(true_name)

    sec = r'\section{Numerical Correction (Phase II)}' + eol
    sec += (r'The equations above solve the simplified arm exactly.  To place'
            r' the real robot at the goal pose $T_d$, the joint vector they'
            r' produce is used as a \emph{seed} for damped least squares'
            r' (Levenberg-Marquardt) against the real arm' + "'" + r's forward'
            r' kinematics:' + eol)
    sec += r'\begin{dmath}' + eol
    sec += r'\Delta q = J^{T}\left(J J^{T} + \lambda^{2} I\right)^{-1} e' + eol
    sec += r'\end{dmath}' + eol
    sec += r'with the residual and the convergence metric' + eol
    sec += r'\begin{dmath}' + eol
    sec += (r'e = \left[ \Delta p ;\; w_{rot}\,\theta\,\hat{a} \right], \qquad'
            r' m = \|\Delta p\| + w_{rot}\,\theta' + eol)
    sec += r'\end{dmath}' + eol
    sec += (r'where $\theta$ and $\hat{a}$ are the angle and axis of the'
            r' orientation error $R_d R^{T}$.  Both expressions use the same'
            r' rotation parameterisation and the same weight $w_{rot}$, so the'
            r' step and the stopping test agree about which of two poses is'
            r' closer.  The damping $\lambda$ rises on a rejected step and'
            r' falls on an accepted one, which is what lets the method pass'
            r' through a singularity, where an undamped Newton step does not'
            r' exist.' + eol)
    sec += (r'Because the seed determines which solution branch the correction'
            r' converges to, the two phases are separate entry points:  the'
            r' caller enumerates the branches, chooses one, and only then'
            r' refines it.' + eol)

    sec += r'\subsection{Generated code}' + eol
    sec += r'\begin{center}\begin{tabular}{ll}' + eol
    sec += r'{\tt IK\_hybrid\_' + ident + r'.py} & the two phases \\' + eol
    sec += (r'{\tt ikin\_' + ident + r'\_approx(T)} & Phase I: branches of the'
            r' simplified arm \\' + eol)
    sec += (r'{\tt refine\_' + ident + r'(T, index)} & Phase II: correct'
            r' branch {\tt index} \\' + eol)
    sec += r'\end{tabular}\end{center}' + eol
    sec += (r'Phase I discards candidate branches that do not reproduce $T_d$'
            r' on the simplified arm;  IKBT enumerates combinations of each'
            r" unknown's solution branches and does not itself filter the"
            r' spurious ones.  Phase II reports {\tt converged}, which can'
            r' legitimately be false:  the real arm may not reach the'
            r' requested pose from the chosen branch, or at all.' + eol)

    return sec.splitlines()


def output_latex_solution(Robot, variables, groups, hybrid=None, R_true=None):
    GRAPH = True
    '''Print out a latex document of the solution equations.

       hybrid / R_true are set on the HYBRID path, where `Robot` is the
       DERIVED arm.  The report is then NAMED FOR and TITLED WITH the true
       robot -- that is the robot the user asked about and the one the
       generated code is for -- and it gains two sections saying which arm was
       actually solved and how the answer gets corrected.  Left at None, this
       function behaves exactly as it always did.'''
    eol = '\n'
    #  ONE pool for the whole report, so every K_i is defined exactly once and
    #  means one thing in the document.  Restarting the numbering per variable
    #  would make K_3 mean something different in each subsection -- the very
    #  ambiguity that ruled out reusing `a_i`.
    pool = SubexprPool(Robot)

    solved_name = Robot.name.replace('test: ','')
    orig_name = (hybrid.get('true_robot') or solved_name) if hybrid else solved_name
    fixed_name = orig_name.replace(r'_', r'\_')
    solved_fixed = solved_name.replace(r'_', r'\_')

    DirName = 'LaTex/'
    fname = DirName + 'ik_solution_'+orig_name+'.tex'
    LF = LatexFile(fname)

    ####################   Intro Section

    introstring = r'''
    \begin{center}
    \section*{Inverse Kinematic Solution for ''' + fixed_name + r'''}
    \today
    \end{center}
    \section{Introduction}
    This report describes ''' + ('a HYBRID inverse kinematic solution for '
    if hybrid else 'closed form inverse kinematics solutions for ') + fixed_name + r'''.   The solution was generated by
    the \href{https://github.com/uw-biorobotics/IKBT}{IK-BT package}
    from the University of Washington Biorobotics Lab.
    The IK-BT package is described in
    \url{https://arxiv.org/abs/1711.05412}.
    IK-BT derives your  equations
    using {\tt Python 3.8} and the {\tt sympy 1.9} module for symbolic mathematics.
    '''

    #  Per BH, on every hybrid report, immediately after the Introduction.
    #  HYBRID ONLY, because the sentence says "the hybrid solution method used
    #  here" -- which is not true of a report for an arm that solved in closed
    #  form.  Dropping the `if` puts it on every report, if that is wanted.
    if hybrid:
        introstring += eol + AI_STATEMENT + eol

    LF.sections.append(introstring.splitlines())

    ####################  Which arm is this, really?

    #  HYBRID ONLY, and it goes SECOND on purpose -- immediately after the
    #  introduction and before any kinematics.  Everything below this point is
    #  generated from Robot.Mech, which on this path is the DERIVED arm, so a
    #  reader who met the closed form first would have no way to know it was
    #  not the robot named on the title page.
    LF.sections.append(hybrid_true_arm_section(hybrid, R_true))

    ####################   Kinematic params

    paramsection = r'''\section{Kinematic Parameters''' + (
        (r''' of the Simplified Arm ''' + solved_fixed) if hybrid else '') + r'''}
    The kinematic parameters for ''' + (solved_fixed if hybrid else 'this robot') + r''' are
    \[ \left [ \alpha_{i-1}, \quad a_{i-1}, \quad d_i, \quad \theta_i \right  ] \]
    \begin{dmath}''' + sp.latex(Robot.Mech.DH) +  r'\end{dmath}'

    #  Declare the invented twist constants HERE:  with the other kinematic
    #  parameters, and before the first equation that uses them.
    paramsection += alpha_definition_section(Robot, eol)

    LF.sections.append(paramsection.splitlines())

    ####################  Joint axis geometry (Pieper condition)

    #  Written by the pieper_id leaf, which ticks ahead of the branch split so
    #  that this statement appears whichever branch produced the solution.
    #  getattr: a Robot restored from a pickle written before this existed will
    #  not carry the attribute, and a missing statement must not break the
    #  report.  (output_FK_equations() below deliberately does not get this --
    #  fkOnly.py never ticks the BT, so nothing would have written it.)
    #  SKIPPED on the hybrid path.  hybrid_true_arm_section() above already
    #  printed this statement, for the TRUE arm, which is the arm the reader
    #  needs it for -- report_gen hands both that section and this one the same
    #  snapshot, so leaving both in printed the identical section twice.
    pieper_section = None if hybrid else getattr(Robot, 'pieper_latex', None)
    if pieper_section:
        LF.sections.append(pieper_section.splitlines())




    ####################  Forward Kinematics

    fksection = r'''\section{Forward Kinematic Equations}
    The forward kinematic equations for this robot are:'''+eol

    fksection += r'\begin{dmath} '+eol

    LHS = ik_lhs()
    RHS = kc.notation_squeeze(Robot.Mech.T_06)   # see kin_cl.mechanism.T_06

    fksection += sp.latex(LHS) + r' \\'+eol

    COLUMNS = True
    if COLUMNS:
        for c in range(4):
            fksection +=  r'\mathrm{Column \quad'+str(c+1)+'}' +eol+sp.latex(RHS[:,c]) + r'\\'+eol
    else:
        fksection += sp.latex(RHS)
    fksection += r'\end{dmath}'+eol

    fksection += 'Note: column numbers use math notation rather than python indeces.'+eol

    LF.sections.append(fksection.splitlines())

    ####################   Unknowns

    unksection = r'\section{Unknown Variables: }'+eol

    # introduce the unknowns and the solution ORDER
    unksection += r'''The unknown variables for this robot are (in solution order): ''' +eol+r'\begin{enumerate}'+eol

    for n in Robot.solution_nodes:
        unk = n.symbol
        print('\noutput_latex: latex unknown v1:',unk, type(unk))
        tmp = '$' +  sp.latex(unk) + '$'
        tmp = theta_expand(tmp)
        tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {}
        print('output_latex: latex unknown v2:',tmp)
        unksection += eol+r'\item {'+tmp+'}'

    unksection += r'\end{enumerate}'+eol

    LF.sections.append(unksection.splitlines())


    ##########################################################################
    ####################   Solutions to IK    ( convert to use Robot.solListMatrix)

    solsection = r'\section{Solutions in Generic Form} '+eol
    solsection += ''' The following equations comprise solutions for each unknown.''' + eol

    # sort the nodes into solution order
    #sorted_node_list = sorted(Robot.solution_nodes)

    Robot.make_LHS_versions() # create final equations including all dependencies, versions, solutions!

    ###################
    #  Print the generic solution equations for each unknown without doing the permuations and combinations
    #
    for node in Robot.solution_nodes:
        if node.solvemethod != '':   # skip variables (typically extra SOA's) that are not used.
            u = node.unknown
            tmp = '$' + sp.latex(node.symbol) + '$'
            tmp = theta_expand(tmp)
            varLHS = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {} for latex

            #new subsection for this variable and solution(s)
            solsection += '\n' +r'\subsection{'+varLHS+r' } '+eol + 'Solution Method: ' + u.solvemethod + eol

            #  Lift the big pieces out FIRST, then print what is left.  The
            #  definitions have to precede the whole align block, not sit
            #  between its rows, so every solution of this variable is split
            #  before any of it is printed.
            rewritten, defs = [], []
            for sol in u.solutions:
                new, newdefs = pool.split(sol)
                rewritten.append(new)
                defs += newdefs

            solsection += definition_block(defs, eol)

            nsolns = u.nsolutions
            ALIGN = nsolns > 1

            solsection += r'\begin{align}' if ALIGN else r'\begin{dmath} '

            for i, sol in enumerate(rewritten):
                thisEOL = r'\\' if (ALIGN and i < nsolns-1) else ''
                eqn = kc.kequation(sp.var(u.solutionNames[i]), sol)
                solsection += str(eqn.LaTexOutput(ALIGN)) + ' ' + thisEOL

            solsection += (r'\end{align} ' if ALIGN else r'\end{dmath} ') + eol

            solsection += eol+eol

    LF.sections.append(solsection.splitlines())

    ###################
    # print the detailed equations for each version of each variable
    solsection = r'\section{Solutions to Generate all Versions} '+eol
    solsection += ''' The following equations are the full set of solutions for each unknown
    incorporating all combinations of dependencies.''' + eol

    for node in Robot.solution_nodes:
        if node.solvemethod != '':   # skip variables (typically extra SOA's) that are not used.
            tmp = '$' + sp.latex(node.symbol) + '$'
            tmp = theta_expand(tmp)
            varLHS = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {} for latex


            #new subsection for this variable and solution
            solsection += '\n' +r'\subsection{'+varLHS+r' } '+eol + 'Solution Method: ' + node.solvemethod + eol

            colindex = node.unknown.solveorder-1  # select the unknown
            #  ONE equation per DISTINCT version.  A variable solved early has
            #  fewer versions than the matrix has rows and SHARES them between
            #  rows -- Puma's th_1 has 2 versions over 8 rows -- so walking the
            #  rows printed each of its equations four times.  Dedup on the
            #  version name (the LHS), keeping first-seen order:  list(set(..))
            #  would work but reorders the output, differently from run to run.
            eqnlist = []
            seen = set()
            for rowindex in range(Robot.nversions):
                eqn = Robot.FinalEqnMatrix[rowindex][colindex]
                if str(eqn.LHS) in seen:
                    continue
                seen.add(str(eqn.LHS))
                eqnlist.append(eqn)

            #  Same treatment as the generic solutions.  This is the section
            #  that needs it most:  a version equation has every dependency
            #  substituted into it, so it is the longest thing in the report --
            #  and the pool is shared, so a piece already named above is
            #  referenced here rather than defined a second time.
            rewritten, defs = [], []
            for eqn in eqnlist:
                new, newdefs = pool.split(eqn.RHS)
                rewritten.append(kc.kequation(eqn.LHS, new))
                defs += newdefs

            solsection += definition_block(defs, eol)

            ALIGN = True
            solsection += r'\begin{align}'
            for i, eqn in enumerate(rewritten):
                thisEOL = r'\\' if i < len(rewritten)-1 else ''
                solsection += str(eqn.LaTexOutput(ALIGN)) + ' ' + thisEOL
            solsection += r'\end{align} ' + eol

            solsection += eol+eol

    LF.sections.append(solsection.splitlines())

    ####################  List the edges of the solution graph

    edgesection = r'\section{Solution Graph (Edges)} '+eol  +  r'''
The following is the abstract representation of solution graph for this manipulator (nodes with parent -1 are roots).  Future: graphic representation. :
\begin{verbatim}
'''

    graph = Robot.notation_graph_edges

    i = 0
    sameline = '     '
    sepstr = sameline
    print('test: Starting Graph output')
    for edge in graph:
        i+=1
        if i%2==0:
            sepstr = eol
        elif i>1:
            sepstr = sameline
        print('test: edge + sepstr: [',str(edge)+sepstr,']')
        edgesection+= str(edge)+ sepstr

    edgesection +=  r'\end{verbatim} '+eol

    LF.sections.append(edgesection.splitlines())


    ####################  Solution Sets

    #  JOINTS ONLY, and in a stable row order -- see pose_columns() and
    #  solution_rows().  A pose is a value per joint;  the sum-of-angle
    #  intermediates are named below the table instead of occupying columns in
    #  it, which is the same split the generated code makes between JOINT_NAMES
    #  and AUX_NAMES.
    keep, jcols, soa = pose_columns(Robot)
    rows = solution_rows(Robot, groups)

    solsection = r'\section{Solution Set}'+eol
    solsection += (r'The following are the sets of joint solutions (poses) for'
                   r' this manipulator, one row per pose, columns in the order'
                   + eol + '$' + '$, $'.join(version_tex(c) for c in jcols)
                   + '$:' + eol)
    solsection += r'\begin{verbatim}' + eol

    for r in rows:
        solsection += str(tuple(r[i] for i in keep))+eol

    solsection += '\end{verbatim}'+eol

    if soa:
        solsection += (r'Sum-of-angle intermediates ($'
                       + '$, $'.join(version_tex(n) for n in soa)
                       + r'$) are computed on the way to these solutions and'
                       r' are fully determined by the joints above, so they are'
                       r' not columns of a pose.' + eol + eol)

    LF.sections.append(solsection.splitlines())



    ####################  Solution sets Table form
    colstr = '|' + 'l|'*len(keep)
    tablestr = (r'\section{Solution Set (table)} \begin{tabular}{' + colstr
                + r'}\hline' + eol)
    #  A HEADER ROW.  Without one the reader has to infer which joint each
    #  column holds from the version names, and solve order is not chain order,
    #  so that inference is wrong as often as not.
    tablestr += (' & '.join(r'\textbf{$' + version_tex(c) + r'$}'
                            for c in jcols) + r'\\\hline' + eol)
    for r in rows:
        cells = [version_tex(r[i]) for i in keep]
        tablestr += ' & '.join('$' + c + '$' for c in cells) + r'\\\hline' + eol
    tablestr += r'\end{tabular}'+eol

    LF.sections.append(tablestr.splitlines())


    ####################  Solution methods
     # Equations evaluated (for result verification or debugging)
    #metsection = r'\section{Equations Used for Solutions}'

    #for node in Robot.solution_nodes:
        #if node.solvemethod == '':  # skip unused SOA vars.
            #continue
                ##print out the equations evaluated
        ## print  'Equation(s):
        #tmp = '$' + sp.latex(node.symbol) + '$'
        #tmp = tmp.replace(r'th_', r'\theta_')
        #tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {} for latex
        #metsection += r'\subsection{'+tmp+' }'+eol
        #metsection += r'Solution Method: '+node.solvemethod

        #for eqn in node.eqnlist:
            #metsection += r'\begin{dmath}'+eol
            #metsection += eqn.LaTexOutput()+eol
            #metsection += r'\end{dmath}'+eol

    #LF.sections.append(metsection.splitlines())


    ####################  Jacobian Matrix

    jsection =r'''\newpage
\section{Jacobian Matrix}

'''

    #  ONE COLUMN PER JOINT.  J66 is stored 6x6 for every robot because the DH
    #  table is always padded to 6 rows, and the surplus columns are NOT zero,
    #  so a 5-DOF arm was shown a sixth joint it does not have.  dof_of() is the
    #  same helper the numeric solver slices with (numeric_ik.py:243).
    ndof = nik.dof_of(Robot.Mech)
    j66result = kc.notation_squeeze(Robot.Mech.J66)[:, :ndof]
    cols = j66result.shape[1]

    jsection += r'\begin{dmath}'+eol
    jsection += '^6J_6  = '+r'\\'+eol

    COLUMNS = True
    if COLUMNS:
        for c in range(cols):
            jsection += r'\mathrm{'+ r' Column \quad'+str(c+1)+ r'}\\'+eol
            jsection += sp.latex(j66result[:,c])+eol
            jsection += r'\\ '+eol
    else:
        jsection += sp.latex(j66result)+eol
    jsection += r'\end{dmath}'+eol

    LF.sections.append(jsection.splitlines())

    ####################  How the approximate answer gets corrected

    #  HYBRID ONLY, and LAST:  it refers to the equations above, and it is the
    #  step that turns them into an answer for the robot on the title page.
    LF.sections.append(hybrid_numeric_section(hybrid, orig_name))

    # Write out the file!!
    LF.output()

#
#
#################################################################################
#
#
#
#      Generate a partial report: only the FK and Jacobian
#
def link_transforms(Mech):
    '''Return [(i, T)] for each REAL link of the mechanism, where T is the
       modified-DH link transformation  {i-1}_{i}T  built from DH row i-1.

       The DH table always has six rows -- shorter arms pad with [0,0,0,0]
       (see "Adding a robot" in CLAUDE.md) -- and a padding row yields the
       identity, which is only noise in the report.  A row is a real joint iff
       it carries a joint variable in d or theta;  that is the same test
       Robot.__init__ uses to find max_index (ik_classes.py).'''

    d, th = 2, 3                     # DH column indices for d_i and theta_i
    #  forward_kinematics() stores the six link transforms in Mech.Ts.  Fall
    #  back to the individual attributes for FK pickles written before that
    #  list existed -- an old fk_eqns/*.p must not crash the report.
    Ts = getattr(Mech, 'Ts', None)
    if Ts is None:
        Ts = [Mech.T_01, Mech.T_12, Mech.T_23, Mech.T_34, Mech.T_45, Mech.T_56]

    out = []
    for i, T in enumerate(Ts):
        if Mech.DH[i, d] == 0 and Mech.DH[i, th] == 0:
            continue                 # padding row, not a joint
        out.append((i, T))
    return out


def link_transform_section(Robot):
    '''LaTeX for the per-link transformation matrices.

       These are the factors whose product is the forward kinematics, so the
       section sits directly after the DH table it is derived from and before
       the assembled FK.'''

    eol = '\n'
    links = link_transforms(Robot.Mech)

    s = r'''\section{Link Transformation Matrices}
Each row of the table above generates one link transformation matrix,
$^{i-1}_{i}T$, relating frame $i$ to frame $i-1$ (modified, or Craig,
DH convention).  The product of these matrices, in order, is the forward
kinematics given in the next section:
\[ ^{0}_{6}T = {}^{0}_{1}T \; {}^{1}_{2}T \; {}^{2}_{3}T \;
               {}^{3}_{4}T \; {}^{4}_{5}T \; {}^{5}_{6}T \]
Throughout, $c_i = \cos \theta_i$ and $s_i = \sin \theta_i$.
'''+eol

    if len(links) < 6:
        s += (r'This mechanism has %d links;  the remaining rows of the '
              r'parameter table are zero padding and are not shown.'
              % len(links)) + eol

    for i, T in links:
        s += r'\begin{dmath}'+eol
        s += r'^{%d}_{%d}T = ' % (i, i+1) + sp.latex(kc.notation_squeeze(T)) + eol
        s += r'\end{dmath}'+eol

    return s


def output_FK_equations(Robot):
    GRAPH = True
    ''' Print out a latex document of the solution equations. '''
    eol = '\n'
    orig_name =  Robot.name.replace('test: ','')
    fixed_name = orig_name.replace(r'_', r'\_')

    DirName = 'LaTex/'
    fname = DirName + 'fk_equations_'+orig_name+'.tex'
    LF = LatexFile(fname)

    ####################   Intro Section

    introstring = r'''
    \begin{center}
    \section*{Forward Kinematic Computations for ''' + fixed_name + r'''}
    \today
    \end{center}
    \section{Introduction}
    This report gives the forward kinematics solutions for '''+fixed_name+r'''.
    These equations are automatically generated by the \href{https://github.com/uw-biorobotics/IKBT}{IK-BT package}
    from the University of Washington Biorobotics Lab.
    The IK-BT package is described in
    \url{https://arxiv.org/abs/1711.05412}.
    IK-BT derives your inverse kinematics equations
    using {\tt Python 3.8} and the {\tt sympy 1.9} module for symbolic mathematics.
    '''

    LF.sections.append(introstring.splitlines())

    ####################   Kinematic params

    paramsection = r'''\section{Kinematic Parameters}
    The kinematic parameters for this robot are
    \[ \left [ \alpha_{i-1}, \quad a_{i-1}, \quad d_i, \quad \theta_i \right  ] \]
    \begin{dmath}''' + sp.latex(Robot.Mech.DH) +  r'\end{dmath}'

    LF.sections.append(paramsection.splitlines())

    ####################  Individual link transforms

    LF.sections.append(link_transform_section(Robot).splitlines())

    ####################  Forward Kinematics

    fksection = r'''\section{Forward Kinematic Equations}
    The forward kinematic equations for this robot are:'''+eol

    fksection += r'\begin{dmath} '+eol

    LHS = ik_lhs()
    RHS = kc.notation_squeeze(Robot.Mech.T_06)   # see kin_cl.mechanism.T_06

    fksection += sp.latex(LHS) + r'= \\'+eol

    COLUMNS = True
    if COLUMNS:
        for c in range(4):
            fksection += r'\mathrm{Column \quad'+str(c+1)+r'}\\'+eol+sp.latex(RHS[:,c]) + r'\\'+eol
    else:
        fksection += sp.latex(RHS)
    fksection += r'\end{dmath}'+eol

    fksection += 'Note: column numbers use math notation rather than python indeces.'+eol

    LF.sections.append(fksection.splitlines())



    ####################  Jacobian Matrix

    jsection =r'''\newpage
\section{Jacobian Matrix}

'''

    #  ONE COLUMN PER JOINT.  J66 is stored 6x6 for every robot because the DH
    #  table is always padded to 6 rows, and the surplus columns are NOT zero,
    #  so a 5-DOF arm was shown a sixth joint it does not have.  dof_of() is the
    #  same helper the numeric solver slices with (numeric_ik.py:243).
    ndof = nik.dof_of(Robot.Mech)
    j66result = kc.notation_squeeze(Robot.Mech.J66)[:, :ndof]
    cols = j66result.shape[1]

    jsection += r'\begin{dmath}'+eol
    jsection += r'^6J_6  = \\'+eol

    COLUMNS = True
    if COLUMNS:
        for c in range(cols):
            jsection += r'\mathrm{Column \quad '+str(c+1)+r'}\\'+eol
            jsection += sp.latex(j66result[:,c])+eol
            jsection += r'\\ '+eol
    else:
        jsection += sp.latex(j66result)+eol
    jsection += r'\end{dmath}'+eol

    LF.sections.append(jsection.splitlines())

    # Write out the file!!
    LF.output()

    print('\n\n\n                       End of LaTex Output work \n\n\n')


#####################################################################
#
#   Test code
#
import unittest


class TestSolver015(unittest.TestCase):
    '''Per-link transformation matrices in the FK report (fkOnly.py).'''

    def setUp(self):
        print('\n\n===============  Test link transform output  =====================')
        return

    def runTest(self):
        self.test_ltA_product_is_the_forward_kinematics()
        self.test_ltB_padding_rows_are_skipped()
        self.test_ltC_section_is_wellformed_latex()

    def robot(self, name='Puma'):
        '''Uses the cached FK pickle, so this stays fast.'''
        from ikbtfunctions.ik_driver import load_robot
        import io, contextlib
        with contextlib.redirect_stdout(io.StringIO()):
            M, R, unknowns = load_robot(name)
        return M, R

    def test_ltA_product_is_the_forward_kinematics(self):
        '''The whole point of the section:  these are the factors of the FK.
           If the report prints matrices whose product is not T_06, it is
           telling the reader something false.'''
        fs = ' link transform FAIL'
        for name in ('Puma', 'Wrist'):
            M, R = self.robot(name)
            prod = sp.eye(4)
            for i, T in link_transforms(M):
                prod = prod * T
            diff = sp.simplify(sp.trigsimp(prod - M.T_06))
            self.assertTrue(all(e == 0 for e in diff),
                            fs + ' (%s: product of link transforms != T_06)' % name)

    def test_ltB_padding_rows_are_skipped(self):
        '''The DH table always has six rows;  shorter arms pad with [0,0,0,0],
           which yields an identity transform -- noise in the report.'''
        fs = ' link transform padding FAIL'
        M, R = self.robot('Wrist')
        links = link_transforms(M)
        self.assertLess(len(links), 6, fs + ' (Wrist should have padding rows)')
        for i, T in links:
            self.assertFalse(M.DH[i, 2] == 0 and M.DH[i, 3] == 0,
                             fs + ' (emitted padding row %d)' % i)
        #  and a full 6-DOF arm must lose nothing
        M6, R6 = self.robot('Puma')
        self.assertEqual(len(link_transforms(M6)), 6,
                         fs + ' (dropped a real link from a 6-DOF arm)')

    def test_ltC_section_is_wellformed_latex(self):
        '''Balanced dmath environments and one matrix per real link.'''
        fs = ' link transform latex FAIL'
        M, R = self.robot('Puma')
        R.Mech = M
        s = link_transform_section(R)
        n = len(link_transforms(M))
        self.assertEqual(s.count(r'\begin{dmath}'), n, fs + ' (wrong matrix count)')
        self.assertEqual(s.count(r'\begin{dmath}'), s.count(r'\end{dmath}'),
                         fs + ' (unbalanced dmath)')
        self.assertIn(r'\section{Link Transformation Matrices}', s, fs)
        for i in range(n):
            self.assertIn(r'^{%d}_{%d}T' % (i, i+1), s,
                          fs + ' (missing label for link %d)' % i)


def run_test():
    suite = unittest.TestLoader().loadTestsFromTestCase(TestSolver015)
    unittest.TextTestRunner(verbosity=2).run(suite)


if __name__ == "__main__":
    run_test()
