#!/usr/bin/python
#
#   texwidth.py --  which equations actually run off the page?
#
#   THE MEASUREMENT PROBLEM.  An equation is too wide when TeX cannot fit it in
#   \textwidth -- which depends on the font, the margins, the environment it
#   sits in (align does not wrap, dmath does), and how much of the line the LHS
#   already used.  None of that is visible from the expression, so every cheap
#   proxy tried here failed on measured data:  character count and count_ops are
#   properties of the EXPRESSION, and overflow is a property of the TYPESET
#   LINE.  See IKdocs/DEV_NOTES.md.
#
#   SO ASK TeX.  pdflatex already reports exactly this, in points, with source
#   line numbers:
#
#       Overfull \hbox (144.6304pt too wide) detected at line 276
#
#   That is ground truth, in the real document at the real geometry, and one
#   subprocess covers the whole report.  (sympy.preview() renders one equation
#   per subprocess, measures each at ITS OWN default geometry rather than this
#   document's, and needs dvipng.)
#
#   IT MUST NEVER COST US THE REPORT.  Every entry point degrades to "no
#   measurement" -- pdflatex missing, a LaTeX error, a timeout -- and the caller
#   then emits the unfolded report, which is correct and merely wide.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os
import re
import shutil
import subprocess


#  'Overfull \hbox (144.6304pt too wide) detected at line 276'
#  'Overfull \hbox (52.85153pt too wide) in paragraph at lines 255--255'
_OVERFULL = re.compile(
    r'Overfull \\hbox \(([0-9.]+)pt too wide\)'
    r'(?:[^\n]*?)(?:detected at line|in paragraph at lines)\s+(\d+)')

#  Marker the generator writes on its own source line, immediately above each
#  equation, so a reported line number can be attributed to an equation.
#  A TeX comment, so it never reaches the page.
MARKER = '%%IKBT-EQ '

DEFAULT_TIMEOUT = 180


def pdflatex_available():
    '''Is there a pdflatex to ask?  The whole module degrades to [] without one.'''
    return shutil.which('pdflatex') is not None


def overfull_by_line(texpath, timeout=DEFAULT_TIMEOUT):
    '''{source line number: points too wide} for one .tex file.

       Returns {} when the measurement could not be made at all -- no pdflatex,
       a crash, a timeout.  {} means "nothing measured", NOT "nothing wrong",
       and the caller must treat it as the former.

       pdflatex is run in the file's own directory (the generated reports
       \\input nothing, but IK_preamble.tex lives beside them) and in
       nonstopmode, because a document that errors still reports the overfull
       boxes it found before erroring -- which are exactly what we want.'''

    if not pdflatex_available():
        return {}

    d = os.path.dirname(os.path.abspath(texpath)) or '.'
    base = os.path.basename(texpath)
    try:
        p = subprocess.run(
            ['pdflatex', '-interaction=nonstopmode', '-halt-on-error=0', base],
            cwd=d, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout)
        out = p.stdout.decode('utf-8', errors='replace')
    except (OSError, subprocess.SubprocessError):
        return {}

    worst = {}
    for pts, line in _OVERFULL.findall(out):
        n = int(line)
        worst[n] = max(worst.get(n, 0.0), float(pts))
    return worst


def ids_by_line(texpath):
    '''{source line number: equation id} from the markers in the file.

       Each marker claims every line from just after itself up to the next
       marker, so an equation spanning several source lines is attributed
       correctly however it was broken.'''

    try:
        with open(texpath, 'r') as f:
            lines = f.read().splitlines()
    except IOError:
        return {}

    out = {}
    current = None
    for i, l in enumerate(lines, 1):
        s = l.lstrip()
        if s.startswith(MARKER):
            current = s[len(MARKER):].strip()
            continue
        if current:
            out[i] = current
    return out


def overfull_ids(texpath, slack_pt=0.0, timeout=DEFAULT_TIMEOUT):
    '''{equation id: points too wide} -- the equations that need shortening.

       slack_pt ignores overflows smaller than that.  TeX reports a box 3.7pt
       over, which is about one character and not worth restructuring an
       equation for;  the caller decides where the line is.

       An overfull line with no marker above it is NOT this module's business
       (a wide table, a long verbatim row) and is dropped rather than guessed at.'''

    by_line = overfull_by_line(texpath, timeout=timeout)
    if not by_line:
        return {}
    ids = ids_by_line(texpath)

    out = {}
    for line, pts in by_line.items():
        if pts <= slack_pt:
            continue
        eid = ids.get(line)
        if eid is None:
            continue
        out[eid] = max(out.get(eid, 0.0), pts)
    return out


#####################################################################
#
#   Test code
#

import unittest


class TestSolver026(unittest.TestCase):
    '''The overflow measurement.

       No pdflatex is required to run these:  what is tested is the parsing and
       the attribution, which is where the mistakes would be.  Whether pdflatex
       reports overfull boxes correctly is pdflatex's business.'''

    def setUp(self):
        print('\n\n===============  Test tex width measurement  =====================')
        return

    def runTest(self):
        self.test_twA_parses_both_report_forms()
        self.test_twB_keeps_the_worst_per_line()
        self.test_twC_marker_claims_its_lines()
        self.test_twD_missing_measurement_is_empty()

    def test_twA_parses_both_report_forms(self):
        '''pdflatex says "detected at line N" for a display equation and
           "in paragraph at lines N--M" for text.  Both must parse.'''
        fs = ' texwidth parse FAIL'
        log = ('Overfull \\hbox (144.6304pt too wide) detected at line 276\n'
               'Overfull \\hbox (52.85153pt too wide) in paragraph at lines 255--255\n')
        got = _OVERFULL.findall(log)
        self.assertEqual(len(got), 2, fs + ' (expected two matches, got %r)' % (got,))
        self.assertEqual(got[0], ('144.6304', '276'), fs)
        self.assertEqual(got[1], ('52.85153', '255'), fs)

    def test_twB_keeps_the_worst_per_line(self):
        '''One source line can overflow several times -- an align block reports
           once per row.  The line's severity is the WORST of them, not the
           last one seen.'''
        fs = ' texwidth worst-per-line FAIL'
        log = ('Overfull \\hbox (10.0pt too wide) detected at line 5\n'
               'Overfull \\hbox (300.0pt too wide) detected at line 5\n'
               'Overfull \\hbox (20.0pt too wide) detected at line 5\n')
        worst = {}
        for pts, line in _OVERFULL.findall(log):
            n = int(line)
            worst[n] = max(worst.get(n, 0.0), float(pts))
        self.assertEqual(worst, {5: 300.0}, fs)

    def test_twC_marker_claims_its_lines(self):
        '''A marker owns every line after it until the next marker, so a
           multi-line equation is attributed to the right id.'''
        import tempfile
        fs = ' texwidth attribution FAIL'
        body = ('preamble\n'
                + MARKER + 'th_4s1\n'
                '\\begin{dmath} a\n'
                '  + b \\end{dmath}\n'
                + MARKER + 'th_5s1\n'
                '\\begin{dmath} c \\end{dmath}\n')
        with tempfile.NamedTemporaryFile('w', suffix='.tex', delete=False) as f:
            f.write(body)
            path = f.name
        try:
            ids = ids_by_line(path)
            self.assertIsNone(ids.get(1), fs + ' (text before any marker was claimed)')
            self.assertEqual(ids.get(3), 'th_4s1', fs)
            self.assertEqual(ids.get(4), 'th_4s1',
                             fs + " (continuation line lost its equation)")
            self.assertEqual(ids.get(6), 'th_5s1', fs)
        finally:
            os.unlink(path)

    def test_twD_missing_measurement_is_empty(self):
        '''A file that cannot be read yields {}, not an exception.  This runs on
           the report path, and a formatting step must never cost the report.'''
        fs = ' texwidth degradation FAIL'
        self.assertEqual(ids_by_line('/nonexistent/nope.tex'), {}, fs)
        self.assertEqual(overfull_by_line('/nonexistent/nope.tex'), {}, fs)
        self.assertEqual(overfull_ids('/nonexistent/nope.tex'), {}, fs)


def run_test():
    t = TestSolver026()
    t.setUp()
    t.runTest()
    print('\n\n            tex width measurement PASSES\n\n')


if __name__ == '__main__':
    unittest.main()
