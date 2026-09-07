#!/usr/bin/python
#
#     Python wrapper to draw graphs in latex using tikz
#

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import sympy as sp

class newnode:
    def __init__(self):
        self.symbol = '' 
        self.order = ''
        self.eqn = ''
        self.Node = []     # Solution node for this variable
        self.children = []
        self.parents = []
        
    def __hash__(self):
        return self.symbol.__hash__()
    
    def __eq__(self, other): #equal judgement, also hashing in python 3.x
        value = False
        if type(self) != type(other):
            return False
        if self.symbol == other.symbol:
            value = True
        return value
    
    def __repr__(self): # string representation
        child = '-none-'
        parent = '-none-'
        if (len(self.children) > 0):
            child = str(self.children[0].symbol)
        if (len(self.parents) > 0):
            parent = str(self.parents[0].symbol)
        return 'Newnode: ' + str(self.symbol) + ' child[0]: ' +child + ', parents[0]: ' + parent + '\n'


def tikz_setup():
    # insert this text at top of latex file, before \begin{document}
    return r'''\usepackage{tikz}
\usetikzlibrary{arrows,shapes,automata,petri,positioning,calc}

\tikzset{
    place/.style={
        circle,
        thick,
        draw=black,
        fill=gray!50,
        minimum size=6mm,
    },
        state/.style={
        circle,
        thick,
        draw=blue!75,
        fill=blue!20,
        minimum size=6mm,
    },
}
'''
 
#def tikz_startpicture():
    #return r"\begin{tikzpicture}[node distance=2cm and 1cm,>=stealth',auto, every place/.style={draw}]"
def tikz_startpicture():
    return r'''\begin{tikzpicture}[sibling distance=10em, every node/.style = {shape=rectangle, rounded corners,
    draw, align=center, top color=white, bottom color=blue!20}]]
\tikzstyle{level 1}=[sibling distance=70mm] 
\tikzstyle{level 2}=[sibling distance=38mm] 
\tikzstyle{level 3}=[sibling distance=10mm] 
'''

#def tikz_place_node(name, label, control_string):
def tikz_place_node( label ):
    #\node [place] (S3) [node distance=1.5cm,below =of right-S1] {S3};
    return r' \node {'+label+'} '

def tikz_place_children(n,f):
        #TO DO: silence this on the console for better output
        print('Place children: ', n)
        if len(n.parents)> 0: # must have a parent else its root!
            print('child { node {$' + sp.latex(n.symbol).replace(r'th_', r'\theta_') + '$}  ', file=f)
        for c in n.children:
            print('   ', file=f)
            tikz_place_children(c,f)
        if len(n.parents) > 0:
            print('}', file=f)

def tikz_place_edge(from_node, to_node, label,control_string = ''):
    #\path[->] (S3) edge [bend left] node {a} (S1);
    return r'\path[->] ('+from_node+') edge ['+control_string+'] node {'+label+'} ('+to_node+');'
                        
def tikz_closepicture():
    return r'; \end{tikzpicture}'


if __name__ == "__main__":
    f = open('test_graph.tex','w')
    print('''\\documentclass[letterpaper]{article}

% Uncomment for bibliog.
%\\bibliographystyle{unsrt}

\\usepackage{graphicx}
\\usepackage{lineno}
\\usepackage{amsmath} ''', file=f)
    print(tikz_setup(), file=f)
    print(r'\begin{document} \section{}', file=f)
    print('%  move above stuff before \begin{document}', file=f)
    print(tikz_startpicture(), file=f)
    print(tikz_place_node('N1', '$N_1$',''), file=f)
    print(tikz_place_node('N2', '$N_2$','right =of N1'), file=f)
    print(tikz_place_node('N3', '$N_3$','below =of N1'), file=f)
    print(tikz_place_node('N4', '$N_4$','below =of N2'), file=f)
    print(tikz_place_edge('N2', 'N3', '$\pm\sqrt{x}$',''), file=f)
    print(tikz_place_edge('N1', 'N3', 'atan2()', 'bend right'), file=f)
    print(tikz_place_edge('N3', 'N4', 'asin()', 'bend right'), file=f)
    print(tikz_closepicture(), file=f)
    print(r'\end{document}', file=f)
    


#####################################################################
#
#   The solution dependency graph, as a TikZ figure
#
#   The report also prints this graph as a list of edges
#   ('Edge:th_4 depends on: th_23'), which is the exact data but does not show
#   the SHAPE of the solve -- which variable unlocks which.  This draws it.
#

import re


def _tex_label(name):
    r"""'th_23' -> '\theta_{23}', 'd_1' -> 'd_{1}'.

       The same two steps the rest of the report uses on a variable name.  Done
       here rather than imported so this module stays standalone -- it is the
       one place that turns a graph into LaTeX, and it should not need the
       report generator to do it."""

    s = re.sub(r'_(\d+)', r'_{\1}', str(name))
    return s.replace('th_', r'\theta_')


def _levels(names, deps):
    """{name: row}, where a variable sits one row below everything it needs.

       A variable with no dependencies is a root at row 0.  Every other one
       goes one row below the deepest thing it depends on, so every arrow
       points DOWNWARD and the drawing reads top to bottom in the order the
       solver could have discovered them.

       The `seen` guard is for a cycle.  There should never be one -- a
       variable cannot depend on something solved after it -- but a drawing
       routine is the wrong place to discover that, so a cycle degrades to a
       finite (if ugly) layout instead of infinite recursion."""

    level = {}

    def depth(n, seen):
        if n in level:
            return level[n]
        if n in seen:
            return 0                      # cycle: stop, do not recurse
        seen = seen | {n}
        d = 0
        for p in deps.get(n, ()):
            if p in names:
                d = max(d, depth(p, seen) + 1)
        level[n] = d
        return d

    for n in names:
        depth(n, set())
    return level


def solution_graph_tikz(order, edges, eol='\n', caption=None, label=None):
    r"""A TikZ figure of the solution graph, or '' if there is nothing to draw.

       order    variable names, in SOLVE order
       edges    iterable with .StartNode and .dependsOn (ikbtbasics Edge)

       AN ARROW MEANS "UNLOCKS":  it runs from a variable to one that becomes
       solvable once it is known -- the reverse of the edge listing's wording
       ('th_4 depends on: th_23').  Both conventions are in use:  software
       dependency graphs (UML, package managers, make) point from the dependent
       to what it needs, while scheduling and dataflow networks (PERT/CPM, task
       graphs) point along topological order.  This graph is a precedence
       network -- th_1 must be solved before th_2 -- so the scheduling
       convention is the right one, and it also makes the arrows agree with the
       drawing, which already runs top to bottom in solution order.  Pointing at
       dependencies ran every arrow backwards against the layout.

       Rows come from the dependency depth rather than from solve order.  Those
       are different: the solver may solve th_5 before th_23 while th_4 needs
       both, and depth is what shows that th_4 is the one waiting.  Within a row
       the solve order is preserved, so the drawing still reads left to right in
       the order the report discusses them.

       NO EDGES IS NOT NOTHING.  An arm whose variables are all independent
       draws a single row with no arrows, and that is worth seeing -- it says
       the solve decoupled completely."""

    order = [str(n) for n in order]
    if not order:
        return ''

    deps = {}
    for e in edges or ():
        a, b = str(e.StartNode), str(e.dependsOn)
        if a in order and b in order and a != b:
            deps.setdefault(a, set()).add(b)

    level = _levels(order, deps)
    rows = {}
    for n in order:                       # solve order preserved within a row
        rows.setdefault(level[n], []).append(n)

    #  Geometry in cm.  Wide enough that a label never touches its neighbour,
    #  short enough that a seven-variable graph does not need its own page.
    DX, DY = 2.3, 1.7

    out = r'\begin{figure}[htb]' + eol + r'\centering' + eol
    #  SHRINK ONLY IF NEEDED.  A row is as wide as the number of variables that
    #  share a dependency depth, and an arm whose variables are largely
    #  independent puts them all on one row -- which can be wider than the text
    #  block.  This idiom scales the picture down to \textwidth when it would
    #  overflow and leaves it alone when it fits, so a small graph is not blown
    #  up to fill the page.
    out += (r'\resizebox{\ifdim\width>\textwidth \textwidth\else\width\fi}{!}{%'
            + eol)
    #  No `font=` key:  \displaystyle is not a font command, and TeX accepts
    #  `font=$\displaystyle$` silently while inserting an empty math group
    #  before every label.  The labels carry their own $...$.
    pos = {}
    out += (r'\begin{tikzpicture}[>=stealth, thick,'
            r' every node/.style={draw, circle, minimum size=9mm,'
            r' inner sep=1pt}]' + eol)

    for r in sorted(rows):
        row = rows[r]
        x0 = -DX * (len(row) - 1) / 2.0   # centre each row on the axis
        for i, n in enumerate(row):
            pos[n] = (x0 + i * DX, -r * DY)
            out += (r'  \node (%s) at (%.2f,%.2f) {$%s$};'
                    % (_tikz_name(n), pos[n][0], pos[n][1], _tex_label(n)) + eol)

    #  Drawn b -> a:  b unlocks a.  deps[a] holds what a needs, so the arrow
    #  is emitted the other way round from how the dependency is stored.
    #
    #  ROUTING.  Rows are stacked directly under one another, so an edge
    #  spanning more than one row would run through the nodes between.  Those
    #  are sent around the outside with out/in angles, on whichever side
    #  carries fewer nodes in the rows they cross -- bending them all the same
    #  way just moves the pile to the other side.  Adjacent-row edges are
    #  drawn straight.
    xof = {n: pos[n][0] for n in order}
    for a in sorted(deps):
        for b in sorted(deps[a]):
            na, nb = _tikz_name(a), _tikz_name(b)
            lo, hi = sorted((level[a], level[b]))
            span = hi - lo

            if span <= 1:
                out += r'  \draw[->] (%s) -- (%s);' % (nb, na) + eol
                continue

            #  who is in the way, and on which side
            left = right = 0
            for n in order:
                if lo < level[n] < hi:
                    if xof[n] < -0.01:
                        left += 1
                    elif xof[n] > 0.01:
                        right += 1
            #  180 = leave/enter on the left, 0 = on the right
            ang = 180 if left <= right else 0
            loose = 1.0 + 0.45 * (span - 1)
            out += (r'  \draw[->] (%s) to[out=%d, in=%d, looseness=%.2f] (%s);'
                    % (nb, ang, ang, loose, na) + eol)

    out += r'\end{tikzpicture}}' + eol
    if caption:
        out += r'\caption{' + caption + '}' + eol
    if label:
        out += r'\label{' + label + '}' + eol
    out += r'\end{figure}' + eol
    return out


def _tikz_name(n):
    """A TikZ node name:  letters and digits only.

       TikZ parses '(' and ',' inside a coordinate, so a name carrying either
       would silently produce a picture with missing arrows rather than an
       error.  Variable names here are already tame (th_2, d_1), but the report
       must not depend on that staying true."""

    return re.sub(r'[^A-Za-z0-9]', '', str(n))
