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
###import numpy as np
##import re
#from solution_graph_v2 import *
#from sys import exit
#import b3 as b3          # behavior trees
#import pickle 
#import helperfunctions as hf
#from kin_cl import *

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
    #name = label.replace(r'\$',r'') # no eqn formatting for internal node name
    #name = label.replace(r'_',r'') # no eqn formatting for internal node name
    #return r' \node [place] ('+ name + ') ['+control_string+'] {'+label+'};'
    return r' \node {'+label+'} '

def tikz_place_children(n,f):
        #TO DO: silence this on the console for better output
        print('Place children: ', n)
        if len(n.parents)> 0: # must have a parent else its root!
            print(operator.rshift(f), 'child { node {$' + sp.latex(n.symbol).replace(r'th_', r'\theta_') + '$}  ')
        for c in n.children:
            print(operator.rshift(f), '   ',)
            tikz_place_children(c,f)
        if len(n.parents) > 0:
            print(operator.rshift(f), '}')

def tikz_place_edge(from_node, to_node, label,control_string = ''):
    #\path[->] (S3) edge [bend left] node {a} (S1);
    return r'\path[->] ('+from_node+') edge ['+control_string+'] node {'+label+'} ('+to_node+');'
                        
def tikz_closepicture():
    return r'; \end{tikzpicture}'


if __name__ == "__main__":
    f = open('test_graph.tex','w')
    print(operator.rshift(f), '''\\documentclass[letterpaper]{article}

% Uncomment for bibliog.
%\\bibliographystyle{unsrt}

\\usepackage{graphicx}
\\usepackage{lineno}
\\usepackage{amsmath} ''')
    print(operator.rshift(f), tikz_setup())
    print(operator.rshift(f), r'\begin{document} \section{}')
    print(operator.rshift(f), '%  move above stuff before \begin{document}')
    print(operator.rshift(f), tikz_startpicture())
    print(operator.rshift(f), tikz_place_node('N1', '$N_1$',''))
    print(operator.rshift(f), tikz_place_node('N2', '$N_2$','right =of N1'))
    print(operator.rshift(f), tikz_place_node('N3', '$N_3$','below =of N1'))
    print(operator.rshift(f), tikz_place_node('N4', '$N_4$','below =of N2'))
    print(operator.rshift(f), tikz_place_edge('N2', 'N3', '$\pm\sqrt{x}$',''))
    print(operator.rshift(f), tikz_place_edge('N1', 'N3', 'atan2()', 'bend right'))
    print(operator.rshift(f), tikz_place_edge('N3', 'N4', 'asin()', 'bend right'))
    print(operator.rshift(f), tikz_closepicture())
    print(operator.rshift(f), r'\end{document}')
    
