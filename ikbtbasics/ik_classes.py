#!/usr/bin/python
#
#     Inverse Kinematics Classes
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
#import numpy as np
import pykinsym as pks
import re
from solution_graph_v2 import *
import matching as mtch
import sys as sys
import b3 as b3          # behavior trees
import pickle
from ikbtfunctions.helperfunctions import *
import ikbtfunctions.graph2latex as gl
#from kin_cl import *
import kin_cl as kc


# generic variables for any manipulator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4, l_5, l_6)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4', 'l_5', 'l_6'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('l_5 l_6')
sp.var('th_12, th_23, th_34, th_45, th_56')
sp.var('th_234') # handle 3-parallel axes
sp.var('c_12 s_12 c_23 s_23 c_34 s_34 c_45 s_45 c_56 s_56 c_13 s_13')
sp.var('x')  # super generic place holder
soa_vars = [th_12, th_23, th_34, th_45, th_56]  # a list of the sum-of-angles variables

soa_expansions= {}
soa_expansions[th_12] = th_1 + th_2
soa_expansions[th_23] = th_2 + th_3
soa_expansions[th_34] = th_3 + th_4
soa_expansions[th_45] = th_4 + th_5
soa_expansions[th_56] = th_5 + th_6


pprotocol = 2
#
#   retrieve forward kinematics from a pickle file if it exists.
#      if it doesn't, compute the FK and store it in a pickle file.
def kinematics_pickle(rname, dh, constants, pvals, vv, unks, test):
    #
    #   Check for a pickle file of combined pre-computed Mech and Robot objects
    #

    pickle_dir = 'fk_eqns/'

    if not os.path.isdir(pickle_dir):  # if this doesn't exist, create it.
        print 'Creating a new pickle directory: ./'+pickle_dir
        os.mkdir(pickle_dir)

    name = pickle_dir + rname + '_pickle.p'

    print 'kinematics pickle: trying to open ', name,' in ', os.getcwd()

    if(os.path.isfile(name)):
        with open(name, 'rb') as pick:
            print '\Trying to read pre-computed forward kinematics from '+name
            [m, R, unknowns]  = pickle.load(pick)
            print 'Successfully read pre-computed forward kinematics'
            print 'pickle contained ', len(unknowns), ' unknowns'
    else:
        #print 'WRONG - quitting, error: ',sys.exc_info()[0]
        #sys.exit
        # set up mechanism object instance
        m = kc.mechanism(dh, constants, vv)
        m.pvals = pvals  # store numerical values of parameters
        print 'Did not find VALID stored pickle file: ', name
        print "Starting Forward Kinematics"
        m.forward_kinematics()
        print "Completed Forward Kinematics"
        print 'Starting Sum of Angles scan (slow!)'

        # set up Robot Object instance
        R = Robot(m, rname)              # set up IK structs etc
        R.scan_for_equations(unks)       # generate equation lists

        # below is commented out for testing and devel of sum_of_angles_transform
        R.sum_of_angles_transform(unks)  # find sum of angles

        R.generate_solution_nodes(unks) # generate solution nodes

        print ' Storing kinematics pickle for '+rname + '('+name+')'
        with open(name,'wb') as pf:
            pickle.dump( [m, R, unks], pf, protocol=pprotocol)
        unknowns = unks    # be sure to return updated unknown list (including SOAs)

    return [m,R,unknowns]


def check_the_pickle(dh1, dh2):   # check that two mechanisms have identical DH params
    flag = False
    if (dh1.shape[0] != dh2.shape[0]):
        print   '   Wrong number of rows!'
        flag = True
    else:
        for r in range(0,dh1.shape[0]):
            for c in [0,1,2,3]:
                if(dh1[r,c] != dh2[r,c]):
                    flag = True
    if(flag):
        print '\n\n  -----------------------------------------------------'
        print     '                    DH parameters Differ '
        print     '                 Pickle file is out of date. '
        print     '                   please remove it and start again'
        print     '  -----------------------------------------------------'
        quit()

## retrieve thxy from thx, thy
def find_xy(thx, thy):
    # lookup table for thxy
    thxy_lookup = {th_1: [th_12], th_2:[th_12, th_23], th_3:[th_23, th_34], \
                    th_4:[th_34, th_45], th_5:[th_45, th_56], th_6:[th_56]}
    # one symbol in common is the th_xy we're looking for
    thx_s = set(thxy_lookup[thx])
    thy_s = set(thxy_lookup[thy])
    thxy_s = thx_s.intersection(thy_s)
    thxy = thxy_s.pop()
    return thxy

#def find_sum(thx,thy):
    # new approach for same problem as find_xy()

#  Class to contain all robot info
class Robot:
    def __init__(self, Mech=None, name="*Unnamed*"):
        self.name = name
        # the following data pertain to the solution tree for this Robot
        self.solveN = 0   # index of current solution in solving sequence
        self.soltag = ''  # suffix tag for current solution level leafs
        self.params = []  # constant dh params such as l_4 etc.
        self.solution_nodes = []  # first one is the root, by solve order
        self.variables_symbols = []
        #
        #   "notations" means specifically labeled solution variables such as
        #          th_1s2  (theta-1, solution 2)
        self.notation_graph = set() #solution nodes notation graph
        self.notation_collections = [] #solution notations divided into subgroups

        self.min_index = 0
        self.max_index = 0
        self.mequation_list = []        # all the 4x4 Matrix FK equations
        #self.kequation_aux_list = []    # kequations: such as eg th_23 = th_2+th_3
        
        # a place to store discovered sum of angles equations
        self.SOA_eqns = kc.matrix_equation()  # embedd these inside a 4x4 equation for 
                                        # downstream compatibility: use SOA_eqns.auxeqns.append(neweqn)

        if(Mech != None):    # in testing situations we only need a "Robot" to keep track of solutions above
            self.Mech = Mech

            self.min_index = 0  # start DOF of the current chain
                                #  min_index starts at 0 for ALL manips.
            #  max index == index of highest unsolved link variable
            #  define indices for DH table
            d = 2      # joint offset DH param index
            th = 3     # joint angle DH param index
            self.max_index = -99
            assert (self.Mech.DH[0,d]!=0 or self.Mech.DH[0,th] != 0), "You do not have a variable in first DH row!"
            for i in [5,4,3,2,1]:
                if(self.Mech.DH[i,d]!=0 or self.Mech.DH[i,th] != 0):
                    self.max_index = i  # end DOF of the current chain
                    break
            assert (self.max_index > 0), "Couldn't find mechanism index"
            #
            #  build up the equations to solve:
            self.mequation_list = Mech.get_mequation_set()  # all the Matrix FK equations
            print 'ik_classes: length Robot.mequation_list: ', len(self.mequation_list)

    def generate_solution_nodes(self, unknowns):
        '''generate solution nodes'''
        for unk in unknowns:
            self.solution_nodes.append(Node(unk))
            self.variables_symbols.append(unk.symbol)

        print self.solution_nodes
        print self.variables_symbols

    # get lists of unsolved equations having 1 and 2 unks
    # class Robot:
    def scan_for_equations(self,variables):
        self.l1 = [] # equations with one unk nown (if any)
        self.l2 = [] # equations with two unknowns
        self.l3p = [] # 3 OR MORE unknowns
        sp.var('x')  #this will be used to generate 'algebraic zero'
        #elist = self.mequation_list.append(self.kequation_aux_list)
        elist = self.mequation_list
        #print '------------------------- elist----'
        #print elist
        print '-------- (aux eqns):'
        print self.SOA_eqns.auxeqns
        print '--------'
        assert (len(elist) > 0), '  not enough equations '
        for eqn in elist:
            lhs = eqn.Td   #4x4 matrix
            rhs = eqn.Ts  #4x4 matrix
            for i in [0,1,2,3]:
                for j in range(0,4):
                    lh1x1 = lhs[i,j]
                    rh1x1 = rhs[i,j]
                    n = count_unknowns(variables, lh1x1) + count_unknowns(variables, rh1x1)
                    e1 = kc.kequation(lh1x1, rh1x1)
                    if(n==1):
                        flag = False

                        if e1 not in self.l1:
                            self.l1.append(e1)   # only append if not already there
                    if(n==2):
                        flag = False

                        if e1 not in self.l2:
                            self.l2.append(e1)    # only append if not already there
                    if(n > 2):

                        if e1 not in self.l3p:
                            self.l3p.append(e1)    # only append if not already there
        #Process the SOA equations
        for e in self.SOA_eqns.auxeqns:
            lhs = e.LHS
            rhs = e.RHS
            n = count_unknowns(variables, lhs) + count_unknowns(variables, rhs)
            if(n==1):
                self.l1.append(kc.kequation(lhs, rhs))  # change from 0, rhs-lhs !!  ************
            if(n==2):
                self.l2.append(kc.kequation(lhs, rhs))

        self.l1 = erank(self.l1) # sort the equations (in place) so solvers get preferred eqns first
        self.l2 = erank(self.l2)
        self.l3p = erank(self.l3p)
        return [self.l1, self.l2, self.l3p]
        #end of scan_for_eqns

#
#   Get equation lists from just a matrix equation
#     (this is used when generating tests NOT from DH params
#
    def scan_Mequation(self,Meqn,variables):
        self.l1 = []
        self.l2 = []
        for eqn in Meqn.get_kequation_list():
            lh1x1 = eqn.LHS  #4x4 matrix
            rh1x1 = eqn.RHS  #4x4 matrix
            n = count_unknowns(variables, lh1x1) + count_unknowns(variables, rh1x1)
            #e1 = kequation(lh1x1, rh1x1) # change from 0,rh1x1-lh1x1 **********
            e1 = eqn
            if(n==1):
                flag = False
                if e1 not in self.l1:

                    self.l1.append(e1)   # only append if not already there
            if(n==2):
                flag = False

                if e1 not in self.l2:
                    self.l2.append(e1)    # only append if not already there
        self.l1 = erank(self.l1) # sort the equations (in place) so solvers get preferred eqns first
        self.l2 = erank(self.l2)
        return [self.l1, self.l2]


    #
    # identify sum of angles terms and transform them to new variable
    #
    def sum_of_angles_transform(self,variables):
        print 'Starting sum-of-angles scan. Please be patient'
        unkn_sums_sym = set() #keep track of joint variable symbols

        #k = equation number
        #i = row, j=col
        nits = len(self.mequation_list) * 3 * 4 # total number of equations
        barlen = nits/2
        it_number = 0
        for k in range(0,len(self.mequation_list)):  # contains duplicates
            Meq = self.mequation_list[k]  # get next matrix equation
            for i in [0,1,2]:   # only first three rows are interesting
                for j in [0,1,2,3]:  # but check all 4 columns
                    it_number += 1
                    #print ' .. '
                    prog_bar(it_number, nits, barlen, 'Sum of Angles')
                    #print 'kij: ', k,i,j
                    
                    #print 'Sum of Angles: eqn,row,col: ', k,i,j
                    # simplify with lasting effect (note: try sp.trigsimp() for faster????)
                    Meq.Ts[i,j] = sp.simplify(Meq.Ts[i,j])  # simplify should catch c1s2+s1c2 etc. (RHS)
                    Meq.Td[i,j] = sp.simplify(Meq.Td[i,j])  # simplify should catch c1s2+s1c2 etc. (LHS)

                    lhs = Meq.Td[i,j]
                    rhs = Meq.Ts[i,j]

                        
                    # simplify LHS
                    lhs, newj, newe = sum_of_angles_sub(self,lhs, variables)
                    if newj:
                        variables.append(newj)
                    if newe:
                        self.kequation_aux_list.append(newe)
                    # simplify RHS
                    rhs, newj, newe= sum_of_angles_sub(self, rhs, variables)
                    if newj:
                        variables.append(newj)
                    if newe:
                        self.kequation_aux_list.append(newe)
                    
                    Meq.Td[i,j] = lhs 
                    Meq.Ts[i,j] = rhs

        prog_bar(-1,100,100, '')  # clear the progress bar

                            #x = raw_input('<enter> to cont...')
        print 'Completed sum-of-angles scan.'


##################
# 
#   substitute th_23 for th_2+th_3 etc.
# (april: separate out for easier testing)

def sum_of_angles_sub(R, expr, variables):   
    thx = sp.Wild('thx') # a theta_x term
    thy = sp.Wild('thy') # a theta_x term
    sgn = sp.Wild('sgn') # 1 or -1
    aw = sp.Wild('aw')
    bw = sp.Wild('bw')
    cw = sp.Wild('cw')
    s1 = sp.Wild('s1')
    s2 = sp.Wild('s2') 
    newjoint = None
    tmpeqn = None
    found2 = found3 = False
    matches = expr.find(sp.sin(aw+bw+cw)) | expr.find(sp.cos(aw+bw+cw))
    #print '- -  - - -'
    #print expr
    #print matches

    for m in matches: # analyze each match
        d  = m.match(sp.cos(aw + bw + cw))
        d1 = m.match(sp.sin(aw + bw + cw))
        if d != None and d1 != None:
            d.update(d1)
        if d == None and d1 != None:
            d = d1
        #  To find sum-of-angles arguments,
        # count number of non-zero elements among aw ... cw
        nzer = 0
        varlist = []
        for k1 in d.keys():
            if d[k1] == 0:
                nzer += 1
            else:
                varlist.append(d[k1])
        #print 'varlist: ', varlist
        if len(varlist) == 2:
            found2 = True
        if len(varlist) == 3:
            found3 = True

        newjoint = None
        tmpeqn = None
        if(found2 or found3):  # we've got a SOA!

            # generate index of the SOA variable
            nil = [] #new index list = 'nil'
            for v in varlist: # build the new subscript
                nil.append( str(get_variable_index(variables, v)) )
            nil.sort()  # get consistent order of indices
            ni = ''
            for c in nil:  # make into a string
                ni += c

            #print 'New index: '+ni
            vexists = False
            # has this SOA been found before?  Did we already make it?
            for v in variables:
                if v.n == int(ni):   # i.e. if th_23 is aready defined
                    th_subval = v
                    vexists = True
            newjoint = None
            tmpeqn = None
            th_new = sp.var('th_'+ni) # create iff doesn't yet exist
            th_subval = th_new
            if not vexists:
                print ":  found new 'joint' (sumofangle) variable: ", th_new
                #  try moving soa equation to Tm.auxeqns
                #unkn_sums_sym.add(th_new) #add into the joint variable set
                newjoint = kc.unknown(th_new)
                newjoint.n = int(ni)  # generate e.g. 234 = 10*2 + 34
                newjoint.solved = False  # just to be clear for count_unknowns
                variables.append(newjoint) #add it to unknowns list
                tmpeqn = kc.kequation(th_new, d[aw] + d[bw] + d[cw])
                print 'sum_of_angles_sub: created new equation:', tmpeqn
                #
                #   Add the def of this SOA to list:  eg  th23 = th2+th3
                #   BUT  it needs to be embedded into a 4x4 mequation so
                #    that solvers can scan it properly
                R.SOA_eqns.auxeqns.append(tmpeqn)
            
            
            
            # substitute new variable into the kinematic equations  ((WHY twice??))
            #self.mequation_list[k].Td[i,j] = Meq.Td[i,j].subs(d[aw] + d[bw] + d[cw], th_subval)
            #self.mequation_list[k].Ts[i,j] = Meq.Ts[i,j].subs(d[aw] + d[bw] + d[cw], th_subval)
            expr = expr.subs(d[aw] + d[bw] + d[cw], th_subval)
            #print 'sum of angles (ik_classes): NEW Eqns (k,i,j)',k,i,j
            #print self.mequation_list[k].Td[i,j]
            #print ' = '
            #print self.mequation_list[k].Ts[i,j]
            #print '========'
 
    return (expr, newjoint, tmpeqn)

def get_variable_index(vars, symb):
    for v in vars:
        if v.n == 0:
            print 'get_variable_index()/ik_classes: at least one index is not initialized for joint variables (or is 0!)'
            quit()
        found = False
        #print 'get_variable_index: v[i], symb, v[i].n ',str(v.symbol),str(symb), v.n
        if v.symbol == symb:
            found = True
            return v.n
    assert found, 'Error: trying to get index of an unknown joint variable' + str(symb)
    #if found == False:
        #print 'Error: trying to get index of an unknown joint variable'
        #print 'symbol: ', symb
        #print vars
        #quit()
    return False



# class kequation()       now moved to kin_cl.py
# class unknown(object)   now moved to kin_cl.py

# matrix_equation class moved to kin_cl.py



# #
#   Print text-based solution graph
#
def output_solution_graph(R):
    print '========== Solution output ================'
    print '          ' + R.name


    for node in R.solution_nodes:
        if node.solveorder != -1: #node is solved
            print '\n\n', node.solveorder, node.symbol, ' by method: ', node.solvemethod, ',  ', node.nsolutions, ' solution(s)'
            print node.solution_with_notations

    # print all edges in graph
    print '========== Solution Graph (Edges) output ================'
    for edge in R.notation_graph:
        print edge
    print '========== End Solution output ================'

#
#      Generate a complete report in latex
#
def output_latex_solution(Robot,variables, groups):
    GRAPH = True
    ''' Print out a latex document of the solution equations. '''

    orig_name =  Robot.name.replace('test: ','')
    fixed_name = orig_name.replace(r'_', r'\_')

    DirName = 'LaTex/'
    defaultname = DirName + 'IK_solution.tex'
    fname = DirName + 'IK_solution_'+orig_name+'.tex'
    f = open(fname, 'w')
    print >> f , r'''
    \begin{center}
    \section*{Inverse Kinematic Solution for ''' + fixed_name + r'''}
    \today
    \end{center}
    \section{Introduction}
    This report describes closed form inverse kinematics solutions for '''+fixed_name+r'''.
    The solution was automatically generated by the IK-BT package from the University of Washington Biorobotics Lab.
    The IK-BT package is described in
    \url{https://arxiv.org/abs/1711.05412}.  IK-BT derives your inverse kinematics equations
    using {\tt Python 2.7} and the {\tt sympy} module for symbolic mathematics.
    '''
    print >> f, r'''\section{Kinematic Parameters}
    The kinematic parameters for this robot are
    \[ \left [ \alpha_{i-1}, \quad a_{i-1}, \quad d_i, \quad \theta_i \right  ] \]
    \begin{dmath}''',
    print >> f, sp.latex(Robot.Mech.DH),
    print >> f, '''\end{dmath}
    '''



    print >>f, r'''\section{Forward Kinematic Equations}
    The forward kinematic equations for this robot are:'''


    LHS = ik_lhs()
    RHS = kc.notation_squeeze(Robot.Mech.T_06)   # see kin_cl.mechanism.T_06
    print >> f, r'\begin{dmath}'
    print >> f, sp.latex(LHS) + r' =  \\'
    print >> f, sp.latex(RHS)
    print >> f, r'\end{dmath}'

    print >> f , r'\section{Unknown Variables: }'

    # introduce the unknowns and the solution ORDER
    print >> f ,'''The unknown variables for this robot are (in solution order): '''
    print >> f ,r'\begin{enumerate}'

    tvars = {}
    for v in variables:
        tvars[v]=v.solveorder
    for v in sorted(tvars, key=tvars.get):
        tmp = '$' + sp.latex(v) + '$'
        tmp = tmp.replace(r'th_', r'\theta_')
        tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {}
        print >> f ,'\item {'+tmp+'}'
    print >> f ,r'\end{enumerate}'



    # print the solutions for each variable (in DH order)
    print >> f ,r'\section{Solutions}'
    print >> f ,''' The following equations comprise the full solution set for this robot.'''

    # sort the nodes into solution order
    sorted_node_list = sorted(Robot.solution_nodes)

    for node in sorted_node_list:
        ALIGN = True
        tmp = '$' + sp.latex(node.symbol) + '$'
        tmp = tmp.replace(r'th_', r'\theta_')
        tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {} for latex
        print >> f ,r'\subsection{'+tmp+' }'
        print >> f , 'Solution Method: ', node.solvemethod




        if (ALIGN):
            print >> f ,r'\begin{align}'
        else:
            print >> f ,r'\begin{dmath}'
        i=0
        nsolns = len(node.solution_with_notations.values())
        for eqn in node.solution_with_notations.values():
            i += 1
            if ALIGN and (i < nsolns):
                tmp2 = r'\\'   # line continuation for align environment
            else:
                tmp2 = ''
            tmp = str(eqn.LaTexOutput(ALIGN))
            # convert division ('/') to \frac{}{} for nicer output
            if re.search(r'/',tmp):
                 tmp = tmp.replace(r'(.+)=(.+)/(.+)', r'\1 = \frac{\2}{\3}')
            print >> f ,tmp, tmp2

        if (ALIGN):
            print >> f ,r'\end{align}'
        else:
            print >> f ,r'\end{dmath}'


    ###########################################################
    #
    #   Future:  Output a graph of the solution dependencies
    #            (not a tree!)
    #
    ###########################################################
    print>>f, r'\section{Solution Graph (Edges)}'
    print >>f, r'''
    The following is the abstract representation of solution graph for this manipulator (nodes with parent -1 are roots):
    \begin{verbatim}
    '''
    graph = Robot.notation_graph

    for edge in graph:
        print >>f, edge

    print>>f, '\end{verbatim}'
    ###########################################################
    #
    #   Output of solution sets
    #
    ###########################################################

    print>>f, r'\section{Solution Sets}'
    print >>f, r'''
    The following are the sets of joint solutions (poses) for this manipulator:
    \begin{verbatim}
    '''
    # groups = mtch.matching_func(Robot.notation_collections, Robot.solution_nodes)

    for g in groups:
        print >>f, g

    print>>f, '\end{verbatim}'

    ###########################################################
    #
    #   Output of Equation Evaluated (Use for verification or debugging)
    #
    ###########################################################
    #################################################
    # Equations evaluated (for result verification or debugging)
    print >>f, r'\section{Equations Used for Solutions}'



    for node in sorted_node_list:
                #print out the equations evaluated
        # print >> f , 'Equation(s):
        tmp = '$' + sp.latex(node.symbol) + '$'
        tmp = tmp.replace(r'th_', r'\theta_')
        tmp = re.sub(r'_(\d+)',  r'_{\1}', tmp)   # get all digits of subscript into {} for latex
        print >> f ,r'\subsection{'+tmp+' }'
        print >> f , 'Solution Method: ', node.solvemethod

        for eqn in node.eqnlist:
            print >>f, r'\begin{dmath}'
            print >>f, eqn.LaTexOutput()
            print >>f, r'\end{dmath}'

    f.close()

    # copy file to default filename (processing of latex simplifier)
    #  after this step  >pdflatex ik_report_template.tex   <<JUST WORKS!>>

    sh.copyfile(fname, defaultname)
#
#  ###########   End of Latex Output Section
#

def erank(list_L):    # rearrange list of eqns by length
                    # by putting shortest eqns last, system will prefer to solve
                    #   shorter equations (i.e. prefer shorter solutions where two exist)

    # since the sorting is from lower to higher
    # it should not be reversed when putting into the list - D.Z.
    sorted_ls = []
    list_d = {}
    for e in list_L:
        count = int(sp.count_ops(e.RHS)) + int(sp.count_ops(e.LHS))
        if count not in list_d.keys():
            list_d[count] = []
        list_d[count].append(e)

    keys = list_d.keys()
    keys = sorted(keys, reverse= False)

    for key in keys:
        sorted_ls.extend(list_d[key])
    return sorted_ls



#############    main       test the library  #########################
#
if __name__ == "__main__":   # tester code for the classes in this file

    #j1 = joint_var(th_12)

    sp.var('a b c d e')

    a = sp.atan2(b,c)   # make sure this function compiles/loads


    # Test .subs operator on atan2() function

    print 'Original Function: ', a
    print 'Substitute b<-e:   ', a.subs(b,e), ' (Expect atanw(e, c))'
    assert(a.subs(b,e) == sp.atan2(e,c))

     ###Test the Left Hand Side Generator

    m = ik_lhs()
    fs = 'ik_lhs() matrix generator FAIL'
    assert (m[0,0] == sp.var('r_11')), fs
    assert (m[0,1] == sp.var('r_12')),fs
    assert (m[3,3] == 1), fs
    assert (m[3,2] == 0), fs
    assert (m[3,1] == 0), fs
    assert (m[3,0] == 0), fs

    #
    ###Test kequation class

    E1 = kc.kequation(0, sp.cos(d))
    E2 = kc.kequation(5, sp.sin(e))
    E3 = kc.kequation(5, d+e+5)

    print "\n\nTesting kequation()"
    print "kequation sample: "
    print E1.LHS, " = ", E1.RHS
    fs = ' kequation method FAIL'
    assert(E1.LHS == 0), fs
    assert(E1.RHS == sp.cos(d)), fs
    assert(E2.RHS == sp.sin(e)), fs
    assert(E3.RHS == d+e+5), fs


    print "---------testing equation print method-----"
    E1.prt()
    print "--------------"


     ####Test unknown class

    ua = kc.unknown(a)
    ub = kc.unknown(b)

    print "\n\nTesting unknown(symbol) (one-arg form)"
    print "Unknown a: ",   ua.symbol
    fs = ' unknown object element "solved" FAIL'
    assert(ua.solved == False), fs
    print "a is solved: ", ua.solved , ' (Expect False)'
    print "Unknown b: ",   ub.symbol
    ub.solved = True
    print "b is solved: ", ub.solved, ' (Expect True)'
    assert(ub.solved == True), fs

      ##Test matrix_equation class


    print "\n\nTesting matrix_equation(T1,T2) class"
    T1 = ik_lhs()
    T2 = sp.zeros(5)
    T2[1,1] = a   # note: a = atan2(b,c) above
    T2[1,2] = a+b
    T2[2,2] = sp.sin(c)
    T2[2,3] = l_1*sp.sin(d) + 2*l_2*sp.cos(d)
    T2[3,1] = c+sp.cos(c)*l_1

    sp.pprint(T2)

    tme = kc.matrix_equation(T1,T2)
    print ''
    print "Mat eqn 1,2: ", tme.Td[1,2], " '=' ", tme.Ts[1,2], "(not a kequation type!)"
    print ''

    sp.var('e22 ')

    fs = 'Matrix Equation Class, FAIL'
    assert(tme.Ts[1,1] == a), fs
    assert(tme.Td[1,1] == sp.var('r_22')), fs
    assert(tme.Ts[1,2] == a+b), fs
    assert(tme.Ts[2,2] == sp.sin(c)), fs
    assert(tme.Ts[2,3] == l_1*sp.sin(d)+2*l_2*sp.cos(d)), fs


    print '           Test equation sorting: '

    e1 = kc.kequation(l_1, sp.sin(th_1) + sp.cos(th_1)*l_1)
    e2 = kc.kequation(l_2, sp.sin(th_1))
    e3 = kc.kequation(l_3, sp.sin(th_1) + sp.cos(th_1)*l_1 + sp.cos(th_3)*l_2)
    l = [e1, e2, e3]

    print 'Original List: '
    for e in l:
         e.prt()

    l= erank(l)  # should sort in place by increasing length of expression


    print 'Sorted List: '
    for e in l:
        e.prt()

    assert (l == [e2, e1, e3]), ' Equation length sorting FAIL'



    # unknown class hash function testing
    tmpv1 = kc.unknown(th_1)
    tmpv2 = kc.unknown(th_1)
    print 'a.symbol: ', tmpv1
    print 'b.symbol: ', tmpv2
    c = set()
    c.add(tmpv1)
    c.add(tmpv2)
    print '-------------'
    print tmpv1.__hash__()
    print tmpv2.__hash__()
    print 'Length of set: ', len(c)
    print '-------------'
    assert(len(c) == 1), "hashing (unknown/variable) class fail"



    ###    Test Robot class
    #   Robot class is tested in updateL.py
    print 'Sum of Angles testing'
    #
    
    
    ############################################################3
    #
    #  basic sum of angles testing:
    s = 'Basic Sum of Angles Testing'
    print '\n\n ' + s + '\n\n'
    
    sp.var('a1 a2 a3')   # note subscripts required
    
    vars01 = [kc.unknown(a1), kc.unknown(a2), kc.unknown(a3)]
    
    i=1
    for v in vars01:   # variables need an index to place them in mechanism order
                       #   must start with 1 for DH/Craig compatibility 
        v.n = i
        i+=1 
   
    term1 = sp.sin(a1+a2)
    term2 = sp.sin(a1+a2+a3)
    term1a, newj, newe = sum_of_angles_sub(term1, vars01)
    fs = ' new equation not correctly established'
    assert str(newj.symbol) == 'th_12', fs
    print 'new eqn', newe
    term2a, newj, newe = sum_of_angles_sub(term2, vars01)
    assert str(newj.symbol) == 'th_123', fs
    print 'new eqn', newe
    
    #print 'Initial Test: '
    #print term1, ' --> ', term1a
    #print term2, ' --> ', term2a
    
    fs = 'Sum of angles: basic test fail'
    assert term1a == sp.sin(th_12), fs
    assert term2a == sp.sin(th_123), fs
    

    #####################################################################
    #
    #   Problem-specific SOA test
    #

    s = 'Problem Specific Sum of Angles Testing'
    print '\n\n ' + s + '\n\n'
    
    sp.var('Px Py Pz')
    unks01 =  [kc.unknown(th_1), kc.unknown(th_2), kc.unknown(th_3), kc.unknown(th_4), kc.unknown(th_5), kc.unknown(th_6)]
    i=1
    for v in unks01:   # variables need an index to place them in mechanism order
                       #   must start with 1 for DH/Craig compatibility 
        v.n = i
        i+=1 
   

    eqnterm = Px*sp.sin(th_2 + th_3 + th_4)*sp.cos(th_1) + Py*sp.sin(th_1)*sp.sin(th_2 + th_3 + th_4) + Pz*sp.cos(th_2 + th_3 + th_4) - a_2*sp.sin(th_3 + th_4) - a_3*sp.sin(th_4) - d_1*sp.cos(th_2 + th_3 + th_4)
    
    term2, newj, newe = sum_of_angles_sub(eqnterm, unks01)
    if newj:
        unks01.append(newj)
    if newe:
        print ' NEW Equation: ', newe
        
    print '---'
    print eqnterm
    print '---'
    print term2
    print '---'
    print 'new unknown list: ', unks01
    
    fs = 'FAIL problem specific SOA tests'
    assert str(term2) == 'Px*sin(th_234)*cos(th_1) + Py*sin(th_1)*sin(th_234) + Pz*cos(th_234) - a_2*sin(th_34) - a_3*sin(th_4) - d_1*cos(th_234)', fs
    assert kc.unknown(th_234) in unks01, fs
    assert kc.unknown(th_34) in unks01, fs
 
 
 
    #####################################################
    #
    #  advanced SOA testing 
    #
    # Generate some SUM of Angles Kin eqns
    
    s = 'Advanced Sum of Angles Testing: DH parameters/UR5'
    print '\n\n ' + s + '\n\n'
    
    
    dh = sp.Matrix([
            [    0    ,    0 ,  d_1 ,     th_1  ],  # based on UR5 but simplified
            [-sp.pi/2 ,    0 ,   0  ,     th_2  ],
            [    0    ,   a_2,   0  ,     th_3  ],
            [    0    ,   a_3,   0  ,     th_4  ],
            [ sp.pi/2 ,   0 ,    0  ,     th_5  ],
            [ sp.pi/2 ,   0 ,    0  ,     th_6  ]
            ])
    vv = [1,1,1,1,1,1]

    variables =  [kc.unknown(th_1), kc.unknown(th_2), kc.unknown(th_3), kc.unknown(th_4), kc.unknown(th_5), kc.unknown(th_6)]
    i=1
    for v in variables:
        v.n = i   # set the variable index (joint number)
        i += 1

    params = [d_1, a_2, a_3]
    pvals = {d_1:1, a_2:1,  a_3:1}  # meters

    robot = 'SOA Test Robot'

    testing = False  # not using this now
    [m, R, tmpvars] = kinematics_pickle(robot, dh, params, pvals, vv, variables, testing)
    print 'GOT HERE: robot name: ', R.name

    variables = tmpvars
    R.name = robot
    R.params = params

    ##   check the pickle in case DH params were changed
    check_the_pickle(m.DH, dh)   # check that two mechanisms have identical DH params

    print '\n the variables: ', variables
    assert len(variables) == 9, 'wrong number of variables'
    print '\n\n'

    # Below replaced by pickle code above

    #m = kc.mechanism(dh, params, vv)
    #m.pvals = pvals  # store numerical values of parameters
    #print "Starting SOA Test Forward Kinematics"
    #m.forward_kinematics()
    #print "Completed Forward Kinematics"
    ##print 'Starting Sum of Angles scan (slow!)'
    # set up Robot Object instance
    #R = Robot(m, 'SOA 2,3 TEST Robot')              # set up IK structs et
    #R.scan_for_equations(variables)       # generate equation lists
    
    
    
    ## below is commented out for testing and devel of sum_of_angles_transform
    
    # need to re-do for debugging if test fails. 
    #R.sum_of_angles_transform(variables)  # find sum of angles



    ####   Temp code:  manually review all the FK matrix equations(!)
    eqn = 0
    row = 2
    col = 3
    print 'Equations: '
    enbr = 0
    for eqn in R.mequation_list:
        print enbr, ':  ', eqn.Td[row,col], '    =    ', eqn.Ts[row,col]
        enbr += 1
    print '\n\n\n'
    ####


    
    #  Now we test some expected RHS results
    sp.var('A1 A2') # two possible answers

    # Assertion 1
    k = 0   # equation
    i = 0   # row
    j = 0   # col
    A1 = R.mequation_list[k].Ts[i,j]
    # expected correct answer:
    A2 = -(sp.sin(th_1)*sp.sin(th_5) - sp.cos(th_1)*sp.cos(th_234)*sp.cos(th_5))*sp.cos(th_6) + sp.sin(th_234)*sp.sin(th_6)*sp.cos(th_1)



    print '\n\n\n'
    print '--- (round 1)'
    print A1          ##   failing to substitute 3-way SOA right now
    print '---'
    print A2
    print '---'
    print sp.simplify(A1-A2)
    print '\n\n\n'
    
    fs = 'ik_classes: sum_of_angles_transform FAILS 3-way sum'
    assert sp.simplify(A1-A2)== 0, fs+' 001'


    # Assertion 2
    sp.var('Px Py Pz')
    k = 5   # equation
    i = 2   # row
    j = 3   # col
    A1 =  R.mequation_list[k].Td[i,j]
    # expected correct answer:
    #A2 =  Px*sp.sin(th_234)*sp.cos(th_1) + Py*sp.sin(th_1)*sp.sin(th_234) + Pz*sp.cos(th_234) - a_2*sp.sin(th_34) - a_3*sp.sin(th_4) - d_1*sp.cos(th_234)
    A2 =  Px*sp.sin(th_234)*sp.cos(th_1) + Py*sp.sin(th_1)*sp.sin(th_234) + Pz*sp.cos(th_234) - a_2*sp.sin(th_34) - a_3*sp.sin(th_4) - d_1*sp.cos(th_234)

    print '\n\n\n'
    print '--- (round 2)'
    print A1          ##   failing to substitute 3-way SOA right now
    print '---'
    print A2
    print '---'
    print sp.simplify(A1-A2)
    print '\n\n\n'
    assert sp.simplify(A1-A2)==0, fs+' 002'


    print '\n\n\n        ik_classes   PASSES all tests \n\n'
