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
import copy
#import numpy as np
import ikbtbasics.pykinsym as pks
import re
import ikbtbasics.solutionGraphV3 as sg
import ikbtbasics.matching as mtch
import sys as sys
import b3 as b3          # behavior trees
import pickle
from ikbtfunctions.helperfunctions import *
import ikbtfunctions.graph2latex as gl
#from kin_cl import *f
import ikbtbasics.kin_cl as kc

# generic variables for any manipulator
((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((h,l_0, l_1, l_2, l_3, l_4, l_5, l_6)) = sp.symbols(('h','l_0', 'l_1', 'l_2', 'l_3', 'l_4', 'l_5', 'l_6'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))
sp.var('l_5 l_6')
sp.var('th_12, th_23, th_34, th_45, th_56')
sp.var('th_123,th_234,th_345,th_456')
sp.var('c_12 s_12 c_23 s_23 c_34 s_34 c_45 s_45 c_56 s_56 c_13 s_13')
sp.var('x')  # super generic place holder
soa_vars = [th_12, th_23, th_34, th_45, th_56]  # a list of the sum-of-angles variables

soa_expansions= {}
soa_expansions[th_12] = th_1 + th_2
soa_expansions[th_23] = th_2 + th_3
soa_expansions[th_34] = th_3 + th_4
soa_expansions[th_45] = th_4 + th_5
soa_expansions[th_56] = th_5 + th_6
soa_expansions[th_123] = th_1 + th_2 + th_3
soa_expansions[th_234] = th_2 + th_3 + th_4
soa_expansions[th_345] = th_3 + th_4 + th_5
soa_expansions[th_456] = th_4 + th_5 + th_6

pprotocol = 2
#
#   retrieve forward kinematics from a pickle file if it exists.
#      if it doesn't, compute the FK and store it in a pickle file.
def kinematics_pickle(rname, dh, constants, pvals, vv, unks, test):
    #
    #   Check for a pickle file of combined pre-computed Mech and Robot objects
    #
    #  TODO: refactor code to get rid of unused "test" argument

    pickle_dir = 'fk_eqns/'

    if not os.path.isdir(pickle_dir):  # if this doesn't exist, create it.
        print('Creating a new pickle directory: ./'+pickle_dir)
        os.mkdir(pickle_dir)

    name = pickle_dir + rname + '_pickle.p'

    print('kinematics pickle: trying to open ', name,' in ', os.getcwd())

    if(os.path.isfile(name)):
        with open(name, 'rb') as pick:
            print('\Trying to read pre-computed forward kinematics from '+name)
            [m, R, unknowns]  = pickle.load(pick)
            print('Successfully read pre-computed forward kinematics')
            print('pickle contained ', len(unknowns), ' unknowns')
    else:
        #print 'WRONG - quitting, error: ',sys.exc_info()[0]
        #sys.exit
        # set up mechanism object instance
        m = kc.mechanism(dh, constants, vv)
        m.pvals = pvals  # store numerical values of parameters
        print('Did not find VALID stored pickle file: ', name)
        print("Starting Forward Kinematics")
        m.forward_kinematics()
        print("Completed Forward Kinematics")
        print('Starting Sum of Angles scan (slow!)')

        # set up Robot Object instance
        R = Robot(m, rname)              # set up IK structs etc
        R.scan_for_equations(unks)       # generate equation lists

        # below is commented out for testing and devel of sum_of_angles_transform
        R.sum_of_angles_transform(unks)  # find sum of angles

        # for Version 3: solution nodes created as they are solved (unknown.set_solved)
        #R.generate_solution_nodes(unks) # generate solution nodes

        print(' Storing kinematics pickle for '+rname + '('+name+')')
        with open(name,'wb') as pf:
            pickle.dump( [m, R, unks], pf, protocol=pprotocol)
        unknowns = unks    # be sure to return updated unknown list (including SOAs)

    return [m,R,unknowns]


def check_the_pickle(dh1, dh2):   # check that two mechanisms have identical DH params
    flag = False
    if (dh1.shape[0] != dh2.shape[0]):
        print('   Wrong number of rows!')
        flag = True
    else:
        for r in range(0,dh1.shape[0]):
            for c in [0,1,2,3]:
                if(dh1[r,c] != dh2[r,c]):
                    flag = True
    if(flag):
        print('''\n\n -----------------------------------------------------
                    DH parameters Differ
                 Pickle file is out of date.
                   please remove it and start again
  -----------------------------------------------------
  ''')
        quit()

##############################################################################33
#
#     Robot Class

#  Class to contain all robot info
class Robot:
    def __init__(self, Mech=None, name="*Unnamed*"):
        self.name = name
        # the following data pertain to the solution tree for this Robot
        self.solveN = 0   # index of current solution in solving sequence
        self.soltag = ''  # suffix tag for current solution level leafs
        self.params = []  # constant dh params such as l_4 etc.
        self.variables_symbols = []
        self.FinalEqnMatrix = []   # will contain a kequation for each unk and each version
        self.nversions = 0   # will have the number of rows in FinalEqnMatrix


        #
        #   "notations" means specifically labeled solution variables such as
        #          th_1s2  (theta-1, solution 2)
        #
        #   "notations" are deprecated in V3 solution set solver method
        #   TODO: ablation test some of these outdated class members

        self.notation_graph_edges = set() #solution edges between nodes notation graph
        self.solution_nodes = []  #   by solve order
        self.notation_collections = [] #solution notations divided into subgroups
        self.min_index = 0
        self.max_index = 0

        # Forward Kinematics Results
        # mequation_list:        all the 4x4 Matrix FK equations
        self.mequation_list = []
        # kequation_aux_list:    sum of angle eqns such as eg th_23 = th_2+th_3
        self.kequation_aux_list = []
        # a matrix equation in which to embed important kequations equations
        #    this is done for compatibility with the id/solvers
    #    self.SOA_eqns = kc.matrix_equation()
        # To add a kequation, use R.kequation_aux_list.append(neweqn)

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
            print('ik_classes: length Robot.mequation_list: ', len(self.mequation_list))

    #   *********  new fcn for final soln versions
    #  generate the final list of versions for this unknown
    #
    # class Robot
    def make_LHS_versions(self):
        nvers = len(self.solListMatrix)  # n versions
        nunks = len(self.solListMatrix[0])  # n solved unknowns (cols)
        for row in range(nvers): # go through versions
            # get subs dict for this row
            subdict = {}
            for col,n in enumerate(self.solution_nodes):
                unk = n.unknown.name
                subdict[unk] = self.solListMatrix[row][col]

            nsols = self.solution_nodes[col].unknown.nsolutions
            #LHS: variable version #
            eqns_row = []
            for col in range(nunks): # go through unks in solution order
                #make an equation
            #tmp = '$' + sp.latex(node.symbol) + '$'
            #tmp = theta_expand(tmp)
                #LHS = sp.var(self.solution_nodes[col].unknown.LHSversionNames[row]) # no
                LHS =  sp.var(self.solution_nodes[col].name + 'v' + str(row+1)) #versions begin w/ 1
                #LHS = theta_expand(LHS)
                #print('\n\n   make_LHS_versions:         LHS: ', LHS , '\n\n')
                thisSol = self.solution_nodes[col].unknown.solutions[row%nsols]
                RHS = thisSol.subs(subdict) # substitute versions for this row
                thiseqn = kc.kequation(LHS,RHS)
                eqns_row.append(thiseqn)
            self.FinalEqnMatrix.append(eqns_row)
            self.nversions += 1
        print('\n\n')
        for r in self.FinalEqnMatrix:
            print(r)
        print('\n\n')


    #
    #  generate the solution vectors for the robot
    #     first as a list of lists, convert to set of tuples for compatibilty w/ V2
    #
    #class Robot
    def create_solution_set(self):
        # go through nodes in solution order
        solListMatrix = []  # a matrix, each row is a set of versions forming a solution
        verListMatrix = []
        for node in self.solution_nodes:
            u = node.unknown
            u_nvers = u.nversions
            u_nsols = u.nsolutions
            n_rows_solnM = len(solListMatrix)

            # if this unk has multiple solutions, multiply rows (versions)
            if u_nsols > 1: # if current nsol > 1
                for i in range(u_nsols-1): # duplicate rows if needed to accomodate nsols of this unk
                    solListMatrix += copy.deepcopy(solListMatrix)[::-1]
                    verListMatrix += copy.deepcopy(verListMatrix)[::-1]

            if n_rows_solnM > 0:
                # add this node's solutions to form a new column
                for i in range(len(solListMatrix)):  # go through each row and add next unk.
                    rs = solListMatrix[i]
                    # LHS version names (version names start w/ 1)
                    u.LHSversionNames.append(u.name + 'v' + str(i+1)) # not modded by nsolutions.  For final LHS of eqn

                    rs.append(u.LHSversionNames[i])           # version-based version names

            else: # first time through
                #print('first solved unk:', u.details())
                for vn in u.versionNames:  #start new row for all solns of first solved unk.
                    solListMatrix.append([vn])
                    #solListMatrix.append([u.name+'v'+'1'])
                    #print('create_solution_sets: creating ', u, solListMatrix)
            #print('---')
            #print(solListMatrix)
        print('====== SOLUTION LIST COMPLETED: ================')

        self.solListMatrix = solListMatrix  # soltions in list-of-lists form (nversions rows by nunknowns colums)
        self.solutionSet = set()            # solutions as a set of tuples (v2 output compatibility)

        for row in solListMatrix:
            print(row)
        #
        #   code/Latex generaters want a set of tuples(!)
        #
            self.solutionSet.add(tuple(row))

        print('======  SOLUTION LIST IS Set of tuples: self.solutionSet ==============')

        return

    # get lists of unsolved equations having 1 and 2 unks
    #
    def scan_for_equations(self,variables):
        self.l1 = [] # equations with one unk nown (if any)
        self.l2 = [] # equations with two unknowns
        self.l3p = [] # 3 OR MORE unknowns
        sp.var('x')  #this will be used to generate 'algebraic zero'
        elist = self.mequation_list
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
        for e in self.kequation_aux_list:
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
#  Testing use only:
#   Get equation lists from just a matrix equation
#     (this is used when generating tests NOT from DH params)
#
    def scan_Mequation(self,Meqn,variables):
        self.l1 = []
        self.l2 = []
        self.l3p = []
        for eqn in Meqn.get_kequation_list():
            lh1x1 = eqn.LHS  #4x4 matrix
            rh1x1 = eqn.RHS  #4x4 matrix
            n = count_unknowns(variables, lh1x1) + count_unknowns(variables, rh1x1)
            e1 = eqn
            if(n==1):
                if e1 not in self.l1:
                    self.l1.append(e1)   # only append if not already there
            if(n==2):
                if e1 not in self.l2:
                    self.l2.append(e1)    # only append if not already there
            if(n > 2):
                if e1 not in self.l3p:
                    self.l3p.append(e1)    # only append if not already there
        self.l1 = erank(self.l1) # sort the equations (in place) so solvers get preferred eqns first
        self.l2 = erank(self.l2)
        self.l3p = erank(self.l3p)
        return [self.l1, self.l2, self.l3p]


    #
    # identify sum of angles terms and transform them to new variable
    #
    def sum_of_angles_transform(self,variables):
        print('Starting sum-of-angles scan. Please be patient')
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
                    prog_bar(it_number, nits, barlen, 'Sum of Angles')
                    #print 'kij: ', k,i,j

                    #print 'Sum of Angles: eqn,row,col: ', k,i,j
                    # simplify with lasting effect (note: try sp.trigsimp() for faster????)
                    Meq.Ts[i,j] = sp.simplify(Meq.Ts[i,j])  # simplify should catch c1s2+s1c2 etc. (RHS)
                    Meq.Td[i,j] = sp.simplify(Meq.Td[i,j])  # simplify should catch c1s2+s1c2 etc. (LHS)

                    lhs = Meq.Td[i,j]
                    rhs = Meq.Ts[i,j]


                    # simplify LHS

                    lhs, newj, newe = sum_of_angles_sub(self, lhs, variables)

                    if newj:
                        variables.append(newj)
                    if newe:
                        self.kequation_aux_list.append(newe)

                    # simplify RHS
                    rhs, newj, newe= sum_of_angles_sub(self, rhs, variables)

                    Meq.Td[i,j] = lhs
                    Meq.Ts[i,j] = rhs

        prog_bar(-1,100,100, '')  # clear the progress bar

                            #x = raw_input('<enter> to cont...')
        print('Completed sum-of-angles scan.')


##################
#
#   substitute th_23 for th_2+th_3 etc.
# (april: separate out for easier testing)

def sum_of_angles_sub(R, expr, variables):
    aw = sp.Wild('aw')
    bw = sp.Wild('bw')
    cw = sp.Wild('cw')
    newjoint = None
    tmpeqn = None
    found2 = found3 = False
    matches = expr.find(sp.sin(aw+bw+cw)) | expr.find(sp.cos(aw+bw+cw))
    #print '- -  - - -'
    #print expr
    #print matches

    # Sort matches so 3-way SOA subs are processed before 2-way ones.
    # Without sorting, set iteration order is non-deterministic which can
    # cause a 2-way sub (e.g. th_34) to fire first, preventing the 3-way
    # sub (e.g. th_234) from matching later.
    def _soa_nargs(m):
        """Return negative count of non-zero wild args so 3-way sorts first."""
        d  = m.match(sp.cos(aw + bw + cw))
        d1 = m.match(sp.sin(aw + bw + cw))
        if d is not None and d1 is not None:
            d.update(d1)
        if d is None and d1 is not None:
            d = d1
        if d is None:
            return 0
        return -sum(1 for v in d.values() if v != 0)

    matches = sorted(matches, key=_soa_nargs)

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
                ni += c   # build up subscript e.g. 234

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
                print(":  found new 'joint' (sumofangle) variable: ", th_new)
                #  try moving soa equation to Tm.auxeqns
                #unkn_sums_sym.add(th_new) #add into the joint variable set
                newjoint = kc.unknown(th_new)
                newjoint.n = int(ni)  # generate e.g. 234 = 10*2 + 34
                newjoint.solved = False  # just to be clear for count_unknowns
                variables.append(newjoint) #add it to unknowns list
                tmpeqn = kc.kequation(th_new, d[aw] + d[bw] + d[cw])
                print('sum_of_angles_sub: created new equation:', tmpeqn)

                #
                #   Add the def of this SOA to list:  eg  th23 = th2+th3
                #   BUT  it needs to be embedded into a 4x4 mequation so
                #    that solvers can scan it properly
                R.kequation_aux_list.append(tmpeqn)

            # substitute new variable into the kinematic equations

            # Problem Dec'21:
            #     If there is a three-way sub, prefer it to a two-way sub.  e.g:
            #     (a+b+c) -> (abc) instead of (a+bc)(!)
            #

            #self.mequation_list[k].Td[i,j] = Meq.Td[i,j].subs(d[aw] + d[bw] + d[cw], th_subval)
            #self.mequation_list[k].Ts[i,j] = Meq.Ts[i,j].subs(d[aw] + d[bw] + d[cw], th_subval)
            expr = expr.subs(d[aw] + d[bw] + d[cw], th_subval)
            #print 'sum of angles (ik_classes): NEW Eqns (k,i,j)',k,i,j
            #print self.mequation_list[k].Td[i,j]
            #print ' = '
            #print self.mequation_list[k].Ts[i,j]
            #print '========'
    if tmpeqn is not None:
        print('sum_of_angles_sub: Ive found a new SOA equation, ', tmpeqn, 'it is a 3-way SOA: ', found3)
    return (expr, newjoint, tmpeqn)

def get_variable_index(vars, symb):
    for v in vars:
        if v.n == 0:
            print('get_variable_index()/ik_classes: at least one index is not initialized for joint variables (or is 0!)')
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
   # testing for these classes and methods now in tests/leavestest.py
   # TBD   properly integrate with unittest module
   pass

