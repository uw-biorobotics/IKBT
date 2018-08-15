#!/usr/bin/python
#
#     Small helper functions for IKBT debugging
#
 # had to separate tests form helperfunctions b/c of circular imports
 #   see tests/helpertest.py

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import unittest
import sys as sys
import sympy as sp

#######################################################################3
#
#    Some helper functions
#
def print_status(bb, label):
    print "    --- ", label, " ---"
    print "            Solved:   " , bb.get('solutions')
    print "            Unknowns: " , bb.get('unknowns')

def print_debug(label):
    print label

def count_unknowns(unknowns, expr):
    n = 0
    for u in unknowns:
        print u.symbol
        if(expr.has(u.symbol) and u.solved == False):
            n += 1
    return n

#return a list of unknown objects that exsits in a exper
def get_unknowns(unknowns, expr):
    us = []
    for u in unknowns:
        if(expr.has(u.symbol) and u.solved == False):
            us.append(u)
    return us

# return a list of all variables in an expression which
#    belong to a list of variables.

def get_variables(variables, expr):
    vs = []
    #print 'get_variables: ', expr
    for v in variables:
        #print 'get_variables: ', v.symbol
        if(expr.has(v.symbol)):
            vs.append(v)
    return vs

# get the varible(unknown) object by its symbol
def find_obj(th_sym, unknowns):
    for unk in unknowns:
        if unk.symbol == th_sym:
            return unk
    return None


#    TODO:   still needed????
# find a variable higher up in the solution tree
#    which has been used in the current solution

def find_ancestor(node, var):
    if node.parent.root:
        print "find_ancestor: never found a solved variable in the solution tree !", var.symbol
        assert(False), 'caller (set_solved) tried to register a solution with unknown in it!'
        return 0
    else:
        if(node.parent.variable == var):  # we found it
            return node.parent
        else:
            return find_ancestor(node.parent, var)



def ik_lhs():  # generate a simple Left Hand side matrix
    m = sp.zeros(4)
    for i in [1,2,3]:
        for j in [1,2,3]:
            v = 'r_'+'{:1}{:1}'.format(i,j)
            m[i-1,j-1] = sp.var(v)

    m[0,3] = 'Px'
    m[1,3] = 'Py'
    m[2,3] = 'Pz'
    m[3,0] = 0
    m[3,1] = 0
    m[3,2] = 0
    m[3,3] = 1
    return m


#
#  Print a progress bar.  lmax = your full job
#                         l = your current iteration
#   if l<0,  clear the bar.
#   default bar length = 40
#
def prog_bar(l, lmax, msg = ''):
    if l<0:
        print ''
        sys.stdout.flush()

        return
    if l>lmax:
        l = lmax
    n = int(lmax*(float(l)/float(lmax))) + 1
    n2 = lmax - n
    stringval = '='*n + '.'*n2 + '] '+msg
    percent =  int(100*l/float(lmax))
    sys.stdout.write(('\r [%2d%s] ['+stringval) % (percent, '%'))
    sys.stdout.flush()
    return


#######################################################################################

#  TODO:  this is no longer needed

####    from Stackexchange(!)
#http://codereview.stackexchange.com/questions/82802/stack-implementation-in-python
class EmptyStackError(Exception):
    def __init__(self):
        super(self.__class__, self).__init__("Stack is empty: cannot pop an empty stack")


class StackFullError(Exception):
    def __init__(self):
        super(self.__class__, self).__init__("Stack is full")


class stack():
    def __init__(self):
        self.data = []
        self.Nmax = 5

    def isFull(self):
        if (len(self.data) > self.Nmax-1):
            return True
        else:
            return False

    def isEmpty(self):
        if (len(self.data) == 0):
            return True
        else:
            return False

    def Clear(self):   # erase the stack
        self.data = []

    def push(self, data):
        if self.isFull():
            raise StackFullError()
        self.data.append(data)
        return data

    def pop(self):
        if self.isEmpty():
            raise EmptyStackError()
        item = self.data[len(self.data) -1]
        del self.data[len(self.data) -1]
        return item


