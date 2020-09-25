# Solution graph
# Node: unknown/variable

# Dianmu Zhang Apr 2017

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
import unittest
import sympy as sp
from ikbtbasics.ik_classes import *
import ikbtbasics.kin_cl as kc
from ikbtbasics.matching import *
import itertools as itt

((th_1, th_2, th_3, th_4, th_5, th_6)) = sp.symbols(('th_1', 'th_2', 'th_3', 'th_4', 'th_5', 'th_6'))
((d_1, d_2, d_3, d_4, d_5, d_6)) = sp.symbols(('d_1', 'd_2', 'd_3', 'd_4', 'd_5', 'd_6'))
((a_2, a_3)) = sp.symbols(('a_2', 'a_3'))

possible_unkns = set([th_1, th_2, th_3, th_4, th_5, th_6, d_1, d_2, d_3, d_4, d_5, d_6]) #didn't count th_XY

global notation_graph

def find_node(nodes, symbol): #equivlent function -> find_obj in helper
    for node in nodes:
        if node.symbol == symbol:
            return node

def goal_search(start, parent_notations, graph):
    '''modified BFS'''
    q = []
    q.append(start)
    
    while(len(q) > 0):
        curr = q[0]
        del q[0]
        if curr in parent_notations:
            return curr
        else:
            next_steps = find_edge(curr, graph)
            q.extend(next_steps)
            
    return None
        
def find_edge(child, graph):
    next_level = []
    for edge in graph:
        if edge.child == child:
            next_level.append(edge.parent)
            
    return next_level
    
def related(start_node, end_node):
    '''DFS: return True if a path exists, for Node types'''
    s = []
    s.append(start_node)
    
    #ancestors = set()
    
    while(len(s) > 0):
        curr = s[-1]
        del s[-1]
        
        if curr == end_node:
            return True
        else:
            next_steps = curr.parents
            s.extend(next_steps)
    return False

    
class Node:
    '''Node is a temp class, will be integrate into unknown/variable, or inhirit from it'''
    def __init__(self, unk):
        self.symbol = unk.symbol
        self.argument = unk.argument
        self.solvemethod = unk.solvemethod
        self.eqnlist = []
        self.nsolutions = 0
        self.solutions = []
        self.assumption = []
        self.sol_notations = set() 
        self.parents = []
        self.solution_with_notations = {} # self.notation : kequation
        self.arguments = {}  # argument needing domain testing
        self.solveorder = -1 
        # upper level parent nodes
        self.upper_level_parents = []
    
    def __lt__(self, other):   # for sorting Nodes
        return self.solveorder < other.solveorder 
    
    def __eq__(self, other): #equal judgement, also hashing in python 3.x
        if other != None:
            return self.symbol == other.symbol
        return False
        
    def __hash__(self): #hash function "inherits" from symbol 
        return self.symbol.__hash__()
        
    def __repr__(self): # string representation
        return self.symbol.__repr__()
        
    def detect_parent(self, R):
        if not len(self.solutions) == 0:
            eqn = self.solutions[0] #solutions is a list of keqn
            print(eqn)
            elements = eqn.atoms(sp.Symbol) # get only symbol elements
            for elem in elements:
                if elem in R.variables_symbols: #swap possible_unkns to unknows symbols
                    parent = find_node(R.solution_nodes, elem)
                    self.parents.append(parent)
                     
            # detect redundancy and eliminate higher order parent
            if len(self.parents) > 1:
                self.upper_level_parents = set()
                for par in self.parents:
                    for other_par in self.parents:
                        if par != other_par and related(par, other_par):
                            self.upper_level_parents.add(other_par)
                # convert to list
                self.upper_level_parents = list(self.upper_level_parents)
                for node in self.upper_level_parents:
                    self.parents.remove(node)
                

                    
    def generate_notation(self, R):
        #pass #TODO: generate individual notation
        #  bh change "footnote" to "subscript"
        #  bh: "notation" means indices eg:  th_2011
        #global notation_graph
        
        # TODO: debug the situation where parents are not related
        # but at different levels
        if len(self.parents) == 0: #root node special case
            if self.nsolutions < 2:
                self.sol_notations.add(self.symbol)
                R.notation_graph.add(Edge(self.symbol, -1))
                R.notation_collections.append([self.symbol])
                #print '//////////////////////// 1 sol'
                #print 'generate_notation: curr: ', self.symbol
                self.solution_with_notations[self.symbol] = kc.kequation(\
                        self.symbol, self.solutions[0])
                self.arguments[self.symbol] = self.argument
            else:
                for i in range(1, self.nsolutions + 1):
                    curr = str(self.symbol) + 's' + str(i) #convert to str, then to symbol?
                    curr = sp.var(curr)
                    self.sol_notations.add(curr)
                    R.notation_graph.add(Edge(curr, -1))
                    R.notation_collections.append([curr]) # add into subgroup

                    curr_solution = self.solutions[i-1]
                    self.solution_with_notations[curr] = kc.kequation(curr, curr_solution)
                    #print '//////////////////////// > 1 sol'
                    #print 'curr: ', curr
                    print(self.argument)
                    self.arguments[curr] = self.argument # simple because root
                    

        else:    # Non-root node
            # (find the deepest level of parents and) get *product* of parents
            # getting the product is safe here because we already
            # trimmed the infeasible pairs from last step (redundency detection)
            parents_notation_list = []
            
            if len(self.parents) == 1: 
                # convert to single symbol to list
                for one_sym in self.parents[0].sol_notations:
                    parents_notation_list.append([one_sym])
            elif len(self.parents) == 2:
                parents_notation_list = itt.product(self.parents[0].sol_notations, \
                                        self.parents[1].sol_notations)
            elif len(self.parents) == 3:
                parents_notation_list = itt.product(self.parents[0].sol_notations, \
                                        self.parents[1].sol_notations, \
                                        self.parents[2].sol_notations)
            elif len(self.parents) == 4:
                parents_notation_list = itt.product(self.parents[0].sol_notations, \
                                        self.parents[1].sol_notations, \
                                        self.parents[2].sol_notations, \
                                        self.parents[3].sol_notations)
            elif len(self.parents) == 5:
                parents_notation_list = itt.product(self.parents[0].sol_notations, \
                                        self.parents[1].sol_notations, \
                                        self.parents[2].sol_notations, \
                                        self.parents[3].sol_notations, \
                                        self.parents[4].sol_notations)
            



            isub = 1


            for parents_tuple in parents_notation_list:
                # find all parents notations, this is done outside of
                # the solution loop because multiple solutions share the same parents
                parents_notations = []
                # look for higher level parent notation connected to this parent
                for parent_sym in parents_tuple:
                    parents_notations.append(parent_sym)    
                    for higher_parent in self.upper_level_parents:
                        goal_notation = goal_search(parent_sym, \
                            higher_parent.sol_notations, R.notation_graph)
                        if goal_notation is not None:
                            parents_notations.append(goal_notation)

                for curr_solution in self.solutions:
                    # creat new symbols and link to graph
                    curr = str(self.symbol) + 's' + str(isub)
                    curr = sp.var(curr)
                    self.sol_notations.add(curr)
                                   

                    #R.notation_collections.append(curr)
                    isub = isub + 1
                    # link to graph
                    for parent_sym in parents_tuple:
                        R.notation_graph.add(Edge(curr, parent_sym))
                    # substitute for solutions
                    rhs = curr_solution

                    expr_notation_list= [curr]
                    for parent in (self.parents + self.upper_level_parents):
                        curr_parent = None

                        for parent_sym in parents_notations:
                            if parent_sym in parent.sol_notations:
                                curr_parent = parent_sym
                                expr_notation_list.append(curr_parent)
                        try:
                            rhs = rhs.subs(parent.symbol, curr_parent)
                            tmp_arg = self.argument.subs(parent.symbol, curr_parent) # also sub the arg
                        except:
                            print("problmematic step: ", parent.symbol)
                            print("solution: ", rhs)
                            print("parents notations")
                            print(parents_notation_list)

                    R.notation_collections.append(expr_notation_list)
                        #parents_notations.remove(curr_parent) 
                        # can't remove here, or won't be able to generate solution for
                        # multiple solution case

                    self.solution_with_notations[curr] = kc.kequation(curr, rhs)
                    self.arguments[curr] = tmp_arg   
                        

    def generate_solutions(self, R):
        '''generate solutions with notation(subscript)'''
        pass
       
            

class Edge:
    def __init__(self, child, parent):
        '''child and parent are notation with subscript type'''
        self.child = child
        self.parent = parent
        
    def __repr__(self):
        return "Edge from child: " + str(self.child) + " to parent: " + str(self.parent)
        
    def __eq__(self, other):
        return self.child == other.child and self.parent == other.parent
        
    def __hash__(self):
        return self.child.__hash__() * self.parent.__hash__() + self.child.__hash__()



class SolutionGraphV2Tests(unittest.TestCase):
    # the tests were designed for v1 (independent module), not suitable for v2
    def test_mock(self):
        print("place holder for real test: solution_graph_v2")
    # def test_find_parent(self):
        # '''test detect_parent: th_2 depends on th_1'''
        # R = Robot()
        # v_th1 = Node(th_1)
        # v_th2 = Node(th_2)
        # R.solution_nodes = [v_th1, v_th2]
        # #v_th1.nodes = allnodes
        # #v_th2.nodes = allnodes
        # v_th2.solutions = [kc.kequation(th_2, sp.sin(th_1))]
        # v_th2.detect_parent(R)
        # #print v_th2.parents
        # self.assertTrue(v_th2.parents[0] == v_th1, 'parent finding error') 
        
        
    # def test_notation_generation(self):
        # '''test notation generation'''
        # global notation_graph
        # notation_graph = set()
        # v_th1 = Node(th_1)
        # v_th2 = Node(th_2)
        # v_th3 = Node(th_3)
        # allnodes = [v_th1, v_th2, v_th3]
        
        # v_th1.nodes = allnodes
        # v_th2.nodes = allnodes
        # v_th3.nodes = allnodes
        
        # v_th1.nsolutions = 1
        # v_th1.solutions = [kc.kequation(th_1, a_2)]
        # v_th1.detect_parent()
        # v_th1.generate_notation()
        # #print v_th1.sol_notations
        # expected_notation_1 = set([th_1])
        # self.assertTrue(expected_notation_1 == v_th1.sol_notations)
        
        # v_th2.nsolutions = 2
        # v_th2.solutions = [kc.kequation(th_2, sp.cos(th_1)), kc.kequation(th_2, -sp.cos(th_1))]
        # v_th2.detect_parent()
        # v_th2.generate_notation()
        # #print v_th2.sol_notations
        # note_a = sp.var(str(th_2) + str(1))
        # note_b = sp.var(str(th_2) + str(0))
        # expected_notation_2 = set([note_a, note_b])
        # self.assertTrue(expected_notation_2 == v_th2.sol_notations)
        
    # def test_redundency(self):
        # '''test for redundency detection'''
        # global notation_graph
        # notation_graph = set()
        # v_th1 = Node(th_1)
        # v_th2 = Node(th_2)
        # v_th3 = Node(th_3)
        # allnodes = [v_th1, v_th2, v_th3]
        
        # v_th1.nodes = allnodes
        # v_th2.nodes = allnodes
        # v_th3.nodes = allnodes
        
        # v_th1.nsolutions = 1
        # v_th1.solutions = [kc.kequation(th_1, a_2)]
        # v_th1.detect_parent()
        # v_th1.generate_notation()
        

        # v_th2.nsolutions = 2
        # v_th2.solutions = [kc.kequation(th_2, sp.cos(th_1)), kc.kequation(th_2, -sp.cos(th_1))]
        # v_th2.detect_parent()
        # v_th2.generate_notation()
        # th_20 = sp.var('th_20')
        # th_21 = sp.var('th_21')
        
        # # th_3 depends on th_2 and th_1(th_1 should be eleminated as redundant ancestor)
        # v_th3.nsolutions = 2
        # v_th3.solutions = [kc.kequation(th_3, sp.sin(th_2) + th_1), kc.kequation(th_3, -sp.sin(th_2) - th_1)]
        # v_th3.detect_parent()
        # v_th3.generate_notation()
        
        # expected_notation_3 = set()
        # th_300 = sp.var('th_300')
        # th_301 = sp.var('th_301')
        # th_310 = sp.var('th_310')
        # th_311 = sp.var('th_311')
        # expected_notation_3 = set([th_300, th_301, th_310, th_311])
        
        # self.assertTrue(v_th3.parents[0] == v_th2)
        # self.assertTrue(expected_notation_3 == v_th3.sol_notations)
        
        
        # expected_graph = set([Edge(th_20, th_1), Edge(th_21, th_1), \
                        # Edge(th_300, th_20), Edge(th_301, th_20), \
                        # Edge(th_310, th_21), Edge(th_311, th_21)])
        # # the notation_graph should have the same edges as expected graph
        # self.assertTrue(expected_graph == notation_graph) 
        # for edge in notation_graph:
            # print edge
            
    # def test_solution_generation(self):
        # '''generate solutions with correct notations'''
        # global notation_graph
        # notation_graph = set()
        # v_th1 = Node(th_1)
        # v_th2 = Node(th_2)
        # v_th3 = Node(th_3)
        # allnodes = [v_th1, v_th2, v_th3]
        

        
        # v_th1.nsolutions = 1
        # v_th1.solutions = [kc.kequation(th_1, a_2)]
        # v_th1.detect_parent()
        # v_th1.generate_notation()
        # v_th1.generate_solutions()
        # #print v_th1.solution_with_notations

        # v_th2.nsolutions = 2
        # v_th2.solutions = [kc.kequation(th_2, sp.cos(th_1)), kc.kequation(th_2, -sp.cos(th_1))]
        # v_th2.detect_parent()
        # v_th2.generate_notation()
        # v_th2.generate_solutions()
        # th_20 = sp.var('th_20')
        # th_21 = sp.var('th_21')
        # #print v_th2.solution_with_notations

        # # th_3 depends on th_2 and th_1(th_1 should be eleminated as redundant ancestor)
        # v_th3.nsolutions = 2
        # v_th3.solutions = [kc.kequation(th_3, sp.sin(th_2) + th_1), kc.kequation(th_3, -sp.sin(th_2) - th_1)]
        # v_th3.detect_parent()
        # v_th3.generate_notation()
        # v_th3.generate_solutions()
        
        # th_300 = sp.var('th_300')
        # th_301 = sp.var('th_301')
        # th_310 = sp.var('th_310')
        # th_311 = sp.var('th_311')
        # print v_th3.solution_with_notations
        
        # #for edge in notation_graph:
            # #print edge
        # #tests
        # expected_1 = {th_1: kc.kequation(th_1, a_2)}
        # expected_2 = {th_20: kc.kequation(sp.var('th_20'), sp.cos(th_1)), th_21: kc.kequation(sp.var('th_21'), -sp.cos(th_1))}
        # expected_3 = {th_311: kc.kequation(th_311, -th_1 - sp.sin(th_21)), th_300: kc.kequation(th_300, th_1 + sp.sin(th_20)), \
                        # th_310: kc.kequation(th_310, th_1 + sp.sin(th_21)), th_301: kc.kequation(th_301, -th_1 - sp.sin(th_20))}
                        
        # self.assertTrue(v_th1.solution_with_notations == expected_1)
        # self.assertTrue(v_th2.solution_with_notations == expected_2)
        # self.assertTrue(v_th3.solution_with_notations == expected_3)
        
if __name__ == '__main__':
    #notation_graph = set()
    #unittest.main() 
    
    print('\n\n===============  Test solution_graph_v2 =====================')
    #testsuite = unittest.TestLoader().loadTestsFromTestCase(SolutionGraphV2Tests)  # replace TEMPLATE 
    #unittest.TextTestRunner(verbosity=2).run(testsuite)
    unittest.main()
   
