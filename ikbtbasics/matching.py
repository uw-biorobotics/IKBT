# Copyright 2017 University of Washington

# Developed by Dianmu Zhang and Blake Hannaford
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.




# Dianmu Zhang
# matching function: this method groups specific joint
# solutions ('notations', e.g. th_1s4) into the correct sets:
#
# [
# [th1s01, th2s01, th3_s01, th4s01 ... etc ],
# [th1s02, th2s02, th3_s02, th4s01 ... etc ],
# etc ....
# ]
#
# which is what control software requires for correct poses
# How to use:
# insert "final_groups = matching_func(R.notation_collections,
# R.solution_nodes)"
# after the BT is ticked, and solution_graph is generated
# example, in test_Robots_new.py line 351

import unittest
import sympy as sp
from ik_classes import *
import kin_cl as kc
from solution_graph_v2 import *

# find subset contains certain symbol
def find_subset(notation_collections, symbol):
    for subset in notation_collections:
        if symbol in subset:
            return subset
# delete some subset
def delete_subset(notation_collections, subset):
    for i in range(len(notation_collections)):
        if subset == notation_collections[i]:
            del notation_collections[i]
            return notation_collections
# sort notation_collections' sets by their contents
# start with the ones with most notations
def sort_by_length(notation_collections):
    notation_d = {}
    max_len = -1
    for subset in notation_collections:
        curr_subset = subset
        try:
            if len(subset) not in notation_d.keys():
                notation_d[len(subset)] = []
            notation_d[len(subset)].append(subset)
            if len(subset) > max_len:
                max_len = len(subset)
        except:
            print "problematic step"
            print subset
        
    return notation_d, max_len
    
# find groups that contains the goals
def search_notation(search_list, goals, start):
    potential_groups = []
    for subgroup in search_list:
        for goal in goals:
            for goal_notation in goal.sol_notations:
                if goal_notation in subgroup:
                    potential_groups.append(subgroup)

    # sort them!
    # 1) number of elements 2) how many common elements
    # assign scores
    group_d = {}
    for pten_group in potential_groups:
        score = len(pten_group)
        for sym in pten_group:
            if sym in start:
                score = score + 1
        if score not in group_d.keys():
            group_d[score] = []
        group_d[score].append(pten_group)

    keys = group_d.keys()
    keys = sorted(keys, reverse= True)
    sorted_potential_groups = []
    for i in keys:
        sorted_potential_groups.extend(group_d[i])

    return sorted_potential_groups


# check to see if all variables are found
def finish_found(check_list):

    for check in check_list:
        if not check:
            return False
    return True
# check off the variables that are found
def mark_off(notation_set, solution_nodes):
    check_list = []
    goals = [] #what to look for
    start_pts = [] #what the set already has
    for node in solution_nodes:
        check_list.append(False)
        for single_notation in notation_set:
            if single_notation in node.sol_notations:
                check_list[-1] = True
        if check_list[-1] == False:
            goals.append(node)
        else:
            start_pts.append(node)

    return check_list, goals, start_pts

# main function of matching
def matching_func(notation_collections, solution_nodes):
    notation_d, max_len = sort_by_length(notation_collections)
    if(max_len) < 1:
        print 'matching.py: bad notation collection'
        quit()
    start_list = notation_d[max_len] # get lists with most variables
    
    final_group = set()
    goals = []
    contained = []

    # go through the start_list, and try find their matches
    for start in start_list:
        if len(start) == len(solution_nodes): # if it has all the unknown symbols
            return notation_d[max_len]
        else:
            # check off the list
            print "looking missing pieces for: "
            print start

            check_list, goals, contained = mark_off(start, solution_nodes)
            # find groups contains the target   
            potential_groups = search_notation(notation_collections, goals, start)  
#
            new_set = start[:]
            new_set = set(new_set)

            # go through the pential groups
            # merge when find match
            for pten_group in potential_groups:
                #print "currently at: %s"%pten_group
                group_to_add = set(pten_group)
                # if there's overlapping, merge
                for single_notation in start:
                    if single_notation in pten_group:
                        has_new_sym = False
                        
                        # check if symbols already in the merged group                        
                        for sym in pten_group:
                            for i in range(len(solution_nodes)):
                                if sym in solution_nodes[i].sol_notations:
                                    if check_list[i]:
                                        has_new_sym = True

                        
                        # check if there's conflict, e.g.: th3s1 vs. th3s2
                        no_conflict = True
                        for sym in pten_group:
                            for sym_in_set in new_set:
                                if not(sym == sym_in_set): 
                                    for i in range(len(solution_nodes)):
                                        if (sym in solution_nodes[i].sol_notations) \
                                            and (sym_in_set in solution_nodes[i].sol_notations):
                                            
                                                no_conflict= False
                        # if none of the symbols clashes
                        if no_conflict:
                            print "no conflicts"
                                

                        if has_new_sym and no_conflict:
                            print "merging (1)"
                            print pten_group
                            print "\n"
                            new_set = new_set.union(group_to_add)
                            check_list, goals, contained = mark_off(new_set, solution_nodes)
                            #print "current goals: %s"%goals


                
                common_ele = group_to_add.intersection(new_set)
                # no commen elements, needs to satisfy two conditions
                if len(common_ele) == 0: 
                    countains_unmarked = False
                    for goal in goals:
                        for single_notation in pten_group:
                            if single_notation in goal.sol_notations:
                                #print "contains unmarked %s"%goal
                                countains_unmarked = True

                    no_repeats = True
                    for pt in contained:
                        for single_notation in pten_group:
                            if single_notation in pt.sol_notations:
                                #print "repeat: %s"%single_notation
                                no_repeats = False

                    if countains_unmarked and no_repeats:
                        print "merging (2)"
                        print pten_group
                        print "\n"
                        new_set = new_set.union(group_to_add)
                        check_list, goals, countained = mark_off(new_set, solution_nodes)


                if finish_found(check_list):
                    new_list = list(new_set)
                    sorted_ls = sort_variables(new_list, solution_nodes)
                    sorted_tp = tuple(sorted_ls)
                    final_group.add(sorted_tp)
                    print "sorted finished list:"
                    print sorted_ls
                    print "\n"
                    

                    break
    # this is sorted
    return final_group

# sort the list
def sort_variables(one_list, solution_nodes):
    # find the symbol one by one
    sorted_list = []
    for node in solution_nodes:
        for ele in one_list:
            if ele in node.sol_notations:
                sorted_list.append(ele)

    return sorted_list
