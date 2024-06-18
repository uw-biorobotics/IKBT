# IKBT Older News Updates


## Dec 2021
* Release v2.2 has refactored one of the more arcane solution nodes (x2y2solver)  so that it instead transforms
the equation for later solution by any solver.  KawasakiRS007L can now be solved!   (Still an obscure bug
with sum-of-angle substitution, see commends in ik_classes.py)

* Latex output has been refactored.   New Latex procedure is simplified as follows:
  1) run IKBT
  2) cd LaTex
  3) pdflatex ik_solutions_ROBOTNAME.tex 
  
* A new top-level is provided to do Forward Kinematics and Jacobian matrix only. 
  1) python fkOnly.py ROBOTNAME
  2) cd LaTex
  3) pdflatex fk_equations_ROBOTNAME.tex 
  
## Nov 2021
We've accumulated experience from many installations with the help of students in 
ECE 543 (University of Washington).  One issue that came up is that with python 3.6.X 
something fails in the complex graph of imports.  This works fine with python 3.8.X.
We haven't attempted to fix this. 

Recent commits to **main** have corrected some remaining python 2 hangovers in Latex outout
and python output (even python 3 code can *generate* pytyon 2 print statements!).

## Sept 2021  Version 2.0
At long last the 3-way sum of angles feature is upgraded to python3 and merged into main branch.
All tests are passing, UR5 and Puma both solve all unknowns.   USE MAIN BRANCH.

## July 2021
Unit test programs were broken for python3.  This is now fixed.  Test programs work and all pass.
## March 2021
Upgraded to Python3 (mostly just adding parens to print statements and new python3-sympy).  New "main" branch for more respectful terminology.

## Feb 2020
BH has fixed the UR5 regression - it now works again and solution output no longer shows sum-of-angles variables that are not needed in the actual solution.   
Use branch **RepairUR5Regression** for latest and greatest version. 

## Feb 2020
BH has fixed the UR5 regression - it now works again and solution output no longer shows sum-of-angles variables that are not needed in the actual solution.   
Use branch **RepairUR5Regression** for latest and greatest version. 

## July 2019
Work aimed at \#15 and \#18 is still ongoing (it's tough!).  But a few smaller bugs have been fixed which are now in the 'testing' branch.   Please try that branch and post issues. THX.

## May 2019  
BH is working on a new implementation of the solution graph and generating the list of solutions (much trickier that it seems
at first!).   I've taken this work to a private repo fork to decluter this page, but will merge and commit shortly. 
This work is aimed at issues \#15 and \#18.  Thanks to you new issue posters!

##  Aug 2018
Sum-of-Angles transform now works for the case of three angles (corresponds to three parallel axes in the 
mechanism).   IKBT can now solve the UR5 and similar robots!