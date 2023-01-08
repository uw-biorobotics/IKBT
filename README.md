# IKBT
A python based system for generating closed-form solutions to the manipulator inverse kinematics problem using
behavior trees for action selection. 
Solutions are fully symbolic and are output as LaTex, Python, and C++.

# Current News

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


# IKBT Overview
## Our contributions to automate closed-form kinematics solving are:
 1. We built an autonomous inverse kinematics solver (IKBT) using a behavior tree to organize solution algorithms. 
 1. We incorporated knowledge frequently used (by human experts) when solving inverse kinematics into a behavior tree. These rule-based solvers applicable to any serial-chain, non-redundant, robot arm.
 1. IKBT generates a dependency {\it graph} of joint variables after solving, generating all possible solutions. 
 1. IKBT provides convenience features such as automatic documentation of the solution in \LaTeX and automatic code generation in Python and C++. 
 1.  Implementation in a modern open-source, cross-platform, programming language (Python) with minimal dependencies outside of the standard Python distribution ({\tt sympy}).
 
# Track Project Forks
 * [ Current list of Forks ](https://github.com/uw-biorobotics/IKBT.git)

# Videos
 * [Introductory Video](https://youtu.be/bLTXuNZPR5k)  (6min)
 * [How to set up IKBT for your own robot arm](https://youtu.be/hXzY5vrvWkU) (6.5min)


# Details
## How to cite: 

Zhang, Dianmu, and Blake Hannaford. "IKBT: solving symbolic inverse kinematics with behavior tree." Journal of Artificial Intelligence Research 65 (2019): 457-486.  [Link](https://www.jair.org/index.php/jair/article/view/11592)

Zhang, Dianmu, and Blake Hannaford. "IKBT: solving closed-form Inverse Kinematics with Behavior Tree." arXiv preprint arXiv:1711.05412 (2017).
[Link](http://arxiv.org/abs/1711.05412)

## Installation Dependencies

You need the following to be installed to run IKBT:

 * Python 2.7.x [(Python Installation)](https://edu.google.com/openonline/course-builder/docs/1.10/set-up-course-builder/check-for-python.html)
 * Sympy python package [(Installation instructions for all OS)](https://github.com/sympy/sympy/wiki/Download-Installation)
 * Latex package (for nice equation output - highly recommended) [(Install Latex)](https://www.latex-project.org/get/)
 
## Tested robots, DH parameters & other technical details to reproduce the results

A list of all DH parameters tested in the paper:
['Puma', 'Chair\_Helper', 'Wrist', 'MiniDD', 'Olson13','Stanford',
'Sims11', 'Srisuan11', 'Axtman13', 'Mackler13', 'Minder13', 'Palm13', 
'Parkman13', 'Frei13', 'Wachtveitl', 'Bartell', 'DZhang', 'Khat6DOF'.]

We suggest you first run the Wrist since it is relatively fast:

 > python ikSolver.py Wrist 

To solve your own problem open the file ikbtfunctions/ik_robots.py and create an entry 
for your robot.  You should copy an entry for an existing robot and edit it's entries. 
Create an "unknown" for each joint variable and package them into the vector "variables".
Enter the DH parameters in matrix form.   Also, enter the name of your robot into the list
of valid names (ikbtfunctions/ik_robots.py, line 31).

DH parameters explained:
The vector "vv" encodes whether each joint is rotary (1) or prismatic (0).   If your 
robot is less than 6 DOF, create empty rows:  [      0 ,     0,   0,   0  ], in the 
DH table so that it has six rows.  Many standard symbols in robot kinematics are pre-defined
for you but if you use any new ones, be sure to define them using sp.var().  See "Wrist" 
for an example in which the three joint variables "A, B, C" are set up for sympy by
sp.var('A B C'). "pvals" is where you can put in the numerical values for all parameters, for 
result verification purposes.

Pre-computed forward kinematics.

Sometimes computation of the forward kinematic equations (and their subsequent 
simplification) can be time consuming.   When debugging an inverse kinematics 
solution (for example modifying the BT), it can slow the cycle if these have to 
be redone each time.   Therefore, the software has a mechanism using Python 
"pickle" files, to cache the forward kinematics computation and not repeat it.
Forward kinematics pickle files are stored in the directory fk_eqns/.  This 
directory will be automatically created if you don't have it.  In some cases you 
may have to delete the pickle file for your robot.  To do that, >rm 
fk_eqns/NAME_pickle.p.  IKBT will generally tell you when you should do this, 
but it is OK to just >rm -rf fk_eqns/ .


