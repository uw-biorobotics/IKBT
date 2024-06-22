# IKBT
A python based system for generating closed-form solutions to the manipulator inverse kinematics problem using
behavior trees for action selection. 
Solutions are fully symbolic and are output as LaTex, Python, and C++.

# Latest News

## June 2024

* We have done a complete re-write of the solution set generation process with seemingly good results.
  We develop the table of solutions directly instead of relying on a solution graph,
  adding rows as each solved variable creates combinations [Explanation and Example](https:../blob/main/IKdocs/solExamp.pdf)  
* One of our main learnings earlier in this project was that robot kinematic equation solutions in general are NOT described 
by a tree structure!  Instead they are a more general graph.  Our previous solution
was overly complex due to lingering assumptions from the tree structure idea. 

* Current status is
that FK and IK solutions seem correct, latex output is correct and shows all versions.  A new section in the Latex output
also gives generic "textbook" style solutions without all the solution set permutations.
We have not yet done a full "closed loop" test on these new solution vectors as described in the JAIR paper.

* New solution version system is fully integrated with Latex, Cpp, and Python generation outputs.

* The new approach to solution sets is explained in a document [IKdocs/solExample.pdf](https:../blob/main/IKdocs/solExamp.pdf).

* There is lots of code from the old method ("v2") that need to be cleaned up, as well as several superfluous class members
  in Robot and unknown classes.


## Jan 2024
* New feature: Can generate python code for Forward Kinematics only.  This code can be sub-optimal, and equations can be over complex
when the alpha parameter is not a nice multiple of pi/2 (sin(al) != {0,1}).  This is the case for a few robots like Raven-II.
Now in this case constants with the sine and cosine values of alpha are automatically created and swapped in to the FK equations.
Generated code initializes the new constants.   Try >python3 fkOnly \<RobotName\>.  Output is in the CodeGen directory.

## [Old News](IKdocs/oldNews.md)

# IKBT Overview
## Our contributions to automate closed-form kinematics solving are:
 1. We built an autonomous inverse kinematics solver (IKBT) using a behavior tree to organize solution algorithms. 
 1. We incorporated knowledge frequently used (by human experts) when solving inverse kinematics into a behavior tree. These rule-based solvers applicable to any serial-chain, non-redundant, robot arm.
 1. IKBT generates a dependency {\it graph} of joint variables after solving, generating all possible solutions. 
 1. IKBT provides convenience features such as automatic documentation of the solution in \LaTeX and automatic code generation in Python and C++. 
 1.  Implementation in a modern open-source, cross-platform, programming language (Python) with minimal dependencies outside of the standard Python distribution ({\tt sympy}).
 
# Track Project Forks
 * [ Current list of Forks ](https://techgaun.github.io/active-forks/index.html#uw-biorobotics/IKBT)

# Videos
 * [Introductory Video](https://youtu.be/bLTXuNZPR5k)  (6min)
 * [How to set up IKBT for your own robot arm](https://youtu.be/hXzY5vrvWkU) (6.5min)


# Details    

### Documenation:  
   * [ Theory of Operation ](https://www.jair.org/index.php/jair/article/view/11592)
   * [ Major Classes ](https:../blob/main/IKdocs/classes.md)
   * [ Solution Set Generation Strategy Doc ](https:../blob/main/IKdocs/solExamp.pdf) (updated June 2024)

## How to cite: 

Zhang, Dianmu, and Blake Hannaford. "IKBT: solving symbolic inverse kinematics with behavior tree." Journal of Artificial Intelligence Research 65 (2019): 457-486.  [Link](https://www.jair.org/index.php/jair/article/view/11592)

Zhang, Dianmu, and Blake Hannaford. "IKBT: solving closed-form Inverse Kinematics with Behavior Tree." arXiv preprint arXiv:1711.05412 (2017).
[Link](http://arxiv.org/abs/1711.05412)

## Installation Dependencies

You need the following to be installed to run IKBT:

 * Python 3.8.x [(Python Installation)](https://edu.google.com/openonline/course-builder/docs/1.10/set-up-course-builder/check-for-python.html)
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


