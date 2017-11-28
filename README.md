# IKBT
A python based system for generating closed-form solutions to the manipulator inverse kinematics problem. 

## Our contributions to automate closed-form kinematics solving are:
 1. We built an autonomous inverse kinematics solver (IKBT) using a behavior tree to organize solution algorithms. 
 1. We incorporated knowledge frequently used (by human experts) when solving inverse kinematics into a behavior tree. These rule-based solvers applicable to any serial-chain, non-redundant, robot arm.
 1. IKBT generates a dependency {\it graph} of joint variables after solving, generating all possible solutions. 
 1. IKBT provides convenience features such as automatic documentation of the solution in \LaTeX and automatic code generation in Python and C++. 
 1.  Implementation in a modern open-source, cross-platform, programming language (Python) with minimal dependencies outside of the standard Python distribution ({\tt sympy}).
 
 
## [Introductory Video](https://youtu.be/bLTXuNZPR5k)

## How to cite: 

Zhang, Dianmu, and Blake Hannaford. "IKBT: solving closed-form Inverse Kinematics with Behavior Tree." arXiv preprint arXiv:1711.05412 (2017).

http://arxiv.org/abs/1711.05412

## Installation Dependencies.

You need the following to be installed to run IKBT:

 * Python 2.7.x [(Python Installation)](https://edu.google.com/openonline/course-builder/docs/1.10/set-up-course-builder/check-for-python.html)
 * Sympy python package [(Installation instructions for all OS)](https://github.com/sympy/sympy/wiki/Download-Installation)
 * Latex package (for nice text output - highly recommended) [(Install Latex)](https://www.latex-project.org/get/)
 




