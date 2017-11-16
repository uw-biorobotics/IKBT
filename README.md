# IKBT
A python based system for generating closed-form solutions to the manipulator inverse kinematics problem. 

Our major contributions to automate closed-form kinematics solving are:
 1. We built an autonomous inverse kinematics solver (IKBT) using a behavior tree to organize solution algorithms. 
 1. We incorporated knowledge frequently used (by human experts) when solving inverse kinematics into a behavior tree. These rule-based solvers applicable to any serial-chain, non-redundant, robot arm.
 1. IKBT generates a dependency {\it graph} of joint variables after solving, generating all possible solutions. 
 1. IKBT provides convenience features such as automatic documentation of the solution in \LaTeX and automatic code generation in Python and C++. 
 1.  Implementation in a modern open-source, cross-platform, programming language (Python) with minimal dependencies outside of the standard Python distribution ({\tt sympy}).
 
How to cite: 

http://arxiv.org/abs/1711.05412




