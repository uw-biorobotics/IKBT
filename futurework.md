
## Approaches for planning further work


1\.  A sufficient condition for solvability of the FK equations is that three axes intersect at a point or three consecutive axes are parallel (interesect at infinity).  If IKBT fails to solve a robot lacking this condition,
we should consider a new hybrid symbolic-numerical version.   

  1\.1  Can identify the numerically smallest DH parameter modification to a robot design which does not have 3 interesecting axes to give
  it 3 interesecting axes?    For example, Kinova as a small offset d5 which apparently breaks its solvability.   By small I mean that 
  it's magnitude of 10mm is small compared to all other dx and lx parameters.    Let's develop a tool which finds a simplified set of DH 
  parameters having three interescting axes which is therefor solvable. 
  
  2\.2  Once we have this, I propose some new BT nodes which can use a gradient descent numerical method (driven by the symbolic Jacobian
  matrix).   A closed form solution to the simplified manipulator will generate a set of ``approximate'' joint solutions to a given 
  end effector configuration.   Then gradient descent will get a suitable numerical solution (based on some specified error metric for 
  end-effector position/orientation) correcting the selected symbolic solution. 
  
  2\.3  The existing BT will be modified to first try to solve the robot in pure symbolic form.   If that fails (Sequence node) a new
  Sequence node  will 1) perform step 1.1 above.  2) solve the simplified manipulator, 3) generate python and C++ code which includes 
  solutions to the simplified manipulator followed by numerical gradient descent code. 
  
  2\.4  Develop test cases for this approach.   For example there may be a robot whose simplified form is not close enough to properly 
  initialize the numerical method.   Or, this hybrid method may break down in the region of singularities of the Jacobian matrix. 
  These failure modes (and others) should be systematically studied.
  

2\.  Consider also expandign the repertoire of ID and Solver nodes to additional mathematical methods.  Are they any other trig identities
or solution methods that we can exploit?   What about the case where we have two equations and two unknowns?  We exploit that idea when we
consider sin(theta_i) and cos(theta_i) to be two "unknowns", but what if we have two equations containing two unknowns (theta_i, theta_j)??
