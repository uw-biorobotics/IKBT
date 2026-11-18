# Thoughts on the recent solution finding problem

We initially posed the problem is that a solver outputs one or more "solutions" which are dependent on previously solved joint variables (dependencies).  Once specific values of the dependencies were solved and classified into "versions" (based on their own dependencies), the versions arising from the current solver could be computed.

Now we have identified an additional situation, where there is, for example 1 solution, but there are two possible values of that solution depending on the value of a variable which has not yet been solved.  We will call these variables "forward-dependencies".  (By the way, we can add forward-dependencies to our solution graph (perhaps as dotted links)).

So besides a single solution and multiple solutions we also have "contingent solutions" which have a definite value (or perhaps a small set of values) but it/they depend on a to-be-solved forward-dependency.

We can storing information after each variable is solved which is sufficient to identify all solution vectors AFTER all unknowns have been solved (which disambiguates those with a forward-dependence)

Now if the forward dependency never gets solved we do not have a symbolic solution and we can just declare "FAILURE".   On the other hand, there are two situations that can arise.  

First, the forward-dependency solution is independent of the earlier solution which uses it.  This is straightforward. 

On the other hand though, we could have a situation in which the forward-dependency variable requires the earlier variable for its solution.  This presents a new problem.  It would be good to figure out which of these situations emerge in our problem robots so far.   Such a "dependency loop" could be 'stable'(converging to one solution) or 'unstable' (where setting the forward dependency changes its own solution via the initial variable). 

I'm thinking we should refactor the solution generation logic to only keep data as the variables are solved and then to generate the solution graph and the solution table AFTER all variables are solved. 



