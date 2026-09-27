## Manual work by BH on IKBT   26-Sept ---

### General work description

Cleaning up verbose comments.
Inserting TODOs where code looks like it might be fixing an old problem we no longer have (etc.).


### Specific files edited:

  * kin_cl.py: comment out chain of "self-reference warning" methods - don't think still needed.
  
  * Working on ik_classes.py - minor edits / comment cleanup
  
  * output_latex.py - only use of "self-reference warning" 
  
  * numeric_ik.py:   TODO: refactor dof_of() to be a new member of mechanism class which 
                       is initialized at _init_() stage.
                       
  * bt_assembly.py:    BT assembly with multiple copies seems overly abstract and complex.  Why are 
       multiple copies of the BT necessary?  Couldn't one tree be re-used with careful management of
       the Blackboard??     Logic of why a dictionary of leaves is required is 
        described in opaque and obtuse comments.   Can this be streamlined?   Original purpose of the 
        whole bt_assembly.py file was to just break up a long code block ikSolver.py into modules.  Did 
        this refactor get out of hand?
  * hybrid_ik.py:    Cleaning up verbose comments.
        Several new TODOs - variable pieper_ok has unclear definition and lots of comments idicates a 
        confusing role for this flag.   TODO: clean this stuff up!!
        
  * parallel_triple:  Clarified comments and user output messages.
  
  * two_eqn_m7.py:   Comment cleanup and TODO suggeted consolidation.
  
  * ik_Driver.py:  Comment cleanup - looking good overall.
  
  * helperfunctions.py:  minor cleanup.
  
  * graph2latex.py: comment tweaks
  
  * output_onevar_python.py:  comment cleanup.  Some testing.   KinovaLite got 4 solutions 1 time
         (instead of 8)
         but I put in a loop and ran thousands more while keeping solution counts and couldn't reproduce it.
      Note that 8 solutions means only two roots of the th_1 root finder. 
      
  * output_hybrid_python.py: comment clarifications and a TODO or two.
  
  * progress.py   TODO: is simplify metering still in use/ still needed??
  
  * subexpressions.py:   Lots of comment cleanup.  
  
  * output_numeric_common:   Comment cleanup.
  
  * output_python.py:  Comment cleanup.
  
  * texwidth.py:  Comment cleanup
  
  * clear_state.py: Comment cleanup
  
  * onevar_ik.py:  Comment cleanup
  
  * symbolic_loop.py: Comment cleanup
  
  * updateL.py: Comment cleanup and TODO
  
  * expected.py:  Do we still need this file for our current testing?
  
  * numercial_closed_loop_sol_check.py:   Deleted from comments waffle words from older versions.
  
  
