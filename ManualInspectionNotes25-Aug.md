### Notes on commit b69c093500f35313bfe64633fa7885ba5e909ea5 (HEAD -> newSolverStrategies)

## BH cleaned out LaTex output, FK pickles, and CodeGen output and did some manual runs: 

1\.   Looking at LaTex output of Puma, a generic minor issue is that Section 7 (Solutions to Generate all Versions) has duplicated output equations.  After getting the solutions and versions for a solved variable it should be converted by list(set(table column)) to eliminate duplicates if present, even though they are duplicated correctly in the table. 


2\.  Generated solution(s) for KinovaLite.   Solver failed on the original arm (check), generated the min-cost simplified arm (check), Sucessfully solved the simplified arm (check).   Ended without any LaTex output (TBD or failure), python output(TBD or failure), or C++(TBD or failure) output.

Subjectively noted good speed improvements in both. 


3\.  Tried Chair_Helper (an "easy", 5DOF arm)  LaTex output looks good.  As noted earlier,  Jacobian matrix should only have as many rows as the number of joints (in this case 5 but six rows are given).   Should be a trivial fix. 

# Reviewed files:

*create_solution_set()*    Edited comments to remove wordy rationales.   Clarified (at least from my point of view).   Need to check the actual need for solIdxMatrix, line 321: since i< n_rows_solnM for all loop iterations, the whole statement always evaluates to appending 0.  It's not clear that self.n_rows_solnM is ever used??


*check_solution_set.py*   for readability, load_generated() and generated_collums() need 1) more descriptive names, 2) clearer (but NOT substantially longer) docstring descriptions --- what are they for?  Generally, docstrings should define input and return values clearly.  No need to explain in docstrings what the code did wrong before!

*solution_check.py*   indeed seems duplicative.   TODO:  systematically compare these two scripts and create a third one with the best features of both to replace them with.   Meantime, standardize which one will be used from now on before the TODO.





