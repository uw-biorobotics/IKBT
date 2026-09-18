
## Closed-loop test of your robot's solution set

`scripts/check_solution_sets.py` validates that all
symbolic solutions (solution versions) returned by ikSolver are correct: 
it generates random poses, and for each pose

1. generates the end effector config `T = FK(pose)`,
2. generates the joint solution versions `vers = IK(T)`,
3. for each version `v`, computes `T' = FK(v)` and asks whether 
`|T - T'| <= epsilon`.

To run it on your own robot, enter your robot info 
in the list of robot names and in
`robot_params()` in `ikbtfunctions/ik_robots.py`.

Then refer to these example command lines: 
```bash
cd ..
python3 -m scripts.check_solution_sets Puma              # one robot
python3 -m scripts.check_solution_sets Puma Stanford     # multiple robots
python3 -m scripts.check_solution_sets Puma --poses 20   # more random poses (default 10)
python3 -m scripts.check_solution_sets Puma --gate       # exit 1 on a defect
python3 -m scripts.check_solution_sets Puma --verbose    # say why each version failed
```

This checker plugs in and tests the **symbolic** solution set. 
Only use this if IKBT produced a fully symbolic solution.
A separate script,
`scripts/numerical_closed_loop_sol_check.py`, 
tests the **generated Python code** and covers
both the symbolic and the hybrid path from one command:

```bash
python3 -m scripts.numerical_closed_loop_sol_check Puma
``` 


# Unit testing IKBT software

There are several ways to test the IKBT functionality as a whole, and also unit tests of
each piece.

> **Run everything from the project root** (the directory called `IKBT`), **not** from
> `tests/`. Module imports are package-relative.
>
> Use a `.` between `tests` and `leavestest` (not a slash!), and omit the `.py`.

For a wider map of the test system — what each command asserts and where every log and
artifact lands — see [`IKdocs/TESTING.md`](../IKdocs/TESTING.md).

## Unit tests

To test all the leaves (solvers):

```bash
cd ..                          # project root, not /tests
python3 -m tests.leavestest
```

To test the "helperfunctions":

```bash
cd ..
python3 -m tests.helpertest
```

### HTML test report output

`leavestest` can generate a nice HTML summary of the results using the package
`HTMLTestRunner.py` by Wai Yip Tung
(<http://tungwaiyip.info/software/HTMLTestRunner.html>).

To generate an HTML test report:

```bash
cd ..
python3 -m tests.leavestest html
```

Then open a new tab in your browser, hit <kbd>Ctrl</kbd>-<kbd>O</kbd>, navigate to the
project page and open the file `IK-BT_testreport.html`.

If necessary, customize `projdir = '<dir>'` in your `leavestest.py` file to place the file
in the right place.

### Testing an individual leaf

If you are having trouble with a particular solver ("leaf"), you can test it individually
— for example the sincos solver:

```bash
cd ..
python3 -m ikbtleaves.sincos_solver
```

### Testing `ik_classes.py`

This file has not been integrated with `unittest` yet. To test:

```bash
cd ..
python3 -m ikbtbasics.ik_classes
```

## Testing system for IKBT leaves

Overall, testing is mostly integrated into the `unittest` python module's framework.
A test method should be defined in each leaf file.

The testing process usually involves setting up a 2-leaf BT whose root is a sequence node:

- the **first** leaf is a special testing-only node which sets up the environment (chiefly
  blackboard information) for testing the second leaf;
- the **second** leaf is the leaf under test.

This tree is ticked to perform the test, and then the outputs are checked with assertions
in `__main__()`.
