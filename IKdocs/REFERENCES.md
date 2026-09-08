# Background reading

Papers behind IKBT's methods. Cited rather than redistributed, except where the licence
explicitly permits it — see **Shipped in this repo** at the end.

See also [parallel_triple_refs.md](parallel_triple_refs.md), which carries the IK-Geo /
subproblem-decomposition literature for the three-parallel-axes case.

## IKBT itself

- Zhang, D. and Hannaford, B. **IKBT: Solving Symbolic Inverse Kinematics with Behavior Tree.**
  *Journal of Artificial Intelligence Research* 65 (2019): 457–486.
  <https://www.jair.org/index.php/jair/article/view/11592> — the theory of operation.
  Shipped here as [IKBT_JAIR_2019.pdf](IKBT_JAIR_2019.pdf).
- Zhang, D. and Hannaford, B. **IKBT: Solving Closed-Form Inverse Kinematics with Behavior
  Tree.** arXiv:1711.05412 (2017). <http://arxiv.org/abs/1711.05412>

## Numerical IK, and why a designer need not insist on a closed form

- Friedman, D.C.W., Kowalewski, T., Jovanovic, R., Rosen, J. and Hannaford, B.
  **Freeing the serial mechanism designer from inverse kinematic solvability constraints.**
  *Applied Bionics and Biomechanics* 7(3) (2010): 209–216.
  doi:[10.1080/11762322.2010.503109](https://doi.org/10.1080/11762322.2010.503109)

  Directly relevant to the hybrid path: a fast numerical IK for a serial manipulator, on the
  argument that with modern computing power a designer can weigh actuator size and safety above
  closed-form solvability. IKBT's hybrid method takes the complementary position — get the closed
  form where it exists, and fall back to damped least squares only where it does not.

## The UR5, IKBT's most instructive failure

`UR5` has three parallel elbow axes and is the robot that motivated
`ikbtleaves/parallel_triple.py` and `ikbtbasics/eqn_sanity.py`.

- Villalobos, J., Sanchez, I.Y. and Martell, F. **Singularity Analysis and Complete Methods to
  Compute the Inverse Kinematics for a 6-DOF UR/TM-Type Robot.** *Robotics* 11(6) (2022): 137.
  doi:[10.3390/robotics11060137](https://doi.org/10.3390/robotics11060137)
  Open access, CC BY 4.0.
- Hawkins, K.P. **Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10 Arms.**
  Georgia Institute of Technology tech report, December 2013.
  <https://repository.gatech.edu/server/api/core/bitstreams/e56759bc-92c8-43df-aa62-0dc47581459d/content>
  A hand derivation for this exact arm — useful for checking generated equations against.
- Kebria, P.M., Al-wais, S., Abdi, H. and Nahavandi, S. **Kinematic and Dynamic Modelling of UR5
  Manipulator.** *IEEE International Conference on Systems, Man, and Cybernetics (SMC)*,
  Budapest, October 2016. ISBN 978-1-5090-1897-0.

## Classical

- Pieper, D.L. **The Kinematics of Manipulators Under Computer Control.** PhD thesis, Stanford
  University, 1968. The source of Pieper's condition, which `ikbtbasics/dh_analysis.py` tests.
- Craig, J.J. **Introduction to Robotics: Mechanics and Control.** Equation numbers such as
  "Craig eqn 4.65" in the source refer to this.

## Shipped in this repo

Only two PDFs are redistributed here, both deliberately:

| file | why it may be shipped |
|---|---|
| `IKBT_JAIR_2019.pdf` | our own paper, JAIR is open access |
| `UR5_Singularity_Analysis_and_Complete_Methods_to_Compu.pdf` | *Robotics* 11(6):137, CC BY 4.0 — redistribution permitted with attribution, given above |

Everything else above is cited by DOI or URL. Three PDFs were removed from `IKdocs/` in Sept 2026
for this reason: the Kebria SMC paper (© 2016 IEEE), the Hawkins tech report (already linked, so
the copy was redundant), and the Friedman paper (© 2010 Taylor & Francis — note Hannaford is a
co-author, so a self-archiving allowance may apply; it was removed as the conservative default,
not because one was checked for).
