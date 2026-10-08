# Eigen, vendored

This is **Eigen 3.4.0**, unmodified. It contains the `Eigen/` header tree plus the upstream
license files (`COPYING.*`) and README.

- Source: https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
- sha256 of that tarball: `8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72`
- License: MPL2 (`COPYING.MPL2`). Eigen's own `COPYING.README` explains the others.

**Why it is vendored.** Generated C++ must build with nothing but a compiler:
`g++ -std=c++17 -O2 <Robot>.cpp`, with no package install and no `-I`. Eigen is header-only,
so a copy here is all that is needed. `Cpp_src/ikbt_types.h` reaches it by a relative
`#include`, and Eigen's own headers include each other relatively too.

**Using a system Eigen instead.** An application that already uses its own Eigen should define
`IKBT_SYSTEM_EIGEN`. `ikbt_types.h` then includes `<Eigen/Dense>` from the include path and
this copy is ignored, so a program never contains two Eigens.

**To upgrade**, replace `Eigen/` and the `COPYING.*` files from a new release, update the
version and hash above, and rerun the checks in `IKdocs/TESTING.md`. Eigen 5.x needs C++14,
which C++17 already covers.
