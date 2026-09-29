//
//  ikbt_search.h --  the 1-D search over the assumed variable
//
//  The C++ twin of output_onevar_python.SEARCH_CORE, mechanism for mechanism.
//
//  GENERIC, WHERE THE PYTHON IS EMITTED PER ROBOT.  The python search has the
//  robot's name in every function because a generated python module stands on
//  numpy alone and has no library to call;  here Cpp_src/ IS that library, so
//  the algorithm is written once and takes the two robot-specific pieces --
//  "what does the closed form give at this value" and "how wrong is it" -- as
//  callbacks.  That is the same code doing the same thing, and it means the
//  search can be unit-tested against a toy whose roots are known, which is
//  how TestSolver029 tests the python one.
//
//  WHAT IT IS FOR.  The conditional closed form solves every OTHER joint
//  given a value for the assumed variable.  Feed it the wrong value and it
//  still returns joint vectors -- they just do not reach the goal pose.  So
//  the pose error is a function of one variable and the true arm's solutions
//  are its ZEROS.
//
//  THE RESOLUTION IS REACHED, NOT ASSUMED, and roots hide in THREE ways, so
//  there are three mechanisms.  Each finds cases the others cannot:
//
//    - a DOUBLING LADDER: scan at n, then 2n, 4n, ... stopping when two
//      successive resolutions agree about the whole set of postures.  Doubling
//      a uniform grid re-uses every coarse point exactly (n is a power of two,
//      so lo + span*(2k)/(2n) is the SAME double as lo + span*k/n) and every
//      sample's error is cached, so the ladder costs little more than its
//      finest rung.  It is the general mechanism and it is NOT proof: two
//      successive grids can step over the same pair.
//
//    - a LOCAL TWIN HUNT: a root a grid hides is BY DEFINITION within one grid
//      step of a root the grid found, so every accepted root gets its
//      neighbourhood re-scanned at 16x the resolution, and anything new found
//      there re-scanned 16x finer again.  Cost goes with the number of roots,
//      not the size of the domain.
//
//    - a DOMAIN-EDGE PROBE: a basin can be narrower than any affordable grid,
//      and when it is, it is pressed against the edge of the branch's domain,
//      because that edge is where the closed form's denominators vanish and
//      its arcsines leave range -- so it is where the pose error moves
//      fastest.  Approaching from inside, the curve RISES first, so the
//      nearest sample is a local MAXIMUM and the ordinary scan never brackets
//      it at any resolution.  So: bisect every finite/infinite transition down
//      to the edge, then walk back inward in HALVING steps.  A geometric
//      ladder resolves a feature at any scale for the price of its logarithm.
//
//  ONE ERROR CURVE PER BRANCH.  The branches are separate functions of the
//  assumed variable with separate zeros and separate domains;  mixing them
//  would bracket minima no single branch has.
//
//  ACCEPTED ONLY IF IT REACHES ZERO.  This is root finding wearing
//  minimisation's clothes: a local minimum that stops above accept_tol is a
//  feature of the curve, not a solution, and returning it is worse than
//  returning nothing.
//
//  SOURCE, not generated -- see Cpp_src/ikbt_types.h.
//
//  Copyright 2026 University of Washington
//
//  Developed by Blake Hannaford
//  BioRobotics Lab, University of Washington

#ifndef IKBT_SEARCH_H
#define IKBT_SEARCH_H

#include <algorithm>
#include <functional>
#include <map>

#include "ikbt_types.h"

namespace ikbt {

//  One solution:  the python twin's dict, field for field.
struct OneVarSolution {
    JointVec q;             // NDOF floats, JOINT_NAMES order, ready for FK
    double known_value;     // the value of the assumed variable it sits at
    int branch;             // which branch of the closed form it came from
    double error;           // ||dp|| + w_rot*theta -- what accept_tol measures
    int n_samples;          // the resolution the answer settled at
};


struct SearchConfig {
    double lo;
    double hi;
    bool periodic;
    int n_samples;          // where the ladder STARTS
    int max_samples;        // ... and where it gives up
    double accept_tol;
    double dedup_tol;
    int refine_iters;

    SearchConfig()
        : lo(-M_PI), hi(M_PI), periodic(true), n_samples(128),
          max_samples(4096), accept_tol(1e-6), dedup_tol(1e-6),
          refine_iters(80) {}
};


//  The two robot-specific pieces.
typedef std::function<std::vector<JointVec>(const Mat4 &, double)> BranchesFn;
typedef std::function<std::vector<double>(const Mat4 &, double)>   ErrorsFn;

typedef std::map<double, std::vector<double> > ErrorCache;


//  n evenly spaced values of the assumed variable, ASCENDING.
//
//  On a periodic domain the two ends are the same point, so the top one is
//  left off and there are exactly n;  on a finite range both ends are wanted,
//  so there are n + 1.
//
//  Written as lo + span * (k / n) rather than by accumulating a step: for n a
//  power of two, k/n is exact in binary and (2k)/(2n) is the SAME double,
//  which is what lets the ladder re-use the coarse grid's cached errors.
inline std::vector<double> sample_values(const SearchConfig &cfg, int n)
{
    const double span = cfg.hi - cfg.lo;
    const int top = cfg.periodic ? n : n + 1;
    std::vector<double> out;
    out.reserve(top);
    for (int k = 0; k < top; ++k)
        out.push_back(cfg.lo + span * (k / (double) n));
    return out;
}


//  A value back into [lo, hi) on a periodic domain.
inline double wrap_value(double t, const SearchConfig &cfg)
{
    if (!cfg.periodic)
        return t;
    const double span = cfg.hi - cfg.lo;
    double r = std::fmod(t - cfg.lo, span);
    if (r < 0.0)
        r += span;               // C's fmod keeps the sign, python's % does not
    return cfg.lo + r;
}


//  errors() memoised on the sample value.  The ladder revisits every coarse
//  sample at every finer resolution, and a sample costs one closed-form solve
//  plus one FK per branch.  Keyed on the double itself, which is exact across
//  a doubling -- see sample_values().
inline const std::vector<double> &cached_errors(const Mat4 &T, double value,
                                                ErrorCache &cache,
                                                const ErrorsFn &errors)
{
    ErrorCache::iterator it = cache.find(value);
    if (it == cache.end())
        it = cache.insert(std::make_pair(value, errors(T, value))).first;
    return it->second;
}


inline double error_of(const Mat4 &T, double t, int b, ErrorCache &cache,
                       const ErrorsFn &errors, const SearchConfig &cfg)
{
    const std::vector<double> &e = cached_errors(T, wrap_value(t, cfg),
                                                 cache, errors);
    return (b < (int) e.size()) ? e[b] : INF;
}


//  (values, curves) at one resolution;  curves[branch][i] is the error.
//
//  EVERY REAL SAMPLE GETS TWO NEIGHBOURS, so every one can be the middle of a
//  bracket.  On a periodic domain that is a wrap: the last sample is repeated
//  below lo and the first above hi, without which a root at either end is a
//  minimum with only one neighbour and is never found -- a whole posture
//  missing, silently.  On a finite (prismatic) range there is nothing to wrap
//  to, so the grid is extended by one step past each end instead: the closed
//  form is as evaluable there as anywhere, and a root found just outside a
//  GUESSED range is worth more than a root not found.
inline void scan_curves(const Mat4 &T, int n, ErrorCache &cache,
                        const ErrorsFn &errors, const SearchConfig &cfg,
                        std::vector<double> &values,
                        std::vector<std::vector<double> > &curves)
{
    values = sample_values(cfg, n);
    curves.clear();
    if (values.empty())
        return;

    std::vector<std::vector<double> > cols;
    cols.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i)
        cols.push_back(cached_errors(T, values[i], cache, errors));

    if (cfg.periodic) {
        const double span = cfg.hi - cfg.lo;
        values.insert(values.begin(), values.back() - span);
        values.push_back(values[1] + span);
        cols.insert(cols.begin(), cols.back());
        cols.push_back(cols[1]);
    } else {
        const double step = (cfg.hi - cfg.lo) / (double) n;
        const double lo_x = values.front() - step;
        const double hi_x = values.back() + step;
        const std::vector<double> e_lo = cached_errors(T, lo_x, cache, errors);
        const std::vector<double> e_hi = cached_errors(T, hi_x, cache, errors);
        values.insert(values.begin(), lo_x);
        values.push_back(hi_x);
        cols.insert(cols.begin(), e_lo);
        cols.push_back(e_hi);
    }

    size_t nb = 0;
    for (size_t i = 0; i < cols.size(); ++i)
        nb = std::max(nb, cols[i].size());

    curves.assign(nb, std::vector<double>(cols.size(), INF));
    for (size_t b = 0; b < nb; ++b)
        for (size_t i = 0; i < cols.size(); ++i)
            curves[b][i] = (b < cols[i].size()) ? cols[i][b] : INF;
}


//  Minimise f on [a, b] by golden section.  -> x, with fx set to f(x).
//
//  Golden section and not a derivative method: near a solution the error is a
//  V, not a parabola -- it is a norm going to zero -- so its slope jumps sign
//  and nothing based on curvature behaves.  Unimodality on the bracket is all
//  this needs.
//
//  It stops when the bracket reaches float resolution rather than always
//  walking `iters` passes:  0.618^80 is far below it, so the tail was
//  deciding between two doubles that are equal, and the ladder calls this
//  many times per resolution.
inline double golden(const std::function<double(double)> &f, double a,
                     double b, int iters, double &fx, double xtol = 1e-15)
{
    const double invphi = (std::sqrt(5.0) - 1.0) / 2.0;
    const double invphi2 = (3.0 - std::sqrt(5.0)) / 2.0;
    double h = b - a;
    double c = a + invphi2 * h;
    double d = a + invphi * h;
    double fc = f(c);
    double fd = f(d);

    for (int i = 0; i < iters; ++i) {
        if (h <= xtol * (1.0 + std::fabs(a) + std::fabs(b)))
            break;
        if (fc < fd) {
            b = d;  d = c;  fd = fc;
            h *= invphi;
            c = a + invphi2 * h;
            fc = f(c);
        } else {
            a = c;  c = d;  fc = fd;
            h *= invphi;
            d = a + invphi * h;
            fd = f(d);
        }
    }
    if (fc < fd) { fx = fc; return c; }
    fx = fd;
    return d;
}


//  Indices i where the curve dips:  curve[i] < curve[i-1] and <= curve[i+1].
//
//  A neighbour of INF is allowed and is often where a real solution is: INF
//  means the branch is undefined there, so the curve falls off a cliff at the
//  edge of that branch's domain and the minimum sits against it.
//
//  Index 0 and the last index are never candidates and do not need to be:
//  scan_curves() has already given every real sample two neighbours.
inline std::vector<size_t> local_minima(const std::vector<double> &curve)
{
    std::vector<size_t> out;
    for (size_t i = 1; i + 1 < curve.size(); ++i) {
        if (!std::isfinite(curve[i]))
            continue;
        if (curve[i] < curve[i - 1] && curve[i] <= curve[i + 1])
            out.push_back(i);
    }
    return out;
}


inline double max_abs_diff(const JointVec &a, const JointVec &b)
{
    if (a.size() != b.size())
        return INF;
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
        worst = std::max(worst, std::fabs(a[i] - b[i]));
    return worst;
}


//  One entry per posture, keeping the most accurate of each.
//
//  Two branches can converge on the same posture -- IKBT enumerates version
//  combinations without discarding duplicates, and a bracket found from
//  either side lands in the same place.
inline std::vector<OneVarSolution> dedup(std::vector<OneVarSolution> found,
                                         double dedup_tol)
{
    std::stable_sort(found.begin(), found.end(),
                     [](const OneVarSolution &x, const OneVarSolution &y) {
                         return x.error < y.error;
                     });
    std::vector<OneVarSolution> unique;
    for (size_t i = 0; i < found.size(); ++i) {
        bool seen = false;
        for (size_t j = 0; j < unique.size() && !seen; ++j)
            seen = (max_abs_diff(found[i].q, unique[j].q) < dedup_tol);
        if (!seen)
            unique.push_back(found[i]);
    }
    return unique;
}


//  Bracket every dip in one branch's sampled curve and refine it.
inline std::vector<OneVarSolution>
roots_in(const Mat4 &T, const std::vector<double> &values,
         const std::vector<double> &curve, int b, ErrorCache &cache,
         const BranchesFn &branches, const ErrorsFn &errors,
         const SearchConfig &cfg)
{
    std::function<double(double)> err_at = [&](double t) {
        return error_of(T, t, b, cache, errors, cfg);
    };

    std::vector<OneVarSolution> out;
    const std::vector<size_t> mins = local_minima(curve);
    for (size_t k = 0; k < mins.size(); ++k) {
        const size_t i = mins[k];
        double e_star = INF;
        double t_star = golden(err_at, values[i - 1], values[i + 1],
                               cfg.refine_iters, e_star);
        if (e_star > cfg.accept_tol)
            continue;                   // a dip, not a root -- see the header
        t_star = wrap_value(t_star, cfg);
        const std::vector<JointVec> qs = branches(T, t_star);
        if (b >= (int) qs.size())
            continue;
        OneVarSolution s;
        s.q = qs[b];
        s.known_value = t_star;
        s.branch = b;
        s.error = e_star;
        s.n_samples = 0;
        out.push_back(s);
    }
    return out;
}


//  Look again, finely, in a window one grid step wide around each root.
inline std::vector<OneVarSolution>
hunt_near(const Mat4 &T, const std::vector<OneVarSolution> &found, double h,
          ErrorCache &cache, const BranchesFn &branches,
          const ErrorsFn &errors, const SearchConfig &cfg,
          int fanout = 16, int rounds = 3)
{
    std::vector<OneVarSolution> known = found;
    std::vector<OneVarSolution> frontier = found;

    for (int r = 0; r < rounds; ++r) {
        std::vector<OneVarSolution> fresh;
        for (size_t s = 0; s < frontier.size(); ++s) {
            const int b = frontier[s].branch;
            const double t0 = frontier[s].known_value;

            std::vector<double> vals;
            std::vector<double> curve;
            vals.reserve(fanout + 1);
            curve.reserve(fanout + 1);
            for (int k = 0; k <= fanout; ++k) {
                const double t = t0 - h + 2.0 * h * (k / (double) fanout);
                vals.push_back(t);
                curve.push_back(error_of(T, t, b, cache, errors, cfg));
            }

            const std::vector<OneVarSolution> got =
                roots_in(T, vals, curve, b, cache, branches, errors, cfg);
            for (size_t g = 0; g < got.size(); ++g) {
                bool seen = false;
                for (size_t u = 0; u < known.size() && !seen; ++u)
                    seen = (max_abs_diff(got[g].q, known[u].q) < cfg.dedup_tol);
                for (size_t u = 0; u < fresh.size() && !seen; ++u)
                    seen = (max_abs_diff(got[g].q, fresh[u].q) < cfg.dedup_tol);
                if (!seen)
                    fresh.push_back(got[g]);
            }
        }
        if (fresh.empty())
            break;
        known.insert(known.end(), fresh.begin(), fresh.end());
        frontier = fresh;
        h = h / (double) fanout;
    }
    return dedup(known, cfg.dedup_tol);
}


//  Adjacent index pairs where one sample is defined and the next is not.
//  INF means the closed form had no answer there -- an arcsine out of range,
//  a division by zero -- so a finite/infinite pair straddles the edge of this
//  branch's domain.
inline std::vector<std::pair<size_t, size_t> >
domain_edges(const std::vector<double> &curve)
{
    std::vector<std::pair<size_t, size_t> > out;
    for (size_t i = 0; i + 1 < curve.size(); ++i)
        if (std::isfinite(curve[i]) != std::isfinite(curve[i + 1]))
            out.push_back(std::make_pair(i, i + 1));
    return out;
}


//  Probe inward from every edge of this branch's domain, in halving steps.
inline std::vector<OneVarSolution>
hunt_edges(const Mat4 &T, const std::vector<double> &values,
           const std::vector<double> &curve, int b, ErrorCache &cache,
           const BranchesFn &branches, const ErrorsFn &errors,
           const SearchConfig &cfg, int bisect = 40, int depth = 40)
{
    std::function<double(double)> err_at = [&](double t) {
        return error_of(T, t, b, cache, errors, cfg);
    };

    std::vector<OneVarSolution> out;
    const std::vector<std::pair<size_t, size_t> > edges = domain_edges(curve);
    for (size_t k = 0; k < edges.size(); ++k) {
        const size_t i = edges[k].first, j = edges[k].second;
        double inside, outside;
        if (std::isfinite(curve[i])) { inside = values[i]; outside = values[j]; }
        else                         { inside = values[j]; outside = values[i]; }
        const double step = std::fabs(outside - inside);
        if (step == 0.0)
            continue;

        //  the edge itself, to a part in 2^bisect of one grid step
        double near = inside, far = outside;
        for (int t = 0; t < bisect; ++t) {
            const double mid = 0.5 * (near + far);
            if (std::isfinite(err_at(mid)))
                near = mid;
            else
                far = mid;
        }

        const double inward = (inside > outside) ? 1.0 : -1.0;
        std::vector<double> probes;
        probes.reserve(depth);
        for (int t = 0; t < depth; ++t)
            probes.push_back(near + inward * step * std::pow(0.5, t));
        std::sort(probes.begin(), probes.end());

        std::vector<double> pcurve;
        pcurve.reserve(probes.size());
        for (size_t t = 0; t < probes.size(); ++t)
            pcurve.push_back(err_at(probes[t]));

        const std::vector<OneVarSolution> got =
            roots_in(T, probes, pcurve, b, cache, branches, errors, cfg);
        out.insert(out.end(), got.begin(), got.end());
    }
    return out;
}


//  Every root found at ONE global resolution.
//
//  Three passes, because roots hide in three ways.  The GRID finds the
//  ordinary ones.  The DOMAIN EDGES are probed geometrically, because a spike
//  too narrow for the grid is pressed against one of them.  And then the
//  neighbourhood of everything found so far is re-scanned, because a root the
//  grid stepped over is within one step of a root it did not.
inline std::vector<OneVarSolution>
solve_at(const Mat4 &T, int n, ErrorCache &cache, const BranchesFn &branches,
         const ErrorsFn &errors, const SearchConfig &cfg)
{
    std::vector<double> values;
    std::vector<std::vector<double> > curves;
    scan_curves(T, n, cache, errors, cfg, values, curves);

    std::vector<OneVarSolution> found;
    for (size_t b = 0; b < curves.size(); ++b) {
        const std::vector<OneVarSolution> a =
            roots_in(T, values, curves[b], (int) b, cache, branches, errors, cfg);
        found.insert(found.end(), a.begin(), a.end());
        const std::vector<OneVarSolution> e =
            hunt_edges(T, values, curves[b], (int) b, cache, branches, errors, cfg);
        found.insert(found.end(), e.begin(), e.end());
    }

    const double step = (cfg.hi - cfg.lo) / (double) n;
    return hunt_near(T, dedup(found, cfg.dedup_tol), step, cache, branches,
                     errors, cfg);
}


inline bool same_postures(const std::vector<OneVarSolution> &a,
                          const std::vector<OneVarSolution> &b, double tol)
{
    if (a.size() != b.size())
        return false;
    for (size_t i = 0; i < a.size(); ++i) {
        bool seen = false;
        for (size_t j = 0; j < b.size() && !seen; ++j)
            seen = (max_abs_diff(a[i].q, b[j].q) < tol);
        if (!seen)
            return false;
    }
    return true;
}


//  Goal pose T -> every joint vector that reaches it.
//
//  THE RESOLUTION IS NOT ASSUMED, IT IS REACHED.  The scan runs at n_samples,
//  then 2*n_samples, and on up, returning as soon as two successive
//  resolutions agree about the whole set of postures.  max_samples is where
//  the checking gives up: the solutions returned there are still SOUND --
//  every one reaches T to within accept_tol -- but their completeness is once
//  again unproven, and the n_samples field reads max_samples to say so.
//
//  An empty result means no solution was FOUND, which is not quite
//  "unreachable".
inline std::vector<OneVarSolution>
onevar_solve(const Mat4 &T, const BranchesFn &branches, const ErrorsFn &errors,
             const SearchConfig &cfg)
{
    ErrorCache cache;
    int n = std::max(2, cfg.n_samples);
    std::vector<OneVarSolution> prev;
    bool have_prev = false;

    for (;;) {
        std::vector<OneVarSolution> sols =
            solve_at(T, n, cache, branches, errors, cfg);
        for (size_t i = 0; i < sols.size(); ++i)
            sols[i].n_samples = n;
        if (have_prev && same_postures(prev, sols, cfg.dedup_tol))
            return sols;
        if (2 * n > cfg.max_samples)
            return sols;
        prev = sols;
        have_prev = true;
        n *= 2;
    }
}

}   // namespace ikbt

#endif   // IKBT_SEARCH_H
