#!/usr/bin/python
#
#   tube_map.py --  the solution graph drawn as a London Underground map
#
#        python3 -m scripts.tube_map Puma            -> graphs/Puma_tube.svg
#        python3 -m scripts.tube_map graphs/Puma_graph.txt -o /tmp/p.svg
#
#   A SIDE SCRIPT.  Nothing in the solver calls it, and it imports nothing from
#   IKBT -- no sympy, no pickle, no solve.  Its whole input is the text file
#   output_latex.py writes next to the report (graphs/<robot>_graph.txt), so
#   drawing is free and can be iterated on without re-solving an arm.
#
#   WHAT THE MAP SAYS
#
#     column   a variable, in solve order, left to right ("fare zones")
#     station  one VERSION of that variable (th_4v5)
#     line     one complete solution -- one row of Robot.solListMatrix
#
#   So a line is a walk from the first variable solved to the last, calling at
#   the version of each variable that this solution uses;  where two solutions
#   agree about a variable they call at the same station, and the map shows
#   them running together and then parting, which is exactly the thing the
#   report's edge list cannot show.  An interchange (the white capsule) is a
#   version SHARED by several solutions.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import os
import re
import sys
import argparse
from xml.sax.saxutils import escape


###############################################################################
#
#    Reading the graph file
#

def read_graph(path):
    '''Parse graphs/<robot>_graph.txt into a dict.

       UNKNOWN KEYWORDS ARE SKIPPED, deliberately:  the writer may gain a line
       type (it already has `true_robot`, which only appears on hybrid solves)
       and an older reader should keep working.  `format` is checked so that a
       genuinely incompatible file says so instead of drawing nonsense.'''

    G = {'robot': '?', 'true_robot': None, 'cols': [], 'edges': [], 'rows': []}

    with open(path) as f:
        for raw in f:
            line = raw.split('#')[0].strip()
            if not line:
                continue
            key, _, rest = line.partition(' ')
            rest = rest.strip()

            if key == 'format':
                if rest.split()[0] != '1':
                    raise ValueError(f'{path}: format {rest} is newer than this '
                                     f'script understands (it knows format 1)')
            elif key == 'robot':
                G['robot'] = rest
            elif key == 'true_robot':
                G['true_robot'] = rest
            elif key == 'col':
                G['cols'].append(_parse_col(rest))
            elif key == 'edge':
                a, b = rest.split()[:2]
                G['edges'].append((a, b))
            elif key == 'row':
                G['rows'].append(rest.split()[1:])   # drop the row number

    if not G['cols'] or not G['rows']:
        raise ValueError(f'{path}: no columns or no rows -- nothing to draw')

    n = len(G['cols'])
    for i, r in enumerate(G['rows']):
        if len(r) != n:
            raise ValueError(f'{path}: row {i+1} has {len(r)} entries but there '
                             f'are {n} columns')
    return G


def _parse_col(rest):
    '''col 3 th_23 nsol=1 nver=4 deps=th_1,th_3 method=simultaneous eqn

       `method` is free text and runs to the end of the line -- solvemethods
       carry spaces and commas ("sinANDcos, best ranked,") -- so it is split
       off first and the rest is then safe to tokenize.'''

    method = ''
    if ' method=' in rest:
        rest, _, method = rest.partition(' method=')
    tok = rest.split()
    col = {'var': tok[1], 'nsol': 0, 'nver': 0, 'deps': [],
           'method': method.strip()}
    for t in tok[2:]:
        k, _, v = t.partition('=')
        if k == 'nsol':
            col['nsol'] = int(v)
        elif k == 'nver':
            col['nver'] = int(v)
        elif k == 'deps':
            col['deps'] = [] if v == '-' else v.split(',')
    return col


###############################################################################
#
#    Where the stations go
#

def station_order(cols, rows):
    '''The stations of each column, top to bottom.

       Starting order is first appearance down the rows, which is the order the
       version names were generated in and is already sensible.  It is then
       improved by BARYCENTRE SWEEPS:  a station moves to the average height of
       the stations its lines call at in the neighbouring column, which is the
       standard layered-graph heuristic and is what stops the map from being a
       ball of wool.  Sweeps that make it worse are thrown away -- the count of
       crossings is cheap to measure, so there is no need to trust the
       heuristic.'''

    order = []
    for c in range(len(cols)):
        seen = []
        for r in rows:
            if r[c] not in seen:
                seen.append(r[c])
        order.append(seen)

    best, best_x = [list(o) for o in order], _crossings(order, rows)
    for sweep in range(6):
        cseq = range(1, len(cols)) if sweep % 2 == 0 else \
               range(len(cols) - 2, -1, -1)
        nbr = -1 if sweep % 2 == 0 else 1
        for c in cseq:
            idx = {s: i for i, s in enumerate(order[c + nbr])}
            bary, hits = {}, {}
            for r in rows:
                bary[r[c]] = bary.get(r[c], 0) + idx[r[c + nbr]]
                hits[r[c]] = hits.get(r[c], 0) + 1
            #  Ties keep the incumbent order:  sorted() is stable, so a station
            #  with no reason to move does not.
            order[c].sort(key=lambda s: bary[s] / hits[s])
        x = _crossings(order, rows)
        if x < best_x:
            best, best_x = [list(o) for o in order], x

    return best


def _crossings(order, rows):
    '''How many times one line crosses another, over the whole map.

       Two lines cross in the gap between two columns when they are the other
       way round at the far end;  comparing station INDEX is enough, since the
       drawing is monotone in the index.'''

    idx = [{s: i for i, s in enumerate(o)} for o in order]
    n = 0
    for c in range(len(order) - 1):
        for i in range(len(rows)):
            for j in range(i + 1, len(rows)):
                a = idx[c][rows[i][c]] - idx[c][rows[j][c]]
                b = idx[c + 1][rows[i][c + 1]] - idx[c + 1][rows[j][c + 1]]
                if a * b < 0:
                    n += 1
    return n


###############################################################################
#
#    Colours:  the real Underground lines, then dashes
#
#  Thirteen colours, and past thirteen lines the palette repeats DASHED, then
#  dotted, then dash-dot -- 52 solutions before two of them look alike.  The
#  greys and the yellow are late in the order on purpose:  they are the hardest
#  to tell apart on white, so an arm with four solutions never sees them.

LINES = [
    ('Central',          '#E32017', None),
    ('Piccadilly',       '#003688', None),
    ('District',         '#00782A', None),
    ('Metropolitan',     '#9B0056', None),
    ('Bakerloo',         '#B36305', None),
    ('Victoria',         '#0098D4', None),
    ('Northern',         '#000000', None),
    ('Elizabeth',        '#6950A1', None),
    ('DLR',              '#00A4A7', None),
    ('Hammersmith & City', '#F3A9BB', None),
    ('Waterloo & City',  '#95CDBA', None),
    ('Jubilee',          '#A0A5A9', None),
    ('Circle',           '#FFD300', None),
]

DASHES = [None, '14,7', '1,7', '14,6,1,6']
DASH_NAME = ['', 'dashed', 'dotted', 'dash-dot']


def line_style(i):
    '''(colour, dash, name) for solution i, counting from 0.'''
    name, colour, _ = LINES[i % len(LINES)]
    k = (i // len(LINES)) % len(DASHES)
    label = name if not k else f'{name} ({DASH_NAME[k]})'
    return colour, DASHES[k], label


###############################################################################
#
#    Text:  th_23v4 -> theta with a real subscript
#

INK = '#17181A'
GREY = '#6B6F76'
ZONE = '#F4F1EA'


def parts_of(name):
    '''[(text, is_subscript)] for a variable or version name.

       th_23v4 -> theta, subscript 23, then v4.  A name that does not have the
       <letters>_<digits> shape (Wrist calls its joints A, B, C) is returned
       whole, which is correct for it.'''

    m = re.match(r'^([A-Za-z]+)_(\d+)(.*)$', name)
    if not m:
        return [(name, False)]
    head, sub, tail = m.groups()
    if head == 'th':
        head = 'θ'
    out = [(head, False), (sub, True)]
    if tail:
        out.append((tail, False))
    return out


def text(x, y, parts, size=13, fill=INK, anchor='middle', weight='normal',
         style='normal', opacity=None):
    '''One <text> element.  parts is a string, or the list from parts_of().'''

    if isinstance(parts, str):
        parts = [(parts, False)]
    body = ''
    for s, sub in parts:
        s = escape(s)
        if sub:
            body += (f'<tspan font-size="{size*0.68:.1f}" dy="{size*0.26:.1f}">'
                     f'{s}</tspan>')
            body += f'<tspan dy="{-size*0.26:.1f}">​</tspan>'
        else:
            body += s
    op = '' if opacity is None else f' opacity="{opacity}"'
    return (f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size:.1f}" '
            f'fill="{fill}" text-anchor="{anchor}" font-weight="{weight}" '
            f'font-style="{style}"{op}>{body}</text>')


###############################################################################
#
#    Paths:  horizontal, 45 degrees, horizontal -- with rounded corners
#

def octolinear(x0, y0, x1, y1, lead):
    '''Waypoints from one station to the next in the Beck manner:  run out
       horizontally, turn 45 degrees, run in horizontally.

       The diagonal is centred in the gap and is a true 45 degrees whenever
       there is room for it.  When the two stations are further apart
       vertically than the gap is wide the diagonal steepens instead, which is
       ugly but honest;  the alternative -- a longer gap -- would set the
       column spacing of the whole map from its single worst pair.'''

    if abs(y1 - y0) < 0.5:
        return [(x0, y0), (x1, y1)]

    span, rise = x1 - x0, abs(y1 - y0)
    if rise <= span - 2 * lead:
        xa = x0 + (span - rise) / 2.0
        xb = xa + rise
    else:
        xa, xb = x0 + lead, x1 - lead
    return [(x0, y0), (xa, y0), (xb, y1), (x1, y1)]


def rounded(pts, r=11.0):
    '''An SVG path through pts with the corners rounded.

       The radius is clipped to half of the shorter adjacent segment, so a
       short segment between two turns cannot make the curves overlap and
       double back.'''

    def cut(p, q, d):                     # point d along p->q
        dx, dy = q[0] - p[0], q[1] - p[1]
        L = max((dx * dx + dy * dy) ** 0.5, 1e-9)
        d = min(d, L / 2.0)
        return (p[0] + dx * d / L, p[1] + dy * d / L)

    d = f'M {pts[0][0]:.1f} {pts[0][1]:.1f}'
    for i in range(1, len(pts) - 1):
        a = cut(pts[i], pts[i - 1], r)
        b = cut(pts[i], pts[i + 1], r)
        d += (f' L {a[0]:.1f} {a[1]:.1f}'
              f' Q {pts[i][0]:.1f} {pts[i][1]:.1f} {b[0]:.1f} {b[1]:.1f}')
    d += f' L {pts[-1][0]:.1f} {pts[-1][1]:.1f}'
    return d


###############################################################################
#
#    The drawing
#

#  Geometry, in px.  COL_DX is set by the column header, which is the widest
#  thing on the map;  GAP and LW by the need to tell eight parallel lines apart
#  at arm's length.
COL_DX, LW, GAP = 215.0, 6.0, 9.0
MARGIN, TITLE_H, HEADER_H, PAD_V = 42.0, 104.0, 104.0, 58.0
LEAD = 30.0            # horizontal room beside a station before a line turns
R_TICK, W_CAP = 5.0, 14.0
ORIGIN_DX = 108.0      # the strip to the left of the first variable


def draw(G, title=None):
    '''The whole SVG, as a string.'''

    cols, rows = G['cols'], G['rows']
    order = station_order(cols, rows)
    ncol, nline = len(cols), len(rows)

    #  Which lines call at each station, in solution order.  That order is the
    #  bundle order too, so lines that run together never swap places between
    #  stations and so never cross INSIDE a bundle.
    at = [{s: [] for s in o} for o in order]
    for i, r in enumerate(rows):
        for c in range(ncol):
            at[c][r[c]].append(i)

    widest = max((len(v) for col in at for v in col.values()), default=1)
    ROW_DY = max(84.0, (widest - 1) * GAP + 62.0)

    nrow = max(len(o) for o in order)
    map_h = (nrow - 1) * ROW_DY + 2 * PAD_V
    legend_h = 30.0 + nline * 23.0
    W = 2 * MARGIN + ORIGIN_DX + ncol * COL_DX
    H = TITLE_H + HEADER_H + map_h + legend_h + MARGIN

    X = [MARGIN + ORIGIN_DX + COL_DX / 2 + c * COL_DX for c in range(ncol)]
    top = TITLE_H + HEADER_H

    #  THE COMMON ORIGIN:  the state just before the first variable is solved.
    #  Every solution starts from the same place -- they have not yet chosen
    #  anything to differ about -- so the map gets one station that all the
    #  lines leave from, which is also what makes the first column read as a
    #  branching rather than as eight unrelated lines starting in mid-air.
    #  It is the one station with no name:  there is no variable here yet.
    X_ORG = MARGIN + ORIGIN_DX / 2
    Y_ORG = top + PAD_V + (nrow - 1) * ROW_DY / 2.0

    def ypos(c, s):
        '''Centre of station s of column c.  Each column is centred vertically,
           so a column with few versions sits opposite the middle of a column
           with many and the lines fan rather than climb.'''
        k = order[c].index(s) + (nrow - len(order[c])) / 2.0
        return top + PAD_V + k * ROW_DY

    def off(c, s, i):
        '''Where line i rides within the bundle at this station:  parallel
           tracks, centred on the station.'''
        b = at[c][s]
        return (b.index(i) - (len(b) - 1) / 2.0) * GAP

    out = []
    out.append(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W:.0f}" '
               f'height="{H:.0f}" viewBox="0 0 {W:.0f} {H:.0f}" '
               f'font-family="Helvetica Neue,Helvetica,Arial,sans-serif">')
    out.append(f'<rect width="{W:.0f}" height="{H:.0f}" fill="#FFFFFF"/>')

    ####  Zones.  One band per variable -- the map's columns are its fare zones.
    for c in range(ncol):
        if c % 2 == 0:
            out.append(f'<rect x="{MARGIN + ORIGIN_DX + c*COL_DX:.1f}" '
                       f'y="{TITLE_H:.1f}" '
                       f'width="{COL_DX:.1f}" '
                       f'height="{HEADER_H + map_h:.1f}" fill="{ZONE}"/>')

    ####  Title block, with a roundel
    out += roundel(MARGIN + 22, TITLE_H / 2 + 4)
    name = title or f'{G["robot"]} Solution Graph'
    out.append(text(MARGIN + 52, TITLE_H / 2 - 4, name, size=27,
                    anchor='start', weight='bold'))
    sub = (f'IKBT  ·  {ncol} variables  ·  '
           f'{nline} solution' + ('s' if nline != 1 else ''))
    if G['true_robot'] and G['true_robot'] != G['robot']:
        sub += f'  ·  closed form for the simplified {G["robot"]}'
    out.append(text(MARGIN + 52, TITLE_H / 2 + 17, sub, size=12.5, fill=GREY,
                    anchor='start'))

    ####  Column headers:  the variable, when it was solved, how, and on what
    for c, col in enumerate(cols):
        y = TITLE_H + 30
        out.append(text(X[c], y, parts_of(col['var']), size=25, weight='bold'))
        nth = ordinal(c + 1)
        out.append(text(X[c], y + 21, f'{nth} solved', size=11.5, fill=GREY))
        out.append(text(X[c], y + 38, col['method'] or '-', size=11.5,
                        fill=GREY, style='italic'))
        if col['deps']:
            need = ', '.join(pretty_ascii(d) for d in col['deps'])
            out.append(text(X[c], y + 55, f'needs {need}', size=11.5, fill=GREY))
        else:
            out.append(text(X[c], y + 55, 'needs nothing', size=11.5,
                            fill=GREY, opacity=0.65))

    ####  The lines.  Drawn before the stations, so a station sits on top of
    ####  the track the way a real map's interchange does.
    for i, r in enumerate(rows):
        colour, dash, _ = line_style(i)
        #  Out of the common origin, in solution order, then on to the first
        #  variable's station.
        y_org = Y_ORG + (i - (nline - 1) / 2.0) * GAP
        pts = [(X_ORG, y_org)]
        for c in range(ncol):
            y = ypos(c, r[c]) + off(c, r[c], i)
            pts += octolinear(X[c - 1] if c else X_ORG, pts[-1][1],
                              X[c], y, LEAD)[1:]
        da = f' stroke-dasharray="{dash}"' if dash else ''
        out.append(f'<path d="{rounded(pts)}" fill="none" stroke="{colour}" '
                   f'stroke-width="{LW}" stroke-linecap="round" '
                   f'stroke-linejoin="round"{da}/>')

    ####  Stations, and their names above them.  The origin first, unnamed.
    o_span = (nline - 1) * GAP
    if nline == 1:
        out.append(f'<circle cx="{X_ORG:.1f}" cy="{Y_ORG:.1f}" r="{R_TICK}" '
                   f'fill="#FFFFFF" stroke="{INK}" stroke-width="2.4"/>')
    else:
        out.append(f'<rect x="{X_ORG - W_CAP/2:.1f}" '
                   f'y="{Y_ORG - o_span/2 - W_CAP/2:.1f}" width="{W_CAP:.1f}" '
                   f'height="{o_span + W_CAP:.1f}" rx="{W_CAP/2:.1f}" '
                   f'fill="#FFFFFF" stroke="{INK}" stroke-width="2.8"/>')

    for c in range(ncol):
        for s in order[c]:
            bund = at[c][s]
            y = ypos(c, s)
            o0 = off(c, s, bund[0])
            o1 = off(c, s, bund[-1])
            if len(bund) == 1:
                out.append(f'<circle cx="{X[c]:.1f}" cy="{y + o0:.1f}" '
                           f'r="{R_TICK}" fill="#FFFFFF" stroke="{INK}" '
                           f'stroke-width="2.4"/>')
                ytop = y + o0 - R_TICK
            else:
                #  An interchange:  one capsule over the whole bundle, which is
                #  how a real map says "these lines meet here".
                h = (o1 - o0) + W_CAP
                out.append(f'<rect x="{X[c] - W_CAP/2:.1f}" '
                           f'y="{y + o0 - W_CAP/2:.1f}" width="{W_CAP:.1f}" '
                           f'height="{h:.1f}" rx="{W_CAP/2:.1f}" '
                           f'fill="#FFFFFF" stroke="{INK}" stroke-width="2.8"/>')
                ytop = y + o0 - W_CAP / 2
            out.append(text(X[c], ytop - 9, parts_of(s), size=13.5,
                            weight='600'))

    ####  Legend.  One entry per solution, naming the row of solListMatrix it
    ####  is -- the map is useless without the way back to the report.
    ly = TITLE_H + HEADER_H + map_h + 22
    out.append(text(MARGIN, ly, 'Solutions', size=14, anchor='start',
                    weight='bold'))
    for i, r in enumerate(rows):
        colour, dash, label = line_style(i)
        y = ly + 22 + i * 23
        da = f' stroke-dasharray="{dash}"' if dash else ''
        out.append(f'<line x1="{MARGIN:.1f}" y1="{y:.1f}" '
                   f'x2="{MARGIN + 46:.1f}" y2="{y:.1f}" stroke="{colour}" '
                   f'stroke-width="{LW}" stroke-linecap="round"{da}/>')
        out.append(text(MARGIN + 58, y + 4.5, f'Solution {i+1}', size=12.5,
                        anchor='start', weight='bold'))
        out.append(text(MARGIN + 140, y + 4.5, label, size=12, fill=GREY,
                        anchor='start', style='italic'))
        out.append(text(MARGIN + 250, y + 4.5,
                        '  ·  '.join(pretty_ascii(v) for v in r),
                        size=12, fill=GREY, anchor='start'))

    out.append('</svg>')
    return '\n'.join(out)


def roundel(cx, cy, r=15.0):
    '''The Underground roundel:  a red ring behind a blue bar.  No text -- at
       this size a word inside it would be unreadable, and the shape alone is
       the thing that says which map this is pretending to be.'''

    return [f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="none" '
            f'stroke="#E32017" stroke-width="5.5"/>',
            f'<rect x="{cx - r - 5:.1f}" y="{cy - 3.6:.1f}" '
            f'width="{2*r + 10:.1f}" height="7.2" fill="#003688"/>']


def pretty_ascii(name):
    '''th_23v4 -> theta-23v4, for the one-line places where tspans cannot go.'''
    return ''.join(s for s, _ in parts_of(name))


def ordinal(n):
    if 10 <= n % 100 <= 20:
        return f'{n}th'
    return f'{n}' + {1: 'st', 2: 'nd', 3: 'rd'}.get(n % 10, 'th')


###############################################################################

def main(argv=None):
    p = argparse.ArgumentParser(
        description='Draw an IKBT solution graph as a subway map (SVG).')
    p.add_argument('robot', help='a robot name, or a path to a _graph.txt file')
    p.add_argument('-o', '--out', help='output SVG (default graphs/<robot>_tube.svg)')
    p.add_argument('-d', '--dir', default='graphs',
                   help='where the graph files live (default: graphs)')
    p.add_argument('-t', '--title', help='override the title')
    a = p.parse_args(argv)

    path = a.robot
    if not os.path.isfile(path):
        path = os.path.join(a.dir, a.robot + '_graph.txt')
    if not os.path.isfile(path):
        #  The file is a by-product of the REPORT, so say the thing that makes
        #  one appear rather than just reporting a missing file.
        print(f'No graph file: {path}')
        print(f'Solve the robot first:   python3 ikSolver.py {a.robot}')
        return 1

    G = read_graph(path)
    out = a.out or os.path.join(os.path.dirname(path) or '.',
                                G['robot'] + '_tube.svg')
    with open(out, 'w') as f:
        f.write(draw(G, title=a.title))

    print(f'{path}  ->  {out}')
    print(f'   {len(G["cols"])} variables, {len(G["rows"])} solutions, '
          f'{sum(len(o) for o in station_order(G["cols"], G["rows"]))} versions')
    return 0


if __name__ == '__main__':
    sys.exit(main())
