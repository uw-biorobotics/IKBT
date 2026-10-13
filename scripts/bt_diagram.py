#!/usr/bin/python
#
#   bt_diagram.py --  the IKBT behavior tree drawn in the BT style guide
#
#        python3 -m scripts.bt_diagram           -> IKdocs/IKBT_BT_p1.svg ... _p3.svg
#        python3 -m scripts.bt_diagram -o /tmp   (directory for the pages)
#
#   A SIDE SCRIPT.  Nothing in the solver calls it.  It builds the tree with
#   bt_assembly.build_default_bt() and walks the live node objects, so the
#   drawing is the tree the code runs, not a hand copy of it that can drift.
#
#   Symbols and fills are those of IKdocs/BT_Symbols.svg (UW Biorobotics, 2018):
#
#     Selector  (b3.Priority)          green circle, "?"
#     Sequence  (b3.Sequence)          yellow circle, arrow
#     Parallel  (b3.OrNode)            pink circle, two arrows -- OrNode ticks
#                                      ALL its children, which is what the
#                                      parallel symbol says
#     Decorator (RepeatUntilSuccess,   grey diamond, loop budget beside it
#                symbolic_loop)
#     Action    (every other leaf)     blue rounded box
#     Continuation                     orange circle, a letter
#
#   PORTRAIT PAGES, one SVG each, for slides.  The whole tree is ~150 nodes and
#   far too wide for a page, so the subtrees named in PAGES are cut out and
#   drawn as panels of their own.  Every cut is a continuation node with its
#   OWN letter, handed out in reading order, and the panel it points to is
#   headed by a continuation node carrying that same letter.
#
#   The symbolic branch is the one subtree used more than once -- three
#   separate instances, plain, ' (onevar)' and ' (hybrid)' -- so its three
#   continuations get three letters and its one panel is headed by all three.
#
#   Copyright 2026 University of Washington
#
#   Developed by Blake Hannaford
#   BioRobotics Lab, University of Washington

import argparse
import os
import re
from xml.sax.saxutils import escape

import b3
from ikbtfunctions.bt_assembly import build_default_bt

TAGS = (' (onevar)', ' (hybrid)')

TOP = None      # the panel holding the root

#  Pages, top to bottom, each a list of panels;  a panel is named by the
#  untagged Name of its top node.  Every name here except TOP is a cut.
PAGES = [
    [TOP, 'One Variable Branch', 'Hybrid Branch'],
    ['Symbolic Branch', 'Try Solve Subtree'],
    ['Work Tools', 'Tan/SinCos + Rank'],
]

SUBTITLE = {
    TOP:                        'bt_assembly.build_default_bt():  three strategies, '
                                'tried in order, then the report',
    'One Variable Branch':      'declare one unknown known, solve the true arm for the rest',
    'Hybrid Branch':            'solve a nearby arm with a Pieper triple, refine numerically',
    'Symbolic Branch':          'one complete symbolic solver;  three separate instances',
    'Try Solve Subtree': 'pick an unknown, try every solver on it',
    'Work Tools':               'the solver Priority -- order decides which strategy owns a variable',
    'Tan/SinCos + Rank':        'both solvers run;  rank keeps the nicer solution',
}

CUTS = {name for page in PAGES for name in page if name is not TOP}

FILL = {'sel': '#a4da77', 'seq': '#ffe071', 'par': '#ff64c8', 'dec': '#cccccc',
        'act': '#9ad3f3', 'cont': '#ff6432', 'root': '#ffffff'}

FONT = "font-family:FreeSans,'Helvetica Neue',Arial,sans-serif"

PAGE_W, PAGE_H = 816, 1056      # US Letter, portrait, at 96 px/in
MARGIN = 32
SLOT = 112        # horizontal room per leaf, at most
LEVEL = 80        # vertical distance between depths
R = 17            # composite circle radius
BOX_H = 40


def plain_name(node):
    name = getattr(node, 'Name', '') or ''
    for t in TAGS:
        name = name.replace(t, '')
    if name.startswith('--unnamed') or name.startswith('*'):
        return ''
    return name


def tag_of(node):
    name = getattr(node, 'Name', '') or ''
    return next((t.strip() for t in TAGS if t in name), '')


def kind(node):
    if isinstance(node, b3.OrNode):
        return 'par'
    if isinstance(node, b3.Priority):
        return 'sel'
    if isinstance(node, b3.Sequence):
        return 'seq'
    if isinstance(node, b3.Decorator):
        return 'dec'
    return 'act'


def children(node):
    kids = list(getattr(node, 'children', None) or [])
    child = getattr(node, 'child', None)
    if child is not None and not isinstance(child, list):
        kids.append(child)
    return kids


class Box:
    '''One drawn node:  what it is, and where it sits once laid out.'''
    def __init__(self, kind, label, kids, extra=''):
        self.kind, self.label, self.kids = kind, label, kids
        self.extra = extra      # decorator budget, continuation letter
        self.x = self.y = 0


class Letters:
    '''Continuation letters, handed out in the order the cuts are met.

       Keyed by the node OBJECT, not its name:  the three symbolic branches
       are three instances and each gets a letter of its own.'''
    def __init__(self):
        self.of = {}            # id(node) -> letter
        self.refs = {}          # panel name -> [(letter, tag), ...]

    def cut(self, node):
        if id(node) not in self.of:
            self.of[id(node)] = chr(ord('A') + len(self.of))
            self.refs.setdefault(plain_name(node), []).append(
                (self.of[id(node)], tag_of(node)))
        return self.of[id(node)]


def to_boxes(node, letters, top=False):
    '''The drawable tree under `node`, with cut subtrees replaced by continuations.'''
    name = plain_name(node)
    if name in CUTS and not top:
        return Box('cont', name, [], letters.cut(node))
    k = kind(node)
    if k == 'act':
        return Box(k, name or type(node).__name__, [])
    extra = ''
    if k == 'dec':
        loop = getattr(node, 'max_loop', -1)
        extra = f'{type(node).__name__}  ≤{loop}' if loop > 0 else type(node).__name__
    return Box(k, name, [to_boxes(c, letters) for c in children(node)], extra)


def layout(box, slot, depth=0, cursor=None):
    '''Tidy layout:  leaves take successive slots, a parent sits over the
       middle of its children.'''
    if cursor is None:
        cursor = [0]
    box.y = depth * LEVEL
    if not box.kids:
        box.x = cursor[0] * slot
        cursor[0] += 1
    else:
        for k in box.kids:
            layout(k, slot, depth + 1, cursor)
        box.x = (box.kids[0].x + box.kids[-1].x) / 2


def walk(box):
    yield box
    for k in box.kids:
        yield from walk(k)


def wrap(text, width):
    words, lines, cur = text.split(), [], ''
    for w in words:
        if cur and len(cur) + 1 + len(w) > width:
            lines.append(cur)
            cur = w
        else:
            cur = f'{cur} {w}' if cur else w
    if cur:
        lines.append(cur)
    return lines


def text(x, y, s, size=11, anchor='middle', extra=''):
    return (f'<text x="{x:.1f}" y="{y:.1f}" text-anchor="{anchor}" '
            f'style="{FONT};font-size:{size}px;{extra}">{escape(s)}</text>')


def line(d):
    return f'<path d="{d}" style="fill:none;stroke:#000;stroke-width:1.2"/>'


ARROW = ('<marker id="arr" orient="auto" refX="0" refY="0" style="overflow:visible">'
         '<path transform="matrix(-1.1,0,0,-1.1,-1.1,0)" '
         'd="M 8.72,4.03 -2.21,0.02 8.72,-4.0 c -1.75,2.37 -1.74,5.62 0,8.04 z" '
         'style="fill:#000;stroke:#000;stroke-width:0.625;stroke-linejoin:round"/></marker>')


def glyph(kind, x, y, extra='', root_img=None, box_w=100):
    '''The style-guide symbol for `kind`, centred on (x, y).'''
    stroke = 'stroke:#000;stroke-width:1.76'
    if kind in ('sel', 'seq', 'par', 'cont', 'root'):
        out = [f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{R}" style="fill:{FILL[kind]};{stroke}"/>']
        if kind == 'sel':
            out.append(text(x, y + 5, '?', 14))
        elif kind == 'seq':
            out.append(f'<path d="M {x - 10.7:.1f},{y:.1f} h 20.4" style="stroke:#000;'
                       f'stroke-width:0.6;marker-end:url(#arr)"/>')
        elif kind == 'par':
            for dy in (-3.6, 3.6):
                out.append(f'<path d="M {x - 10.7:.1f},{y + dy:.1f} h 20.4" style="stroke:#000;'
                           f'stroke-width:0.6;marker-end:url(#arr)"/>')
        elif kind == 'cont':
            out.append(text(x, y + 5, extra, 14, extra='font-weight:bold'))
        elif kind == 'root' and root_img:
            out.append(f'<image x="{x - 10:.1f}" y="{y - 10:.1f}" width="20" height="20" '
                       f'preserveAspectRatio="none" xlink:href="{root_img}"/>')
        return out
    if kind == 'dec':
        s = 24
        return [f'<path d="M {x:.1f},{y - s:.1f} L {x + s:.1f},{y:.1f} L {x:.1f},{y + s:.1f} '
                f'L {x - s:.1f},{y:.1f} z" style="fill:{FILL["dec"]};stroke:#000;'
                f'stroke-width:1.875;stroke-linejoin:round"/>']
    # action
    return [f'<rect x="{x - box_w / 2:.1f}" y="{y - BOX_H / 2:.1f}" width="{box_w:.1f}" '
            f'height="{BOX_H}" rx="16" ry="16" style="fill:{FILL["act"]};stroke:#000;'
            f'stroke-width:1.875"/>']


def half_height(box):
    return {'act': BOX_H / 2, 'dec': 24}.get(box.kind, R)


def draw_panel(root, refs, name, ox, oy, width, root_img=None):
    '''SVG for one panel with its top-left at (ox, oy).  Returns (svg, height).

       `refs` are the (letter, tag) continuations that point at this panel;
       they head it, standing where the cut subtree's parent would be.'''
    out = []
    ncols = sum(1 for b in walk(root) if not b.kids)
    slot = min(SLOT, width / ncols)
    box_w = slot - 8
    chars = max(9, int(box_w / 6.2))

    letters = ', '.join(l for l, _ in refs)
    title = f'{letters}:  {name}' if refs else 'IKBT Behavior Tree'
    out.append(text(ox, oy + 16, title, 16, 'start', 'font-weight:bold'))
    out.append(text(ox, oy + 34, SUBTITLE[name], 11.5, 'start', 'fill:#555;font-style:italic'))

    head = 52 + LEVEL           # titles, then the heading node(s)
    layout(root, slot)
    span = ncols * slot
    shift = ox + (width - span) / 2 + slot / 2      # centre the panel
    for b in walk(root):
        b.x += shift
        b.y += oy + head + 26

    #  Heading:  the root on the top panel, the continuation(s) elsewhere.
    hy = root.y - LEVEL
    if refs:
        spread = 76
        xs = [root.x + (i - (len(refs) - 1) / 2) * spread for i in range(len(refs))]
        ym = hy + R + (root.y - half_height(root) - hy - R) / 2
        for (letter, tag), hx in zip(refs, xs):
            out.append(line(f'M {hx:.1f},{hy + R:.1f} V {ym:.1f} H {root.x:.1f} '
                            f'V {root.y - half_height(root):.1f}'))
            out += glyph('cont', hx, hy, letter)
            if len(refs) > 1:
                out.append(text(hx, hy - R - 6, tag or 'plain', 10.5,
                                extra='fill:#444;font-style:italic'))
    else:
        out.append(line(f'M {root.x:.1f},{hy + R:.1f} V {root.y - half_height(root):.1f}'))
        out += glyph('root', root.x, hy, root_img=root_img)
        out.append(text(root.x + R + 6, hy + 4, 'Root', 10.5, 'start',
                        'fill:#444;font-style:italic'))

    # edges first, so symbols sit on top of them
    for b in walk(root):
        if not b.kids:
            continue
        y0 = b.y + half_height(b)
        ym = b.y + LEVEL / 2 + 4
        for k in b.kids:
            out.append(line(f'M {b.x:.1f},{y0:.1f} V {ym:.1f} H {k.x:.1f} '
                            f'V {k.y - half_height(k):.1f}'))

    for b in walk(root):
        out += glyph(b.kind, b.x, b.y, b.extra, box_w=box_w)
        if b.kind == 'act':
            lines = wrap(b.label, chars)
            y = b.y + 4 - 6.5 * (len(lines) - 1)
            for ln in lines:
                out.append(text(b.x, y, ln, 10.5))
                y += 13
        elif b.kind == 'cont':
            for i, ln in enumerate(wrap(b.label, chars)):
                out.append(text(b.x, b.y + R + 13 + 12 * i, ln, 10.5,
                                extra='fill:#444;font-style:italic'))
        else:
            # composite / decorator:  Name to the right of the symbol
            dx = (24 if b.kind == 'dec' else R) + 5
            lines = wrap(b.label, min(16, chars)) if b.label else []
            if b.kind == 'dec':
                lines += b.extra.split('  ')     # class, then budget
            y = b.y + 4 - 6.5 * (len(lines) - 1)
            for i, ln in enumerate(lines):
                style = 'fill:#444;font-style:italic'
                if b.kind == 'dec' and i >= len(lines) - 2:
                    style = 'fill:#666'
                out.append(text(b.x + dx, y, ln, 10, 'start', style))
                y += 12

    bottom = max(b.y + half_height(b) + (28 if b.kind == 'cont' else 0) for b in walk(root))
    return out, bottom - oy + 8


def legend(ox, oy, width):
    out = [text(ox, oy + 16, 'Symbols  (IKdocs/BT_Symbols.svg)', 14, 'start', 'font-weight:bold')]
    items = [('sel', 'Selector  (b3.Priority):  first non-FAILURE child wins'),
             ('seq', 'Sequence  (b3.Sequence):  FAILs at first failing child'),
             ('par', 'Parallel  (b3.OrNode):  ticks ALL children'),
             ('dec', 'Decorator:  loop, budget ≤N'),
             ('act', 'Action leaf'),
             ('cont', 'Continuation:  the panel headed by this letter')]
    col_w = width / 2
    for i, (k, desc) in enumerate(items):
        col, row = i % 2, i // 2
        x = ox + 30 + col * col_w
        y = oy + 50 + row * 52
        out += glyph(k, x, y, 'A', box_w=50)
        out.append(text(x + 36, y + 4, desc, 11, 'start'))
    return out, 50 + 2 * 52 + 30


def root_image():
    '''The root-node icon from the style guide, if the guide is there.'''
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    try:
        with open(os.path.join(here, 'IKdocs', 'BT_Symbols.svg')) as f:
            m = re.search(r'xlink:href="(data:image/png;base64,[^"]+)"', f.read())
        return re.sub(r'\s+', '', m.group(1)) if m else None
    except OSError:
        return None


def find_untagged(node, name):
    '''The untagged instance of the subtree whose top node is `name`.'''
    if plain_name(node) == name and not tag_of(node):
        return node
    for c in children(node):
        hit = find_untagged(c, name)
        if hit is not None:
            return hit
    return None


def page_svg(body, content_h, number, total):
    '''One portrait page;  content taller than the page is scaled to fit.'''
    avail = PAGE_H - 2 * MARGIN - 20
    s = min(1.0, avail / content_h)
    dx = MARGIN + (PAGE_W - 2 * MARGIN) * (1 - s) / 2
    return '\n'.join([
        '<?xml version="1.0" encoding="UTF-8" standalone="no"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
        f'width="8.5in" height="11in" viewBox="0 0 {PAGE_W} {PAGE_H}">',
        f'<defs>{ARROW}</defs>',
        '<rect width="100%" height="100%" fill="#fff"/>',
        f'<g transform="translate({dx:.1f},{MARGIN}) scale({s:.4f})">',
        *body,
        '</g>',
        text(PAGE_W - MARGIN, PAGE_H - MARGIN + 8, f'IKBT behavior tree   {number} / {total}',
             10, 'end', 'fill:#888'),
        '</svg>', ''])


def build_pages():
    bt = build_default_bt()
    letters = Letters()
    img = root_image()
    width = PAGE_W - 2 * MARGIN
    gap = 36

    #  Box trees first, in reading order, so the letters run A, B, C ... down
    #  the pages and every panel knows who points at it before it is drawn.
    boxes = {name: to_boxes(bt.root if name is TOP else find_untagged(bt.root, name),
                            letters, top=True)
             for page in PAGES for name in page}

    pages = []
    for n, page in enumerate(PAGES, 1):
        body, y = [], 0
        for i, name in enumerate(page):
            if i:
                body.append(f'<path d="M 0,{y - gap / 2:.1f} H {width}" '
                            f'style="stroke:#bbb;stroke-width:1;stroke-dasharray:4,4"/>')
            svg, h = draw_panel(boxes[name], letters.refs.get(name, []), name,
                                0, y, width, img)
            body += svg
            y += h + gap
        if n == len(PAGES):
            body.append(f'<path d="M 0,{y - gap / 2:.1f} H {width}" '
                        f'style="stroke:#bbb;stroke-width:1;stroke-dasharray:4,4"/>')
            svg, h = legend(0, y, width)
            body += svg
            y += h
        pages.append((body, y))
    return [page_svg(body, h, n, len(pages)) for n, (body, h) in enumerate(pages, 1)]


def main():
    ap = argparse.ArgumentParser(description='Draw the IKBT behavior tree as portrait SVG pages.')
    ap.add_argument('-o', '--outdir', default='IKdocs')
    args = ap.parse_args()
    for n, svg in enumerate(build_pages(), 1):
        path = os.path.join(args.outdir, f'IKBT_BT_p{n}.svg')
        with open(path, 'w') as f:
            f.write(svg)
        print(f'wrote {path}')


if __name__ == '__main__':
    main()
