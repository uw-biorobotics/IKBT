#Gary11 is 6DOF, orignal submission has forward kinematics errors
# identified new solving methods: tangent, then 2 unkn 2 equations patern (but with more unkn), stuck

## -- End pasted text --
term:
cos(th₁)⋅cos(th₂ + th₃ - th₄ + th₅)
term:
sin(-th₁ + th₂ + th₃ - th₄ + th₅) + sin(th₁ + th₂ + th₃ - th₄ + th₅)
────────────────────────────────────────────────────────────────────
                                 2
term:
-sin(th₁)
term:
-d₆⋅sin(th₁) + l₁⋅sin(th₁) + l₂⋅cos(th₁)⋅cos(th₂) + l₃⋅cos(th₁)⋅cos(th₂ + th₃)
term:
sin(th₁)⋅cos(th₂ + th₃ - th₄ + th₅)
term:
cos(-th₁ + th₂ + th₃ - th₄ + th₅) - cos(th₁ + th₂ + th₃ - th₄ + th₅)
────────────────────────────────────────────────────────────────────
                                 2
term:
cos(th₁)
term:
d₆⋅cos(th₁) - l₁⋅cos(th₁) + l₂⋅sin(th₁)⋅cos(th₂) + l₃⋅sin(th₁)⋅cos(th₂ + th₃)
term:
sin(th₂ + th₃ - th₄ + th₅)
term:
-cos(th₂ + th₃ - th₄ + th₅)
term:
0
term:
l₂⋅sin(th₂) + l₃⋅sin(th₂ + th₃)
term:
0
term:
0
term:
0
term:
1

Solved th_1 with tansolver

After solving th_1
new T
cos(th₂ + th₃ - th₄ + th₅)
-sin(th₂ + th₃ - th₄ + th₅)
0
l₂⋅cos(th₂) + l₃⋅cos(th₂ + th₃)
0
0
-1
-l₁
sin(th₂ + th₃ - th₄ + th₅)
cos(th₂ + th₃ - th₄ + th₅)
0
l₂⋅sin(th₂) + l₃⋅sin(th₂ + th₃)
0
0
0
1

stuck here!
