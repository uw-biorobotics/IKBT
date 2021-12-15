import sympy as sp
 
 
sp.var('Px Pz')
sp.var('th_1 th_23 th_2 th_3 th_4 th_5 th_6')
sp.var('a_2 a_3 d_4')
print('prelim test data for x2z2_solver')

Td = sp.zeros(4)
Ts = sp.zeros(4)
Td[0,3] =  Px
Ts[0,3] =  a_3*sp.cos(th_23) - d_4*sp.sin(th_23) + a_2 * sp.cos(th_2)

Td[2,3] =  -Pz
Ts[2,3] =  a_3*sp.sin(th_23) + d_4*sp.cos(th_23) + a_2 * sp.sin(th_2)

td12 = Td[0,3] * Td[0,3]
ts12 = Ts[0,3] * Ts[0,3]

td22 = Td[2,3] * Td[2,3]
ts22 = Ts[2,3] * Ts[2,3]

print('LHS: x2+z2:')
print(td12+td22)
print('= RHS: x2+z2:')
print(ts12+ts22)
 
