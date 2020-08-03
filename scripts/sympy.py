from sympy import *
x,y,z = symbols("x y z")
id0, iq0, th0, zd,zq,vd0,vq0,zavg,zdiff = symbols("id0, iq0, th0, zd,zq,vd0,vq0,zavg,zdiff")
init_printing()

f1 = (zd+zq)/2 - zavg
f2 = zd - zq - zdiff

solve([f1,f2],[zd,zq])

a = Matrix([[cos(th0), -sin(th0)],[sin(th0), cos(th0)]])

b = Matrix([[1/(zavg + zdiff/2), 0],[0, 1/(zavg - zdiff/2)]])
c = Matrix([[cos(th0), sin(th0)],[-sin(th0), cos(th0)]])
d = Matrix([[vd0], [vq0]])