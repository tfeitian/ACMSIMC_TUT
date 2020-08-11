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

zdh,rdh,zqh,rqh,Ld,Lq,Vinj,w,t = symbols("zdh,rdh,zqh,rqh,Ld,Lq,Vinj,w,t")

zdh = rdh +I*w*Ld
zqh = rqh + I*w*Lq

zdiff = zdh-zqh

zavg = 1/2*(zdh+zqh)

idsh = Vinj*cos(w*t)/(zdh*zqh)*(zavg-1/2*zdiff*cos(2*th0))

pprint((idsh*sin(w*t)).simplify())

iqsh = Vinj * cos(w*t)/zdh/zqh*(-1/2*zdiff*sin(2*th0))
