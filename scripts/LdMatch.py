import numpy as np
import matplotlib.pyplot as pyplot
from scipy.optimize import curve_fit
def f_fit(x, a, b, c, d):
    return d*x**3 + a*x**2+b*x+c
x=list(range(9))
for i in np.arange(-4, 4, 1):
    x[i]=i
x.sort()
y=[0.098,0.135,0.17,0.205,0.238, 0.265, 0.28, 0.295, 0.301]
y=[50, 50, 50, 47, 42, 34, 28, 22, 20]
p_fit, prov = curve_fit(f_fit, x, y)
print(p_fit)  #三个元素
print(p_fit[0])  #其中a,b,c分别元素
pyplot.plot(x,y)

a=[]
for i in x:
    a.append(f_fit(i, p_fit[0], p_fit[1], p_fit[2], p_fit[3]))
pyplot.plot(x,a)
pyplot.show()