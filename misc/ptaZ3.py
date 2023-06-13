from z3 import *

#x = Real('x')
#y = Real('y')
#s = Solver()
#s.add(x + y > 5, x > 1, y > 1)
#print(s.check())
#print(s.model())


# Simple implementation of the ptaMPC.rkt 
t1 = Int('t1')
t2 = Int('t2')
t3 = Int('t3')
l2 = Int('l2')
l3 = Int('l3')

b2 = Bool('b2')
b3 = Bool('b3')

def b2i(b):
    val = 1 if b else 0
    return val

s = Optimize()
s.add(2 <= t1, t1 <= 3 ,t2 <= 6, t3 <= 6, l2 >= 3, l3 >= 3, t2 == t1+l2, t3 == t1 + l3)
#s.add(t2 <= 6, t3 <= 6, l2 >= 3, l3 >= 3)
#s.add(t2 == t1+l2, t3 == t1+l3)
sol = s.minimize(l2+l3+t1)
m = s.model()
print(m)

x = Real('x')
opt = Optimize()
opt.add(x >= 0, x<=10)
h = opt.minimize(x)
print (opt.check())
print (opt.upper(h)) 
print (opt.model())
