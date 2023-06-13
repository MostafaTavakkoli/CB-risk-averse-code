import sys
sys.path.append(r'C:\Users\baltaefe\Repos\z3\build\python')
from z3 import *
import numpy as np

# State Transitions
B = np.matrix([[0, 0, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1]])

# Incidence matrix 
Ainc = np.matrix([[1, 1, 0, 0], [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]])
# Convenient format in the event space
Atilde = np.transpose(Ainc)*B

# horizon
N = 2

# state
n_e = 4

# List to create z3 variables. Cannot directly work with these
Bs = [[Bool('b%s%s' % (j, i) ) for i in range(n_e)] for j in range(N)]
Atil = [[Bool('a%s%s' % (i, j)) for i in range(n_e)] for j in range(n_e)]
Xs = [Bool('x%s' % i) for i in range(n_e)]
# print(Bs)

# Do a loop with list instead
def makeArr2(st):
    s = st + '%s%s'
    execStr = s + "=Bool('" + s + "')"
    return execStr

for i in range(N):
    for j in range(n_e):
        exec(makeArr2('b') % (i, j, i, j))

for i in range(n_e):
    for j in range(n_e):
        exec(makeArr2('a') % (i, j, i, j))


# Optimization engine
opt = Optimize()
# opt.add(If(a[0] == 1, 1, 0))

def z3MultB(A,x):
    r =  [[A[j][i]*x[i] for i in range(len(x))] for j in range(len(x))]
    return r

# Price matrix for the transitions
p = np.array([1,2,1,1])

# Control constraints loop
# Control in step k (b00, b01, b02, b03) * Atilde => Enabled edges (undirected)
# u_k = Bs[0]
# enble_kp1 = Atil * np.reshape(u_k, (4,1))
# r = z3MultB(Atilde.tolist(), u_k)

# initial conditions
AtildeT = np.transpose(Atilde)
x_0 = np.array([1,0,0,0])

# Using the graph, get the incident events
enable_k = Ainc*np.reshape(x_0, (n_e, 1))
# Get the constraint format of the enabled events
enabledC_k = [enable_k.tolist()[i][0]*If(Bs[0][i], 1, 0) for i in range(n_e)]
# Add the constraints for the initial condition
opt.add(Sum(enabledC_k)==1)

# What needs to happen automatically
'''
opt.add( Sum(If(b00, 1, 0), If(b01, 1, 0)) == 1, \
         If(b00, Xor(b10, b12), True), \
         If(b01, Xor(b11, b13), True), \
        # If(b02, Xor(b12, b13), True), Sum(If(b12, 1, 0), If(b13, 1, 0)) == 1, \
        # If(b03, Xor(b12, b13), True), Sum(If(b12, 1, 0), If(b13, 1, 0)) == 1)
         Sum(If(b10,1,0),If(b11,1,0),If(b12,1,0),15*If(b13,1,0)) == 1)
'''


# A pseudo spec automaton
opt.add(Xor(b12, b13)) # Use state relationships instead?



for j in range(N):

    # This is the global SAT constraint that says: one action per horizon step
    cns = [If(Bs[j][i], 1, 0) for i in range(n_e)]
    opt.add(Sum(cns)==1)
    
    # For fun
    u_k = Bs[j]
    r = z3MultB(Atilde.tolist(), u_k)
    cns2 = []

    # State dynamics not very useful at the moment
    x_kp1 = z3MultB(B.tolist(),u_k)

    # This is where the FSA connectivity is encoded as SAT constraints
    # Do this for the N-1 transitions
    if j < N-1:
        # for each of the possible actions
        for i in range(len(u_k)):
            # This condition is in row-space form for enabled edges after each action
            e_kp1 = np.transpose(Atilde)[i].tolist()
            # Get the indices of ones in the list
            inx_kp1 = [idx for idx,val in enumerate(e_kp1[0]) if val==1]
            # Create the constraint format
            inxBs_kp1 = [If(Bs[j+1][k], 1, 0) for k in inx_kp1]
            cns2.append([u_k[i], inxBs_kp1])
        
        # Assign the constraints
        for i in range(len(cns2)):
            opt.add(If(cns2[i][0], Sum(cns2[i][1]) == 1, True))

    x_k = z3MultB(B.tolist(), u_k)
    print(x_k)

    '''
    cns = []
    for k in range(4):
        for l in range(4):
            if type(r[l][k]) == z3.z3.BoolRef:
                if r[l][k] not in cns:
                #    cns = [r[k][l]*If(Bs[j][i],1,0) for i in range(n_e)]
                    cns.append(If(r[k][l],1,0)) 
    # opt.add(Sum(cns)==1)
    '''

# Final state constraints?
x_f = np.array([0,0,0,1])

# Objective function for the optimization
objFun = [If(Bs[i][j], 1, 0) for i in range(N) for j in range(n_e)]
opt.minimize(Sum(objFun))

print(opt)
print(opt.check())
print(opt.model())
# m = opt.model()
