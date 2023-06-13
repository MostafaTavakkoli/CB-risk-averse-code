import sys
import time
sys.path.append(r'C:\Users\baltaefe\Repos\z3\build\python')
# sys.path.append(r'C:\Users\Efe Balta\Documents\Repos\z3\build\python')
from z3 import *
import numpy as np


# Some global functions 
# Do a loop with list instead
def makeArr1(st, ty, flavor = 'p'):
    # Plain or enumeration format
    if flavor == 'p':
        s = st
    else:
        s = st + '%s'

    # Type of the variable
    if ty == 'bool':
        execStr = s + "=Bool('" + s + "')"
    elif ty == 'int':
        execStr = s + "=Int('" + s + "')"
    elif ty == 'real':
        execStr = s + "=Real('" + s + "')"
    else:
        execStr = s + "=Bool('" + s + "')"
    return execStr

def makeArr2(st):
    s = st + '%s%s'
    execStr = s + "=Bool('" + s + "')"
    return execStr

def z3MultB(A,x):
            r =  [[A[j][i]*x[i] for i in range(len(x))] for j in range(len(x))]
            return r

class ptaSolver:
    def __init__(self, pta, optType = 'min', pr = [1,1]):
        self.pta = pta
        self.optType = optType
        self.Horizon = 8
        self.opt = Optimize()
        self.Ainc = self.setAinc()
        self.B = self.setB()
        self.P = self.setP()
        self.n_e = self.set_n_e()
        self.pr = pr

    # set funcitons extract the pta out of the PTA
    def setAinc(self):
        return self.pta.Ainc

    def setB(self):
        return self.pta.B
        
    def setP(self):
        return self.pta.P

    def set_n_e(self):
        return len(self.pta.transList)

    # Utilities
    def solvePTA (self):

        # State Transitions
        # B = np.matrix([[0, 0, 0, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 1]])
        B = self.B

        # Incidence matrix 
        # Ainc = np.matrix([[1, 1, 0, 0], [1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 1]])
        # Ainc = np.matrix([[1, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1], [0, 0, 1, 1]])
        Ainc = self.Ainc
        # Ainc = np.transpose(self.pta.AincBD)

        # Convenient format in the event space
        Atilde = np.transpose(Ainc)*np.matrix(B)
        # Atilde = np.transpose(Ainc)*np.matrix(B)
        #Atilde = np.matrix(Ainc)*B

        # horizon
        N = self.Horizon

        # event
        n_e = self.n_e

        # state
        n_s = len(self.pta.stateList)

        # List to create z3 variables. Cannot directly work with these
        # Bs = [[Bool('b%s%s' % (j, i) ) for i in range(n_e)] for j in range(N)]
        Bstr = [tr.name + '_%s' for tr in self.pta.transList]

        # Bs = [[exec(Bstr[i] % (i, j, i, j)) for i in range(n_e)] for j in range(N)]
        Bs = [[Bool(Bstr[i] % j ) for i in range(n_e)] for j in range(N)]

        Atil = [[Bool('a%s%s' % (i, j)) for i in range(n_e)] for j in range(n_e)]
        Xs = [Bool('x%s' % i) for i in range(n_e)]


        for i in range(N):
            for j in range(n_e):
                str1 = makeArr1(Bstr[j],'bool')
                exec(str1 % (i, i))



        # Optimization engine
        opt = self.opt

        # Price matrix for the transitions
        p = self.P

        ## Clock Dynamics
        # Get the clocks
        cLocalstr = [tr.locClock.name for tr in self.pta.stateList]
        cLoc = [Real(cLocalstr[i]) for i in range(len(self.pta.stateList))]

        cGlostr = [tr.gloClock.name for tr in self.pta.stateList]
        cGlo = [Real(cGlostr[i]) for i in range(len(self.pta.stateList))]

        count = 0
        for i in self.pta.stateList:
           str1 = makeArr1(i.locClock.name,'int')
           exec(str1)
           str1 = makeArr1(i.gloClock.name,'int')
           exec(str1)
           #initialize
           if i.name == self.pta.currState[0].name:
                indexCurrentState = count
           else:
                count+=1

        # Add the constraints
        for i in self.pta.stateList:
            coList = i.invs
            for cdx in coList:
                co = cdx.getConstrToAdd()
                if cdx.soft:
                    opt.add_soft(eval(co), cdx.weight)
                else:
                    opt.add(eval(co))

            # Clock additions may be multiple for accepting states
            coListC = i.invsC
            # if there are multiple
            if (len(coListC) > 1):
                co = []
                for (ct,con) in enumerate(coListC):
                    s = con.getConstrToAdd()
                    s1 = 'If(' + s + ',1,0)' # not used right now
                    k = self.pta.incoming[i.name]
                    appStr = []
                    for j in range(N):
                        # Get the b_i(N) 
                        objString = str(k[ct]) + '_' + str(j)
                        appStr.append(objString)
                    s2 = 'Or(' + ','.join(appStr) + ')'
                    co1 = 'Implies(' + s2 +','+s+')'
                    opt.add(eval(co1))
                    co2 = 'Implies(' + s +','+s2+')'
                    opt.add(eval(co2))
                    #co.append('Implies(' + s1 +','+s2+')')
                #c0 = 'Sum(' + ','.join(co) + ')==1'
                #opt.add(eval(c0))
                print(opt.check())
            elif (len(coListC) == 1):
                co = coListC[0].getConstrToAdd()
                opt.add(eval(co))
            else:
                continue
        dumbStr = 'eBuffr_globl == eBuffr_local'
        opt.add(eval(dumbStr))

        # Apply the guard constraints
        for e in self.pta.transList:
            if e.guard:
                s = e.guard.getConstrToAdd()
                k = e.name # assume 1 guard for now
                appStr = []
                for j in range(N):
                    # Get the b_i(N) 
                    objString = str(k) + '_' + str(j)
                    appStr.append(objString)
                s2 = 'Or(' + ','.join(appStr) + ')'
                co1 = 'Implies(' + s2 +','+s+')'
                opt.add(eval(co1))
                co2 = 'Implies(' + s +','+s2+')'
                opt.add(eval(co2))
                a = 3


        # Explicit slack variables here
        g1 = Real('g1')
        g2 = Real('g2')
        gArr = [g1, g2]
        dumbStr2 = '3-g1<=AGV3_Q_local'
        opt.add(eval(dumbStr2))
        dumbStr2 = '3-g2<=AGV4_Q_local'
        opt.add(eval(dumbStr2))
        opt.add(eval('0<= g1'))
        opt.add(eval('0<= g2'))

        # Control constraints loop

        # initial conditions
        x_0 =np.zeros(n_s, dtype=np.int)
        x_0[indexCurrentState]=1

        # Using the graph, get the incident events
        enable_k = np.dot(np.transpose(Ainc), x_0)
        # Get the constraint format of the enabled events
        enabledC_k = [enable_k.tolist()[i]*If(Bs[0][i], 1, 0) for i in range(n_e)]
        # Add the constraints for the initial condition
        opt.add(Sum(enabledC_k)==1)


        # A pseudo spec automaton GET THIS WITH FUNCTION   
        
        ############# add the seqSAT here #### 
        ##### Get this from the inputs of the function automatically
        label1 = Bool('label1')
        label2 = Bool('label2')
        # CNC1
        if self.pr[0]:
            predicate1 = 'Or('+\
                ','.join([str(x) for x in np.transpose(Bs)[7]])+\
                    ','+','.join([str(x) for x in np.transpose(Bs)[8]])+') == label1'
            opt.add(eval(predicate1))
        # CNC2
        if self.pr[1]:
            predicate2 = 'Or('+\
                ','.join([str(x) for x in np.transpose(Bs)[15]])+\
                    ','+','.join([str(x) for x in np.transpose(Bs)[16]])+') == label2'
            opt.add(eval(predicate2))
        opt.add(eval('And(label1,label2)'))
        
        ######################################

        for j in range(N):

            # This is the global SAT constraint that says: one action per horizon step
            cns = [If(Bs[j][i], 1, 0) for i in range(n_e)]
            opt.add(Sum(cns)==1)
    
            # For fun
            u_k = Bs[j]
            # r = z3MultB(np.transpose(Atilde).tolist(), u_k)
            cns2 = []
    
            # This is where the FSA connectivity is encoded as SAT constraints
            # Do this for the N-1 transitions
            if j < N-1:
                # for each of the possible actions
                for i in range(len(u_k)):
                    # This condition is in row-space form for enabled edges after each action
                    e_kp1 = np.transpose(Atilde)[i].tolist()
                    # Get the indices of ones in the list
                    inx_kp1 = [idx for idx,val in enumerate(e_kp1[0]) if val==1]                   
                    # Create the constraint format [this one uses mXOR]
                    inxBs_kp1 = [If(Bs[j+1][k], 1, 0) for k in inx_kp1]

                    cns2.append([u_k[i], inxBs_kp1])
        
                # Assign the constraints
                for i in range(len(cns2)):
                    # This one uses only mXOR with ito true trick
                    # opt.add(If(cns2[i][0], Sum(cns2[i][1]) == 1, True))

                    # This one uses logical implication
                    opt.add(Implies(cns2[i][0], Sum(cns2[i][1]) == 1))

            #x_k = z3MultB(B, u_k)
            #print(x_k)

        # Final state constraints?
        x_f = np.array([0,0,0,1])


        # objective function here
        z1 = Int('z1')
        # clocksLoc = [c.locClock.name for c in self.pta.stateList]
        # clocksGlo = [c.gloClock.name for c in self.pta.stateList]
        # zippedS = zip
        # Assuming the first clock is the initial global clock
        # S = [cGlo[0]]
        
        # obj = [()]
        # opt.add(z1 == p[1]*Bs[0][0]*cLoc[1] + p[2]*Bs[0][1]*cLoc[2] + p[1]*Bs[1][2]*cLoc[1] + \
        #    p[2]*Bs[1][3]*cLoc[2])
        
        # Look at the transitions and get the corresponding states
        # A lot of string tricks here

        # Put the initial ones here
        subs = [ str(p[0]) + '*' + str(cLoc[0])]
        for i in range(len(p)):
            for j in range(N):
                # Initial state will not have any incoming states
                if i == 0:
                    continue
                else:
                    for k in self.pta.incoming[self.pta.stateList[i].name]:
                        objString = str(p[i]) + '*' + str(cLoc[i]) + '*' + str(k) + '_' + str(j)
                        subs.append(objString)
                        objString = str(p[i]) + '*' + str(cGlo[i]) + '*' + str(k) + '_' + str(j)
                        subs.append(objString)
        
        objS = 'z1 ==' + '+'.join(subs)  + '+' +'60*g1'+ '+' +'40*g2'
        opt.add(eval(objS))
        # opt.add(z1 == p[0]*cGlo[0] + p[1]*Bs[0][3]*cLoc[1] + p[2]*Bs[0][4]*cLoc[2] + p[3]*cGlo[3])
        opt.minimize(z1)

        # Objective function for the optimization
        # objFun = [If(Bs[i][j], 1, 0) for i in range(N) for j in range(n_e)]
        # opt.minimize(Sum(objFun))

        print(opt)

        t0 = time.clock()
        try:
            opt.check()
            opt.model()
        except:
            print('Unsat')
            t1 = time.clock() - t0
            print("Time elapsed: ", t1 - t0) # CPU seconds elapsed (floating point)
            return
        t1 = time.clock() - t0
        print("Time elapsed: ", t1 - t0) # CPU seconds elapsed (floating point)
        del t0
        # Print results for SAT and model
        #print(opt.check())
        #print(opt.model())

        # Get the model 
        m = opt.model()
        Us, localClocks, globalClocks, slackVars = self.reportRes(m, cLoc, cGlo, Bs, gArr, verbose = False)
        return Us, localClocks, globalClocks, slackVars, t1
    
    def reportRes(self, m, cLoc, cGlo, Bs, gArr, verbose = True):
        # print('\n')
        # print('Objective Function:')
        # print(m[z1])
        localClocks = {}
        mDict = {str(d):m[d] for d in m}
        print('\n')
        print('Local Clocks:')
        print('----------------')
        for cL in cLoc:
            print(str(cL) + ':' + str(mDict[str(cL)]))
            localClocks[str(cL)] = float(str(mDict[str(cL)]))

        globalClocks = {}    
        print('\n')
        print('Global Clocks:')
        print('----------------')
        for gL in cGlo:
            print(str(gL) + ':' + str(mDict[str(gL)]))
            globalClocks[str(gL)] = float(str(mDict[str(gL)]))

        slackVars = {}
        print('\n')
        print('Slack Variables:')
        print('----------------')
        #print('g1 :' + str(m[7]))
        #print('g2 :' + str(m[127]))
        for gL in gArr:
            print(str(gL) + ':' + str(mDict[str(gL)]))
            slackVars[str(gL)] = float(str(mDict[str(gL)]))
        Us = {}
        if verbose:
            print('\n')
            print('Events:')
            print('----------------')
            for ct, idx in enumerate(Bs):
                print('u_%s =' % ct)
                for b in idx:
                    print(str(b) + ':' + str(m[b]))
        else:
            print('\n')
            print('Events:')
            print('----------------')
            for ct, idx in enumerate(Bs):
                print('u_%s =' % ct)
                for trNum, b in enumerate(idx):
                    if m[b]:
                        print(str(b) + ':' + str(m[b]))
                        Us[ct] = trNum
                    else:
                        continue
                print('')
        print('z1 = ',str(mDict['z1']))
        return Us, localClocks, globalClocks, slackVars
    
   # def reportOpt(self):