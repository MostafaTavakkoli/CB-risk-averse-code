import sys
import time
from z3 import *
import numpy as np

# Check if needed
# sys.path.append(r'C:\Users\ikoval\Desktop\z3-master\build\python')
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
    def __init__(self, pta, horizon, optType = 'min', uk = 0, printReport = 0, soft_violation_limit = -1):
        self.pta = pta
        self.optType = optType
        self.Horizon = horizon
        self.opt = Optimize()
        self.Ainc = self.setAinc()
        self.B = self.setB()
        self.P = self.setP()
        self.n_e = self.set_n_e()
        # self.pr = pr
        self.uk = uk if not self.pta.startAtInit else 0
        self.printReport = printReport
        self._soft_violation_limit = soft_violation_limit # -1 as the default infty value
        self._soft_constr_counter = 0
        self._soft_constr_list = []

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
        Bstr = [tr.name + '_%s' for tr in self.pta.transList]

        Bs = [[Bool(Bstr[i] % j ) for i in range(n_e)] for j in range(N)]
        Trs = [Bool(tr.name) for tr in self.pta.transList]

        Atil = [[Bool('a%s%s' % (i, j)) for i in range(n_e)] for j in range(n_e)]
        Xs = [Bool('x%s' % i) for i in range(n_e)]


        for i in range(N):
            for j in range(n_e):
                str1 = makeArr1(Bstr[j],'bool')
                exec(str1 % (i, i))
                
        # make the boolean variables for uk 
        for j in range(n_e):
            str1 = makeArr1(self.pta.transList[j].name,'bool','p')
            exec(str1)

        # Optimization engine
        opt = self.opt

        # if there is uk to apply, apply it
        if self.uk != 0:
            # opt.add(eval(self.pta.transList[np.where(self.uk==1)[0][0]].name+' == True'))
            for k in range(len(self.uk)):
                if self.uk[k]:
                    opt.add(eval(self.pta.transList[k].name+' == True'))
                else:
                    opt.add(eval(self.pta.transList[k].name+' == False'))

        # Price matrix for the transitions
        p = self.P

        ## Clock Dynamics
        # Get the clocks
        cLocalstr = [tr.locClock.name for tr in self.pta.stateList]
        cLoc = [Int(cLocalstr[i]) for i in range(len(self.pta.stateList))]

        cGlostr = [tr.gloClock.name for tr in self.pta.stateList]
        cGlo = [Int(cGlostr[i]) for i in range(len(self.pta.stateList))]

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
                    # For each soft constraint, put a soft id bool that 
                    # is equal to the soft constraint's hard version
                    exec_str = makeArr1('s_idx_','bool','counter')
                    exec(exec_str % (self._soft_constr_counter,self._soft_constr_counter))
                    # s_idx_counter will be true iff soft constraint is not violated 
                    opt.add(eval('('+'s_idx_'+str(self._soft_constr_counter)+'==('+co+'))'))
                    # add the variable to the list so that we can later use it
                    self._soft_constr_list.append('s_idx_'+str(self._soft_constr_counter))
                    # iterate the counter so that each soft constraint has a unique id
                    self._soft_constr_counter += 1
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
                        
                    # to apply uk in here
                    if self.uk != 0: 
                        appStr.append(str(k[ct]))
                            
                    s2 = 'Or(' + ','.join(appStr) + ')'
                    co1 = 'Implies(' + s2 +','+s+')'
                    opt.add(eval(co1))
                    co2 = 'Implies(' + s +','+s2+')'
                    opt.add(eval(co2))
                    #co.append('Implies(' + s1 +','+s2+')')
                #c0 = 'Sum(' + ','.join(co) + ')==1'
                #opt.add(eval(c0))
                #print(opt.check())
            elif (len(coListC) == 1):
                co = coListC[0].getConstrToAdd()
                opt.add(eval(co))
            else:
                continue
        #dumbStr = 'eBuffr_globl == eBuffr_local'
        #opt.add(eval(dumbStr))

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


        # Explicit slack variables here NOT IMPLEMENTED IN THIS SOLVER
        # g1 = Int('g1')
        # g2 = Int('g2')
        # gArr = [g1, g2]
        # dumbStr2 = '3<=AGV3_Q_local'
        # opt.add(eval(dumbStr2))
        # dumbStr2 = '3<=AGV4_Q_local'
        # opt.add(eval(dumbStr2))
        # opt.add(eval('0<= g1'))
        # opt.add(eval('0<= g2'))

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
        ##### THIS ONE IS FROM THE SIMULATION STUDY
        # label1 = Bool('label1')
        # label2 = Bool('label2')
        # # CNC1
        # if self.pr[0]:
        #     predicate1 = 'Or('+\
        #         ','.join([str(x) for x in np.transpose(Bs)[7]])+\
        #             ','+','.join([str(x) for x in np.transpose(Bs)[8]])+') == label1'
        #     opt.add(eval(predicate1))
        # # CNC2
        # if self.pr[1]:
        #     predicate2 = 'Or('+\
        #         ','.join([str(x) for x in np.transpose(Bs)[15]])+\
        #             ','+','.join([str(x) for x in np.transpose(Bs)[16]])+') == label2'
        #     opt.add(eval(predicate2))
        # opt.add(eval('And(label1,label2)'))
        ######################################

        label = Bool('label')
        predStr = []
        # get the number of the corresponding transition in Bs
        for state in self.pta.markedStates:

            x_a = [1 if x.name == state.name else 0 for x in self.pta.stateList]
            arr = np.transpose(B).dot(x_a)
            idx = np.where(arr == 1)
            pSt = [','.join([str(x) for x in np.transpose(Bs)[i]]) for i in idx[0]]
            predStr.append(pSt)
            # predicate = 'Or('+\
            #     ','.join([str(x) for x in np.transpose(Bs)[7]])+\
            #         ','+','.join([str(x) for x in np.transpose(Bs)[8]])+') == label1'
        predicate = 'Or('+ ','.join([i[0] for i in predStr]) + ') == label'
        
        opt.add(eval(predicate))
        opt.add(eval('label == True'))
       
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

        # Final state constraints?
        x_f = np.array([0,0,0,1])


        # objective function here
        z1 = Int('z1')
        
        # Look at the transitions and get the corresponding states
        # A lot of string tricks here

        # Put the initial ones here
        subs = [ str(p[0]) + '*(' + str(cLoc[0]) + '+' +  str(cGlo[0]) + ')']
        for i in range(len(p)):
            # for j in range(N):
                # Initial state will not have any incoming states
            if self.pta.stateList[i].name not in self.pta.incoming:
                continue
            else:
                for k in self.pta.incoming[self.pta.stateList[i].name]:
                    trString = 'Or('+','.join([str(k) + '_' + str(j) for j in range(N)]) +')'
                    objString = str(p[i]) + '*' + str(cLoc[i]) + '*' + trString
                    subs.append(objString)
                    
                    objString = str(p[i]) + '*' + str(cGlo[i]) + '*' + trString
                    subs.append(objString)
            # for time consistency in simulation
            # Initial state will not have any incoming states
            if self.uk != 0:
                if i == 0:
                    continue
                else:
                    for k in self.pta.incoming[self.pta.stateList[i].name]:                 
                                subs.append(str(p[i]) + '*' + str(cLoc[i]) + '*' + str(k))
                                
                                subs.append(str(p[i]) + '*' + str(cGlo[i]) + '*' + str(k))

        objS = 'z1 ==' + '+'.join(subs)
        opt.add(eval(objS))
        opt.minimize(z1)

        # Push the solver to create a backtracking point
        opt.push()
        # if a soft_violation limit is set
        if self._soft_violation_limit >= 0:
            # If the limit is more than the soft constraints, pass
            if len(self._soft_constr_list) <= self._soft_violation_limit:
                pass
            else:
                # Get the variables in z3 form
                z3_soft_constr_list = [Bool(cstr) for cstr in self._soft_constr_list]
                # If the constraint is violated, the var will be false
                cns = [If(cstr, 0, 1) for cstr in z3_soft_constr_list]
                # Total number of falses should be less than the limit
                opt.add(Sum(cns)<=self._soft_violation_limit)
                # TODO add opt.pop() here to redo computation with different violation limit

        #t0 = time.clock()
        t0 = time.perf_counter()
        try:
            opt.check()
            opt.model()
        except:
            print('Unsat')
            t1 = time.perf_counter() - t0
            #print("Time elapsed: ", t1 - t0) # CPU seconds elapsed (floating point)
            return
        t1 = time.perf_counter() - t0
        #print("Time elapsed: ", t1 - t0) # CPU seconds elapsed (floating point)
        del t0

        # Get the model
        m = opt.model()
        if self.printReport == 1 and opt.check() != unsat:
            print(opt.check())
            Us, localClocks, globalClocks, slackVars = self.reportRes(m, cLoc, cGlo, Bs, verbose=False)

        return {**self.toJSON_model(m, Bs), **self.toJSON_cGlo(m, cGlo), **self.toJSON_cLoc(m, cLoc),\
                **self.toJSON_violations(m, Bs, cLoc, cGlo)}

    def reportRes(self, m, cLoc, cGlo, Bs, verbose = True):
        # print('\n')
        # print('Objective Function:')
        # print(m[z1])
        localClocks = {}
        print('\n')
        print('Local Clocks:')
        print('----------------')
        for cL in cLoc:
            print(str(cL) + ':' + str(m[cL]))
            localClocks[str(cL)] = m[cL].as_long()

        globalClocks = {}
        print('\n')
        print('Global Clocks:')
        print('----------------')
        for gL in cGlo:
            print(str(gL) + ':' + str(m[gL]))
            globalClocks[str(gL)] = m[gL].as_long()

        slackVars = {}
        # print('\n')
        # print('Slack Variables:')
        # print('----------------')
        # #print('g1 :' + str(m[7]))
        # #print('g2 :' + str(m[127]))
        # for gL in gArr:
        #     print(str(gL) + ':' + str(m[gL]))
        #     slackVars[str(gL)] = m[gL].as_long()

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
            pathstateList = []
            print('\n')
            print('Events:')
            print('----------------')
            for ct, idx in enumerate(Bs):
                print('u_%s =' % ct)
                for trNum, b in enumerate(idx):
                    if m[b]:
                        print(str(b) + ':' + str(m[b]))
                        Us[ct] = trNum
                        q=str(b)
                        for trans in self.pta.transList:
                            if trans.name == q[:-2]:
                                pathstateList.append(str(trans.source.name))
                    else:
                        continue
                print('')
        print('States:')
        print('----------------')
        print(pathstateList)
        #print(str(pathstateList[i]) for i in pathstateList)
        print('\n')
        print('Violations:')
        print('----------------')

        # Checking which constraints were violated
        for state in self.pta.stateList:
            coList = state.invs

            for cdx in coList:
                co = cdx.getConstrToAdd()
                co_evaluated = co

                for clockName in localClocks:
                    co_evaluated = co_evaluated.replace(clockName, str(localClocks[clockName]))

                for clockName in globalClocks:
                    co_evaluated = co_evaluated.replace(clockName, str(globalClocks[clockName]))

                s = Solver()
                s.add(eval(co_evaluated))

                if s.check() == unsat:

                    # better visualization for the constraint
                    print(state.name+":")
                    print_string = co
                    print_string = print_string.replace(state.name,"")
                    print_string = print_string.replace("_globl","G")
                    print_string = print_string.replace("_local","L")
                    print_string = print_string.replace("Or","Or ")

                    # print out the constraint and it's valuation
                    print("\tConstraint: " + print_string)
                    print("\tValuation: "+str(co_evaluated))

                del s

        return Us, localClocks, globalClocks, slackVars

    def toJSON_model(self, m, Bs):

        return_dict = {}

        for ct, idx in enumerate(Bs):
            for b in idx:
                if m[b]:
                    return_dict[str(b)] = 'test'
                else:
                    continue

        return {"edges":return_dict}

    def toJSON_cGlo(self, m, cGlo):

        return_dict = {}

        for gL in cGlo:
            return_dict[str(gL)] = m[gL].as_long()

        return {"globalClocks":return_dict}

    def toJSON_cLoc(self, m, cLoc):

        return_dict = {}

        for cL in cLoc:
            return_dict[str(cL)] = m[cL].as_long()

        return {"localClocks":return_dict}

    def toJSON_violations(self, m, Bs, cLoc, cGlo):

        localClocks = {}
        for cL in cLoc:
            localClocks[str(cL)] = m[cL].as_long()

        globalClocks = {}
        for gL in cGlo:
            globalClocks[str(gL)] = m[gL].as_long()

        return_violations = []
        # Checking which constraints were violated
        for state in self.pta.stateList:
            coList = state.invs

            for cdx in coList:
                co = cdx.getConstrToAdd()
                co_evaluated = co

                for clockName in localClocks:
                    co_evaluated = co_evaluated.replace(clockName, str(localClocks[clockName]))

                for clockName in globalClocks:
                    co_evaluated = co_evaluated.replace(clockName, str(globalClocks[clockName]))

                s = Solver()
                s.add(eval(co_evaluated))

                if s.check() == unsat:
                    return_violations.append([state,co])

                del s

        return {"violations": return_violations}
