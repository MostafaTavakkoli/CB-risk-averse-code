import time
import src.ptaDefs as pd
import src.ptaSolver_wSlackFromNoSlack as ps
import numpy as np

tagv1 = 1
tcnc1 = 15
tcnc2 = 10
tqc = 2
tagvb = 3

# This example is the Factory_Master layout in the MPC final project

# Define the states of the PTA
eBuffr = pd.ptaState('eBuffr')
AGV1_E = pd.ptaState('AGV1_E')
CNC1_B1 = pd.ptaState('CNC1_B1')
CNC1   = pd.ptaState('CNC1',cost=3)
CNC1_B2 = pd.ptaState('CNC1_B2')
CNC2_B1 = pd.ptaState('CNC2_B1')
CNC2   = pd.ptaState('CNC2',cost=3)
CNC2_B2 = pd.ptaState('CNC2_B2')
AGV2_1 = pd.ptaState('AGV2_1')
AGV2_2 = pd.ptaState('AGV2_2')
qCheck = pd.ptaState('qCheck')
AGV3_Q = pd.ptaState('AGV3_Q', cost = 3)
AGV4_Q = pd.ptaState('AGV4_Q', cost = 2)
tBuffr = pd.ptaState('tBuffr')

# Define guards
guar1 = pd.Constr(2, '<=', qCheck.locClock.name)
guar2 = pd.Constr(2, '<=', qCheck.locClock.name)

# Define the transitions of the PTA
tr0 = pd.Transition(eBuffr, AGV1_E, 'tr0')
tr1 = pd.Transition(AGV1_E, CNC1_B1, 'tr1')
tr2 = pd.Transition(AGV1_E, CNC2_B1, 'tr2')
tr3 = pd.Transition(CNC1_B1, CNC1, 'tr3')
tr4 = pd.Transition(CNC2_B1, CNC2, 'tr4')
tr5 = pd.Transition(CNC1_B1, CNC1_B2, 'tr5')
tr6 = pd.Transition(CNC2_B1, CNC2_B2, 'tr6')
tr7 = pd.Transition(CNC1, CNC1_B2, 'tr7')
tr8 = pd.Transition(CNC2, CNC2_B2, 'tr8')
tr9 = pd.Transition(CNC1_B2, AGV2_1, 'tr9')
tr10 = pd.Transition(CNC2_B2, AGV2_2, 'tr10')
tr11 = pd.Transition(AGV2_1, qCheck, 'tr11')
tr12 = pd.Transition(AGV2_2, qCheck, 'tr12')
tr13 = pd.Transition(qCheck, AGV3_Q, 'tr13')#, guard = guar1)
tr14 = pd.Transition(qCheck, AGV4_Q, 'tr14')#, guard = guar2)
tr15 = pd.Transition(AGV3_Q, tBuffr, 'tr15')
tr16 = pd.Transition(AGV4_Q, tBuffr, 'tr16')

transList = [tr0, tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9, tr10, tr11, tr12, tr13, tr14, tr15, tr16]

### The MPC loop
Nmpc = 8
globalTime = 0
horizon = 8
# initial conditions
currentState = eBuffr
n_s, n_e = 14, 17
x_k = np.zeros(n_s, dtype=np.int)
x_k[0] = 1
Xlist = [x_k]
Ulist = []
Slist = [currentState]
Tlist = [globalTime]
Zlist = []
execTimes = []
slackDict = {}
uk = 0

for k_mpc in range(Nmpc):
    #### Adaptive Nmpc

    # Constraints evaluation after discrete transition
    c1 = pd.Constr(1, '<=', eBuffr.locClock.name)
    c2 = pd.Constr(tagv1, '<=', AGV1_E.locClock.name)
    c3 = pd.Constr(tcnc1, '<=', CNC1.locClock.name)
    c4 = pd.Constr(tcnc1+4, '<=', AGV2_1.gloClock.name)
    c5 = pd.Constr(1, '<=', AGV2_1.locClock.name)
    c6 = pd.Constr(tcnc2, '<=', CNC2.locClock.name)
    c7 = pd.Constr(tcnc1+11, '<=', AGV2_2.gloClock.name)
    c8 = pd.Constr(1, '<=', AGV2_2.locClock.name)
    co1l = pd.Constr(tqc,'<=', qCheck.locClock.name)
    co1g = pd.Constr(tqc,'<=', qCheck.gloClock.name)
    co3l2 = pd.Constr(0, '<', AGV3_Q.locClock.name)
    co3l3 = pd.Constr(0, '<', AGV4_Q.locClock.name)
    if k_mpc>0:
        # To keep the global time consistent throughout the simulation
        consistency = pd.Constr(Slist[k_mpc-1].gloClock.name, '==', globalTime)
        # add it to a non-used state invariant
        eBuffr.invs = [consistency]
        
    ###### activate these for the slack solution ####
    tf = pd.Constr(23,'>=', tBuffr.gloClock.name)
    tBuffr.invs = [tf]
    #################################################

    # assigns private constraints
    eBuffr.invs.append(c1)
    AGV1_E.invs = [c2]
    CNC1.invs = [c3]
    AGV2_1.invs = [c4,c5]
    CNC2.invs = [c6]
    AGV2_2.invs = [c7,c8]
    qCheck.invs = [co1l,co1g]
    AGV3_Q.invs = [co3l2]
    AGV4_Q.invs = [co3l3]

    stateList = [eBuffr, AGV1_E, CNC1_B1, CNC1, CNC1_B2, AGV2_1,\
         CNC2_B1, CNC2, CNC2_B2, AGV2_2, qCheck, AGV3_Q, AGV4_Q, tBuffr]

    #### create the PTA
    A = pd.PTA(stateList,transList,[currentState],[tBuffr])

    #### solve the optimization
    if horizon == 4:
        ######### horizon = 4 #####
        solver = ps.ptaSolver(A,horizon, 'min',[1,0],uk) if k_mpc <= 3 else ps.ptaSolver(A,horizon, 'min',[0,1],uk)
        ###########################
    elif horizon == 6:
        ######### horizon = 6 #####
        if k_mpc <= 3:
            solver = ps.ptaSolver(A,horizon, 'min',[1,0],uk)
        elif k_mpc == 13: # not necessary
            solver = ps.ptaSolver(A,horizon, 'min',[1,1],uk)
        else:
            solver = ps.ptaSolver(A,horizon, 'min',[0,1],uk)
        ###########################
    else:
        ######### horizon = 8 #####
        solver = ps.ptaSolver(A,horizon, 'min',[1,1],uk) if k_mpc <= 3 else ps.ptaSolver(A,horizon, 'min',[0,1],uk)
        ###########################

    Us, localClocks, globalClocks, slackVars, execT, z1 = solver.solvePTA()
    execTimes.append(execT)
    slackDict[k_mpc] = slackVars
    Zlist.append(z1)

    #### apply the control action
    # get the control action
    uk = np.zeros(n_e, dtype=np.int)
    uk[Us[0]] = 1
    Ulist.append(uk)

    # update the time
    globalTime += localClocks[currentState.name+'_local']
    # globalTime = globalClocks[currentState.name+'_globl']
    Tlist.append(globalTime)

    # get the next state
    xk = np.dot(A.B, uk)
    Xlist.append(xk)
    currentState = stateList[np.where(xk == 1)[0][0]]
    Slist.append(currentState)
    
    #### reset invariants for the next loop
    A.resetStateInvs()
    del(solver)

print(execTimes)
print(np.mean(execTimes), np.std(execTimes))
print([x.name for x in Slist])
print(Tlist)
print([np.where(x==1)[0][0] for x in Ulist])
print(slackDict)
print(Zlist)