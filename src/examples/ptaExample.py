import time
import src.ptaDefs as pd
import src.ptaSolver as ps

tagv1 = 1
tcnc1 = 15
tcnc2 = 10
tqc = 2
tagvb = 3



# Define the states of the PTA
qCheck = pd.ptaState('qCheck')
AGV3_Q = pd.ptaState('AGV3_Q')
AGV4_Q = pd.ptaState('AGV4_Q')
tBuffr = pd.ptaState('tBuffr')
storage = pd.ptaState('storage')

# Define the transitions of the PTA
tr1 = pd.Transition(qCheck, AGV3_Q, 'tr1')
tr3 = pd.Transition(AGV3_Q, tBuffr, 'tr3')
tr2 = pd.Transition(qCheck, AGV4_Q, 'tr2')
tr4 = pd.Transition(AGV4_Q, tBuffr, 'tr4')
tr5 = pd.Transition(tBuffr, storage, 'tr5')


# Constraints
co1l = pd.Constr(tqc,'<=', qCheck.locClock.name)
co1g = pd.Constr(tqc,'<=', qCheck.gloClock.name)
co2l = pd.Constr(tagvb, '<=', AGV3_Q.locClock.name)
co2g = pd.Constr(AGV3_Q.gloClock.name, '<=', 6)
#co3l = pd.Constr(tagvb, '<=', AGV4_Q.locClock.name)
#co3g = pd.Constr(AGV4_Q.gloClock.name, '<=', 6)

# assigns private constraints
qCheck.invs = [co1l, co1g]
AGV3_Q.invs = [co2l, co2g]
#AGV4_Q.invs = [co3l, co3g]

## Mostafa Edits
## Han test2
# Failed States
# work with global constraints for now
# pd.ptaState().addConstr([co1l, co2l, co2g, co3g, co3l])
markedstateList = [storage]
remainingstateList = markedstateList

stateList = [qCheck, AGV3_Q, AGV4_Q, tBuffr, storage]
transList = [tr1, tr2, tr3, tr4, tr5]
A = pd.PTA(stateList, transList, [qCheck], remainingstateList)
solver = ps.ptaSolver(A, 5,'min',0,1)
## Here for some reason, the solution stays at tBuff no matter how wide the horizon is defined.
#t0 = time.time()
solver.solvePTA()
#t1 = time.time() - t0
#print("Time elapsed: ", t1) # CPU seconds elapsed (floating point)