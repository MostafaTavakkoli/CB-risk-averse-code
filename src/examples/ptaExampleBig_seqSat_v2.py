import time
import src.ptaDefs as pd
import src.ptaSolver as ps

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
CNC1   = pd.ptaState('CNC1')
CNC1_B2 = pd.ptaState('CNC1_B2')
CNC2_B1 = pd.ptaState('CNC2_B1')
CNC2   = pd.ptaState('CNC2')
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
gg = pd.Constr('AGV1_E','||',['tBuffr','asd','efd'])
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


# Constraints
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
co2l = pd.Constr(tagvb, '<=', AGV3_Q.locClock.name)
#co2l = pd.Constr(tagvb, '<=', AGV3_Q.locClock.name,1,3)
# co2g = pd.Constr(AGV3_Q.gloClock.name, '<=', 6)
co3l = pd.Constr(tagvb, '<=', AGV4_Q.locClock.name)
#co3l = pd.Constr(tagvb, '<=', AGV4_Q.locClock.name,1,2)
co3l2 = pd.Constr(0, '<', AGV3_Q.locClock.name)
co3l3 = pd.Constr(0, '<', AGV4_Q.locClock.name)
# co3g = pd.Constr(AGV4_Q.gloClock.name, '<=', 6)


# assigns private constraints
eBuffr.invs = [c1]
AGV1_E.invs = [c2]
CNC1.invs = [c3]
AGV2_1.invs = [c4,c5]
CNC2.invs = [c6]
AGV2_2.invs = [c7,c8]
qCheck.invs = [co1l,co1g]
AGV3_Q.invs = [co2l]
AGV4_Q.invs = [co3l]
# AGV3_Q.invs = [co3l2]
# AGV4_Q.invs = [co3l3]

######
# Input a list of accepting states for the pta
# We will find the corresponding discrete transitions and create the necessary constraints
# Note that we have implemented a single label right now, so all the marked states have 
# the same label
markedStates = [CNC1,CNC2]

stateList = [eBuffr, AGV1_E, CNC1_B1, CNC1, CNC1_B2, AGV2_1, CNC2_B1, CNC2, CNC2_B2, AGV2_2, qCheck, AGV3_Q, AGV4_Q, tBuffr]
transList = [tr0, tr1, tr2, tr3, tr4, tr5, tr6, tr7, tr8, tr9, tr10, tr11, tr12, tr13, tr14, tr15, tr16]
A = pd.PTA(stateList,transList,eBuffr,markedStates)
solver = ps.ptaSolver(A, 8,'min')
# t0 = time.clock()
solver.solvePTA()
# t1 = time.clock() - t0
# print("Time elapsed: ", t1 - t0) # CPU seconds elapsed (floating point)