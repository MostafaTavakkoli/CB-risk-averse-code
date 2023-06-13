import numpy as np


# Very simple class definitions to create a Priced Timed Automaton


class ptaState:
    globalConsts = []
##
    def __init__(self, name='', comment='', cost=1):
        self.name = name
        self.comment = comment
        self.invs = []
        self.invsC = []
        self.cost = cost
        self.locClock = Clock(str(self.name + '_local'))
        self.gloClock = Clock(str(self.name + '_globl'))

    def addConstr(self, c):
        for _ in c:
            ptaState.globalConsts.append(_)

    def resetInvs(self):
        self.invs = []
        self.invsC = []


class Transition:
    def __init__(self, source, dest, name='', guard=[], sync=[], reset=[]):
        self.name = name
        self.guard = guard
        self.sync = sync
        self.reset = reset
        self.source = source
        self.dest = dest

    def getGuards(self):
        return self.guard

    def getReset(self):
        return self.reset

    def getSource(self):
        return self.source

    def getDest(self):
        return self.dest


class Clock:
    def __init__(self, name='', cost=1, init=0):
        self.name = name
        self.cost = cost
        self.init = init
        self.value = init

    def incr(self):
        self.value = self.value + self.cost

    def getCost(self):
        return self.cost

    def getName(self):
        return self.name

    def reset(self, toValue=0):
        self.value = toValue


class Constr:
    def __init__(self, lhs, constrType, rhs, soft=0, weight=1, guard=0):
        self.constrType = constrType
        self.lhs = lhs
        self.rhs = rhs
        self.soft = soft

        # Check to make sure weight is nonegative
        if (weight >= 0):
            self.soft = soft
            self.weight = weight
        else:
            self.soft = 0
            self.weight = 1

    def getConstr(self):
        # Get the constraint as a pytuple
        if self.soft:
            return (str(self.lhs), str(self.constrType), str(self.rhs), str(self.weight))
        else:
            return (str(self.lhs), str(self.constrType), str(self.rhs))

    def getConstrToAdd(self):
        # Returning different strings for the eval in the solver is unnecessary since
        # the soft constraint can accept the condition and weight separately
        # see Z3_optimize_assert_soft() in 
        # https://z3prover.github.io/api/html/group__capi.html#ga4fec5d5997cd90dcc01cd3cddae73915
        # 
        # if self.weight:
        #    return str(self.lhs) + str(self.constrType) + str(self.rhs) + ','  + str(self.weight)
        # else:
        if self.constrType in {'Or', 'OR', 'or', '||'}:
            if len(self.rhs) > 1 and isinstance(self.rhs, list):
                rhsStr = ','.join([str(x) for x in self.rhs])
            else:
                rhsStr = str(self.rhs) if isinstance(self.rhs, str) else str(self.rhs[0])
            return 'Or(' + str(self.lhs) + ',' + rhsStr + ')'
        elif self.constrType in {'And', 'AND', 'and', '&&'}:
            if len(self.rhs) > 1 and isinstance(self.rhs, list):
                rhsStr = ','.join([str(x) for x in self.rhs])
            else:
                rhsStr = str(self.rhs) if isinstance(self.rhs, str) else str(self.rhs[0])
            return 'And(' + str(self.lhs) + ',' + rhsStr + ')'
        else:
            return str(self.lhs) + str(self.constrType) + str(self.rhs)

    def __str__(self):
        return str(self.getConstr())


class PTA:
    def __init__(self, stateList=[], transList=[], initState=[], markedStates=[], startAtInit = 0):
        self.stateList = stateList
        self.transList = transList
        self.initState = initState
        self.markedStates = markedStates
        self.currState = initState
        self.startAtInit = startAtInit
        if self.startAtInit: 
            self.prunePTA()
        self.out, self.incoming = self.sourceDest()
        self.Ainc = self.createIncMat()
        self.AincBD = self.createIncMatBD()
        self.B = self.createBMat()
        self.P = self.createPMat()
        self.addClockConstr()

    def prunePTA(self):
        if not self.initState:
            print('No initial state is given to start at')
            return
        else:
            states_to_remove = []
            trans_to_remove = []
            for trans in self.transList:
                if trans.dest == self.initState:
                    trans_to_remove.append(trans)
                    states_to_remove.append(trans.source)

            comp_list = [] # check if this is growing
            while comp_list!=states_to_remove:
                comp_list = states_to_remove
                for states in states_to_remove:
                    for trans in self.transList:
                        if trans.dest == states:
                            trans_to_remove.append(trans)
                            states_to_remove.append(trans.source)

            for state in states_to_remove:
                idx = self.stateList.index(state)
                self.stateList.pop(idx)

            for trans in trans_to_remove:
                idx = self.transList.index(trans)
                self.transList.pop(idx)

            return

    def sourceDest(self):
        _sources = []
        _destinations = []

        out = dict()
        incoming = dict()
        for t in self.transList:
            _sources.append([t, t.source])
            _destinations.append([t, t.dest])

        for i in _sources:
            if i[1].name in out:
                out[i[1].name].append(i[0].name)
            else:
                out[i[1].name] = [i[0].name]

        for i in _destinations:
            if i[1].name in incoming:
                incoming[i[1].name].append(i[0].name)
            else:
                incoming[i[1].name] = [i[0].name]

        return out, incoming

    def createIncMat(self):
        Ainc = []
        # Out overwrites the incoming for the directed graph representation
        sDict = {**self.incoming, **self.out}
        for i in self.stateList:
            a = []
            for j in self.transList:
                if j.name in sDict[i.name]:
                    a.append(1)
                else:
                    a.append(0)
            Ainc.append(a)
        return Ainc

    def createIncMatBD(self):
        # Create the incidence Matrix for the bidirectional case
        AincBD = []
        for i in self.transList:
            a = []
            for s in self.stateList:
                if i.source.name == s.name or i.dest.name == s.name:
                    a.append(1)
                else:
                    a.append(0)
            AincBD.append(a)
        return AincBD

    def createBMat(self):
        B = []
        for i in self.stateList:
            b = []
            for j in self.transList:
                if i.name in self.incoming:
                    if j.name in self.incoming[i.name]:
                        b.append(1)
                    else:
                        b.append(0)
                else:
                    b.append(0)
            B.append(b)
        return B

    def createPMat(self):
        P = [p.cost for p in self.stateList]
        return P

    def addClockConstr(self):
        for t in self.transList:
            source = t.source
            dest = t.dest
            c1 = Constr(dest.gloClock.name, '==', str(source.gloClock.name) + '+' + str(dest.locClock.name))
            dest.invsC.append(c1)

        for s in self.stateList:
            c1 = Constr(0, '<=', s.gloClock.name)
            s.invs.append(c1)
            c2 = Constr(0, '<=', s.locClock.name)
            s.invs.append(c2)

    def resetStateInvs(self):
        for states in self.stateList:
            states.resetInvs()
