import src.ptaDefs as pd
import src.ptaSolver as ps
import sys
import os
import networkx as nx
from jsonInterface import *

os.chdir(os.path.dirname(os.path.abspath(__file__)))

def solve(input_name, debug_var):
    # Initialization
    stateList = []
    transList = []
    markedStates = []
    stateHash = {}

    # Use the graph to find the MPC horizon (longest distance)
    nxGraph = nx.DiGraph()

    # parse json file
    with open(input_name, encoding='utf-8') as data_file:
        data = json.loads(data_file.read())

    constraint_information = data["constraintInformation"]
    pta = data["environmentModel"]

    # put all the states with defined constraints from pta.json in the stateList
    counter = 0  # counting all of the states
    stateCosts = pta["stateCost"]

    # Normalize
    costList = list(stateCosts.values())
    scalingFactor = 0.7/max(costList)
    for k in stateCosts:
        # stateCosts[k] = round(stateCosts[k]*scalingFactor,1)
        stateCosts[k] = 1

    violation_map = {}

    for state in pta["states"]:
        offsetValue = [0, pta["globalClockOffset"]]
        ptaState = pd.ptaState(state["name"], cost=stateCosts[state["name"]])

        clockNames = [ptaState.locClock.name, ptaState.gloClock.name]
        clockNames_JSON = [state["localClock"]["clockName"], state["globalClock"]["clockName"]]

        constraintsValues = [state["localClock"]["constraints"], state["globalClock"]["constraints"]]

        pta["currentState"]["name"]
        if state["name"] == pta["currentState"]["name"]:
            offsetValue[0] = pta["localClockOffset"]
            ptaState.invs.append(pd.Constr(clockNames[0], '==', clockNames[1]))


        # Check lower bounds, equalities, and upper bound for the clocks
        for index in [0, 1]:
            clockName = clockNames[index]
            clockName_JSON = clockNames_JSON[index]
            offset = offsetValue[index]
            constraints = constraintsValues[index]

            # cl >= lower bound (indicates the minimum time a product must spend at a state)
            for bound in constraints["lowerBounds"]:
                value_list, weight = get_bound_information(bound, constraint_information)
                constraint = pd.Constr(value_list[0] - offset, '<=', clockName, soft=1, weight=weight)
                ptaState.invs.append(constraint)

                # save the mapping from z3 constraint to bound for printing
                bound["clock"] = clockName_JSON
                bound["type"] = "lowerBound"
                violation_map[constraint.getConstrToAdd()] = [state["name"],bound]

            # cl == upper bound (indicates the time a product can must spend at a state, e.g. due to physical constraints)
            for bound in constraints["equalities"]:
                value_list, weight = get_bound_information(bound, constraint_information)
                constraint = pd.Constr(clockName, '==', value_list[0] - offset, soft=1, weight=weight)
                ptaState.invs.append(constraint)

                # save the mapping from z3 constraint to bound for printing
                bound["clock"] = clockName_JSON
                bound["type"] = "equality"
                violation_map[constraint.getConstrToAdd()] = [state["name"],bound]

            # cl <= upper bound (indicates the minimum time a product can spend at a state)
            for bound in constraints["upperBounds"]:
                value_list, weight = get_bound_information(bound, constraint_information)
                constraint = pd.Constr(clockName, '<=', value_list[0] - offset, soft=1, weight=weight)
                ptaState.invs.append(constraint)

                # save the mapping from z3 constraint to bound for printing
                bound["clock"] = clockName_JSON
                bound["type"] = "upperBound"
                violation_map[constraint.getConstrToAdd()] = [state["name"],bound]

        # Band gap indicates that a product can't be in the state for a specific time
        # Global clock based, but could be done for local clocks, if necessary
        # Note that cg-cl indicated the starting time of being in the state
        for bound in state["globalClock"]["constraints"]["bandGaps"]:
            value_list, weight = get_bound_information(bound, constraint_information)

            lowerGapValue = str(value_list[0] - offsetValue[1])
            upperGapValue = str(value_list[1] - offsetValue[1])
            # # Cannot be in the gap lb<cg-cl or ub>cg-cl
            # between_1 = lowerGapValue + ' > ' + clockNames[1] + ' - ' + clockNames[0]
            # between_2 = upperGapValue + ' < ' + clockNames[1] + ' - ' + clockNames[0]
            # betweenBound = 'Not(And(' + between_1 + ',' + between_2 + '))'  # ,0,1)'
            # # If start before the gap (cg-cl)<=ub -> cg <=lb.
            # # Implies was simplified by negating the first statement and making it an OR
            # lowerBound_1 = clockNames[1] + ' - ' + clockNames[0] + ' > ' + lowerGapValue
            # lowerBound_2 = clockNames[1] + ' <= ' + lowerGapValue
            # lowerBound = 'Or(' + between_2 + ',' + lowerBound_2 + ')'
            lowerBound_1 = clockNames[1] + ' - ' + clockNames[0] + ' > ' + upperGapValue
            lowerBound_2 = clockNames[1] + ' <= ' + lowerGapValue
            lowerBound = 'Or(' + lowerBound_1 + ',' + lowerBound_2 + ')'
            # Add both constraints to indicate that the product can't be in the state
            #constraint1 = pd.Constr(betweenBound, '', '', soft=1, weight=weight)
            constraint2 = pd.Constr(lowerBound, '', '', soft=1, weight=weight)
            #ptaState.invs.append(constraint1)
            ptaState.invs.append(constraint2)

            # save the mapping from z3 constraint to bound for printing
            bound["clock"] = clockNames_JSON[1]
            bound["type"] = "bandGap"
            #violation_map[constraint1.getConstrToAdd()] = [state["name"],bound]
            violation_map[constraint2.getConstrToAdd()] = [state["name"],bound]

        stateList.append(ptaState)
        stateHash[state["name"]] = counter
        counter += 1

    # Add the start state
    startStatePTA = stateList[stateHash[pta["currentState"]["name"]]]
    # Start state constraint should be included cg=cl
    #startStatePTA.invs.append(pd.Constr(startStatePTA.locClock.name, '==', startStatePTA.gloClock.name))


    # Put transitions into the PTA
    for tran in pta["events"]:

        in1 = stateHash[tran["parentName"]]
        in2 = stateHash[tran["childName"]]

        name = tran["name"]
        ptaTransition = pd.Transition(stateList[in1], stateList[in2], name)
        transList.append(ptaTransition)

        # Add transitions to graph (for horizon)
        nxGraph.add_edge(tran["parentName"], tran["childName"])

    # Find the MPC horizon (longest distance)
    nxHorizon = len(nx.dag_longest_path(nxGraph, 1))

    # Add marked states
    for endState in pta["markedStates"]:
        markedStates.append(stateList[stateHash[endState["name"]]])

    # Information for debuging
    if debug_var == 1:
        print("Cycles in PTA:" + str(len(list(nx.simple_cycles(nxGraph)))))
        print("Shortest path:" + str(nx.shortest_path(nxGraph, startStatePTA.name, markedStates[0].name)))
        for state in stateList:
            print(state.name + ":")
            for inv in state.invs:
                printString = "\t" + "".join(inv.getConstr())
                printString = printString.replace(state.name, "")
                printString = printString.replace("_globl", "G")
                printString = printString.replace("_local", "L")
                printString = printString.replace("Or", "Or ")
                print(printString)
                # print(inv)

        # drawing the graph
        # import matplotlib.pyplot as plt  # For visualization
        # nx.draw(nxGraph, with_labels=True)
        # plt.savefig("simple_path.png")  # save as png
        # plt.show()  # display

    # Put everything into solver
    A = pd.PTA(stateList, transList, startStatePTA, markedStates, startAtInit=1)
    solver = ps.ptaSolver(A, nxHorizon + 2, 'min', printReport=debug_var, soft_violation_limit = 2)
    return solver.solvePTA(), pta, violation_map, offsetValue[1]


def main(input_name, result_name, debug_var=0):
    solution, pta, violation_map, global_offset = solve(input_name, debug_var)
    write_solution(solution, pta, result_name, violation_map, global_offset)

    return result_name


if __name__ == "__main__":  
    debug_input_Name = 'tick61(0)_smallOrder1_Part1'
    input_name, result_name, debug_var = read_JSONFile(sys.argv, debug_input_Name)

    # Run everything (with exceptions) if debugging
    if debug_var == 1:
        main(input_name, result_name, debug_var)
    else:
        # Catch exception and write empty solution if not debugging
        try:
            main(input_name, result_name)
        except Exception as e:
            write_empty_solution(result_name)
