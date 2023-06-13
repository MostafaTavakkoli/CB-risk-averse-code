import json


def read_JSONFile(systemInput, debug_input_Name):

    # Get the directories used in the simulation
    try:
        with open("directories.json", encoding='utf-8') as data_file:
            directories = json.loads(data_file.read())

    except IOError:
        print("Directories not accessible")

    z3JSON_Directory = directories["Directory"]["z3_JSON_Directory"]

    # automatically set the debug to 0 if called from repast
    if len(systemInput) > 1:
        debug_var = 0
    else:
        debug_var = 1
        debug_part_name_time = debug_input_Name

    # If not debugging, provide (via command line) the file names where to read the input and write the result
    if debug_var == 0:
        input_name = systemInput[1]
        result_name = systemInput[2]
    else:
        input_name = z3JSON_Directory+debug_part_name_time + '_input.json'
        result_name = z3JSON_Directory + '\\test_' + debug_part_name_time + '_result.json'

    return input_name, result_name, debug_var


def get_bound_information(bound, constraint_information):

    # get the weight for the specific bound
    weight = bound["weight"]

    # only hard constraints: weight is -1
    if constraint_information["hardConstraintOnly"] or weight < 0:
        weight = -1
        #weight = 100

    # if the priority of soft constraints agent is less: weight is 0
    elif len(bound["agent"].split("priority")) == 2 and \
            int(bound["agent"].split("priority")[1]) > constraint_information["priority"]:
        weight = 0

    value_list = bound["value"]
    return value_list, weight


def write_solution(solution, pta, result_name, violation_map, global_offset):

    # Initialize for solution
    solution_formatted = {}
    index = 0
    name = 'notAName'
    flag = False

    # Go through the output and put it into a json format
    for tran_solved in solution["edges"]:
        if flag:
            break
        tran_name_prev = name

        # Obtain the corresponding edge in the PTA
        for tran in pta["events"]:
            # Check all of the solutions in the solved path
            if tran["name"] in tran_solved:
                name = tran["name"]
                # Check the path consistency
                if name == tran_name_prev:
                    break
                    flag = true

                solution_formatted[index] = {}

                # Add the time for transition
                for gLo in solution["globalClocks"]:
                    if tran["parentName"] in gLo:
                        # print_dict[index]["transition"] = tran

                        solution_formatted[index]["transition"] = name
                        solution_formatted[index]["timeOfTransition"] = solution["globalClocks"][gLo]
                        index += 1

    print_dict = {"solution": solution_formatted}

    # Go through all of the violations and add it to the output JSON
    #violation_dictionary = solution["violations"]  # Dictionary that maps states to violations
    #violation_list = {}
    #for state in violation_dictionary:
    #   violation_list[state.name] = violation_map[violation_dictionary[state]]

    # Go through all of the violations and add it to the output JSON
    violation_Solution = solution["violations"]  # Dictionary that maps states to violations
    violation_list = []
    for violation in violation_Solution:
        violation_list.append(violation_map[violation[1]])

    print_dict["constraintViolations"] = violation_list

    print_dict["timeOffset"] = global_offset

    # Write it to a JSON file
    with open(result_name, 'w', encoding='utf-8') as f:
        json.dump(print_dict, f)


def write_empty_solution(result_name):
    print_dict = {"solution": [], "constraintViolations":{}}

    # Write it to a JSON file
    with open(result_name, 'w', encoding='utf-8') as f:
        json.dump(print_dict, f)
