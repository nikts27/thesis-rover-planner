/**
 * @file rover_verify.c
 * @brief A standalone tool to verify the validity of a solution plan.
 *
 * This program takes a PDDL problem file and a solution file as input.
 * It simulates the execution of the plan step-by-step, starting from the
 * initial state described in the problem file. It checks if each action
 * in the plan is applicable and, if the final state reached after executing
 * all actions satisfies the problem's goal conditions.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "auxiliary.h"
#include "parser.h"

/**
 * @brief Displays a syntax message for incorrect command-line arguments.
 */
void syntax_message_verify() {
	printf("Usage:\n\n");
	printf("\trover_verify <problem-file> <solution-file>\n\n");
}

/**
 * @brief Verifies if a solution file contains a valid plan for a given problem.
 *
 * This function performs the core validation logic:
 * 1. Parses the problem file to establish the initial state.
 * 2. Reads the solution file line by line.
 * 3. For each line, it parses the action and its parameters.
 * 4. It calls `apply_action` to simulate the action's execution on the current state.
 * 5. If any action is invalid, the verification fails.
 * 6. After all actions are executed, it calls `is_solution` to check if the goal is met.
 *
 * @param problem_file Path to the PDDL problem file.
 * @param solution_file Path to the solution plan file.
 * @return 0 on successful validation, -1 on failure.
 */
int verify_solution(char* problem_file, char* solution_file) {
    FILE* fp;
    char line[MAX_LINE];
    char tokens[MAX_TOKENS][MAX_TOKEN_LENGTH];
    int err, num_params, result, line_num = 0;
    State next_state; // A temporary state to hold the result of an action

    // 1. Load the initial state from the problem file.
    State* state = parse_pddl_file(problem_file);
    if (state == NULL) {
        printf("Error loading problem file\n");
        return -1;
    }

    // 2. Open the solution file for reading.
    fp = fopen(solution_file, "r");
    if (fp == NULL) {
        printf("Error: Could not open solution file: %s\n", solution_file);
        free(state); // Clean up memory
        return -1;
    }

    // 3. Process each action in the solution file.
    while (fgets(line, sizeof(line), fp)) {
        line_num++;
        trim(line);
        if (strlen(line) > 0) { // Skip empty lines or header info
            int token_count = tokenize(line, tokens);

            if (token_count>0 && strcmp(tokens[0], "(") == 0) {
                // Identify the action and parse its parameters.
                if (strcmp(tokens[1], "navigate") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int wp1 = get_object_number(tokens[3]);
                  int wp2 = get_object_number(tokens[4]);

                  if (rover < 0 || wp1 < 0 || wp2 < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, wp1, wp2};

                  result = apply_action(state, 0, params, &next_state, &err);
                } else if (strcmp(tokens[1], "recharge") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int wp = get_object_number(tokens[3]);

                  if (rover < 0 || wp < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, wp};

                  result = apply_action(state, 1, params, &next_state, &err);
                } else if (strcmp(tokens[1], "sample_soil") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int store = get_object_number(tokens[3]);
                  int wp = get_object_number(tokens[4]);

                  if (rover < 0 || store < 0 || wp < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, store, wp};

                  result = apply_action(state, 2, params, &next_state, &err);
                } else if (strcmp(tokens[1], "sample_rock") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int store = get_object_number(tokens[3]);
                  int wp = get_object_number(tokens[4]);

                  if (rover < 0 || store < 0 || wp < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, store, wp};

                  result = apply_action(state, 3, params, &next_state, &err);
                } else if (strcmp(tokens[1], "drop") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int store = get_object_number(tokens[3]);

                  if (rover < 0 || store < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, store};

                  result = apply_action(state, 4, params, &next_state, &err);
                } else if (strcmp(tokens[1], "calibrate") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int camera = get_object_number(tokens[3]);
                  int objective = get_object_number(tokens[4]);
                  int wp = get_object_number(tokens[5]);

                  if (rover < 0 || camera < 0 || objective < 0 || wp < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, camera, objective, wp};

                  result = apply_action(state, 5, params, &next_state, &err);
                } else if (strcmp(tokens[1], "take_image") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int wp = get_object_number(tokens[3]);
                  int objective = get_object_number(tokens[4]);
                  int camera = get_object_number(tokens[5]);
                  int mode = get_mode_index(tokens[6]);

                  if (rover < 0 || wp < 0 || objective < 0 || camera < 0 || mode < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, wp, objective, camera, mode};

                  result = apply_action(state, 6, params, &next_state, &err);
                } else if (strcmp(tokens[1], "communicate_soil_data") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int wp1 = get_object_number(tokens[3]);
                  int wp2 = get_object_number(tokens[4]);
                  int wp3 = get_object_number(tokens[5]);

                  if (rover < 0 || wp1 < 0 || wp2 < 0 || wp3 < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, wp1, wp2, wp3};

                  result = apply_action(state, 7, params, &next_state, &err);
                } else if (strcmp(tokens[1], "communicate_rock_data") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int wp1 = get_object_number(tokens[3]);
                  int wp2 = get_object_number(tokens[4]);
                  int wp3 = get_object_number(tokens[5]);

                  if (rover < 0 || wp1 < 0 || wp2 < 0 || wp3 < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, wp1, wp2, wp3};

                  result = apply_action(state, 8, params, &next_state, &err);
                } else if (strcmp(tokens[1], "communicate_image_data") == 0) {
                  int rover = get_object_number(tokens[2]);
                  int objective = get_object_number(tokens[3]);
                  int mode = get_mode_index(tokens[4]);
                  int wp1 = get_object_number(tokens[5]);
                  int wp2 = get_object_number(tokens[6]);

                  if (rover < 0 || objective < 0 || mode < 0 || wp1 < 0 || wp2 < 0) {
                     printf("Error parsing action at line %d: %s\n", line_num, line);
                     fclose(fp);
                     return -1;
                  }

                  int params[] = {rover, objective, mode, wp1, wp2};

                  result = apply_action(state, 9, params, &next_state, &err);
                } else {
                  printf("Unknown action '%s' at line %d\n", tokens[1], line_num);
                  fclose(fp);
                  free(state);
                  return -1;
                }
                // 4. Check if the action was successfully applied.
                if (result == 1) {
                  // If successful, update the main state.
                  memcpy(state, &next_state, sizeof(State));
                } else {
                  // If not, the plan is invalid.
                  printf("Error: Action at line %d is not applicable.\n -> %s\n", line_num, line);
                  fclose(fp);
                  free(state);
                  return -1;
                }
            }
        }
    }

    // 5. After all actions, check if the final state is a goal state.
    if (!is_solution(*state)) {
        printf("Error: Plan executed successfully, but the final state is not a goal state.\n");
        fclose(fp);
        free(state);
        return -1;
    }

    // Success
    fclose(fp);
    free(state);

    // Print success message with statistics
    printf("Solution is valid!\n");
    printf("Total actions: %d\n", line_num-2);
    printf("Total recharges: %d\n", state->recharges);

    return 0;
}


/**
 * @brief Main entry point for the verifier.
 */
int main(int argc, char* argv[]) {
    // Check for the correct number of command-line arguments.
    if (argc != 3) {
        syntax_message_verify();
        return -1;
    }

    // Call the main verification function.
    if (verify_solution(argv[1], argv[2]) == 0) {
        return 0; // Success
    }
    return -1; // Failure
}
