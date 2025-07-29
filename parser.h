/**
 * @file parser.h
 * @brief Handles the parsing of PDDL problem files.
 *
 * This file contains all the necessary functions to read a PDDL problem file,
 * tokenize its content, and populate the initial State struct. It is specifically
 * tailored to the 'Rover' domain and does not parse the domain file itself.
 * It also includes a validation function to ensure the parsed state is consistent.
 */

#ifndef PARSER_H_INCLUDED
#define PARSER_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "auxiliary.h"

/**
 * @brief Removes leading and trailing whitespace from a string.
 * @param str The string to be trimmed in-place.
 */
void trim(char *str) {
    char *start = str;
    char *finish = str + strlen(str) - 1;

    while(isspace(*start)) start++;
    while(finish > start && isspace(*finish)) finish--;

    *(finish + 1) = '\0';
    memmove(str, start, strlen(start) + 1);
}

/**
 * @brief Splits a line of text into tokens.
 *
 * This function handles spaces, tabs, and special PDDL characters like parentheses
 * to break down a line into a list of strings (tokens).
 * @param line The input string line.
 * @param tokens A 2D char array to store the output tokens.
 * @return The number of tokens found.
 */
int tokenize(char *line, char tokens[MAX_TOKENS][MAX_TOKEN_LENGTH]) {
    int amount = 0;
    char *token;
    char line_copy[MAX_LINE];
    strcpy(line_copy, line);

    if (line_copy[0] == '(') {
        strcpy(tokens[amount++], "(");
        memmove(line_copy, line_copy + 1, strlen(line_copy));
    }

    token = strtok(line_copy, " \t\n");
    while(token != NULL && amount < MAX_TOKENS) {
        int len = strlen(token);
        if (len > 0 && token[len-1] == ')') {
            token[len-1] = '\0';
            if (strlen(token) > 0) {
                strcpy(tokens[amount++], token);
            }
            if (amount < MAX_TOKENS) {
                strcpy(tokens[amount++], ")");
            }
        } else {
            strcpy(tokens[amount++], token);
        }
        token = strtok(NULL, " \t\n");
    }

    return amount;
}

/**
 * @brief Converts an object name (e.g., "rover1", "waypoint2") to its integer index.
 *
 * PDDL uses named objects, but the planner's internal state uses integer indices
 * for efficiency. This function extracts the number from the object name.
 * For example, "waypoint3" becomes 2 (assuming 0-based indexing).
 * @param object_name The name of the PDDL object.
 * @return The integer index of the object, or -1 on error.
 */
int get_object_number(const char *object_name) {
    int len = strlen(object_name);
    int i = 0;

    // Search the first number in the string.
    // If a number is found return it, else return -1
    while (i < len && !isdigit(object_name[i])) {
        i++;
    }

    if (i < len) {
        return atoi(object_name + i);
    }

    return -1;
}

/**
 * @brief Converts a camera mode name (e.g., "colour") to its integer index.
 * @param mode_name The string name of the mode.
 * @return The integer index (0, 1, or 2), or -1 on error.
 */
int get_mode_index (const char *mode_name){
    if (strcmp(mode_name, "colour") == 0) return 0;
    if (strcmp(mode_name, "high_res") == 0) return 1;
    if (strcmp(mode_name, "low_res") == 0) return 2;
    return -1;
}

/**
 * @brief Validates the consistency and integrity of a parsed State.
 *
 * Performs numerous checks to ensure that the initial state read from the file
 * is valid according to the rules of the 'Rover' domain (e.g., rover positions are valid,
 * equipment matches capabilities, etc.).
 * @param state A pointer to the State to be validated.
 * @return 1 if the state is valid, 0 otherwise.
 */
int is_valid_state(State *state) {
    if (state == NULL) {
        printf("Error: State is NULL\n");
        return 0;
    }

    // Check that we have valid number of objects
    if (num_rovers <= 0 || num_rovers > MAX_ROVERS) {
        printf("Error: Invalid number of rovers: %d\n", num_rovers);
        return 0;
    }
    if (num_waypoints <= 0 || num_waypoints > MAX_WAYPOINTS) {
        printf("Error: Invalid number of waypoints: %d\n", num_waypoints);
        return 0;
    }
    if (num_cameras < 0 || num_cameras > MAX_CAMERAS) {
        printf("Error: Invalid number of cameras: %d\n", num_cameras);
        return 0;
    }
    if (num_stores < 0 || num_stores > MAX_STORES) {
        printf("Error: Invalid number of stores: %d\n", num_stores);
        return 0;
    }
    if (num_objectives <= 0 || num_objectives > MAX_OBJECTIVES) {
        printf("Error: Invalid number of objectives: %d\n", num_objectives);
        return 0;
    }

    // Check rover positions and attributes
    for (int i = 0; i < num_rovers; i++) {
        // Check if rover position is valid
        if (state->rovers[i].position < 0 || state->rovers[i].position >= num_waypoints) {
            printf("Error: Rover %d has invalid position: %d\n", i, state->rovers[i].position);
            return 0;
        }

        // Check rover energy
        if (state->rovers[i].energy < 0) {
            printf("Error: Rover %d has negative energy: %d\n", i, state->rovers[i].energy);
            return 0;
        }

        // Check traversal matrix
        for (int j = 0; j < num_waypoints; j++) {
            for (int k = 0; k < num_waypoints; k++) {
                if (state->rovers[i].can_traverse[j][k] != 0 && state->rovers[i].can_traverse[j][k] != 1) {
                    printf("Error: Rover %d has invalid traversal value from waypoint %d to %d\n", i, j, k);
                    return 0;
                }

                // Check if waypoint j is visible from k when traversal is possible
                if (state->rovers[i].can_traverse[j][k] && !(state->waypoints[j].visible_waypoints & (1 << k))) {
                    printf("Error: Rover %d can traverse from waypoint %d to %d, but they are not visible to each other\n", i, j, k);
                    return 0;
                }
            }
        }
    }

    // Check waypoints
    for (int i = 0; i < num_waypoints; i++) {
        // Check soil and rock samples are valid (0 or 1)
        if (state->waypoints[i].has_soil_sample != 0 && state->waypoints[i].has_soil_sample != 1) {
            printf("Error: Waypoint %d has invalid soil sample value\n", i);
            return 0;
        }
        if (state->waypoints[i].has_rock_sample != 0 && state->waypoints[i].has_rock_sample != 1) {
            printf("Error: Waypoint %d has invalid rock sample value\n", i);
            return 0;
        }

        // Check sun visibility
        if (state->waypoints[i].in_sun != 0 && state->waypoints[i].in_sun != 1) {
            printf("Error: Waypoint %d has invalid sun visibility\n", i);
            return 0;
        }

        // Check visibility bitmap (at least one waypoint should be visible from each waypoint)
        if (state->waypoints[i].visible_waypoints == 0) {
            printf("Warning: Waypoint %d has no visible waypoints\n", i);
        }
    }

    // Check lander position
    if (state->lander.lander_position < 0 || state->lander.lander_position >= num_waypoints) {
        printf("Error: Lander has invalid position: %d\n", state->lander.lander_position);
        return 0;
    }

    // Check cameras
    for (int i = 0; i < num_cameras; i++) {
        // Check camera rover association
        if (state->cameras[i].rover_id < 0 || state->cameras[i].rover_id >= num_rovers) {
            printf("Error: Camera %d has invalid rover association: %d\n", i, state->cameras[i].rover_id);
            return 0;
        }

        // Check calibration status
        if (state->cameras[i].calibrated != 0 && state->cameras[i].calibrated != 1) {
            printf("Error: Camera %d has invalid calibration status\n", i);
            return 0;
        }

        // Check if at least one objective is a calibration target
        if (state->cameras[i].calibration_targets == 0) {
            printf("Error: Camera %d has no calibration targets\n", i);
            return 0;
        }

        // Check if camera supports at least one mode
        if (state->cameras[i].modes_supported == 0) {
            printf("Error: Camera %d doesn't support any mode\n", i);
            return 0;
        }
    }

    // Check stores
    for (int i = 0; i < num_stores; i++) {
        // Check rover association
        if (state->stores[i].rover_id < 0 || state->stores[i].rover_id >= num_rovers) {
            printf("Error: Store %d has invalid rover association: %d\n", i, state->stores[i].rover_id);
            return 0;
        }

        // Check fullness status
        if (state->stores[i].is_full != 0 && state->stores[i].is_full != 1) {
            printf("Error: Store %d has invalid fullness status\n", i);
            return 0;
        }
    }

    // Check objectives
    for (int i = 0; i < num_objectives; i++) {
        // Check visibility bitmap (at least one waypoint should be visible from each objective)
        if (state->objectives[i].visible_waypoints == 0) {
            printf("Error: Objective %d is not visible from any waypoint\n", i);
            return 0;
        }
    }

    // Validate goal conditions
    int has_goal = 0;
    // Check if there's at least one goal
    for (int i = 0; i < num_waypoints; i++) {
        if (goal.communicated_soil_data[i] == 1 ||
            goal.communicated_rock_data[i] == 1) {
            has_goal = 1;
            break;
        }
    }

    if (!has_goal) {
        for (int i = 0; i < num_objectives; i++) {
            for (int j = 0; j < num_modes; j++) {
                if (goal.communicated_image_data[i][j] == 1) {
                    has_goal = 1;
                    break;
                }
            }
            if (has_goal) break;
        }
    }

    if (!has_goal) {
        printf("Warning: No goal conditions found\n");
    }

    // If all checks , the state is valid
    return 1;
}

/**
 * @brief Reads a PDDL problem file and constructs the initial State.
 *
 * This is the main function of the parser. It reads the file line by line,
 * uses a simple state machine to identify the :objects, :init, and :goal sections,
 * and populates the global `goal` struct and the initial `State` struct accordingly.
 * @param filename The name of the PDDL problem file to parse.
 * @return A pointer to the newly allocated and initialized State, or NULL on error.
 */
State* parse_pddl_file(const char *filename){
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Error opening file: %s\n", filename);
        return NULL;
    }

    State *state = (State*)malloc(sizeof(State));
    memset(state, 0, sizeof(State));
    memset(&goal, 0, sizeof(Goal));

    char line[MAX_LINE];
    char tokens[MAX_TOKENS][MAX_TOKEN_LENGTH];

    // Initialize object counters
    num_rovers = 0;
    num_waypoints = 0;
    num_cameras = 0;
    num_stores = 0;
    num_objectives = 0;
    num_modes = 0;

    // Temporary storage for objects before we know their final counts
    char obj_names[MAX_TOKENS][MAX_TOKEN_LENGTH];
    char obj_types[MAX_TOKENS][MAX_TOKEN_LENGTH];
    int obj_count = 0;

    // Simple state machine flags to track which section of the file we are in
    int in_objects = 0;
    int in_init = 0;
    int in_goal = 0;

    while (fgets(line, MAX_LINE, file)) {
        trim(line);
        if (strlen(line) > 0) { // Skip empty lines
            int token_count = tokenize(line, tokens);

            if (token_count > 0) {
                // --- State Machine: Section Recognition ---
                if(strstr(line, ":objects")) {
                    in_objects = 1;
                    in_init = 0;
                    in_goal = 0;
                }
                else if(strstr(line, ":init")) {
                    in_objects = 0;
                    in_init = 1;
                    in_goal = 0;
                }
                else if(strstr(line, ":goal")) {
                    in_objects = 0;
                    in_init = 0;
                    in_goal = 1;
                }
                else {
                    // --- Section Processing ---
                    if(in_objects) {
                        // Store object names and their types temporarily
                        for(int i = 0; i < token_count; i++) {
                            if(strcmp(tokens[i], "-") == 0 && i > 0) {
                                for(int j = 0; j < i; j++) {
                                    strcpy(obj_names[obj_count], tokens[j]);
                                    strcpy(obj_types[obj_count], tokens[i+1]);
                                    obj_count++;
                                }
                            }
                        }
                    }

                    // Parse predicates and fluents to set the initial state
                    // Example: (at rover0 waypoint1) -> state->rovers[0].position = 1;
                    if(in_init && tokens[0][0] == '(') {
                        if(strcmp(tokens[1], "visible") == 0) {
                            int wp1 = get_object_number(tokens[2]);
                            int wp2 = get_object_number(tokens[3]);
                            if (wp1<0 || wp2<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->waypoints[wp1].visible_waypoints |= (1 << wp2);
                        }
                        else if(strcmp(tokens[1], "at_soil_sample") == 0) {
                            int wp = get_object_number(tokens[2]);
                            if (wp<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->waypoints[wp].has_soil_sample = 1;
                        }
                        else if(strcmp(tokens[1], "at_rock_sample") == 0) {
                            int wp = get_object_number(tokens[2]);
                            if (wp<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->waypoints[wp].has_rock_sample = 1;
                        }
                        else if(strcmp(tokens[1], "in_sun") == 0) {
                            int wp = get_object_number(tokens[2]);
                            if (wp<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->waypoints[wp].in_sun = 1;
                        }
                        else if(strcmp(tokens[1], "at_lander") == 0) {
                            int wp = get_object_number(tokens[3]);
                            if (wp<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->lander.lander_position = wp;
                        }
                        else if(strcmp(tokens[1], "channel_free") == 0) {
                            state->lander.channel_free = 1;
                        }
                        else if(strcmp(tokens[1], "=") == 0 && strcmp(tokens[2], "(recharges") == 0) {
                            state->recharges = atoi(tokens[4]);
                        }
                        else if(strcmp(tokens[1], "=") == 0 && strcmp(tokens[2], "(energy") == 0) {
                            int rover_idx = get_object_number(tokens[3]);
                            if (rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->rovers[rover_idx].energy = atoi(tokens[5]);
                        }
                        else if(strcmp(tokens[1], "in") == 0) {
                            int rover_idx = get_object_number(tokens[2]);
                            int wp = get_object_number(tokens[3]);
                            if (wp<0 || rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                            }
                            state->rovers[rover_idx].position = wp;
                        }
                        else if(strcmp(tokens[1], "available") == 0) {
                           int rover_idx = get_object_number(tokens[2]);
                           if (rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->rovers[rover_idx].available = 1;
                        }
                        else if(strcmp(tokens[1], "can_traverse") == 0) {
                           int rover_idx = get_object_number(tokens[2]);
                           int wp1 = get_object_number(tokens[3]);
                           int wp2 = get_object_number(tokens[4]);
                           if (wp1<0 || wp2<0 || rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->rovers[rover_idx].can_traverse[wp1][wp2] = 1;
                        }
                        else if(strcmp(tokens[1], "equipped_for_soil_analysis") == 0) {
                           int rover_idx = get_object_number(tokens[2]);
                           if (rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->rovers[rover_idx].equipped_soil = 1;
                        }
                        else if(strcmp(tokens[1], "equipped_for_rock_analysis") == 0) {
                           int rover_idx = get_object_number(tokens[2]);
                           if (rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->rovers[rover_idx].equipped_rock = 1;
                        }
                        else if(strcmp(tokens[1], "equipped_for_imaging") == 0) {
                           int rover_idx = get_object_number(tokens[2]);
                           if (rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->rovers[rover_idx].equipped_imaging = 1;
                        }
                        else if(strcmp(tokens[1], "empty") == 0) {
                           int store_idx = get_object_number(tokens[2]);
                           if (store_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->stores[store_idx].is_full = 0;
                        }
                        else if(strcmp(tokens[1], "store_of") == 0) {
                           int store_idx = get_object_number(tokens[2]);
                           int rover_idx = get_object_number(tokens[3]);
                           if (rover_idx<0 || store_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->stores[store_idx].rover_id = rover_idx;
                        }
                        else if(strcmp(tokens[1], "calibration_target") == 0) {
                           int camera_idx = get_object_number(tokens[2]);
                           int objective_idx = get_object_number(tokens[3]);
                           if (camera_idx<0 || objective_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->cameras[camera_idx].calibration_targets |= (1 << objective_idx);
                        }
                        else if(strcmp(tokens[1], "on_board") == 0) {
                           int camera_idx = get_object_number(tokens[2]);
                           int rover_idx = get_object_number(tokens[3]);
                           if (camera_idx<0 || rover_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->cameras[camera_idx].rover_id = rover_idx;
                        }
                        else if(strcmp(tokens[1], "calibrated") == 0) {
                           int camera_idx = get_object_number(tokens[2]);
                           if (camera_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->cameras[camera_idx].calibrated = 1;
                        }
                        else if(strcmp(tokens[1], "supports") == 0) {
                           int camera_idx = get_object_number(tokens[2]);
                           int mode_idx = get_mode_index(tokens[3]);
                           if (camera_idx<0 || mode_idx<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->cameras[camera_idx].modes_supported |= (1 << mode_idx);
                        }
                        else if(strcmp(tokens[1], "visible_from") == 0) {
                           int objective_idx = get_object_number(tokens[2]);
                           int wp = get_object_number(tokens[3]);
                           if (objective_idx<0 || wp<0) {
                                printf("Error reading line %s. Program terminates\n", line);
                                free(state);
                                fclose(file);
                                return NULL;
                           }
                           state->objectives[objective_idx].visible_waypoints |= (1 << wp);
                        }
                        else {
                            printf("Error reading line %s. Program terminates\n", line);
                            free(state);
                            fclose(file);
                            return NULL;
                        }
                    }

                    // Parse goal conditions and populate the global `goal` struct
                    // Example: (communicated_soil_data waypoint5) -> goal.communicated_soil_data[5] = 1;
                    if(in_goal && tokens[0][0] == '(') {
                        if(strcmp(tokens[1], "communicated_soil_data") == 0) {
                            int wp = get_object_number(tokens[2]);
                            if (wp >= 0) {
                                goal.communicated_soil_data[wp] = 1;
                            }
                        }
                        else if(strcmp(tokens[1], "communicated_rock_data") == 0) {
                            int wp = get_object_number(tokens[2]);
                            if (wp >= 0) {
                                goal.communicated_rock_data[wp] = 1;
                            }
                        }
                        else if(strcmp(tokens[1], "communicated_image_data") == 0) {
                            int objective = get_object_number(tokens[2]);
                            int mode = get_mode_index(tokens[3]);
                            if (objective >= 0 && mode >= 0) {
                                goal.communicated_image_data[objective][mode] = 1;
                            }
                        }
                    }
                }
            }
        }
    }

    // Count objects by type
    for(int i = 0; i < obj_count; i++) {
        if(strcmp(obj_types[i], "rover") == 0) {
            num_rovers++;
        }
        else if(strcmp(obj_types[i], "waypoint") == 0) {
            num_waypoints++;
        }
        else if(strcmp(obj_types[i], "camera") == 0) {
            num_cameras++;
        }
        else if(strcmp(obj_types[i], "store") == 0) {
            num_stores++;
        }
        else if(strcmp(obj_types[i], "objective") == 0) {
            num_objectives++;
        }
        else if(strcmp(obj_types[i], "mode") == 0) {
            num_modes++;
        }
    }

    fclose(file);

    // Finally, validate the constructed state
    if (is_valid_state(state)){
        return state;
    }
    else {
        printf("Invalid state contained in file %s. Program terminates.\n",filename);
        return NULL;
    }
}

#endif // PARSER_H_INCLUDED
