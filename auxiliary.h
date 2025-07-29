/**
 * @file auxiliary.h
 * @brief Defines all core data structures, constants, and global variables for the planner.
 *
 * This file serves as the central repository for data representation. It defines
 * structs for every object in the 'Rover' domain (Rover, Waypoint, Camera, etc.),
 * the main State struct that encapsulates the entire world state, the search tree node
 * struct, and global variables used for tracking the solution and problem dimensions.
 */

#ifndef AUXILIARY_H_INCLUDED
#define AUXILIARY_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// --- General Purpose Constants ---
#define MAX_LINE 1024
#define MAX_TOKENS 100
#define MAX_TOKEN_LENGTH 256

// --- Domain-Specific Constants ---
// These define the maximum number of objects of each type the planner can handle.
#define MAX_ROVERS 10
#define MAX_WAYPOINTS 30
#define MAX_SAMPLES 20
#define MAX_CAMERAS 10
#define MAX_OBJECTIVES 10
#define MAX_STORES 10
#define MAX_MODES 3

// --- PDDL Object Structures ---

/**
 * @struct Rover
 * @brief Represents a single rover agent.
 */
typedef struct {
    int position;          // Current waypoint ID.
    int energy;            // Current energy level.
	int available;         // Rover availability flag.
    int has_soil_analysis; // Bitmap: i-th bit is 1 if rover has analysis for waypoint i.
    int has_rock_analysis; // Bitmap: i-th bit is 1 if rover has analysis for waypoint i.
    int equipped_soil;     // Flag: can this rover analyze soil?
    int equipped_rock;     // Flag: can this rover analyze rock?
    int equipped_imaging;  // Flag: can this rover take images?
    int can_traverse[MAX_WAYPOINTS][MAX_WAYPOINTS]; // Adjacency matrix for traversable paths.
    int have_image[MAX_OBJECTIVES][MAX_MODES];      // Matrix: 1 if rover has image of objective in mode.
} Rover;

/**
 * @struct Waypoint
 * @brief Represents a location on the map.
 */
typedef struct {
    int has_soil_sample;    // Flag: is a soil sample currently at this waypoint?
    int has_rock_sample;    // Flag: is a rock sample currently at this waypoint?
    int communicated_soil;  // Flag: has soil data from this waypoint been communicated?
	int communicated_rock;  // Flag: has rock data from this waypoint been communicated?
    int in_sun;             // Flag: can a rover recharge here?
	int visible_waypoints;  // Bitmap: i-th bit is 1 if waypoint i is visible from here.
} Waypoint;

/**
 * @struct Camera
 * @brief Represents a camera instrument.
 */
typedef struct {
    int calibrated;          // Flag: is the camera currently calibrated?
    int rover_id;            // ID of the rover this camera is on.
	int calibration_targets; // Bitmap: i-th bit is 1 if objective i is a valid calibration target.
	int modes_supported;     // Bitmap: i-th bit is 1 if mode i is supported.
} Camera;

/**
 * @struct Store
 * @brief Represents a rover's storage unit for samples.
 */
typedef struct {
    int is_full;    // Flag: is the store currently full?
    int rover_id;   // ID of the rover that owns this store.
} Store;

/**
 * @struct Objective
 * @brief Represents an imaging target.
 */
typedef struct {
	int communicated_image; // Bitmap: i-th bit is 1 if an image in mode i has been communicated.
    int visible_waypoints;  // Bitmap: i-th bit is 1 if this objective is visible from waypoint i.
} Objective;

/**
 * @struct Lander
 * @brief Represents the main lander.
 */
typedef struct {
    int lander_position; // Waypoint ID where the lander is located.
	int channel_free;    // Flag: is the communication channel to the lander free?
} Lander;

// --- Goal and State Structures ---

/**
 * @struct Goal
 * @brief Represents the goal conditions of the problem.
 * Stores which data items need to be communicated.
 */
typedef struct {
    int communicated_soil_data[MAX_WAYPOINTS];
    int communicated_rock_data[MAX_WAYPOINTS];
    int communicated_image_data[MAX_OBJECTIVES][MAX_MODES];
} Goal;

/**
 * @struct Action
 * @brief Represents a single action in a solution plan.
 */
typedef struct {
    int action_type;    // Integer ID representing the action (e.g., 0 for navigate).
    int num_params;     // Number of parameters for this action.
    char param_names[6][MAX_TOKEN_LENGTH]; // String names of the parameters.
    int h;              // Heuristic value of the state after this action.
    int f;              // F-value of the state after this action.
} Action;

/**
 * @struct State
 * @brief Encapsulates the entire state of the world at a given time.
 * This is the primary data structure passed around during the search.
 */
typedef struct {
    Rover rovers[MAX_ROVERS];
    Waypoint waypoints[MAX_WAYPOINTS];
    Camera cameras[MAX_CAMERAS];
    Store stores[MAX_STORES];
	Objective objectives[MAX_OBJECTIVES];
	Lander lander;
	int recharges; // Counter for the number of recharge actions taken.
} State;

/**
 * @struct tree_node
 * @brief Represents a single node in the search tree.
 */
struct tree_node
{
    State currState;            // The world state this node represents.
    int depth;                  // The depth of the node in the tree (g-cost in terms of steps).
    int h;				        // The heuristic value (estimated cost to goal).
    int g;				        // The actual cost from the root to this node (energy spent).
    int f;				        // The evaluation function value (f = g + h for A*, f = h for Best-First).
    struct tree_node *parent;	// Pointer to the parent node (NULL for the root).
    Action action_taken;        // The action that led from the parent to this node.
};

// --- Global Variables ---

Goal goal; // Stores the goal conditions parsed from the problem file.

int solution_length;	// The length of the final solution plan.
int total_recharges;    // The total number of recharges in the final plan.
int total_energy;       // The total energy cost of the final plan.
Action *solution;		// A dynamic array to store the sequence of actions in the solution.

// Statistics for performance tracking.
int total_inserts = 0, total_extracts = 0;
int step_count = 0;

// Counts of the different object types in the current problem.
int num_rovers;
int num_waypoints;
int num_cameras;
int num_stores;
int num_objectives;
int num_modes;

/**
 * @brief Applies an action to a state to generate a new state.
 *
 * Checks if all preconditions for the given action are met in the current state.
 * If they are, it applies the action's effects to produce the next state.
 * @param current The current state.
 * @param action_type The integer ID of the action to apply.
 * @param params An array of integer parameters for the action.
 * @param next A pointer to the State struct where the new state will be stored.
 * @param energy_spent A pointer to an integer to store the energy cost of the action.
 * @return 1 if the action was applied successfully, 0 otherwise.
 */
int apply_action(State *current, int action_type, int *params, State *next, int *energy_spent) {
	// copy current state
	memcpy(next, current, sizeof(State));

	*energy_spent = 0;

//	printf("Applying action %d to rover%d - \n", action_type, params[0]);

	switch (action_type) {
		case 0: // navigate
		{
            int rover = params[0];
			int from = params[1];
			int to = params[2];

			if (!current->rovers[rover].available) return 0;
			if (current->rovers[rover].energy < 8) return 0;
			if (!(current->waypoints[from].visible_waypoints & (1 << to))) return 0;
			if (!current->rovers[rover].can_traverse[from][to]) return 0;
			if (current->rovers[rover].position != from) return 0;
			if (from == to) return 0;

			next->rovers[rover].position = to;
			next->rovers[rover].energy -= 8;
			*energy_spent = 8;

			break;
		}
		case 1: // recharge
		{
			int rover = params[0];
			int waypoint = params[1];

			if (!current->waypoints[waypoint].in_sun) return 0;
			if (current->rovers[rover].position != waypoint) return 0;
            if (current->rovers[rover].energy >= 8) return 0;

			next->rovers[rover].energy += 20;
			next->recharges += 1;

			break;
		}
		case 2: // sample_soil
		{
			int rover = params[0];
			int store = params[1];
			int waypoint = params[2];

			if (current->rovers[rover].position != waypoint) return 0;
			if (current->rovers[rover].energy < 3) return 0;
			if (!current->waypoints[waypoint].has_soil_sample) return 0;
			if (!current->rovers[rover].equipped_soil) return 0;
			if (current->stores[store].rover_id != rover) return 0;
			if (current->stores[store].is_full) return 0;
			if (!goal.communicated_soil_data[waypoint]) return 0;
			if (current->waypoints[waypoint].communicated_soil) return 0;

			next->stores[store].is_full = 1;
			next->rovers[rover].energy -= 3;
			*energy_spent = 3;
			next->rovers[rover].has_soil_analysis |= (1 << waypoint); // Set bit for this waypoint
			next->waypoints[waypoint].has_soil_sample = 0;

			break;
		}
		case 3: // sample_rock
		{
			int rover = params[0];
			int store = params[1];
			int waypoint = params[2];

			if (current->rovers[rover].position != waypoint) return 0;
			if (current->rovers[rover].energy < 5) return 0;
			if (!current->waypoints[waypoint].has_rock_sample) return 0;
			if (!current->rovers[rover].equipped_rock) return 0;
			if (current->stores[store].rover_id != rover) return 0;
			if (current->stores[store].is_full) return 0;
			if (!goal.communicated_rock_data[waypoint]) return 0;
			if (current->waypoints[waypoint].communicated_rock) return 0;

			next->stores[store].is_full = 1;
			next->rovers[rover].energy -= 5;
			*energy_spent = 5;
			next->rovers[rover].has_rock_analysis |= (1 << waypoint); // Set bit for this waypoint
			next->waypoints[waypoint].has_rock_sample = 0;

			break;
		}
		case 4: // drop
		{
			int rover = params[0];
			int store = params[1];

			if (current->stores[store].rover_id != rover) return 0;
			if (!current->stores[store].is_full) return 0;

			next->stores[store].is_full = 0;

			break;
		}
		case 5: // calibrate
		{
			int rover = params[0];
			int camera = params[1];
			int objective = params[2];
			int waypoint = params[3];

			if (!current->rovers[rover].equipped_imaging) return 0;
			if (current->rovers[rover].energy < 2) return 0;
			if (!(current->cameras[camera].calibration_targets & (1 << objective))) return 0;
			if (current->rovers[rover].position != waypoint) return 0;
			if (!(current->objectives[objective].visible_waypoints & (1 << waypoint))) return 0;
			if (current->cameras[camera].rover_id != rover) return 0;

			next->rovers[rover].energy -= 2;
			next->cameras[camera].calibrated = 1;

			*energy_spent = 2;

			break;
		}
		case 6: // take_image
		{
			int rover = params[0];
			int waypoint = params[1];
			int objective = params[2];
			int camera = params[3];
			int mode = params[4];

			if (!current->cameras[camera].calibrated) return 0;
			if (current->cameras[camera].rover_id != rover) return 0;
			if (!current->rovers[rover].equipped_imaging) return 0;
			if (!(current->cameras[camera].modes_supported & (1 << mode))) return 0;
			if (!(current->objectives[objective].visible_waypoints & (1 << waypoint))) return 0;
			if (current->rovers[rover].position != waypoint) return 0;
			if (current->rovers[rover].energy < 1) return 0;
			if (!goal.communicated_image_data[objective][mode]) return 0;
			if (current->objectives[objective].communicated_image & (1 << mode)) return 0;

			next->rovers[rover].have_image[objective][mode] = 1;
			next->cameras[camera].calibrated = 0;
			next->rovers[rover].energy -= 1;

			*energy_spent = 1;

			break;
		}
		case 7: // communicate soil data
		{
			int rover = params[0];
			int sample_waypoint = params[1];
            int rover_waypoint = params[2];
			int lander_waypoint = params[3];

			if (current->rovers[rover].position != rover_waypoint ) return 0;
			if (current->lander.lander_position != lander_waypoint) return 0;
			if (!(current->rovers[rover].has_soil_analysis & (1 << sample_waypoint))) return 0;
			if (!(current->waypoints[rover_waypoint].visible_waypoints & (1 << lander_waypoint))) return 0;
			if (!current->rovers[rover].available) return 0;
			if (!current->lander.channel_free) return 0;
			if (current->rovers[rover].energy < 4) return 0;
			if (!goal.communicated_soil_data[sample_waypoint]) return 0;
			if (current->waypoints[sample_waypoint].communicated_soil) return 0;

			next->waypoints[sample_waypoint].communicated_soil = 1;
			next->rovers[rover].energy -= 4;

			*energy_spent = 4;

			break;
		}
		case 8: // communicate rock data
		{
			int rover = params[0];
			int sample_waypoint = params[1];
			int rover_waypoint = params[2];
			int lander_waypoint = params[3];

			if (current->rovers[rover].position != rover_waypoint ) return 0;
			if (current->lander.lander_position != lander_waypoint) return 0;
			if (!(current->rovers[rover].has_rock_analysis & (1 << sample_waypoint))) return 0;
			if (!(current->waypoints[rover_waypoint].visible_waypoints & (1 << lander_waypoint))) return 0;
			if (!current->rovers[rover].available) return 0;
			if (!current->lander.channel_free) return 0;
			if (current->rovers[rover].energy < 4) return 0;
			if (!goal.communicated_rock_data[sample_waypoint]) return 0;
			if (current->waypoints[sample_waypoint].communicated_rock) return 0;

			next->waypoints[sample_waypoint].communicated_rock = 1;
			next->rovers[rover].energy -= 4;

			*energy_spent = 4;

			break;
		}
		default: // last possible action: communicate image data
		{
			int rover = params[0];
			int objective  = params[1];
			int mode = params[2];
			int rover_waypoint = params[3];
			int lander_waypoint = params[4];

			if (current->rovers[rover].position != rover_waypoint ) return 0;
			if (current->lander.lander_position != lander_waypoint) return 0;
			if (!current->rovers[rover].have_image[objective][mode]) return 0;
			if (!(current->waypoints[rover_waypoint].visible_waypoints & (1 << lander_waypoint))) return 0;
			if (!current->rovers[rover].available) return 0;
			if (!current->lander.channel_free) return 0;
			if (current->rovers[rover].energy < 6) return 0;
			if (!goal.communicated_image_data[objective][mode]) return 0;
			if (current->objectives[objective].communicated_image & (1 << mode)) return 0;

			next->objectives[objective].communicated_image |= (1 << mode);
			next->rovers[rover].energy -= 6;

			*energy_spent = 6;

			break;
		}
	}
//	printf("SUCCESS\n");
	return 1;
}

/**
 * @brief Checks if a given state satisfies all goal conditions.
 * @param nodeState The state to check.
 * @return 1 if it is a goal state, 0 otherwise.
 */
int is_solution(State nodeState) {
    // Check communicated soil data
    for (int wp = 0; wp < num_waypoints; wp++) {
        if (goal.communicated_soil_data[wp] && !nodeState.waypoints[wp].communicated_soil) {
            return 0;
        }
    }

    // Check communicated rock data
    for (int wp = 0; wp < num_waypoints; wp++) {
        if (goal.communicated_rock_data[wp] && !nodeState.waypoints[wp].communicated_rock) {
            return 0;
        }
    }

    // Check communicated image data
    for (int o = 0; o < num_objectives; o++) {
        for (int m = 0; m < num_modes; m++) {
            if (goal.communicated_image_data[o][m] && !(nodeState.objectives[o].communicated_image & (1 << m))) {
                return 0;
            }
        }
    }

    return 1;
}

/**
 * @brief Prints a detailed representation of a state to the console for debugging.
 * @param state The state to print.
 */
void print_state(State *state) {
    printf("----- Current State -----\n");

    // Rovers
    printf("Rovers (%d):\n", num_rovers);
    for (int i = 0; i < num_rovers; i++) {
        printf("  Rover %d -> Position: %d, Energy: %d, Available: %d\n",
               i, state->rovers[i].position, state->rovers[i].energy, state->rovers[i].available);
        printf("    Equipped for Soil: %d, Rock: %d, Imaging: %d\n",
               state->rovers[i].equipped_soil, state->rovers[i].equipped_rock, state->rovers[i].equipped_imaging);
        printf("    Has Soil Analysis: %d, Has Rock Analysis: %d\n",
               state->rovers[i].has_soil_analysis, state->rovers[i].has_rock_analysis);
        printf("    Can Traverse:\n");
        for (int j = 0; j < num_waypoints; j++) {
            for (int k = 0; k < num_waypoints; k++) {
                if (state->rovers[i].can_traverse[j][k]) {
                    printf("      [%d -> %d]\n", j, k);
                }
            }
        }
    }

    // Waypoints
    printf("Waypoints (%d):\n", num_waypoints);
    for (int i = 0; i < num_waypoints; i++) {
        printf("  Waypoint %d -> Soil Sample: %d, Rock Sample: %d, Communicated Soil: %d, Communicated Rock: %d\n",
               i, state->waypoints[i].has_soil_sample, state->waypoints[i].has_rock_sample,
               state->waypoints[i].communicated_soil, state->waypoints[i].communicated_rock);
        printf("    In Sun: %d, Visible Waypoints Bitmap: %d\n",
               state->waypoints[i].in_sun, state->waypoints[i].visible_waypoints);
    }

    // Cameras
    printf("Cameras (%d):\n", num_cameras);
    for (int i = 0; i < num_cameras; i++) {
        printf("  Camera %d -> Rover: %d, Calibrated: %d, Calibration Targets Bitmap: %d\n, Modes supported Bitmap: %d\n",
               i, state->cameras[i].rover_id, state->cameras[i].calibrated, state->cameras[i].calibration_targets,
               state->cameras[i].modes_supported);
    }

    // Stores
    printf("Stores (%d):\n", num_stores);
    for (int i = 0; i < num_stores; i++) {
        printf("  Store %d -> Rover: %d, Full: %d\n",
               i, state->stores[i].rover_id, state->stores[i].is_full);
    }

    // Objectives
    printf("Objectives (%d)\n", num_objectives);
    printf("Modes (%d)\n", num_modes);
    for (int i = 0; i < num_objectives; i++) {
        printf("  Objective %d -> Communicated Image Bitmap: %d, Visible Waypoints Bitmap: %d\n",
               i, state->objectives[i].communicated_image, state->objectives[i].visible_waypoints);
    }

    // Lander
    printf("Lander -> Position: %d, Channel Free: %d\n",
           state->lander.lander_position, state->lander.channel_free);

    // Have Image Matrix
    printf("Have Image Matrix:\n");
    for (int r = 0; r < num_rovers; r++) {
        for (int o = 0; o < num_objectives; o++) {
            for (int m = 0; m < num_modes; m++) {
                if (state->rovers[r].have_image[o][m]) {
                    printf("  Rover %d has image of Objective %d in Mode %d\n", r, o, m);
                }
            }
        }
    }

    // Recharges Used
    printf("Recharges Used: %d\n", state->recharges);

    // Goal Conditions
    printf("Goal Conditions:\n");
    for (int i = 0; i < num_waypoints; i++) {
        if (goal.communicated_soil_data[i] || goal.communicated_rock_data[i]) {
            printf("  Waypoint %d -> Communicated Soil: %d, Rock: %d\n",
                   i, goal.communicated_soil_data[i], goal.communicated_rock_data[i]);
        }
    }
    for (int i = 0; i < num_objectives; i++) {
        for (int j = 0; j < num_modes; j++) {
            if (goal.communicated_image_data[i][j]) {
                printf("  Objective %d -> Communicated Image in Mode %d\n", i, j);
            }
        }
    }

    printf("-------------------------\n");

}

/**
 * @brief Prints the final solution plan to the console.
 */
void print_solution() {
    if (solution == NULL || solution_length == 0) {
        printf("No solution found.\n");
        return;
    }

    printf("\n=== Solution Found! (%d steps) ===\n", solution_length);
    for (int i = 0; i < solution_length; i++) {
        switch(solution[i].action_type) {
            case 0:  printf("( navigate "); break;
            case 1:  printf("( recharge "); break;
            case 2:  printf("( sample_soil "); break;
            case 3:  printf("( sample_rock "); break;
            case 4:  printf("( drop "); break;
            case 5:  printf("( calibrate "); break;
            case 6:  printf("( take_image "); break;
            case 7:  printf("( communicate_soil_data "); break;
            case 8:  printf("( communicate_rock_data "); break;
            default: printf("( communicate_image "); break;
        }

        for (int j = 0; j < solution[i].num_params; j++) {
            printf("%s ", solution[i].param_names[j]);
        }
        printf(")\n");
    }
    printf("==================================\n");
}

#endif // AUXILIARY_H_INCLUDED
