/**
 * @file planner.c
 * @brief Main file for the domain-dependent planner.
 *
 * This file contains the core implementation of the search algorithm (A* and Best-First Search),
 * the duplicate detection mechanism using a Hash Table, the node expansion logic,
 * and the main program flow management.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// --- Include custom header files ---
#include "parser.h"       // Logic for parsing the PDDL problem file.
#include "auxiliary.h"    // Auxiliary data structures (State, Rover, Waypoint, etc.).
#include "minheap.h"      // Min-Heap implementation for the frontier.
#include "heuristic.h"    // Heuristic function implementations.
#include "solution.h"     // Functions for extracting and writing the solution.
#include "uthash.h"       // External library for Hash Table management.
#include "bloom.h"        // Library for Bloom Filter management.

// --- Constants for algorithm selection ---
#define best	1   // Represents the Best-First Search algorithm.
#define astar	2   // Represents the A* algorithm.

#define TIMEOUT	 600	// Maximum execution time in seconds.

/**
 * @struct StateKey
 * @brief A compact, "flat" representation of a State.
 *
 * This structure is used as the key for the Hash Table. It converts the complex
 * data from the main State struct into a combination of simple integers and bitmaps,
 * enabling efficient comparison and storage of visited states.
 */
typedef struct {
    int rover_positions[MAX_ROVERS];
    int energy_levels[MAX_ROVERS];
    int has_soil_analysis;          // Bitmap
    int has_rock_analysis;          // Bitmap
    unsigned int have_image_bm[MAX_ROVERS]; // Bitmap for images
    int has_soil_sample;            // Bitmap
    int has_rock_sample;            // Bitmap
    int communicated_soil_sample;   // Bitmap
    int communicated_rock_sample;   // Bitmap
    int cameras_calibrated;         // Bitmap
    int full_stores;                // Bitmap
    int communicated_image;         // Bitmap
    int recharges;
} StateKey;

/**
 * @struct state_entry
 * @brief Entry structure for the Hash Table (using uthash).
 *
 * Contains the state key (StateKey) and the necessary handle (hh)
 * required by the uthash library to manage hash entries.
 */
typedef struct  {
    StateKey key;
    UT_hash_handle hh; // Handle used by uthash
} state_entry;

// --- Global Variables ---
state_entry *state_set = NULL; // The Hash Table storing the closed set of states.
BloomFilter *bf;               // Pointer to the Bloom Filter (optional mechanism).
MinHeap *frontier;             // The search frontier (open set), implemented as a Min-Heap.
time_t t1;                     // Search start time for timeout checking.
clock_t c1, c2;                // Variables for measuring CPU time.

/**
 * @brief Creates a compact StateKey from a full State struct.
 * @param s The full State to be converted.
 * @param key The resulting StateKey.
 */
void make_state_key(State *s, StateKey *key) {
    memset(key, 0, sizeof(StateKey));

    // Convert dynamic information into bitmaps for a compact representation.
    for (int r = 0; r < num_rovers; r++) {
        key->rover_positions[r] = s->rovers[r].position;
        key->energy_levels[r] = s->rovers[r].energy;
        if (s->rovers[r].has_soil_analysis) key->has_soil_analysis |= (1 << r);
        if (s->rovers[r].has_rock_analysis) key->has_rock_analysis |= (1 << r);
        for (int o = 0; o < num_objectives; o++) {
            for (int m = 0; m < num_modes; m++) {
                if (s->rovers[r].have_image[o][m]) {
                    key->have_image_bm[r] |= (1U << (o * num_modes + m));
                }
            }
        }
    }

    for (int w = 0; w < num_waypoints; w++) {
        if (s->waypoints[w].has_soil_sample) key->has_soil_sample |= (1 << w);
        if (s->waypoints[w].has_rock_sample) key->has_rock_sample |= (1 << w);
        if (s->waypoints[w].communicated_soil) key->communicated_soil_sample |= (1 << w);
        if (s->waypoints[w].communicated_rock) key->communicated_rock_sample |= (1 << w);
    }

    for (int c = 0; c < num_cameras; c++) {
        if (s->cameras[c].calibrated) key->cameras_calibrated |= (1 << c);
    }

    for (int st = 0; st < num_stores; st++) {
        if (s->stores[st].is_full) key->full_stores |= (1 << st);
    }

    for (int o = 0; o < num_objectives; o++) {
        if (s->objectives[o].communicated_image) key->communicated_image |= (1 << o);
    }

    key->recharges = s->recharges;
}

/**
 * @brief Adds a new state key to the Hash Table (closed set).
 * @param key The key to be added.
 */
void add_to_state_set(StateKey *key) {
    state_entry *entry = malloc(sizeof(state_entry));
    entry->key = *key;
    HASH_ADD(hh, state_set, key, sizeof(StateKey), entry);
}

/**
 * @brief Checks if a state key already exists in the Hash Table.
 * @param key The key to check.
 * @return 1 if the state exists, 0 otherwise.
 */
int state_exists(StateKey *key) {
    state_entry *entry;
    HASH_FIND(hh, state_set, key, sizeof(StateKey), entry);
    return (entry != NULL);
}

/**
 * @brief Initializes the Bloom Filter.
 */
void initialize_bloom() {
    bf = bloom_create(2000000, 0.01);
    if (!bf) {
        printf("Failed to initialize bloom filter\n");
        exit(1);
    }
}

/**
 * @brief Checks a new node for duplicate states to detect loops.
 *
 * This function creates a key for the node's state and checks if it exists
 * in the closed set (Hash Table). We can use additionaly a bloom filter check, for more security.
 * If not, it adds it.
 * @param node The search tree node to check.
 * @return 1 if the state is new (no loop), 0 if a loop is detected.
 */
int check_with_parents(struct tree_node *node) {
    StateKey sk;
    make_state_key(&node->currState, &sk);

    // Using the key directly now
    //if (bloom_check(bf, &sk, sizeof(StateKey))) {
        if (state_exists(&sk)) {
            return 0; // Loop detected
        }
    //}

    //bloom_add(bf, &sk, sizeof(StateKey));
    add_to_state_set(&sk);
    return 1; // No loop
}

/**
 * @brief Checks if the search has exceeded the predefined timeout.
 */
void check_timeout() {
    if (difftime(time(NULL), t1) > TIMEOUT) {
        printf("Timeout reached. Aborting...\n");
        printf("Heap stats: inserts=%d, extracts=%d\n", total_inserts, total_extracts);
        exit(1);
    }
}

/**
 * @brief Displays a syntax message for incorrect command-line arguments.
 */
void syntax_message() {
	printf("planner <method> <input-file> <output-file>\n\n");
	printf("where: ");
	printf("<method> = best|astar\n");
	printf("<input-file> is a file containing a PDDL problem description.\n");
	printf("<output-file> is the file where the solution will be written.\n");
}

/**
 * @brief Parses the search method from command-line arguments.
 * @param s The string argument representing the method.
 * @return The integer constant for the method, or -1 if invalid.
 */
int get_method(char* s) {
    if (strcmp(s,"best")==0) return best;
    if (strcmp(s,"astar")==0) return astar;
    return -1;
}

/**
 * @brief Adds a new search tree node to the frontier (Min-Heap).
 * @param node The node to be added.
 * @return 0 on success.
 */
int add_frontier_in_order(struct tree_node *node) {
    insert_node(frontier, node->f, node);
    return 0;
}

/**
 * @brief Generates the PDDL parameter name for a given action parameter index.
 * @param child The child node being created.
 * @param param The integer value of the parameter (e.g., waypoint ID).
 * @param index The position of the parameter in the action's signature.
 */
void get_param_name(struct tree_node *child, int param, int index) {
    switch(child->action_taken.action_type){
        case 0:
        case 1:
        {
            sprintf(child->action_taken.param_names[index], "waypoint%d", param);
            break;
        }
        case 2:
        case 3:
        {
            switch(index) {
               case 1:
               {
                    sprintf(child->action_taken.param_names[index], "store%d", param);
                    break;
               }
               default:
               {
                   sprintf(child->action_taken.param_names[index], "waypoint%d", param);
                   break;
               }

            }
            break;
        }
        case 4:
        {
            sprintf(child->action_taken.param_names[index], "store%d", param);
            break;
        }
        case 5:
        {
            switch(index) {
               case 1:
               {
                    sprintf(child->action_taken.param_names[index], "camera%d", param);
                    break;
               }
               case 2:
               {
                   sprintf(child->action_taken.param_names[index], "objective%d", param);
                   break;
               }
               default:
               {
                   sprintf(child->action_taken.param_names[index], "waypoint%d", param);
                   break;
               }
            }
            break;
        }
        case 6:
        {
            switch(index) {
               case 1:
               {
                    sprintf(child->action_taken.param_names[index], "waypoint%d", param);
                    break;
               }
               case 2:
               {
                   sprintf(child->action_taken.param_names[index], "objective%d", param);
                   break;
               }
               case 3:
               {
                   sprintf(child->action_taken.param_names[index], "camera%d", param);
                   break;
               }
               default:
               {
                   switch (param){
                       case 0:
                       {
                           sprintf(child->action_taken.param_names[index], "colour");
                           break;
                       }
                       case 1:
                       {
                           sprintf(child->action_taken.param_names[index], "high_res");
                           break;
                       }
                       default:
                       {
                           sprintf(child->action_taken.param_names[index], "low_res");
                           break;
                       }
                   }
                   break;
               }
            }
            break;
        }
        case 7:
        case 8:
        {
            sprintf(child->action_taken.param_names[index], "waypoint%d", param);
            break;
        }
        default:
        {
            switch(index) {
               case 1:
               {
                   sprintf(child->action_taken.param_names[index], "objective%d", param);
                   break;
               }
               case 2:
               {
                   switch (param){
                       case 0:
                       {
                           sprintf(child->action_taken.param_names[index], "colour");
                           break;
                       }
                       case 1:
                       {
                           sprintf(child->action_taken.param_names[index], "high_res");
                           break;
                       }
                       default:
                       {
                           sprintf(child->action_taken.param_names[index], "low_res");
                           break;
                       }
                   }
                   break;
               }
               default:
               {
                   sprintf(child->action_taken.param_names[index], "waypoint%d", param);
                   break;
               }
            }
            break;
        }

    }
}
/**
 * @brief Adds a new child node to the search tree.
 *
 * This function sets the child's properties (parent, depth, g-cost),
 * checks for loops, calculates its heuristic and f-values, and adds it to the frontier.
 * @return 0 on success, -1 on memory error.
 */
int add_child(struct tree_node *current_node, int action_type, struct tree_node *child, int method, int *params, int param_count, int energy_spent){
    int err = 0;

    child->parent = current_node;
    child->depth = current_node->depth + 1;
    child->g = current_node->g + energy_spent;
    child->action_taken.action_type = action_type;
    child->action_taken.num_params = param_count;
    sprintf(child->action_taken.param_names[0], "rover%d", params[0]);
    for (int i=1; i<param_count; i++){
        get_param_name(child, params[i], i);
    }
    switch (action_type) {
       case 7:
       case 8:
       case 9:
       {
            sprintf(child->action_taken.param_names[5], "general");
            child->action_taken.num_params++;
            break;
       }
    }

    if(!check_with_parents(child)){
        free(child);
        child = NULL;
    }
    else {
        child->h=heuristic(child->currState);
        if (method==best) {
            child->f = child->h;
        }
        else {
            child->f = child->h + child->g;
        }

        err = add_frontier_in_order(child);
    }

    return err;
}

// Helper function to safely create and try to add a child node
int try_add_child(struct tree_node *parent_node, int action_type, int *params, int param_count, int method) {
    step_count++;
    if (step_count % 1000 == 0) {
        check_timeout();
    }

    struct tree_node *child = malloc(sizeof(struct tree_node));
    if (child == NULL) return -1;

    int energy_spent;

    if (apply_action(&parent_node->currState, action_type, params, &child->currState, &energy_spent)) {
        int err = add_child(parent_node, action_type, child, method, params, param_count, energy_spent);
        if (err < 0) return -1;
    } else {
        free(child);
    }

    return 0;
}

// Helper function to try actions that require 2 parameters
int try_two_param_action(struct tree_node *node, int rover, int param2, int action_type, int method) {
    int params[2] = {rover, param2};
    return try_add_child(node, action_type, params, 2, method);
}

// Helper function to try actions that require 3 parameters
int try_three_param_action(struct tree_node *node, int rover, int param2, int param3, int action_type, int method) {
    int params[3] = {rover, param2, param3};
    return try_add_child(node, action_type, params, 3, method);
}

// Helper function to try actions that require 4 parameters
int try_four_param_action(struct tree_node *node, int rover, int param2, int param3, int param4,
                         int action_type, int method) {
    int params[4] = {rover, param2, param3, param4};
    return try_add_child(node, action_type, params, 4, method);
}

// Helper function to try actions that require 5 parameters
int try_five_param_action(struct tree_node *node, int rover, int param2, int param3, int param4,
                         int param5, int action_type, int method) {
    int params[5] = {rover, param2, param3, param4, param5};
    return try_add_child(node, action_type, params, 5, method);
}
/**
 * @brief Expands a node by generating all its possible successor states (children).
 *
 * This is the core function for generating the search tree. It iterates through all
 * possible actions for all rovers and creates new child nodes for each valid action.
 * Extensive pruning is used to avoid generating obviously invalid or unhelpful states.
 * @param current_node The node to expand.
 * @param method The search algorithm being used (astar or best).
 * @return 1 on success, -1 on memory error.
 */
int find_children(struct tree_node *current_node, int method) {
    State *s = &current_node->currState;
    int rover, store, cam, wp, wp2, obj, mode, pos;
    int lander_pos = s->lander.lander_position;

    for (rover = 0; rover < num_rovers; rover++) {
        if (!s->rovers[rover].available) {
            continue;
        }

        pos = s->rovers[rover].position;

        // RECHARGE (1)
        if (s->waypoints[pos].in_sun && s->rovers[rover].energy < 8) {
            if (try_two_param_action(current_node, rover, pos, 1, method) < 0) return -1;
        }

        // SAMPLE_SOIL (2)
        if (s->rovers[rover].equipped_soil && s->rovers[rover].energy >= 3 &&
            goal.communicated_soil_data[pos] && !s->waypoints[pos].communicated_soil &&
            s->waypoints[pos].has_soil_sample) {
            for (store = 0; store < num_stores; store++) {
                if (s->stores[store].rover_id == rover && !s->stores[store].is_full) {
                    if (try_three_param_action(current_node, rover, store, pos, 2, method) < 0) return -1;
                }
            }
        }

        // SAMPLE_ROCK (3)
        if (s->rovers[rover].equipped_rock && s->rovers[rover].energy >= 5 &&
            goal.communicated_rock_data[pos] && !s->waypoints[pos].communicated_rock &&
            s->waypoints[pos].has_rock_sample) {
            for (store = 0; store < num_stores; store++) {
                // ΚΛΑΔΕΜΑ: Το store πρέπει να ανήκει στο rover και να είναι άδειο.
                if (s->stores[store].rover_id == rover && !s->stores[store].is_full) {
                    if (try_three_param_action(current_node, rover, store, pos, 3, method) < 0) return -1;
                }
            }
        }

        if (s->rovers[rover].equipped_imaging) {
            for (cam = 0; cam < num_cameras; cam++) {
                if (s->cameras[cam].rover_id != rover) continue;

                for (obj = 0; obj < num_objectives; obj++) {
                    // CALIBRATE (5)
                    if (s->rovers[rover].energy >= 2 &&
                        (s->objectives[obj].visible_waypoints & (1 << pos)) &&
                        (s->cameras[cam].calibration_targets & (1 << obj))) {
                        if (try_four_param_action(current_node, rover, cam, obj, pos, 5, method) < 0) return -1;
                    }

                    // TAKE_IMAGE (6)
                    for (mode = 0; mode < num_modes; mode++) {
                        if (s->cameras[cam].calibrated &&
                            s->rovers[rover].energy >= 1 &&
                            (s->cameras[cam].modes_supported & (1 << mode)) &&
                            (s->objectives[obj].visible_waypoints & (1 << pos)) &&
                            goal.communicated_image_data[obj][mode] &&
                            !(s->objectives[obj].communicated_image & (1 << mode))) {
                            if (try_five_param_action(current_node, rover, pos, obj, cam, mode, 6, method) < 0) return -1;
                        }
                    }
                }
            }
        }

        if (s->lander.channel_free && (s->waypoints[pos].visible_waypoints & (1 << lander_pos))) {
            // COMMUNICATE_SOIL_DATA (7)
            if (s->rovers[rover].energy >= 4) {
                for (wp = 0; wp < num_waypoints; wp++) {
                    if (goal.communicated_soil_data[wp] &&
                        !s->waypoints[wp].communicated_soil &&
                        (s->rovers[rover].has_soil_analysis & (1 << wp))) {
                        if (try_four_param_action(current_node, rover, wp, pos, lander_pos, 7, method) < 0) return -1;
                    }
                }
            }

            // COMMUNICATE_ROCK_DATA (8)
            if (s->rovers[rover].energy >= 4) {
                for (wp = 0; wp < num_waypoints; wp++) {
                    if (goal.communicated_rock_data[wp] &&
                        !s->waypoints[wp].communicated_rock &&
                        (s->rovers[rover].has_rock_analysis & (1 << wp))) {
                        if (try_four_param_action(current_node, rover, wp, pos, lander_pos, 8, method) < 0) return -1;
                    }
                }
            }

            // COMMUNICATE_IMAGE_DATA (9)
            if (s->rovers[rover].energy >= 6) {
                for (obj = 0; obj < num_objectives; obj++) {
                    for (mode = 0; mode < num_modes; mode++) {
                        if (goal.communicated_image_data[obj][mode] &&
                            !(s->objectives[obj].communicated_image & (1 << mode)) &&
                            s->rovers[rover].have_image[obj][mode]) {
                            if (try_five_param_action(current_node, rover, obj, mode, pos, lander_pos, 9, method) < 0) return -1;
                        }
                    }
                }
            }
        }

        // DROP (4)
        for (store = 0; store < num_stores; store++) {
            if (s->stores[store].rover_id == rover && s->stores[store].is_full) {
                if (try_two_param_action(current_node, rover, store, 4, method) < 0) return -1;
            }
        }

        // NAVIGATE (0)
        for (wp2 = 0; wp2 < num_waypoints; wp2++) {
            if (pos != wp2 &&
                s->rovers[rover].energy >= 8 &&
                (s->waypoints[pos].visible_waypoints & (1 << wp2)) &&
                s->rovers[rover].can_traverse[pos][wp2]) {
                if (try_three_param_action(current_node, rover, pos, wp2, 0, method) < 0) return -1;
            }
        }
    }

    return 1; // Process completed!
}


/**
 * @brief Initializes the search process.
 *
 * Creates the root node of the search tree from the initial state,
 * initializes the frontier (Min-Heap), and adds the root node to it.
 * Also precomputes shortest paths.
 * @param initState The initial state of the problem.
 * @param method The search algorithm to be used.
 */
void initialize_search(State initState, int method)
{
	struct tree_node *root=NULL;	// the root of the search tree.

	precompute_shortest_paths(&initState);

	//initialize_bloom();

	//Initialize frontier
	frontier = createMinHeap(1000);

	// Initialize search tree
	root=(struct tree_node*) malloc(sizeof(struct tree_node));
	root->parent=NULL;
	root->action_taken.action_type=-1;
    root->currState=initState;
    root->depth = 0;

	root->g=0;
	root->h=heuristic(root->currState);
	if (method==best)
		root->f=root->h;
	else
		root->f=root->g+root->h;

	// Add the initial root to the frontier
	add_frontier_in_order(root);
}

/**
 * @brief The main search loop.
 *
 * Implements the Search algorithms framework. It repeatedly extracts
 * the most promising node from the frontier, checks if it's a solution,
 * and expands it to generate its children.
 * @param method The search algorithm (astar or best).
 * @return A pointer to the solution node, or NULL if no solution is found.
 */
struct tree_node *search(int method) {
	struct tree_node *current_node;

	while (!is_empty_heap(frontier))
	{
		// Extract the best node from the frontier
		HeapNode minNode = extract_min(frontier);
		total_extracts++;
        current_node = (struct tree_node*) minNode.node;

		if (is_solution(current_node->currState)){
            printf("Heap stats: inserts=%d, extracts=%d\n", total_inserts, total_extracts);
		    free(frontier->nodeArray);
            free(frontier);
            return current_node;
		}

		// Expand the current node to find its children
		int err=find_children(current_node, method);

		if (err<0){
            printf("Memory exhausted while creating new frontier node. Search is terminated...\n");
            return NULL;
        }
	}

	return NULL;
}

/**
 * @brief Main entry point of the program.
 *
 * Handles command-line argument parsing, calls the PDDL parser,
 * initiates the search, and prints the final solution and statistics.
 */
int main(int argc, char** argv) {
    if (argc!=4) {
		printf("Wrong number of arguments. Use correct syntax:\n");
		syntax_message();
		return -1;
	}

	int method=get_method(argv[1]); // The search algorithm that will be used to solve the problem.
	if (method<0) {
		printf("Wrong method. Use correct syntax:\n");
		syntax_message();
		return -1;
	}

	// Parse the PDDL problem file to get the initial state
	State *initial_state = parse_pddl_file(argv[2]);
	if (initial_state == NULL) {
        return -1;
	}

	printf("Solving %s using %s...\n",argv[2],argv[1]);
	c1 = clock();
	t1 = time(NULL);

	// Set up the initial data structures for the search
	initialize_search(*initial_state, method);

	// Start the main search loop
	struct tree_node *solution_node = search(method);

	c2 = clock();

	// Clean up memory
	//bloom_free(bf);
	HASH_CLEAR(hh, state_set);

	// If a solution was found, reconstruct and print the plan
	if (solution_node!=NULL)
		extract_solution(solution_node);
	else
		printf("No solution found.\n");

    if (solution_node!=NULL) {
		printf("Solution found! (%d steps) (Total recharges: %d)\n",solution_length,total_recharges);
		printf("(Total energy spent: %d)\n", total_energy);
		printf("Time spent: %f secs\n",((float) c2-c1)/CLOCKS_PER_SEC);
		write_solution_to_file(argv[3]);
	}

    return 0;
}
