/**
 * @file heuristic.h
 * @brief Implements all heuristic functions for the planner.
 *
 * This file contains the logic for estimating the cost from a given state to the goal.
 * It includes the main heuristic function called by the planner, which implements the
 * most advanced heuristic (H4 - Optimal Assignment). It also contains crucial helper
 * functions used by all heuristics, such as the precomputation of shortest paths
 * (Floyd-Warshall) and the calculation of relaxed costs for individual goals.
 */

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include <stdio.h>
#include <math.h>
#include "auxiliary.h"

// A large integer value to represent infinity, used in shortest path calculations.
#define INT_MAX 100000
// A simple macro to find the maximum of two numbers.
#define max(a, b) ((a) > (b) ? (a) : (b))

/**
 * @var dist
 * @brief Global 3D array to store precomputed shortest path distances.
 *
 * `dist[rover][from_waypoint][to_waypoint]` stores the minimum energy cost
 * for a specific rover to travel between two waypoints. This is precomputed
 * once at the start of the search using the Floyd-Warshall algorithm.
 */
int dist[MAX_ROVERS][MAX_WAYPOINTS][MAX_WAYPOINTS];

/**
 * @struct GoalCost
 * @brief A helper struct to store the relaxed cost of achieving a single goal.
 *
 * This is used to evaluate every possible way a single goal can be achieved by any rover
 * and store the best option.
 */
typedef struct {
    int cost;       // The estimated minimum energy cost to achieve this goal.
    int rover_id;   // The ID of the rover that can achieve this goal with the minimum cost.
} GoalCost;

/**
 * @brief Precomputes all-pairs shortest paths using the Floyd-Warshall algorithm.
 *
 * This function is called once at the beginning of the search. It populates the global
 * `dist` matrix with the minimum travel cost between any two waypoints for each rover,
 * considering their specific traversal capabilities.
 * @param nodeState The initial state, used to get rover traversal permissions.
 */
void precompute_shortest_paths(State *nodeState) {
    for (int rover = 0; rover < num_rovers; rover++) {
        for (int i = 0; i < num_waypoints; i++) {
            for (int j = 0; j < num_waypoints; j++) {
                if (i == j) dist[rover][i][j] = 0;
                else if (nodeState->rovers[rover].can_traverse[i][j] && (nodeState->waypoints[i].visible_waypoints & (1 << j))) dist[rover][i][j] = 8;
                else dist[rover][i][j] = INT_MAX;
            }
        }
        for (int k = 0; k < num_waypoints; k++) {
            for (int i = 0; i < num_waypoints; i++) {
                for (int j = 0; j < num_waypoints; j++) {
                    if (dist[rover][i][k] != INT_MAX && dist[rover][k][j] != INT_MAX) {
                        int through_k = dist[rover][i][k] + dist[rover][k][j];
                        if (through_k < dist[rover][i][j]) dist[rover][i][j] = through_k;
                    }
                }
            }
        }
    }
}

/**
 * @brief Finds the nearest waypoint to a given point that has line-of-sight to the lander.
 *
 * This is a crucial helper function for calculating communication costs.
 * @param rover The rover for which to calculate paths.
 * @param from_wp The starting waypoint.
 * @param state The current state.
 * @return The ID of the nearest communication-enabled waypoint.
 */
int find_nearest_comm_point(int rover, int from_wp, State *state) {
    int lander_pos = state->lander.lander_position;
    if (state->waypoints[from_wp].visible_waypoints & (1 << lander_pos)) return from_wp;
    int min_dist = INT_MAX;
    int best_wp = -1;
    for (int wp = 0; wp < num_waypoints; wp++) {
        if (!(state->waypoints[wp].visible_waypoints & (1 << lander_pos))) continue;
        int d = dist[rover][from_wp][wp];
        if (d < min_dist) {
            min_dist = d;
            best_wp = wp;
        }
    }
    return best_wp;
}


/**
 * @brief Calculates the minimum relaxed cost for every unfulfilled goal.
 *
 * This is the core cost estimation function. It iterates through all goals
 * (soil, rock, image) and, for each one, calculates the minimum possible cost
 * for any rover to achieve it, ignoring resource contention. These costs are
 * the building blocks for all heuristics.
 * @param state The current state to evaluate.
 * @param costs An output array to be filled with GoalCost structs.
 * @param count An output parameter storing the number of unfulfilled goals found.
 */void calculate_all_goal_costs(State *state, GoalCost costs[], int *count) {
    *count = 0;


    // --- Soil Goals ---
    // Calculates travel + sample + travel_to_comm + communicate costs for each rover
    for (int wp = 0; wp < num_waypoints; wp++) {
        if (!goal.communicated_soil_data[wp] || state->waypoints[wp].communicated_soil) continue;
        for (int r = 0; r < num_rovers; r++) {
            int current_rover_cost = INT_MAX;
            if (state->rovers[r].has_soil_analysis & (1 << wp)) {
                int comm_point = find_nearest_comm_point(r, state->rovers[r].position, state);
                if (comm_point != -1) current_rover_cost = dist[r][state->rovers[r].position][comm_point] + 4;
            } else if (state->rovers[r].equipped_soil && state->waypoints[wp].has_soil_sample) {
                int travel_to_sample = dist[r][state->rovers[r].position][wp];
                if (travel_to_sample != INT_MAX) {
                    int comm_point = find_nearest_comm_point(r, wp, state);
                    if (comm_point != -1) current_rover_cost = travel_to_sample + 3 + dist[r][wp][comm_point] + 4;
                }
            }
            if (current_rover_cost != INT_MAX) {
                costs[*count] = (GoalCost){current_rover_cost, r};
                (*count)++;
            }
        }
    }


    // --- Rock Goals ---
    // Similar calculation for rock goals
    for (int wp = 0; wp < num_waypoints; wp++) {
        if (!goal.communicated_rock_data[wp] || state->waypoints[wp].communicated_rock) continue;
        for (int r = 0; r < num_rovers; r++) {
            int current_rover_cost = INT_MAX;
            if (state->rovers[r].has_rock_analysis & (1 << wp)) {
                 int comm_point = find_nearest_comm_point(r, state->rovers[r].position, state);
                 if (comm_point != -1) current_rover_cost = dist[r][state->rovers[r].position][comm_point] + 4;
            } else if (state->rovers[r].equipped_rock && state->waypoints[wp].has_rock_sample) {
                int travel_to_sample = dist[r][state->rovers[r].position][wp];
                if (travel_to_sample != INT_MAX) {
                    int comm_point = find_nearest_comm_point(r, wp, state);
                    if (comm_point != -1) current_rover_cost = travel_to_sample + 5 + dist[r][wp][comm_point] + 4;
                }
            }
            if (current_rover_cost != INT_MAX) {
                costs[*count] = (GoalCost){current_rover_cost, r};
                (*count)++;
            }
        }
    }


    // --- Image Goals ---
    // Similar calculation for image goals, including calibration cost
    for (int obj = 0; obj < num_objectives; obj++) {
        for (int mode = 0; mode < num_modes; mode++) {
            if (!goal.communicated_image_data[obj][mode] || (state->objectives[obj].communicated_image & (1 << mode))) continue;
            for (int r = 0; r < num_rovers; r++) {
                int current_rover_cost = INT_MAX;
                if (state->rovers[r].have_image[obj][mode]) {
                    int comm_point = find_nearest_comm_point(r, state->rovers[r].position, state);
                    if (comm_point != -1) current_rover_cost = dist[r][state->rovers[r].position][comm_point] + 6;
                } else if (state->rovers[r].equipped_imaging) {
                    int has_camera = 0;
                    for (int c = 0; c < num_cameras; c++) if (state->cameras[c].rover_id == r && (state->cameras[c].modes_supported & (1 << mode))) { has_camera = 1; break; }
                    if (!has_camera) continue;


                    int best_shoot_cost = INT_MAX;
                    for (int shoot_wp = 0; shoot_wp < num_waypoints; shoot_wp++) {
                        if (!(state->objectives[obj].visible_waypoints & (1 << shoot_wp))) continue;
                        int travel_cost = dist[r][state->rovers[r].position][shoot_wp];
                        if (travel_cost == INT_MAX) continue;
                        int comm_point = find_nearest_comm_point(r, shoot_wp, state);
                        if (comm_point != -1) {
                            int total = travel_cost + 2 + 1 + dist[r][shoot_wp][comm_point] + 6;
                            if (total < best_shoot_cost) best_shoot_cost = total;
                        }
                    }
                    if(best_shoot_cost < current_rover_cost) current_rover_cost = best_shoot_cost;
                }
                 if (current_rover_cost != INT_MAX) {
                    costs[*count] = (GoalCost){current_rover_cost, r};
                    (*count)++;
                }
            }
        }
    }
}


/**
 * @brief An admissible heuristic for the additional energy cost due to recharges.
 *
 * Given a set of tasks assigned to rovers, this function calculates a lower bound
 * on the extra energy needed for recharging. It's admissible because it only considers
 * the cost to travel to the nearest recharge station, not the full cycle.
 * @param state The current state.
 * @param assigned_costs An array where assigned_costs[r] is the energy cost of the task assigned to rover r.
 * @return The estimated additional energy cost for recharges.
 */int calculate_energy_cost_for_assignment(State *state, int assigned_costs[MAX_ROVERS]) {
    int total_recharge_cost = 0;
    for (int r = 0; r < num_rovers; r++) {
        if (assigned_costs[r] == 0) continue; // No task assigned to this rover


        int work_cost = assigned_costs[r];
        int available_energy = state->rovers[r].energy;


        if (work_cost > available_energy) {

            // Find cost to travel to the nearest recharge station
            int min_recharge_dist = INT_MAX;
            for (int wp = 0; wp < num_waypoints; wp++) {
                if (state->waypoints[wp].in_sun) {
                    int d = dist[r][state->rovers[r].position][wp];
                    if (d < min_recharge_dist) min_recharge_dist = d;
                }
            }

            if (min_recharge_dist != INT_MAX) {
                total_recharge_cost += min_recharge_dist;
            } else {
                return INT_MAX; // Cannot solve energy deficit
            }
        }
    }
    return total_recharge_cost;
}




/**
 * @brief Comparison function for qsort.
 *
 * Used to sort the `GoalCost` array in descending order of cost, so that the
 * most expensive goals can be considered first.
 */int compareGoalCosts(const void *a, const void *b) {
    GoalCost *costA = (GoalCost *)a;
    GoalCost *costB = (GoalCost *)b;
    return (costB->cost - costA->cost);
}


/**
 * @brief The main heuristic function (implements H4 - Optimal Assignment).
 *
 * This is the function called by the search algorithm to get the h-value of a state.
 * It works as follows:
 * 1. Calculate the relaxed cost for every possible rover-goal pairing.
 * 2. Sort these potential tasks in descending order of cost.
 * 3. Greedily assign the most expensive, non-conflicting tasks to each rover.
 * (i.e., each rover can only be assigned one task).
 * 4. Sum the costs of these assigned tasks.
 * 5. Add an admissible estimate for any necessary recharging costs.
 * The result is a highly informed, admissible heuristic value.
 * @param nodeState The state for which to calculate the heuristic value.
 * @return The estimated cost to reach the goal.
 */int heuristic(State nodeState) {
    if (is_solution(nodeState)) return 0;


    // Array to hold all possible goal-rover pairings
    GoalCost all_costs[ (MAX_WAYPOINTS * 2 + MAX_OBJECTIVES * MAX_MODES) * MAX_ROVERS ];
    int goal_count = 0;

    // 1. Calculate all individual goal costs
    calculate_all_goal_costs(&nodeState, all_costs, &goal_count);


    if (goal_count == 0) return 0;

    // 2. Sort tasks by cost, descending
    qsort(all_costs, goal_count, sizeof(GoalCost), compareGoalCosts);


    int h_tasks = 0;
    int rover_used[MAX_ROVERS] = {0};
    int assigned_costs[MAX_ROVERS] = {0}; // Store cost of task assigned to each rover


    // 3. Greedily assign the most expensive tasks to available rovers
    for (int i = 0; i < goal_count; i++) {
        int rover_id = all_costs[i].rover_id;
        if (rover_id != -1 && !rover_used[rover_id]) {
            h_tasks += all_costs[i].cost;
            assigned_costs[rover_id] = all_costs[i].cost; // Store the cost for energy calculation
            rover_used[rover_id] = 1;
        }
    }


    // 4. Add the admissible energy cost for the assignment
    int h_energy = calculate_energy_cost_for_assignment(&nodeState, assigned_costs);


    if (h_energy == INT_MAX) return INT_MAX;


    int final_h = h_tasks + h_energy;

    // Ensure the heuristic value is non-negative and does not overflow
    return (final_h < 0) ? 0 : ((final_h > INT_MAX) ? INT_MAX : final_h);
}


#endif // HEURISTIC_H
