/**
 * @file solution.h
 * @brief Handles the extraction and formatting of the final solution plan.
 *
 * This file contains the functions responsible for taking a solution node
 * (a leaf node in the search tree that satisfies the goal conditions) and
 * reconstructing the sequence of actions that leads from the initial state
 to that solution. It also handles writing the formatted plan to an output file.
 */

#ifndef SOLUTION_H
#define SOLUTION_H

#include "auxiliary.h"

/**
 * @brief Reconstructs the solution plan by backtracking from the solution node.
 *
 * Given a leaf node of the search tree that represents a solution, this function
 * traverses upwards towards the root node using the parent pointers. At each step,
 * it records the action taken to reach the current node. The final sequence of
 * actions is stored in reverse order and then saved into the global `solution` array.
 *
 * @param solution_node A pointer to the leaf node of the search tree that is a solution.
 */
void extract_solution(struct tree_node *solution_node) {
    int i;

    // A temporary node to traverse the tree upwards without losing the original pointer.
	struct tree_node *temp_node = solution_node;

    // The length of the solution is the depth of the solution node.
	solution_length = solution_node->depth;

    // Store final statistics from the solution state.
	total_recharges = solution_node->currState.recharges;
	total_energy = solution_node->g; // The g-cost of the solution node is the total energy spent.

    // Allocate memory for the global solution array.
	solution = (Action*) malloc(solution_length * sizeof(Action));
	if (solution == NULL) {
        printf("Memory allocation for solution failed!\n");
        return;
    }

	temp_node = solution_node;
	i = solution_length;

    // Backtrack from the solution node to the root.
	while (temp_node->parent != NULL)
	{
		i--; // Decrement index before storing to fill the array from the end to the beginning.
		solution[i] = temp_node->action_taken;

        // Optionally store the h and f values of each step for debugging/analysis.
		solution[i].h = temp_node->h;
		solution[i].f = temp_node->f;

		temp_node = temp_node->parent; // Move up to the parent node.
	}
}

/**
 * @brief Writes the extracted solution plan to a specified file.
 *
 * This function opens the output file and iterates through the global `solution` array,
 * printing each action and its parameters in a PDDL-like format. It also includes
 * summary statistics at the beginning of the file.
 *
 * @param filename The name of the file where the solution will be written.
 */
void write_solution_to_file(char* filename){
    int i;
	FILE *fout;

	fout = fopen(filename, "w");
	if (fout == NULL) {
		printf("Cannot open output file to write solution.\n");
		return;
	}

    // Write summary statistics at the top of the file.
	fprintf(fout, "Solution length: %d\n", solution_length);
	fprintf(fout, "Total recharges uses: %d\n", total_recharges);

    // Iterate through the solution plan and print each action.
	for (i = 0; i < solution_length; i++){
        // Print the action name based on its integer type ID.
        switch(solution[i].action_type) {
            case 0: fprintf(fout, "( navigate "); break;
            case 1: fprintf(fout, "( recharge "); break;
            case 2: fprintf(fout, "( sample_soil "); break;
            case 3: fprintf(fout, "( sample_rock "); break;
            case 4: fprintf(fout, "( drop "); break;
            case 5: fprintf(fout, "( calibrate "); break;
            case 6: fprintf(fout, "( take_image "); break;
            case 7: fprintf(fout, "( communicate_soil_data "); break;
            case 8: fprintf(fout, "( communicate_rock_data "); break;
            default: fprintf(fout, "( communicate_image_data "); break;
        }

        // Print the string parameters for the action.
        for (int j = 0; j < solution[i].num_params; j++){
            fprintf(fout, "%s ", solution[i].param_names[j]);
        }

        // Close the parenthesis and add h/f values for analysis.
        fprintf(fout, ") h=%d, f=%d\n", solution[i].h, solution[i].f);
	}

	fclose(fout);
}

#endif // SOLUTION_H
