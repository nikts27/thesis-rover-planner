Domain-Dependent Planner for the 'Rover' Problem (IPC 2023)
===========================================================

This repository contains the complete source code for the undergraduate thesis titled: **"Comparative Study of Domain-Dependent and Domain-Independent Planners for the 'Rover' problem (IPC 2023)"**, submitted to the Department of Applied Informatics at the University of Macedonia.

📝 Project Description
----------------------

This project features a fully functional, domain-dependent planner written entirely in C. It is specifically designed to solve the numeric planning problem **'Rover'**, one of the official benchmarks from the International Planning Competition (IPC) 2023.

The primary goal of this research was to empirically investigate the performance trade-off between a specialized, hand-crafted planner and state-of-the-art, general-purpose (domain-independent) planners. The results demonstrated that while general planners excel in satisficing planning, this specialized approach achieved superior performance in optimal planning, solving more problems than any competitor in the IPC.

✨ Core Methodology
------------------

The planner's intelligence is derived from a series of five custom-designed, evolutionary heuristic functions (H0 to H4). These heuristics encode progressively deeper domain knowledge about the 'Rover' problem, particularly concerning rover parallelism and energy management, allowing the search algorithm to be guided with high precision.

The core search algorithm is a flexible Best-First Search engine that can operate as either **Greedy Best-First Search** (for satisficing planning) or **A\*** (for optimal planning).

🛠️ How to Compile
------------------

The project is written in standard C and can be compiled using GCC. From the root directory, run the following command to create an executable named rover\_planner:

`   gcc -O3 -pg planner.c bloom.c -o rover_planner -lm   `

A standalone utility to verify solution files is also included and can be compiled with:

`   gcc -O3 rover_verify.c -o rover_verify   `

🚀 How to Run
-------------

To execute the planner, use the following format from the command line:

`./rover_planner <algorithm> <problem_file> <solution_file>`   

### Arguments:

* `<algorithm>`: The search algorithm to use.
    
    *   astar: For optimal search (uses the A\* algorithm).
        
    *   best: For satisficing search (uses Greedy Best-First Search).
        
* `<problem_file>`  : The path to the PDDL problem file you want to solve.
    
*  `<solution_file>` : The path where the output solution plan will be saved.
    

### Example:

To solve the problem p08.pddl using the A\* algorithm and save the solution to solution\_p08.txt:

`   ./rover_planner astar problems/p08.pddl solutions/solution_p08.txt   `

📂 Project Structure
--------------------

The source code is organized into several key files:

*   planner.c: Contains the main search loop, node expansion logic, and duplicate detection.
    
*   heuristic.h: The core of the project. Implements all five heuristic functions (H0-H4).
    
*   auxiliary.h: Defines all the core data structures for the planner (State, Rover, Waypoint, etc.).
    
*   parser.h: A dedicated parser for reading and interpreting the PDDL problem files.
    
*   minheap.h: An efficient Min-Heap implementation for the search frontier (open list).
    
*   solution.h: Functions for reconstructing the plan from the solution node and writing it to a file.
    
*   rover\_verify.c: A standalone program to verify the correctness of a generated solution plan.    

📄 License
----------

This project is licensed under the **MIT License**. See the LICENSE file for more details.
