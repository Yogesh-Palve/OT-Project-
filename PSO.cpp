#include <iostream>
#include <random>
#include <time.h>
#include <chrono>
#include <cmath>
#include <fstream>
#include <string>

#include <algorithm>
#include <functional>
#include <iterator>
#include <vector>
#include <array>
#include "PSO.h"

using namespace std;

// ==================================================================================
// MAIN PSO PATH PLANNING FUNCTION
// ==================================================================================
void particleSwarm(int argc, char **argv)
{
    // ------------------------------------------------------------------------------
    // Define start and goal (target) positions
    // ------------------------------------------------------------------------------
    Coord target; 
    target.x = DESTINATION_X;
    target.y = DESTINATION_Y;
    Coord start;
    start.x = START_X;
    start.y = START_Y;

    // ------------------------------------------------------------------------------
    // Random number setup for initializing particle positions and velocities
    // ------------------------------------------------------------------------------
    int seed = 1; // Incremental seed to ensure different sequences per run/call
    double lower_bound = -1;
    double upper_bound = 1;

    double *position_x = generate_random(SWARM_SIZE, lower_bound, upper_bound, seed++);
    double *position_y = generate_random(SWARM_SIZE, lower_bound, upper_bound, seed++);
    double *velocity_x = generate_random(SWARM_SIZE, lower_bound, upper_bound, seed++);
    double *velocity_y = generate_random(SWARM_SIZE, lower_bound, upper_bound, seed++);

    // ------------------------------------------------------------------------------
    // Pack initial random values into a temporary structure (for convenience)
    // ------------------------------------------------------------------------------
    Coord States[SWARM_SIZE];
    for (int i = SWARM_SIZE; i--;)
    {
        States[i].x     = position_x[i];
        States[i].y     = position_y[i];
        States[i].velx  = velocity_x[i];
        States[i].vely  = velocity_y[i];
    }

    // ------------------------------------------------------------------------------
    // Core PSO data structures
    // ------------------------------------------------------------------------------
    Coord   particle_states[SWARM_SIZE];   // Current position & velocity of each particle
    double  particle_fitness[SWARM_SIZE];  // Current fitness (distance to goal)
    Coord   local_best_pos[SWARM_SIZE];    // Personal best position of each particle
    double  local_best_fit[SWARM_SIZE];    // Personal best fitness
    Coord   global_best_pos;               // Swarm's global best position found so far
    double  global_best_fit;               // Global best fitness

    // Initialize global best to start position (will be updated immediately)
    global_best_pos.x = start.x;
    global_best_pos.y = start.y;

    // ------------------------------------------------------------------------------
    // Containers for logging data across iterations (for visualization/plotting)
    // ------------------------------------------------------------------------------
    int cols = SWARM_SIZE + 1;
    int rows = NO_OF_ITERS + 1;
    int initial_value = 0;

    int counter = 1; // Tracks number of iterations actually executed
    vector<Coord> global_pos(rows);                     // Global best per iteration
    global_pos[0].x = global_best_pos.x;
    global_pos[0].y = global_best_pos.y;

    vector<vector<double>> particle_pos_x(rows, vector<double>(cols, initial_value));
    vector<vector<double>> particle_pos_y(rows, vector<double>(cols, initial_value));

    vector<Coord> path_coord; // Final smoothed path (post-processing)

    vector<vector<double>> local_pos_x(rows, vector<double>(cols, initial_value));
    vector<vector<double>> local_pos_y(rows, vector<double>(cols, initial_value));

    global_best_fit = fitness(global_best_pos, global_pos[0]);

    // ==================================================================================
    // OBSTACLE INITIALIZATION – Randomly place circular obstacles (avoiding start/goal)
    // ==================================================================================
    double center_x[NUM_OBSTACLE];
    double center_y[NUM_OBSTACLE];
    for (int i = NUM_OBSTACLE; i--;)
    {
    init:
        int c_x = (POS_MULTIPLE - 200) * (generate_random(1, -1, 1, seed++)[0]);
        int c_y = (POS_MULTIPLE - 200) * (generate_random(1, -1, 1, seed++)[0]);
        bool stinobs = inCircle(start.x, start.y, c_x, c_y, R);
        bool tarinobs = inCircle(target.x, target.y, c_x, c_y, R);
        if (!stinobs && !tarinobs)
        {
            center_x[i] = c_x;
            center_y[i] = c_y;
        }
        else
        {
            cout << "start or target in obstacle..... Replanning" << endl;
            goto init; // Regenerate this obstacle if it covers start or goal
        }
    }

    // ==================================================================================
    // FIRST ITERATION – Initialize swarm with feasible random positions
    // ==================================================================================
    double thresh = (R * 0.1); // Safety margin for obstacle avoidance
    for (int i = SWARM_SIZE; i--;)
    {
        bool final_val_init = false;
        int ct = 500; // Safety counter to prevent infinite loops
        do
        {
            // Scale normalized random values [-1,1] → search space
            particle_states[i].x = POS_MULTIPLE * (States[i].x);
            particle_states[i].y = POS_MULTIPLE * (States[i].y);

            // Generate new normalized values for next use
            double new_x = generate_random(1, 0, 1, seed++)[0];
            States[i].x = new_x;
            double new_y = generate_random(1, 0, 1, seed++)[0];
            States[i].y = new_y;

            // Clamp to environment boundaries
            if (particle_states[i].x > UPPER_BOUNDARY) particle_states[i].x = UPPER_BOUNDARY;
            if (particle_states[i].y > UPPER_BOUNDARY) particle_states[i].y = UPPER_BOUNDARY;
            if (particle_states[i].x < LOWER_BOUNDARY) particle_states[i].x = LOWER_BOUNDARY;
            if (particle_states[i].y < LOWER_BOUNDARY) particle_states[i].y = LOWER_BOUNDARY;

            // Reject and regenerate if particle is inside any obstacle
            final_val_init = obstacle_avoidance(NUM_OBSTACLE, particle_states[i], start,
                                                center_x, center_y, R, thresh, seed,
                                                LOWER_BOUNDARY, UPPER_BOUNDARY);
            ct--;
        } while ((final_val_init) && (ct != 0));

        // Initialize velocity (scaled from normalized random values)
        particle_states[i].velx = VEL_MULTIPLE * (States[i].x);
        particle_states[i].vely = VEL_MULTIPLE * (States[i].y);

        // Evaluate fitness (typically Euclidean distance to goal)
        particle_fitness[i] = fitness(particle_states[i], global_pos[0]);

        // Set initial personal best = current position
        local_best_pos[i] = particle_states[i];
        local_best_fit[i] = particle_fitness[i];

        // Update global best if this particle is better
        if (local_best_fit[i] > global_best_fit)
        {
            global_best_pos = particle_states[i];
            global_best_fit = local_best_fit[i];
        }

        // Log data for iteration 1
        local_pos_x[1][i] = local_best_pos[i].x;
        local_pos_y[1][i] = local_best_pos[i].y;
        global_pos[1] = global_best_pos;
        particle_pos_x[1][i] = particle_states[i].x;
        particle_pos_y[1][i] = particle_states[i].y;
    }

    // Display results after first iteration
    epochDisplay(1, global_best_fit, global_best_pos);

    // ==================================================================================
    // MAIN PSO LOOP – From iteration 2 to NO_OF_ITERS (or convergence)
    // ==================================================================================
    int flag = 1, cnt = 0;
    for (int iter_ctr = 2; iter_ctr <= NO_OF_ITERS; iter_ctr++)
    {
        // --------------------------------------------------------------------------        
        // Update velocity and position for all particles
        // --------------------------------------------------------------------------
        for (int i = SWARM_SIZE; i--;)
        {
            bool final_val = false;
            int ct = 500;
            do
            {
                double rp = generate_random(1, 0, 1, seed++)[0]; // Cognitive random factor
                double rg = generate_random(1, 0, 1, seed++)[0]; // Social random factor

                // Adaptive inertia weight (from literature – exponential decay)
                double w, c = 2.3;
                w = WMIN + (WMAX - WMIN) * exp(-pow(((c * iter_ctr) / NO_OF_ITERS), 2.0));

                // Classic PSO velocity update with inertia, cognitive, and social components
                particle_states[i].velx = (w * particle_states[i].velx +
                                          C1 * rp * (local_best_pos[i].x - particle_states[i].x)) +
                                          C2 * rg * (global_best_pos.x - particle_states[i].x);

                particle_states[i].vely = (w * particle_states[i].vely +
                                          C1 * rp * (local_best_pos[i].y - particle_states[i].y)) +
                                          C2 * rg * (global_best_pos.y - particle_states[i].y);

                // Clamp velocity to maximum allowed
                particle_states[i].velx = max(min(particle_states[i].velx, V_MAX), -V_MAX);
                particle_states[i].vely = max(min(particle_states[i].vely, V_MAX), -V_MAX);

                // Update position
                particle_states[i].x += particle_states[i].velx;
                particle_states[i].y += particle_states[i].vely;

                // Enforce obstacle-free position (reject & retry if colliding)
                final_val = obstacle_avoidance(NUM_OBSTACLE, particle_states[i], global_best_pos,
                                               center_x, center_y, R, thresh, seed,
                                               LOWER_BOUNDARY, UPPER_BOUNDARY);
                ct--;
            } while ((final_val) && (ct != 0));

            // Log current particle positions
            particle_pos_x[iter_ctr][i] = particle_states[i].x;
            particle_pos_y[iter_ctr][i] = particle_states[i].y;
        }

        // --------------------------------------------------------------------------
        // Update personal (local) and global bests based on new fitness values
        // --------------------------------------------------------------------------
        for (int i = SWARM_SIZE; i--;)
        {
            particle_fitness[i] = fitness(particle_states[i], global_pos[iter_ctr - 1]);

            // Update personal best if current position is better
            if (particle_fitness[i] > local_best_fit[i])
            {
                local_best_pos[i] = particle_states[i];
                local_best_fit[i] = particle_fitness[i];

                // Update global best if this personal best is the best so far
                if (local_best_fit[i] > global_best_fit)
                {
                    global_best_pos = local_best_pos[i];
                    global_best_fit = local_best_fit[i];
                    global_best_pos.velx = particle_states[i].velx;
                    global_best_pos.vely = particle_states[i].vely;
                }
            }

            // Log local and global bests for this iteration
            local_pos_x[iter_ctr][i] = local_best_pos[i].x;
            local_pos_y[iter_ctr][i] = local_best_pos[i].y;
            global_pos[iter_ctr] = global_best_pos;
        }

        // --------------------------------------------------------------------------
        // Local minima detection and escape mechanism (after a few iterations)
        // --------------------------------------------------------------------------
        if (iter_ctr > 3)
        {
            bool stuck = fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-1].x) < 0.2 &&
                         fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-2].x) < 0.2 &&
                         fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-3].x) < 0.2 &&
                         fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-1].y) < 0.2 &&
                         fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-2].y) < 0.2 &&
                         fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-3].y) < 0.2;

            if (stuck)
            {
                cout << "Local Minima detected: Applying perturbation..." << endl;
                bool final_val = false;
                int cs = 500;
                do
                {
                    double alpha = 0.2, beta = 0.3;
                    double r3 = generate_random(1, 0, 1, seed++)[0];

                    // Perturb global best velocity and position slightly
                    global_best_pos.velx = alpha * global_best_pos.velx + beta * r3;
                    global_best_pos.vely = alpha * global_best_pos.vely + beta * r3;

                    Coord gbest;
                    gbest.x = global_best_pos.x + global_best_pos.velx;
                    gbest.y = global_best_pos.y + global_best_pos.vely;

                    global_best_pos = gbest;
                    global_best_fit = fitness(gbest, global_pos[iter_ctr - 1]);

                    final_val = obstacle_avoidance(NUM_OBSTACLE, global_best_pos, global_best_pos,
                                                   center_x, center_y, R, thresh, seed,
                                                   LOWER_BOUNDARY, UPPER_BOUNDARY);
                    cs--;
                } while (final_val && cs != 0);
            }
        }

        // Display progress for current iteration
        epochDisplay(iter_ctr, global_best_fit, global_best_pos);

        // --------------------------------------------------------------------------
        // Convergence check: reached goal?
        // --------------------------------------------------------------------------
        counter++;
        if (shortfit(global_best_pos, target) <= TARGET_TOLERANCE)
        {
            cout << "\n\nReached near goal. SUCCESS " << endl;
            flag = 0;
            break;
        }

        // --------------------------------------------------------------------------
        // Early termination if stagnated for too long (likely trapped)
        // --------------------------------------------------------------------------
        else if (iter_ctr >= LOCAL_CONV_TOLERANCE)
        {
            bool converged_locally = fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-1].x) < 0.1 &&
                                     fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-2].x) < 0.1 &&
                                     fabs(global_pos[iter_ctr].x - global_pos[iter_ctr-3].x) < 0.1 &&
                                     fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-1].y) < 0.1 &&
                                     fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-2].y) < 0.1 &&
                                     fabs(global_pos[iter_ctr].y - global_pos[iter_ctr-3].y) < 0.1;

            if (converged_locally)
            {
                cout << "\n\nCouldn't find a feasible path / Converged at a local minima. FAILED" << endl;
                flag = 1;
                break;
            }
        }
    }

    // ==================================================================================
    // POST-PROCESSING: Build a cleaner path from global best history
    // ==================================================================================
    Coord path_cd = start;
    path_coord.push_back(path_cd);

    for (int iter_ctr = 0; iter_ctr < counter; iter_ctr++)
    {
        if (iter_ctr > 3)
        {
            // Keep points that consistently get closer to the goal (simple smoothing)
            bool improving = shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr-1], target) &&
                             shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr-2], target) &&
                             shortfit(global_pos[iter_ctr], target) < shortfit(global_pos[iter_ctr-3], target);

            if (improving)
                path_coord.push_back(global_pos[iter_ctr]);
            else
                path_coord.push_back(global_pos[iter_ctr-1]);
        }
        else
        {
            path_coord.push_back(global_pos[0]);
        }
    }

    // Final result output
    cout << endl << "Final Global Best Position: (" 
         << global_best_pos.x << "," << global_best_pos.y << ")" << endl;

    // ==================================================================================
    // EXPORT ALL DATA TO CSV for plotting in Python/MATLAB
    // ==================================================================================
    write_to_csv(particle_pos_x, particle_pos_y, global_pos,
                 local_pos_x, local_pos_y, SWARM_SIZE, counter,
                 center_x, center_y, R, NUM_OBSTACLE, path_coord);

    cout << "Run Successful" << endl;
}

// ==================================================================================
// MAIN FUNCTION – Entry point
// ==================================================================================
int main(int argc, char **argv)
{
    srand(time(0));
    clock_t startTime = clock();
    cout << "Initiating Optimization ..." << endl;

    particleSwarm(argc, argv);

    // Note: Bug in original code – prints startTime instead of duration
    cout << "Optimization Completed in " 
         << (float)(clock() - startTime) / CLOCKS_PER_SEC << " seconds" << endl;

    return 0;
}
// Pseudo code
/*
 Start
 Initialize swarm with Random Position (x0) and velocity vectors (v0)
 for Each Particle
 Evaluate Fitness
 If fitness(xt) > fitness(gbest)
 gbest=xt
 If fitness(xt) > fitness(pbest)
 pbest=xt
 Update Velocity
 v(t+1)=W*vt + c1*rand(0,1)*(pbest-xt)+c2*rand(0,1)*gbest-xt)
 Update Position
 X(t+1) = Xt+V(t+1)

 Go to next particle

 If Terminate
 gbest is the output
 Else goto for Each Particle
 */


