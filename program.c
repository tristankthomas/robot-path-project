/* Solution to comp20005 Assignment 2, 2022 semester 1.

   Authorship Declaration:

   (1) I certify that the program contained in this submission is completely
   my own individual work, except where explicitly noted by comments that
   provide details otherwise.  I understand that work that has been developed
   by another student, or by me in collaboration with other students,
   or by non-students as a result of request, solicitation, or payment,
   may not be submitted for assessment in this subject.  I understand that
   submitting for assessment work developed by or in collaboration with
   other students or non-students constitutes Academic Misconduct, and
   may be penalized by mark deductions, or by other penalties determined
   via the University of Melbourne Academic Honesty Policy, as described
   at https://academicintegrity.unimelb.edu.au.

   (2) I also certify that I have not provided a copy of this work in either
   softcopy or hardcopy or any other form to any other student, and nor will
   I do so until after the marks are released. I understand that providing
   my work to other students, regardless of my intention or any undertakings
   made to me by that other student, is also Academic Misconduct.

   (3) I further understand that providing a copy of the assignment
   specification to any form of code authoring or assignment tutoring
   service, or drawing the attention of others to such services and code
   that may have been made available via such a service, may be regarded
   as Student General Misconduct (interfering with the teaching activities
   of the University and/or inciting others to commit Academic Misconduct).
   I understand that an allegation of Student General Misconduct may arise
   regardless of whether or not I personally make use of such solutions
   or sought benefit from such actions.

   Signed by: Tristan Thomas (1269492)
   Dated:     5/5/2022

*/

/* ========================================================================== */

/* -- Library inclusions -- */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

/* - Constant definitions - */
#define MAX_COLS 100
#define MAX_ROWS 100
#define MAX_OBSTA MAX_COLS * MAX_ROWS - 1
#define OBSTA_COLS 4

#define STAGE1 1
#define STAGE2 2
#define STAGE3 3

#define X_MIN 0
#define X_MAX 1
#define Y_MIN 2
#define Y_MAX 3

#define HOME_Y 0
#define HOME_X 0

#define RIGHT 1
#define LEFT 2
#define TOP 3
#define BOTTOM 4

#define OBSTACLE '|'
#define REACHABLE 'R'
#define UNEXPLORED '-'

/* rough approximation of an infinite cost (will always be largest for 
   a 100 x 100 world) */
#define MAX_COST 1000000

#define DIAG_COST 3
#define CROSS_COST 2

#define TYPE_REACH 0
#define TYPE_COST 1


/* ------- Typedefs ------- */

/* obstacle 2d array */
typedef int one_obstacles_t[OBSTA_COLS];
typedef one_obstacles_t obstacles_t[MAX_OBSTA];
/* coordinates 2d array (each cell a character type) */
typedef char char_cols_t[MAX_COLS];
typedef char_cols_t char_coords_t[MAX_ROWS];
/* coordinates 2d array (each cell an integer cost) */
typedef int int_cols_t[MAX_COLS];
typedef int_cols_t int_coords_t[MAX_ROWS];

/* structure defining robots world */
typedef struct {

    obstacles_t obstacles;
    char_coords_t coords_type;
    int_coords_t coords_cost;
    int n_rows, n_cols, n_obstas;

} robot_world_t;


/* -- Function prototypes - */
void read_data(robot_world_t *world, int max_cols, int max_rows);
void do_stage1(robot_world_t *world, int stage);
void do_stage2(robot_world_t *world, int stage);
void do_stage3(robot_world_t *world, int stage);
void print_stage(int stage);
void print_obstacle(robot_world_t world, int obst_num);
int obstacle_tagger(robot_world_t *world);
int ovrl_zone_tagger(robot_world_t *world, int x_start, int y_start, 
    char marker, int type);
int indiv_cell_check(robot_world_t *world, int x, int y, char marker, 
    int type);
char conversion(int num);
int edge_detect(robot_world_t *world, int x, int y, int type);
int unreach_zone_tagger(robot_world_t *world, char marker);
void print_cell_stats(robot_world_t world, int num, int stage);


/* ============================== Main function ============================= */

int main(int argc, char *argv[]) {
    robot_world_t robot_world;
    /* initialised number of obstacles */
    robot_world.n_obstas = 0;

    /* reads input data into structure */
    read_data(&robot_world, MAX_COLS, MAX_ROWS);
    
    /* initialises all cost and type coordinates as the max cost and '-' 
       (signifying unexplored) respectively */
    for (int i = 0; i < robot_world.n_rows; i++) {
        for (int j = 0; j < robot_world.n_cols; j++) {
            robot_world.coords_type[i][j] = UNEXPLORED;
            robot_world.coords_cost[i][j] = MAX_COST;
        }
    }

    do_stage1(&robot_world, STAGE1);
    do_stage2(&robot_world, STAGE2);
    do_stage3(&robot_world, STAGE3);

    printf("ta daa!\n");

	return 0;
}

/* ============================ Reading function ============================ */
/* This function was adapted from  Figure 8.6 on page 136 of the subject 
   textbook, written by Alistair Moffat, and accessed via 
   https://people.eng.unimelb.edu.au/ammoffat/ppsaa/c/
   Altered to the specific struct data and includes error printing */

/* Reads the cell dimensions into n_rows and n_cols of struct and the obstacle 
   dimensions into a two dimensional array */

void read_data(robot_world_t *world, int max_cols, int max_rows) {
    int x_min, x_max, y_min, y_max;
    /* scans for the first two ints */
    if (scanf("%d %d", &world->n_cols, &world->n_rows) != 2) {
        printf("Dimensions not entered correctly\n");
        exit(EXIT_FAILURE);
    }

    /* error if entered dimensions are out of initially defined bounds */
    if (world->n_cols > max_cols || world->n_rows > max_rows) {
        printf("Array is too large\n");
        exit(EXIT_FAILURE);
    }

    /* scans the remaining obstacles */
    while(scanf("%d %d %d %d", &x_min, &x_max, &y_min, &y_max) == 4) {

        world->obstacles[world->n_obstas][X_MIN] = x_min;
        world->obstacles[world->n_obstas][X_MAX] = x_max;
        world->obstacles[world->n_obstas][Y_MIN] = y_min;
        world->obstacles[world->n_obstas][Y_MAX] = y_max;

        world->n_obstas++;

        /* obstacle bounds check */
        if (x_min < HOME_X || x_max > world->n_cols - 1 || 
                y_min < HOME_Y || y_min > world->n_rows - 1) {
            printf("Obstacle is out of bounds\n");
            exit(EXIT_FAILURE);
            
        }

    }

    return;
}


/* ================================= Stage 1 ================================ */
/* The names of the next three functions were taken from the sample solution, 
    written by Alistair Moffat, and accessed via the LMS */

/* Stage 1 outputs the following
    - the world dimensions
    - number of obstacles in world
    - obstacle dimensions */

    
void do_stage1(robot_world_t *world, int stage) {
    int i;
    /* print world dimensions */
    print_stage(stage);
    printf("world is %2d cells wide x %2d cells high\n", 
        world->n_cols, world->n_rows);

    /* prints out number of obstacles */
    print_stage(stage);
    printf("world contains %d obstacles\n", world->n_obstas);

    /* prints areas of each obstacle */
    for (i = 0; i < world->n_obstas; i++) {
        print_stage(stage);
        print_obstacle(*world, i);
    }

    printf("\n");

    return;

}


/* ================================= Stage 2 ================================ */

/* Stage 2 outputs the following
    - number of reachable cells
    - number of obstacle cells 
    - all the zones of unreachable cells
    - NOTE: in the array coords, reachable cells will be marked '1', obstacle 
      cells will be marked '0' and unreachable cells will be marked from 'a' 
      to 'z' depending on number of zones. */

void do_stage2(robot_world_t *world, int stage) {
    int num_reach = 1, prev_num_reach, num_obsta, num_zone = 0;
    char tag = 'a';

    /* marking obstacle cells */
    num_obsta = obstacle_tagger(world);


    while (1) {
        prev_num_reach = num_reach;

        /* keeps tagging whilst changes are preset */
        num_reach += ovrl_zone_tagger(world, HOME_X, HOME_Y, REACHABLE, 
            TYPE_REACH);

        /* exits loop if no consecutive changes */
        if (prev_num_reach == num_reach) {
            break;
        }

    }
    /* prints number of reachable cells */
    print_cell_stats(*world, num_reach, stage);
    printf("reachable\n");


    while (1) {
        /* tags unreachable regions until no more unreachable regions */
        num_zone = unreach_zone_tagger(world, tag++);

        if (!num_zone) {
            break;
        }
        /* minus 1 since tag is incremented by time it reaches statement */
        print_cell_stats(*world, num_zone, stage);
        printf("in unreachable zone %c\n", tag - 1);

    }
    
    /* prints number of obstacles */
    print_cell_stats(*world, num_obsta, stage);
    printf("obstacles\n");

    printf("\n");

    return;

}

/* ================================= Stage 3 ================================ */

/* Stage 3 outputs the following
    - map where pipe represents obstacle, and the shortest route to the cell
      from home base is represented by 2 for rectilinear and 3 for diagonal */

void do_stage3(robot_world_t *world, int stage) {
    int num_changes = 1, prev_num_changes;

    while(1) {
        /* keeps checking reachable zone until lowest price for all cells
           is present (by no more changes occuring) */
        prev_num_changes = num_changes;
        num_changes += ovrl_zone_tagger(world, HOME_X, HOME_Y, REACHABLE, 
            TYPE_COST);

        if (prev_num_changes == num_changes) {
            break;
        }
    }


    for (int i = world->n_rows - 1; i >= 0; i--) {

        /* prints every second row */
        if (i % 2 == 0) {
            /* prints map x axis */
            print_stage(stage);
            if (i % 10 == 0) {
                printf("%2d + ", i);
            } else {
                printf("   | ");
            }
            /* prints contents of map */
            for (int j = 0; j < world->n_cols; j++) {
                    
                    if (i == HOME_Y && j == HOME_X) {
                        printf("%c", REACHABLE);
                    } else if (world->coords_type[i][j] == REACHABLE) {
                        /* prints reachable region with costs (in correct char 
                        formatting) */
                        printf("%c", conversion(world->coords_cost[i][j]));
                    } else {
                        /* for all other regions prints as stage 2 tagging */
                        printf("%c", world->coords_type[i][j]);
                    }
                
            }

            printf("\n");

        }
        
    }

    printf("\n");

    return;

}


/* ========================================================================== */
/* ============================ Helper functions ============================ */
/* ========================================================================== */

/* ====================== Calculation helper functions ====================== */

/* Tags all the obstacles as char '|' if obstacle at that coordinate */

int obstacle_tagger(robot_world_t *world) {
    int i, j, k, num_obsta = 0;

    for (i = 0; i < world->n_obstas; i++) {
        /* goes from min to max of x and y of each obstacle */
        for (j = world->obstacles[i][Y_MIN]; 
                j <= world->obstacles[i][Y_MAX]; j++) {

            for (k = world->obstacles[i][X_MIN]; 
                    k <= world->obstacles[i][X_MAX]; k++) {
                
                if (world->coords_type[j][k] != OBSTACLE) {
                    /* tags as obstacle if wasn't previously an obstacle 
                       (to avoid counter duplicates) */
                    world->coords_type[j][k] = OBSTACLE;
                    num_obsta++;
                }
                
            }    
        }
    }

    return num_obsta;

}

/* ========================================================================== */

/* Robot moves through each cell and if cell is reachable tags all adjacent 
   non reachable cells as char 'R' (indicating reachable). Increments 
   tot_changes if a change is made. */

int ovrl_zone_tagger(robot_world_t *world, int x_start, int y_start, 
        char marker, int type) {
    int tot_changes = 0;
    /* initialise starting points */
    world->coords_type[y_start][x_start] = marker;
    world->coords_cost[y_start][x_start] = 0;

    for (int i = y_start; i < world->n_rows; i++) {

        for (int j = HOME_X; j < world->n_cols; j++) {

            if (world->coords_type[i][j] == marker) {
               /* undertakes an individual cell check if current cell has the
                  specified marker and tags with marker if necessary */ 
                tot_changes += indiv_cell_check(world, j, i, marker, type);

            }

        }
    }

    return tot_changes;

}

/* ========================================================================== */

/* Checks for all adjacent cells (rectilinear and diagonal) and depending on
   inputted type, either tags with reachable if not an obstacle or outside array
   or tags with a lower cost if type is cost 
   NOTE: Although it is unlikely that the cells outside the bounds have the 
        reachable character the edge detectors are there just in case */

int indiv_cell_check(robot_world_t *world, int x, int y, char marker, 
        int type) {
    /* define adjacents x, y values */
    int right_x = x + 1;
    int left_x = x - 1; 
    int up_y = y + 1;
    int down_y = y - 1;

    int changes = 0;

    int cost = world->coords_cost[y][x];

    /* flags for edges (1 if edge is present) */
    int right_edge = edge_detect(world, x, y, RIGHT);
    int left_edge = edge_detect(world, x, y, LEFT);
    int top_edge = edge_detect(world, x, y, TOP);
    int bottom_edge = edge_detect(world, x, y, BOTTOM);

    /* flags for obstacles (1 if obstacle is present) */
    int right_obst = world->coords_type[y][right_x] == OBSTACLE;
    int left_obst = world->coords_type[y][left_x] == OBSTACLE;
    int up_obst = world->coords_type[up_y][x] == OBSTACLE;
    int down_obst = world->coords_type[down_y][x] == OBSTACLE;
    int up_right_obst = world->coords_type[up_y][right_x] == OBSTACLE;
    int up_left_obst = world->coords_type[up_y][left_x] == OBSTACLE;
    int down_right_obst = world->coords_type[down_y][right_x] == OBSTACLE;
    int down_left_obst = world->coords_type[down_y][left_x] == OBSTACLE;

    /* only performs tag if there are no obstacles or edges in direction 
       NOTE: the comments in check to right are the same for all directions */
    /* check to the right */
    if (!right_edge && !right_obst) {
        /* stage 2 tagging (tags if cell is reachable) */
        if (world->coords_type[y][right_x] != marker && type == TYPE_REACH) {

            world->coords_type[y][right_x] = marker;
            changes++;

        }
        
        /* stage 3 tagging (tags if cost can be lower) */
        if (cost + CROSS_COST < world->coords_cost[y][right_x] && 
                type == TYPE_COST) {

            world->coords_cost[y][right_x] = cost + CROSS_COST;
            changes++;

        }

    }

    /* checks to the left */
    if (!left_edge && !left_obst) {

        if (world->coords_type[y][left_x] != marker && type == TYPE_REACH) {

            world->coords_type[y][left_x] = marker;
            changes++;
        }

        if (cost + CROSS_COST < world->coords_cost[y][left_x] && 
                type == TYPE_COST) {

            world->coords_cost[y][left_x] = cost + CROSS_COST;
            changes++;
            
        }

    }

    /* checks above */
    if (!top_edge && !up_obst) {

        if (world->coords_type[up_y][x] != marker && type == TYPE_REACH) {

            world->coords_type[up_y][x] = marker;
            changes++;
        }

        if (cost + CROSS_COST < world->coords_cost[up_y][x] && 
                type == TYPE_COST) {

            world->coords_cost[up_y][x] = cost + CROSS_COST;
            changes++;
        }

    }

    /* checks below */
    if (!bottom_edge && !down_obst) {

        if (world->coords_type[down_y][x] != marker && type == TYPE_REACH) {

            world->coords_type[down_y][x] = marker;
            changes++;
        }

        if (cost + CROSS_COST < world->coords_cost[down_y][x] && 
                type == TYPE_COST) {

            world->coords_cost[down_y][x] = cost + CROSS_COST;
            changes++;
        }

    }

    /* checks top right */
    if ((!right_edge && !top_edge) && 
             (!up_right_obst && !right_obst && !up_obst)) {
        
        if (world->coords_type[up_y][right_x] != marker && 
                type == TYPE_REACH) {

            world->coords_type[up_y][right_x] = marker;
            changes++;
        }

        if (cost + DIAG_COST < world->coords_cost[up_y][right_x] && 
                type == TYPE_COST) {

            world->coords_cost[up_y][right_x] = cost + DIAG_COST;
            changes++;
        }
        

    }

    /* checks top left */
    if ((!left_edge && !top_edge) && 
            (!up_left_obst && !left_obst && !up_obst)) {
        
        if (world->coords_type[up_y][left_x] != marker && 
                type == TYPE_REACH) {

            world->coords_type[up_y][left_x] = marker;
            changes++;
        }

        if (cost + DIAG_COST < world->coords_cost[up_y][left_x] && 
                type == TYPE_COST) {

            world->coords_cost[up_y][left_x] = cost + DIAG_COST;
            changes++;
        }


    }

    /* checks bottom right */
    if ((!right_edge && !bottom_edge) && 
            (!down_right_obst && !right_obst && !down_obst)) {

        if (world->coords_type[down_y][right_x] != marker && 
                type == TYPE_REACH) {

            world->coords_type[down_y][right_x] = marker;
            changes++;
        }

        if (cost + DIAG_COST < world->coords_cost[down_y][right_x] && 
                type == TYPE_COST) {

            world->coords_cost[down_y][right_x] = cost + DIAG_COST;
            changes++;
        }

    }

    /* checks bottom left */
    if ((!left_edge && !bottom_edge) && 
            (!down_left_obst && !left_obst && !down_obst)) {
        
        if (world->coords_type[down_y][left_x] != marker && 
                type == TYPE_REACH) {

            world->coords_type[down_y][left_x] = marker;
            changes++;
        }

        if (cost + DIAG_COST < world->coords_cost[down_y][left_x] && 
                type == TYPE_COST) {

            world->coords_cost[down_y][left_x] = cost + DIAG_COST;
            changes++;
        }

    }

    return changes;

}

/* ========================================================================== */

/* Checks if the inputted coordinate is at the edge of the world */

int edge_detect(robot_world_t *world, int x, int y, int type) {

    /* right edge detection */
    if (type == RIGHT) {
        return x == world->n_cols - 1;
    }
    
    /* left edge detection */
    if (type == LEFT) {
        return x == HOME_X;
    }

    /* top edge detection */
    if (type == TOP) {
        return y == world->n_rows - 1;
    }

    /* bottom edge detection */
    if (type == BOTTOM) {
        return y == HOME_Y;

    }

    return 0;

}


/* ========================================================================== */

/* Finds closest unexplored cell and tags from that cell
   onwards until no changes */

int unreach_zone_tagger(robot_world_t *world, char marker) {
    int num_unreach = 1, prev_num_unreach, i, j;
    for (i = HOME_Y; i < world->n_rows; i++) {

        for (j = HOME_X; j < world->n_cols; j++) {
            /* finds nearest untouched cell */
            if (world->coords_type[i][j] == UNEXPLORED) {

                while (1) {     
                    /* performs check starting at untouched cell until no more 
                       changes */
                    prev_num_unreach = num_unreach;
                    num_unreach += ovrl_zone_tagger(world, j, i, marker, 
                        TYPE_REACH);

                    if (prev_num_unreach == num_unreach) {
                        break;
                    }

                }
                /* returns the number of cell changes for each region 
                   (used in output) */
                return num_unreach;
            }

        }
    }

    return 0;
}


/* ======================= Formating helper functions ======================= */

/* Converts an integer value to a character '.' if last digit between 4 and 9
   and converts to the second last digit as a char if last digit between 0
   and 3 */

char conversion(int num) {
    char string[MAX_COST];
    int length;
    /* stores decimal integer as string into array 'string' */
    length = sprintf(string, "%d", num);

    if (string[length - 1] >= '0' && string[length - 1] <= '3') {
        
        if (length == 1) {
            /* when num has no second last digit */
            return '0';

        } else {
            /* second last digit */
            return string[length - 2];
        }
        
    } else if (string[length - 1] >= '4' && string[length - 1] <= '9') {
        /* period for other range of last digit */
        return '.';
    }

    return '0';
    
}

/* ========================================================================== */

/* Prints an obstacle in the correct format for stage 1 */

void print_obstacle(robot_world_t world, int obst_num) {

    printf("obstacle %2d covers [%2d,%2d] x [%2d,%2d]\n", obst_num, 
        world.obstacles[obst_num][X_MIN], world.obstacles[obst_num][X_MAX],
        world.obstacles[obst_num][Y_MIN], world.obstacles[obst_num][Y_MAX]);

    return;

}

/* ========================================================================== */

/* Prints first part of the lines in stage 2 (rest of line depends on case) */

void print_cell_stats(robot_world_t world, int num, int stage) {

    print_stage(stage);
    printf("%4d of %4d cells are ", num, world.n_cols * world.n_rows);

    return;
}

/* ========================================================================== */

/* Prints the current stage in the correct formatting */

void print_stage(int stage) {

    printf("S%d, ", stage);

    return;
}

/* =========================== programming is fun =========================== */
