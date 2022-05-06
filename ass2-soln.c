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
#define FIRST_Y 0
#define FIRST_X 0

#define RIGHT 1
#define LEFT 2
#define TOP 3
#define BOTTOM 4

#define OBSTACLE '0'
#define REACHABLE '1'
#define UNEXPLORED '-'


/* ------- Typedefs ------- */

typedef char cols_t[MAX_COLS];
typedef int one_obst_t[OBSTA_COLS];
typedef cols_t coords_t[MAX_ROWS];
typedef one_obst_t obsts_t[MAX_OBSTA];

typedef struct {
    coords_t coords_type;
    obsts_t obstacles;
    int n_rows, n_cols, n_obstas;
} robot_world_t;


/* -- Function prototypes - */
void read_data(robot_world_t *world, int max_rows, int max_cols);
void do_stage1(robot_world_t world, int stage);
void do_stage2(robot_world_t world, int stage);
void print_stage(int stage);
void print_blank(void);
void print_obstacle(robot_world_t world, int obst_num);
void obstacle_tagger(robot_world_t *world);
int ovrl_reachable_tagger(robot_world_t *world);
int indiv_reachable_tagger(robot_world_t *world, int x, int y);
int edge_detect(robot_world_t *world, int x, int y, int type);

/* test function */
void print_world(robot_world_t world);
// int full_obstacle_check(robot_world_t world, int row, int col);
// int edge_check(robot_world_t world, int y, int x);
// int indiv_obstacle_check(robot_world_t world, int y, int x, int obsta, int type);


/* ============================== Main function ============================= */

int main(int argc, char *argv[]) {
    robot_world_t robot_world;
    
    read_data(&robot_world, MAX_ROWS, MAX_COLS);

    /* initialises all coordinates as type '-' */
    for (int i = 0; i < robot_world.n_rows; i++) {
        for (int j = 0; j < robot_world.n_cols; j++) {
            robot_world.coords_type[i][j] = UNEXPLORED;

        }

    }
    printf("%c\n", robot_world.coords_type[1][1]);

    do_stage1(robot_world, STAGE1);
    do_stage2(robot_world, STAGE2);
    
        


    
    /*printf("%d x %d\n", robot_world.n_cols, robot_world.n_rows);
    for (int i = 0; i < robot_world.nobsts; i++) {
        printf("%d: [%d,%d]x[%d,%d]\n", i, robot_world.obstacles[i][X_MIN], 
            robot_world.obstacles[i][X_MAX], robot_world.obstacles[i][Y_MIN], 
            robot_world.obstacles[i][Y_MAX]);
    }*/


	return 0;
}

/* ============================ Reading function ============================ */

void read_data(robot_world_t *world, int max_rows, int max_cols) {
    int x_min, x_max, y_min, y_max, num = 0;
    /* scans for the first two ints */
    if (scanf("%d %d", &world -> n_cols, &world -> n_rows) != 2) {
        exit(EXIT_FAILURE);
    }

    /* scans the remaining obstacles */
    while(scanf("%d %d %d %d", &x_min, &x_max, &y_min, &y_max) == 4) {
        world->obstacles[num][X_MIN] = x_min;
        world->obstacles[num][X_MAX] = x_max;
        world->obstacles[num][Y_MIN] = y_min;
        world->obstacles[num][Y_MAX] = y_max;
        /* post-increment */
        world->n_obstas = num++;

    }
    world->n_obstas++;
    return;
}


/* ================================= Stage 1 ================================ */

/* Stage 1 outputs the following
    - the world dimensions
    - number of obstacles in world
    - obstacle dimensions */

      

void do_stage1(robot_world_t world, int stage) {
    int i;
    /* print world dimensions */
    print_stage(stage);
    printf("world is %2d cells wide x %2d cells high\n", 
        world.n_cols, world.n_rows);

    /* prints out number of obstacles */
    print_stage(stage);
    printf("world contains %d obstacles\n", world.n_obstas);

    /* prints areas of each obstacle */
    for (i = 0; i < world.n_obstas; i++) {
        print_stage(stage);
        print_obstacle(world, i);
    }

    print_blank();
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

void do_stage2(robot_world_t world, int stage) {

    int i = 0, total_reach = 1, prev_total_reach;

    /* marking home base cell */
    world.coords_type[FIRST_Y][FIRST_X] = REACHABLE; // home base
    /* marking obstacle cells */
    obstacle_tagger(&world);

    printf("AFTER OBSTACLES ADDED\n");
    print_world(world);
    while (1) {
        prev_total_reach = total_reach;
        total_reach += ovrl_reachable_tagger(&world);

        if (prev_total_reach == total_reach) {
            break;
        }
        printf("AFTER RUN %d:\n", i);
        print_world(world);
        
        i++;
        
        

    }
    printf("ovrl_changes = %d\n", total_reach);

    // ovrl_reachable_tagger(&world);

    // printf("AFTER FIRST RUN THROUGH\n");
    // print_world(world);


}

/* ========================================================================== */
/* ============================ Helper functions ============================ */
/* ========================================================================== */

/* ====================== Calculation helper functions ====================== */

/* Tags all the obstacles as char '0' if obstacle at that coordinate */
void obstacle_tagger(robot_world_t *world) {
    for (int i = 0; i < world->n_obstas; i++) {

        for (int j = world->obstacles[i][Y_MIN]; 
                j <= world->obstacles[i][Y_MAX]; j++) {

            for (int k = world->obstacles[i][X_MIN]; 
                    k <= world->obstacles[i][X_MAX]; k++) {
                
                world->coords_type[j][k] = OBSTACLE;
            }    
        }
    }


}

/* ========================================================================== */

/* Robot moves through each cell and if cell is reachable tags all adjacent 
   non reachable cells as char '1' (indicating reachable). Returns 1 if a 
   change is made. */
int ovrl_reachable_tagger(robot_world_t *world) {
    int tot_changes = 0;
    for (int i = FIRST_Y; i < world->n_rows; i++) {

        for (int j = FIRST_X; j < world->n_cols; j++) {

            if (world->coords_type[i][j] == REACHABLE) {
                
                tot_changes += indiv_reachable_tagger(world, j, i);

            }

        }
    }

    return tot_changes;




}


/* ========================================================================== */

/* Checks for all adjacent cells (8 checks) and tags with reachable if not an 
   obstacle or outside array */
int indiv_reachable_tagger(robot_world_t *world, int x, int y) {
    int right_x = x + 1;
    int left_x = x - 1; 
    int up_y = y + 1;
    int down_y = y - 1;

    int changes = 0;

    /* flags for edges */
    int right_edge = edge_detect(world, x, y, RIGHT);
    int left_edge = edge_detect(world, x, y, LEFT);
    int top_edge = edge_detect(world, x, y, TOP);
    int bottom_edge = edge_detect(world, x, y, BOTTOM);

    /* check to the right */
    if (!right_edge && world->coords_type[y][right_x] != OBSTACLE && 
            world->coords_type[y][right_x] != REACHABLE) {

        world->coords_type[y][right_x] = REACHABLE;
        changes++;

    }

    /* checks to the left */
    if (!left_edge && world->coords_type[y][left_x] != OBSTACLE &&
            world->coords_type[y][left_x] != REACHABLE) {

        world->coords_type[y][left_x] = REACHABLE;
        changes++;

    }

    /* checks above */
    if (!top_edge && world->coords_type[up_y][x] != OBSTACLE &&
            world->coords_type[up_y][x] != REACHABLE) {

        world->coords_type[up_y][x] = REACHABLE;
        changes++;

    }

    /* checks below */
    if (!bottom_edge && world->coords_type[down_y][x] != OBSTACLE && 
            world->coords_type[down_y][x] != REACHABLE) {

        world->coords_type[down_y][x] = REACHABLE;
        changes++;

    }

    return changes;

}


/* ========================================================================== */

/* checks if the inputted coordinate is the edge of the world */

int edge_detect(robot_world_t *world, int x, int y, int type) {

    /* right edge detection */
    if (type == RIGHT) {
        return x == world->n_cols - 1;
    }
    

    /* left edge detection */
    if (type == LEFT) {
        return x == FIRST_X;
    }

    /* top edge detection */
    if (type == TOP) {
        return y == world->n_rows - 1;
    }

    /* bottom edge detection */
    if (type == BOTTOM) {
        return y == FIRST_Y;

    }

    return 0;

}

/* ========================================================================== */


// int full_obstacle_check(robot_world_t world, int y, int x) {
//     /* coordinate to either side of x and y */
//     int right_x = x + 1;
//     int left_x = x - 1;
//     int up_y = y + 1;
//     int down_y = y - 1;

    
//     for (int i = 0; i < world.n_obstas; i++) {
//         /* checks for obstacle to the right */
//         indiv_obstacle_check(world, y, x, i, RIGHT);
        
//         /* checks for obstacle to the left */


//         /* checks for obstacle above */


        
        
//     }
//     return 0;
    
// }


/* ========================================================================== */

// int indiv_obstacle_check(robot_world_t world, int y, int x, int obsta, int type) {

//     if (type == RIGHT) {
//         return (x + 1 == world.obstacles[obsta][X_MIN] && 
//                 (y >= world.obstacles[obsta][Y_MIN] && 
//                 y <= world.obstacles[obsta][Y_MAX]));

//     } else if (type == LEFT) {
//         return (x - 1 == world.obstacles[obsta][X_MAX] && 
//                 (y >= world.obstacles[obsta][Y_MIN] && 
//                 y <= world.obstacles[obsta][Y_MAX]));


//     } else if (type == TOP) {
//         return (y + 1 == world.obstacles[obsta][Y_MIN] && 
//                 (x >= world.obstacles[obsta][X_MIN] && 
//                 x <= world.obstacles[obsta][X_MAX]));

//     } else if (type == BOTTOM) {
//         return (y - 1 == world.obstacles[obsta][Y_MAX] && 
//                 (x >= world.obstacles[obsta][X_MIN] && 
//                 x <= world.obstacles[obsta][X_MAX]));


//     }

// }


/* ========================================================================== */

// int edge_check(robot_world_t world, int row, int col) {

//     return (row == FIRST_Y || row == world.n_rows - 1 ||
//         col == FIRST_COL || col == world.n_cols - 1);


// }


/* ======================= Formating helper functions ======================= */

void print_stage(int stage) {

    printf("S%d, ", stage);

    return;
}


/* ========================================================================== */

void print_obstacle(robot_world_t world, int obst_num) {

    printf("obstacle %2d covers [%2d,%2d] x [%2d,%2d]\n", obst_num, 
        world.obstacles[obst_num][X_MIN], world.obstacles[obst_num][X_MAX],
        world.obstacles[obst_num][Y_MIN], world.obstacles[obst_num][Y_MAX]);

}

/* ========================================================================== */

void print_blank(void) {
    printf("\n");
}



/* ============================= Test functions ============================= */

void print_world(robot_world_t world) {

    for (int i = world.n_rows - 1; i >= -1; i--) {
        if (i != -1) {
            printf("%2d ", i);
        }

        for (int j = world.n_cols - 1; j >= 0; j--) {

            if (i == -1) {
                if (j == world.n_cols - 1) {
                    printf("\n");
                    printf("   ");
                }
                printf(" %2d", j);
            } else {
                printf("%3c", world.coords_type[i][j]);
            
            }
            
        }
        printf("\n");
    }

    printf("\n");
    printf("\n");


}