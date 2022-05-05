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
#define MAX_OBSTA MAX_COLS * MAX_ROWS
#define OBSTA_COLS 4
#define STAGE1 1
#define STAGE2 2
#define STAGE3 3
#define X_MIN 0
#define X_MAX 1
#define Y_MIN 2
#define Y_MAX 3


/* ------- Typedefs ------- */

typedef int cols_t[MAX_COLS];
typedef int one_obst_t[OBSTA_COLS];
typedef cols_t coords_t[MAX_ROWS];
typedef one_obst_t obsts_t[MAX_OBSTA];

typedef struct {
    coords_t coords;
    obsts_t obstacles;
    int n_rows, n_cols, n_obstas;
} robot_world_t;


/* -- Function prototypes - */
void read_data(robot_world_t *world, int max_rows, int max_cols);
void do_stage1(robot_world_t world, int stage);
void print_stage(int stage);
void print_blank(void);
void print_obstacle(robot_world_t world, int obst_num);


/* ============================== Main function ============================= */

int main(int argc, char *argv[]) {
    robot_world_t robot_world;
    read_data(&robot_world, MAX_ROWS, MAX_COLS);

    do_stage1(robot_world, STAGE1);


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



/* ========================================================================== */
/* ============================ Helper functions ============================ */
/* ========================================================================== */




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