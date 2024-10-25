#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>


#define UP 0
#define	DOWN 1
#define	LEFT 2
#define	RIGHT 3


#define ROW 16
#define COL 16

#define MAX_SIZE 1000

typedef struct {
    int row;
    int col;
    int value;
}coord;

typedef struct cell_infos{
	// variables for north,east,south,west walls
	bool walls[4];
	bool visited;
    int angle_update;
    bool dead;
}cell_info;
typedef struct wall_mazes{
	cell_info cells[16][16];
}wall_maze;
#endif // MAIN_H