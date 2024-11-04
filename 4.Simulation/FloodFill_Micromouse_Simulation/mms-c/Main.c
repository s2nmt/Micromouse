#include "Main.h"

#include "API.h"
#include "queue.h"
#include "queue_int.h"
#include "stack.h"
#include <stdlib.h>
void log_out(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}
void intToStr(int num, char *str) {
    sprintf(str, "%d", num);
}
char* intToStrver(int num) {
    // Cấp phát bộ nhớ cho chuỗi (đủ để chứa số lớn nhất có thể và ký tự '\0')
    char *str = (char*)malloc(12 * sizeof(char));  // đủ cho số nguyên 32-bit
    if (str != NULL) {
        sprintf(str, "%d", num);  // Chuyển đổi số thành chuỗi
    }
    return str;
}

void init_arr(int arr[ROW][COL], int row, int col) {
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < col; j++) {
            arr[i][j] = -1;
        }
    }
}

wall_maze maze;
Queue myQueue;



void update_wall_debug(int (* arr)[ROW][COL])
{
    char dir;
    bool clear_=0;
    for(int i= 0;i<16;i++)
    {
        for(int j = 0;j<16;j++)
        {
            char value[20];
            intToStr((*arr)[i][j], value);

            for(int k = 0;k<4;k++)
            {
                clear_=maze.cells[i][j].walls[k];
              
                if(k==0)dir='n';
                else if(k==1)dir='s';
                else if(k==2)dir='w';
                else dir='e';
                if(clear_)API_setWall(i,j,dir);
                else API_clearWall(i,j,dir);
                
            }
            if(maze.cells[i][j].visited==true)
            {
                API_setColor(i,j,'g'); //g-green r-red b-Blue
                
            }
            else API_clearColor(i,j); //g-green r-red b-Blue
            if(maze.cells[i][j].dead==true)
            {
                API_setText(i,j,"Dead"); //g-green r-red b-Blue
                API_setColor(i,j,'r'); //g-green r-red b-Blue
           
            }
            else API_setText(i,j,value);
        }
    }
}
void check_and_fill(int arr[ROW][COL],int row,int col,int value)
{
    if(row<0 ||col<0||row>=16||col>=16||arr[row][col]!=-1)return;
    value+=1;
    coord point={row,col,value};
    pushQueue(&myQueue,point);
    arr[row][col]=value;
}
void init_flood(int arr[ROW][COL],int row,int col)
{
    int count_=0;
    coord point={row,col,count_};
    pushQueue(&myQueue,point);
    arr[row][col]=0;
    coord point2={row+1,col,count_};
    pushQueue(&myQueue,point2);
    arr[row+1][col]=0;
    coord point3={row,col+1,count_};
    pushQueue(&myQueue,point3);
    arr[row][col+1]=0;
    coord point4={row+1,col+1,count_};
    pushQueue(&myQueue,point4);
    arr[row+1][col+1]=0;
    while (!isEmptyQueue(&myQueue)) {
        
        coord frontCoord = peekQueue(&myQueue); 
        // printQueue(&myQueue);
        popQueue(&myQueue); 
        
        check_and_fill(arr,frontCoord.row+1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row-1,frontCoord.col,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col+1,frontCoord.value);
        check_and_fill(arr,frontCoord.row,frontCoord.col-1,frontCoord.value);
    }
}
void init_maze()
{
    for(int i =0;i<16;i++)
    {
        for(int j=0;j<16;j++)
        {
            maze.cells[i][j].visited=0;
            maze.cells[i][j].angle_update=90;
            maze.cells[i][j].dead=0;
            for(int k = 0 ;k<4;k++)maze.cells[i][j].walls[k]=0;
        }
    }
}
int main(int argc, char* argv[]) {
    log_out("Running...");
    initializeQueue(&myQueue);
    int arr[ROW][COL];
    init_arr(arr,ROW,COL);
    init_flood(arr,7,7);
    init_maze();
    API_setColor(0, 0, 'r');
    API_setColor(7, 7, 'r');

    API_setText(0, 0, "Start");
    API_setText(7, 7, "Goal");
    coord start={0,0,arr[0][0]};
    coord dest={7,7,arr[7][7]};

    update_wall_debug(&arr);
    int angle_now=90;
    coord new_coord;
    log_out(intToStrver(API_wallFront()));
    log_out(intToStrver(API_wallLeft()));
    log_out(intToStrver(API_wallRight()));

    API_moveForward();
    log_out(intToStrver(API_wallFront()));
    log_out(intToStrver(API_wallLeft()));
    log_out(intToStrver(API_wallRight()));
    API_moveForward();
    log_out(intToStrver(API_wallFront()));
    log_out(intToStrver(API_wallLeft()));
    log_out(intToStrver(API_wallRight()));
    API_turnRight();
    log_out(intToStrver(API_wallFront()));
    log_out(intToStrver(API_wallLeft()));
    log_out(intToStrver(API_wallRight()));
}
