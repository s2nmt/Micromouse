#include "Main.h"

#include "API.h"
#include "queue.h"
#include "queue_int.h"
#include "stack.h"
#include <stdlib.h>
const  int rows=16;
const  int cols=16;

void log_out(char* text) {
    fprintf(stderr, "%s\n", text);
    fflush(stderr);
}
bool isValid(int x, int y) {
    return (x >= 0 && x < rows && y >= 0 && y < cols);
}
char* intToStrver(int num) {
  
    char *str = (char*)malloc(12 * sizeof(char));  
    if (str != NULL) {
        sprintf(str, "%d", num);  
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
const int dx[] = {1, -1, 0, 0};
const int dy[] = {0, 0, -1, 1};

wall_maze maze;
Queue myQueue;
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
        printf("%d%d%d\n",frontCoord.row,frontCoord.col,frontCoord.value);
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
void intToStr(int num, char *str) {
    sprintf(str, "%d", num);
}
void update_wall_debug(int (* arr)[ROW][COL])
{
    char dir;
    bool clear_=0;
    int count_=0;
    for(int i= 0;i<16;i++)
    {
        for(int j = 0;j<16;j++)
        {
            char value[20];
            intToStr((*arr)[i][j], value);
            count_ ++;
            printf("counter: %d value: %s\n",count_, value);
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
                // log("blue");
                API_setColor(i,j,'g'); //g-green r-red b-Blue
                
            }
            else API_clearColor(i,j); //g-green r-red b-Blue
            if(maze.cells[i][j].dead==true)
            {
                API_setText(i,j,"Dead"); //g-green r-red b-Blue
                API_setColor(i,j,'r'); //g-green r-red b-Blue
           
            }
            // value+="-R:"+std::to_string(i)+"-C:"+std::to_string(j);
            else API_setText(i,j,value);
        }
    }
}
cell_info cell_direction_adjust(cell_info cell)
{
    cell_info cell_new;
    cell_new=cell;

    for(int i=0;i<4;i++)
    {
        int ind = i;

        switch(cell.angle_update)
        {
            case 90:
                break;
            case 270:
                if(i%2==0)ind+=1;
                else ind-=1;
                break;
            case 0:
                if(i==0 || i ==1)ind+=2;
                else if(i==2)ind=1;
                else ind=0;
                break;
            case 180:
                if(i==2 || i ==3)ind-=2;
                else if(i==0)ind=3;
                else ind=2;
                break;
        }
        cell_new.walls[i]=cell.walls[ind]; 
    }  
    return cell_new;
}

cell_info update_walls(int angle_now,int row,int col)
{
    cell_info new_cell;
    new_cell.angle_update=angle_now;
    new_cell.walls[UP]=API_wallFront();
    new_cell.walls[DOWN]=0;
    new_cell.walls[LEFT]=API_wallLeft();
    new_cell.walls[RIGHT]=API_wallRight();
    new_cell.dead=0;
    new_cell.visited=1;
    maze.cells[row][col]=cell_direction_adjust(new_cell);
    if(new_cell.walls[UP]==1&&new_cell.walls[LEFT]==1&&new_cell.walls[RIGHT]==1&&row!=0&&col!=0)
    {
        log_out("dead");
        maze.cells[row][col].dead=1;
    }
    for(int i=0;i<4;i++)
    {
        int newRow=row+dy[i]; // 0 0 -1 1
        int newCol=col+dx[i]; // 1 -1 0 0
        if(isValid(newRow,newCol))
        {
            if(i==UP)maze.cells[newRow][newCol].walls[DOWN]=maze.cells[row][col].walls[UP];
            else if(i==LEFT)maze.cells[newRow][newCol].walls[RIGHT]=maze.cells[row][col].walls[LEFT];
            else if(i==RIGHT)maze.cells[newRow][newCol].walls[LEFT]=maze.cells[row][col].walls[RIGHT];
        }
    }
    return new_cell;
}
void go_to_cell(int *angle_now,int dir)
{
    switch(dir)
            {
                case -1:
                    log_out("not dir");
                    break;
                case UP:
                    log_out("forward");
                    API_moveForward();
                    break;
                case DOWN:
                    log_out("Down");
                    *angle_now-=180;
                    API_turnRight();
                    API_turnRight();
                    API_moveForward();
                    break;
                case LEFT:
                    log_out("Left");
                    *angle_now+=90;
                    API_turnLeft();
                    API_moveForward();
                    break;
                case RIGHT:
                    log_out("right");
                    *angle_now-=90;
                    API_turnRight();
                    API_moveForward();
                    break;
                default:
                    break;
            }
            *angle_now = *angle_now % 360;
            // Đảm bảo góc không bị âm
            if (*angle_now < 0) {
                *angle_now += 360;
            }
}

bool check_wall_angle(cell_info cell,int *dir)
{
    switch(cell.angle_update)
    {
        case 90:
            break;
        case 270:
            if(*dir%2==0)*dir+=1;
            else *dir-=1;
            break;
        case 0:
            if(*dir==0 || *dir ==1)*dir+=2;
            else if(*dir==2)*dir=1;
            else *dir=0;
            break;
        case 180:
             if(*dir==2 || *dir ==3)*dir-=2;
            else if(*dir==0)*dir=3;
            else *dir=2;
            break;
    }
    return cell.walls[*dir];
}

coord get_min_neighbour(cell_info cell_wall,coord cur, int (*arr)[ROW][COL],bool change_)
{
    int min_neightbor=255;
    coord next_step;
    next_step.value=-1;
    int ind;
    for (int dir = 0; dir < 4; ++dir) {
        int newRow = cur.row + dy[dir]; // 0 0 -1 1
        int newCol = cur.col + dx[dir]; //1 -1 0 0 
        ind=dir;
        bool check_=cell_wall.walls[dir];
        if(change_)check_=check_wall_angle(cell_wall,&ind);
        fprintf(stderr, "%d", check_);
        fflush(stderr);

        if(isValid(newRow,newCol) && !check_)
        { 
            if((*arr)[newRow][newCol]<=min_neightbor)
            { 
                min_neightbor=(*arr)[newRow][newCol];
                next_step.row=newRow;
                next_step.col=newCol;
                next_step.value=ind;
            }
        }
    }
    // fprintf(stderr, "\n");
    // fflush(stderr);
    return next_step;
}

void flood(Stack *stack_flood,int (*arr)[ROW][COL])
{
    // log_out("flood");
    coord cur_stack;
    coord next_step;

    while(!isEmptyStack(stack_flood))
    {

        cur_stack=peekStack(stack_flood);
        popStack(stack_flood); 
        int min_neightbor=255;
        bool check_;

        next_step=get_min_neighbour(maze.cells[cur_stack.row][cur_stack.col],cur_stack,arr,0);

        min_neightbor=(*arr)[next_step.row][next_step.col];
        // fprintf(stderr, "next_step.row: %d next_step.col: %d min_neightbor: %d min_neightbor[1][2]: %d *arr[cur_stack.row][cur_stack.col]-1: %d\n",next_step.row,next_step.col, min_neightbor,*arr[1][2],*arr[cur_stack.row][cur_stack.col]-1);
        // fflush(stderr);

        if((*arr)[cur_stack.row][cur_stack.col]-1 != min_neightbor )
        {
            for(int i =0 ;i<4;i++)
            {
                coord cur_add;
                cur_add.row= cur_stack.row + dy[i]; // 0 0 -1 1
                cur_add.col= cur_stack.col + dx[i]; //1 -1 0 0 
                check_=maze.cells[cur_stack.row][cur_stack.col].walls[i];
                if(isValid(cur_add.row,cur_add.col) &&(*arr)[cur_add.row][cur_add.col]!=0&&!check_)
                {
                    pushStack(stack_flood,cur_add);
                }
            }
            if((*arr)[cur_stack.row][cur_stack.col]!=0)(*arr)[cur_stack.row][cur_stack.col]=min_neightbor+1;
        }
        int stack_size=sizeStack(stack_flood);

    
        if(stack_size>=35){
            log_out("full stack");
            for(int i=0;i<stack_size;i++)
            {
                popStack(stack_flood);
            }
            return;
        }
    }

}
coord floodfill(coord start,coord dest,int (*arr)[ROW][COL],int *angle_now)
{
    Queue path_queue;
    initializeQueue(&path_queue);

    pushQueue(&path_queue,start);
    coord cur=start;
    cell_info new_cell;

    Stack stack_flood;
    initializeStack(&stack_flood);

    pushStack(&stack_flood,start);

    int path_distance_value_find=0;
    coord next_step;

    while(1)
    {
        if(!isEmptyQueue(&path_queue)) // dua ra quyet dinh va go
        {
            cur = peekQueue(&path_queue);

            new_cell=update_walls(*angle_now,cur.row,cur.col);
            // fprintf(stderr, "new_cell: %d%d%d\n",*angle_now, cur.row,cur.col);

            // fflush(stderr);

            if((*arr)[cur.row][cur.col]==(*arr)[dest.row][dest.col]){
                log_out("find dest");
                break;
            }
            flood(&stack_flood,arr);
            popQueue(&path_queue); 
            // log_out("floodfill");
            next_step=get_min_neighbour(new_cell,cur,arr,1);
            pushQueue(&path_queue,next_step);
            pushStack(&stack_flood,next_step);
            go_to_cell(angle_now,next_step.value);    
            path_distance_value_find++; 
        }
        else{
            log_out("empty Queue- break");
            break;
        }
        update_wall_debug(arr);
    }

    while(!isEmptyQueue(&path_queue)) popQueue(&path_queue); 
    
    printf("total_cost: %d\n",path_distance_value_find);
    coord p_return={next_step.row,next_step.col,0};
    return p_return;
}   
void init_flood_start(int (*arr)[ROW][COL],int row_,int col_,int back_)
{
    int count_=0;
    for(int i=0;i<16;i++)
    {
        for(int j = 0 ;j<16;j++)
        {
            (*arr)[i][j]=-1;
            if(back_==2&&maze.cells[i][j].visited==false){
                (*arr)[i][j]=255;
                maze.cells[i][j].dead=true;
            }
           
        }
    }
    if(back_!=1)
    {
        coord point2={row_+1,col_,count_};
        pushQueue(&myQueue,point2);
        (*arr)[row_+1][col_]=0;
        coord point3={row_,col_+1,count_};
        pushQueue(&myQueue,point3);
        (*arr)[row_][col_+1]=0;
        coord point4={row_+1,col_+1,count_};
        pushQueue(&myQueue,point4);
        (*arr)[row_+1][col_+1]=0;
    }
    coord point={row_,col_,count_};
    pushQueue(&myQueue,point);
    (*arr)[row_][col_]=0;
    while(!isEmptyQueue(&myQueue))
    {
        coord frontCoord = peekQueue(&myQueue); 
        popQueue(&myQueue); 
          for (int i = 0; i < 4; ++i) {
                int newRow = frontCoord.row + dy[i]; // 0 0 -1 1
                int newCol = frontCoord.col + dx[i]; //1 -1 0 0 
                bool check_=maze.cells[frontCoord.row][frontCoord.col].walls[i];
                if(!check_)check_and_fill(*arr,newRow,newCol,frontCoord.value);
          }
          if(sizeQueue(&myQueue)>120){
            log_out("fulllll");
            break;
          }
    } 
}
void shorted_path_go(int (*arr)[ROW][COL],int angle_now,coord start,coord dest)
{
   
    QueueInt next_dir_path;
    initializeQueueInt(&next_dir_path);

    // cell_info new_cell;

    int save_row,save_col;
    coord cur=start;
    // int angle=angle_now;
    for(int i=0;i<(*arr)[start.row][start.col];i++)
    {
            int next_dir=-1;
            int newRow;
            int newCol;
            for (int dir = 0; dir < 4; ++dir) {
                newRow = cur.row + dy[dir]; // 0 0 -1 1
                newCol = cur.col + dx[dir]; //1 -1 0 0 
                bool check_=maze.cells[cur.row][cur.col].walls[dir];
                if(isValid(newRow,newCol) && !check_)
                {
                    if((*arr)[newRow][newCol]<(*arr)[cur.row][cur.col])
                    { 
                        next_dir=dir;
                        save_row=newRow;
                        save_col=newCol;
                    }
                }
            }
            if(next_dir!=-1)
            {
                cur.row=save_row;
                cur.col=save_col;
                pushQueueInt(&next_dir_path,next_dir);
                // next_dir_path.push(next_dir);
                char value[20];
                intToStr((*arr)[save_row][save_col], value);
                API_setColor(save_row,save_col,'g');
                API_setText(save_row,save_col,value);
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

    coord start={0,0,arr[0][0]};
    coord dest={7,7,arr[7][7]};

    API_setColor(0, 0, 'r');
    API_setColor(7, 7, 'r');

    API_setText(0, 0, "Start");
    API_setText(7, 7, "Goal");

    update_wall_debug(&arr);
    int angle_now=90;
    coord new_coord;
    new_coord = floodfill(start,dest,&arr,&angle_now);
    // update_wall_debug(&arr);
    // init_flood_start(&arr,0,0,1);
    // log_out("done2");
    // new_coord=floodfill(new_coord,start,&arr,&angle_now);
    // init_flood_start(&arr,7,7,2);
    // shorted_path_go(&arr,angle_now,new_coord,dest);
    return 0;
}