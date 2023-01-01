/*
 * File:              new_robot_controller.hpp
 * Date:              20/08/2022
 * Author:            Aleksander Kalcina
 * Modifications:
 * Platform:          Windows
 * Notes:             header file for new_robot_controller
 */
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <iomanip>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits>
#include <cmath>
#include <string>
#include <webots/GPS.hpp>
#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>
#include <cmath>
#include <webots/Display.hpp>
#include <webots/Lidar.hpp>
#include <webots/Camera.hpp>
#include <webots/Keyboard.hpp>


#define M 5
#define N 9

#define UP_KEY 'W'
#define DOW_KEY 'S'
#define RIG_KEY 'D'
#define LEF_KEY 'A'


using namespace std;
using namespace webots;
static const double BaseSpeed = 2.0;


int HorizontalWall[6][9];
int VerticalWall[5][10];
int final;
using namespace std;

/* The following functions are used for the A Star algorithm */
class Node {
public:
    int x, y;
    int f, g, h;

    Node(int a, int b) {
        this->x = a;
        this->y = b;
    }

    bool operator<(const Node &a) const {
        return this->f > a.f;
    }
};

int addr[M][N];
bool visit[N][N];
int path[N][N][2];
int realF[N][N];
int att[4][2] = {
        {-1, 0},
        {0,  -1},
        {0,  1},
        {1,  0},
};
priority_queue<Node> que;

void PrintPath(int a, int b);
void AStar(int x0, int y0, int x1, int y1);
bool NodeIsLegal(int x, int y, int x0, int y0);
bool OK(int x, int y, int i);
int Mahuattan(int x, int y, int x1, int y1);
void PrintPath(int i, int i1, int i2, int i3);


class MazeRobot : public Robot {
public:
    MazeRobot();

    void Maze_control_add_cv();
    void Maze_control_for_known_end();
    void Manual_Control();

private:
    int timeStep;
    Motor *motors[4];
    Camera *camera[4];
    int Maze_Switch_word;
    int detect_floor;
    GPS *Gps;
    Display *display;
    Lidar *lid;
    Keyboard *keyboard;
};