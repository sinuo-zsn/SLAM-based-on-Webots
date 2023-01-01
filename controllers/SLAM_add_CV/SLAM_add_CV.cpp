/*
 * File:              SLAM_add_CV.cpp
 * Date:              18/08/2022
 * Author:            Sinuo Zhao
 * Modifications:
 * Platform:          Windows
 * Notes:             for PhaseD capability part
 * reference:         for A star algorithm
 *                    http://t.csdn.cn/G4nIm
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

#define M 5
#define N 9

using namespace std;
using namespace webots;
static const double BaseSpeed = 2.0;

int HorizontalWall[6][9];
int VerticalWall[5][10];
int final;


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
int Mahuattan(int x, int y, int x1, int y1);
void PrintPath(int i, int i1, int i2, int i3);

void AStar(int x0, int y0, int x1, int y1) {
    Node node(x0, y0);
    node.g = 0;
    node.h = Mahuattan(x0, y0, x1, y1);
    node.f = node.g + node.h;
    realF[x0][y0] = node.f;
    que.push(node);
    while (!que.empty()) {
        Node node_top = que.top();
        visit[node_top.x][node_top.y] = true;
        if (node_top.x == x1 && node_top.y == y1) {
            break;
        }
        que.pop();
        for (int i = 0; i < 4; i++) {
            Node node_next(node_top.x + att[i][0], node_top.y + att[i][1]);
            if (NodeIsLegal(node_next.x, node_next.y, node_top.x, node_top.y)) {
                node_next.g = node_top.g + int(10 * (sqrt(pow(att[i][0], 2) + pow(att[i][i], 2))));
                node_next.h = Mahuattan(node_next.x, node_next.y, x1, y1);
                node_next.f = node_next.g + node_next.h;
                if (node_next.f < realF[node_next.x][node_next.y] || !visit[node_next.x][node_next.y]) {
                    realF[node_next.x][node_next.y] = node_next.f;
                    path[node_next.x][node_next.y][0] = node_top.x;
                    path[node_next.x][node_next.y][1] = node_top.y;
                    que.push(node_next);
                }
            }
        }
    }
}
void PrintPath(int x1, int y1, int x0, int y0) {
    if (path[x1][y1][0] == -1 || path[x1][y1][1] == -1) {
        final = 999;

    }


    int x = x1, y = y1;
    int a, b;
    while (x != -1 && y != -1) {
        addr[x][y] = 2;
        a = path[x][y][0];
        b = path[x][y][1];
        if (a == x0 && b == y0) {
            final = x * 10 + y;
        }
        x = a;
        y = b;
    }

}
bool OK(int x, int y, int i) {
    if (i == 1) {
        if (VerticalWall[x][y + 1] == 0) {
            return true;
        } else {
            return false;
        }
    } else if (i == 0) {
        if (VerticalWall[x][y] == 0) {
            return true;
        } else {
            return false;
        }
    } else if (i == 2) {
        if (HorizontalWall[x][y] == 0) {
            return true;
        } else {
            return false;
        }
    } else if (i == 3) {
        if (HorizontalWall[x + 1][y] == 0) {
            return true;
        } else {
            return false;
        }
    } else {
        //the end grid
        return true;
    }
}
bool NodeIsLegal(int x0, int y0, int x1, int y1) {
    if (x0 == x1 && y0 == y1) return true;

    //边界判断
    if (x0 < 0 || y0 < 0 || x0 > M - 1 || y0 > N - 1) return false;

    int direction;
    if (x1 - x0 == 0 && y1 - y0 == 1) {
        direction = 0;
    } else if (x1 - x0 == 0 && y1 - y0 == -1) {
        direction = 1;
    } else if (x1 - x0 == 1 && y1 - y0 == 0) {
        direction = 2;
    } else { direction = 3; }
    //障碍物判断
    if (!OK(x1, y1, direction)) return false;

    return true;
}
int Mahuattan(int x, int y, int x1, int y1) {
    return pow(x1 - x, 2) + pow(y1 - y, 2);
}//

/* The following functions are used for search all map */
int VisitMap[M][N];
int find_end();
int find_end() {
    int x1, y1;
    for (int i = 0; i <= 4; i++) {
        for (int j = 0; j < 9; j++) {
            if (VisitMap[i][j] == 0) {
                x1 = i;
                y1 = j;
                return x1 * 10 + y1;
            }
        }
    }
    return 404;
}

/* initialization for main function MazeRobot*/
class MazeRobot : public Robot {
public:
    MazeRobot();

    void Maze_control();

private:
    int timeStep;
    Motor *motors[4];
    Camera *camera[4];
    int Maze_Switch_word;
    int detect_floor;
    GPS *Gps;
    Display *display;
    Lidar *lid;
};


MazeRobot::MazeRobot() {

    timeStep = 64;//64ms
    motors[0] = getMotor("wheel1");
    motors[1] = getMotor("wheel2");
    motors[2] = getMotor("wheel3");
    motors[3] = getMotor("wheel4");
    motors[0]->setPosition(std::numeric_limits<double>::infinity());
    motors[1]->setPosition(std::numeric_limits<double>::infinity());
    motors[2]->setPosition(std::numeric_limits<double>::infinity());
    motors[3]->setPosition(std::numeric_limits<double>::infinity());
    motors[0]->setVelocity(0.0);
    motors[1]->setVelocity(0.0);
    motors[2]->setVelocity(0.0);
    motors[3]->setVelocity(0.0);

    camera[0] = getCamera("camera");//>
    camera[1] = getCamera("camera(1)");//^
    camera[2] = getCamera("camera(2)");//<
    camera[3] = getCamera("camera(3)");//V


    camera[0]->enable(timeStep);
    camera[1]->enable(timeStep);
    camera[2]->enable(timeStep);
    camera[3]->enable(timeStep);
    camera[0]->recognitionEnable(timeStep);
    camera[1]->recognitionEnable(timeStep);
    camera[2]->recognitionEnable(timeStep);
    camera[3]->recognitionEnable(timeStep);

    Gps = getGPS("gps");
    Gps->enable(timeStep);

    lid = getLidar("lidar");
    lid->enable(timeStep);
    lid->enablePointCloud();

    display = getDisplay("display");

    Maze_Switch_word = 0;
    detect_floor = 9;
}


void MazeRobot::Maze_control() {
    const double *g = Gps->getValues();
    int time = 0;
    int x0 = 0, y0 = 0;
    fill(VisitMap[0], VisitMap[0] + M * N, 0);
    VisitMap[0][0] = -1;
    int m[200][280];

    while (find_end() != 404) {
        int x1 = ceil(find_end() / 10);
        int y1 = find_end() - x1 * 10;
        detect_floor = 9;
        Maze_Switch_word = 0;
        while (step(timeStep) != -1) {
            if (Maze_Switch_word == 8) {
                break;
            }
            switch (Maze_Switch_word) {
                // draw the map use display
                case 0: {
                    const double *g = Gps->getValues();
                    const LidarPoint *n = lid->getLayerPointCloud(0);
                    for (int i = 1; i < 1024; i++) {
                        if (sqrt((n[i].x - n[i - 1].x) * (n[i].x - n[i - 1].x) +
                                 (n[i].z - n[i - 1].z) * (n[i].z - n[i - 1].z)) < 0.008)//滤波，把干扰信号过滤掉
                        {
                            int x = int((g[0] + n[i].z) * 122.5 + 91);
                            int y = int((g[2] - n[i].x) * 122.5 + 50);
                            m[x][y] = 1;
                            display->drawPixel(int((g[0] + n[i].z) * 122.5 + 90), int((g[2] - n[i].x) * 122.5 + 50));
                        }
                    }
                    Maze_Switch_word++;
                    break;
                }
                // detect the wall from display
                // get the HorizontalWall and VerticalWall
                case 1: {
                    int n = 0;
                    //V wall
                    for (int i = 0; i < 5; i++) {
                        for (int j = 0; j < 10; j++) {
                            n = 0;
                            for (int column = i * 20 + 3; column < (i + 1) * 20 - 3; column++) {
                                for (int row = j * 20 - 3; row < j * 20 + 3; row++) {
                                    if (m[row][column] == 1) {
                                        n++;
                                    }
                                }
                            }
                            if (n > 12) {
                                VerticalWall[i][j] = 1;
                            } else {
                                VerticalWall[i][j] = 0;
                            }
                        }
                    }

                    //H wall
                    for (int i = 0; i < 6; i++) {
                        for (int j = 0; j < 9; j++) {
                            n = 0;
                            for (int column = i * 20 - 3; column < i * 20 + 3; column++) {
                                for (int row = j * 20 + 3; row < (j + 1) * 20 - 3; row++) {
                                    if (m[row][column] == 1) {
                                        n++;

                                    }
                                }
                            }
                            if (n > 12) {
                                HorizontalWall[i][j] = 1;
                            } else {
                                HorizontalWall[i][j] = 0;
                            }
                        }
                    }
                    Maze_Switch_word = 2;
                    break;

                }
                // A Star algorithm
                case 2: {
                    fill(visit[0], visit[0] + N * N, false);
                    fill(path[0][0], path[0][0] + N * N * 2, -1);
                    fill(realF[0], realF[0] + N * N, 0);
                    fill(addr[0], addr[0] + M * N, 0);
                    while (!que.empty()) {
                        que.pop();
                    }

                    
                    AStar(x0, y0, x1, y1);
                    PrintPath(x1, y1, x0, y0);

                    Maze_Switch_word++;
                    break;

                }
                //judge direction
                case 3: {
                    //judge dirction
                    if (final == 999) {
                        Maze_Switch_word = 8;
                    }
                    if (final == x0 * 10 + y0 + 1) {
                        Maze_Switch_word = 4;
                        x0 = ceil(final / 10);
                        y0 = final - x0 * 10;

                    } else if (final == (x0 + 1) * 10 + y0) {
                        Maze_Switch_word = 5;
                        x0 = ceil(final / 10);
                        y0 = final - x0 * 10;

                    } else if (final == x0 * 10 + y0 - 1) {
                        Maze_Switch_word = 6;
                        x0 = ceil(final / 10);
                        y0 = final - x0 * 10;

                    } else if (final == (x0 - 1) * 10 + y0) {
                        Maze_Switch_word = 7;
                        x0 = ceil(final / 10);
                        y0 = final - x0 * 10;

                    } else if (final == x1 * 10 + y1) {
                        Maze_Switch_word = 8;
                    }

                    break;
                }
                // four movement case
                case 4: {
                    // >
                    VisitMap[x0][y0] = -1;
                    motors[0]->setVelocity(-BaseSpeed);
                    motors[1]->setVelocity(-BaseSpeed);
                    motors[2]->setVelocity(-BaseSpeed);
                    motors[3]->setVelocity(-BaseSpeed);
                    if (detect_floor == 9) {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {
                            Maze_Switch_word = 9;
                            time = 0;
                            break;
                        }
                    } else {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * (y0 + 1), 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {
                            while (1){
                                motors[0]->setVelocity(0);
                                motors[1]->setVelocity(0);
                                motors[2]->setVelocity(0);
                                motors[3]->setVelocity(0);
                                motors[0]->setPosition(0);
                                motors[1]->setPosition(0);
                                motors[2]->setPosition(0);
                                motors[3]->setPosition(0);
                            }
                        }

                    }
                    time++;
                    break;
                }
                case 5: {
                    // v
                    VisitMap[x0][y0] = -1;
                    motors[0]->setVelocity(BaseSpeed);
                    motors[1]->setVelocity(-BaseSpeed);
                    motors[2]->setVelocity(-BaseSpeed);
                    motors[3]->setVelocity(BaseSpeed);
                    if (detect_floor == 9) {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {
                            Maze_Switch_word = 9;
                            time = 0;
                            break;

                        }
                    } else {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * (x0 + 1), 2) < 0.0002) {
                            while (1){
                                motors[0]->setVelocity(0);
                                motors[1]->setVelocity(0);
                                motors[2]->setVelocity(0);
                                motors[3]->setVelocity(0);
                                motors[0]->setPosition(0);
                                motors[1]->setPosition(0);
                                motors[2]->setPosition(0);
                                motors[3]->setPosition(0);
                            }

                        }

                    }

                    time++;
                    break;
                }
                case 6: {
                    // <
                    VisitMap[x0][y0] = -1;
                    motors[0]->setVelocity(BaseSpeed);
                    motors[1]->setVelocity(BaseSpeed);
                    motors[2]->setVelocity(BaseSpeed);
                    motors[3]->setVelocity(BaseSpeed);
                    if (detect_floor == 9) {

                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {

                            Maze_Switch_word = 9;
                            time = 0;
                            break;

                        }
                    } else {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * (y0 - 1), 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {
                            while (1){
                                motors[0]->setVelocity(0);
                                motors[1]->setVelocity(0);
                                motors[2]->setVelocity(0);
                                motors[3]->setVelocity(0);
                                motors[0]->setPosition(0);
                                motors[1]->setPosition(0);
                                motors[2]->setPosition(0);
                                motors[3]->setPosition(0);
                            }
                        }

                    }
                    time++;
                    break;
                }
                case 7: {
                    // ^
                    VisitMap[x0][y0] = -1;
                    motors[0]->setVelocity(-BaseSpeed);
                    motors[1]->setVelocity(BaseSpeed);
                    motors[2]->setVelocity(BaseSpeed);
                    motors[3]->setVelocity(-BaseSpeed);
                    if (detect_floor == 9) {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * x0, 2) < 0.0002) {

                            Maze_Switch_word = 9;
                            time = 0;
                            break;

                        }
                    } else {
                        if (time >= 125 &&
                            pow(g[0] + 0.663 - 0.167 * y0, 2) + pow(g[2] + 0.326 - 0.167 * (x0 - 1), 2) < 0.0002) {
                            while (1){
                                motors[0]->setVelocity(0);
                                motors[1]->setVelocity(0);
                                motors[2]->setVelocity(0);
                                motors[3]->setVelocity(0);
                                motors[0]->setPosition(0);
                                motors[1]->setPosition(0);
                                motors[2]->setPosition(0);
                                motors[3]->setPosition(0);
                            }
                        }

                    }
                    time++;
                    break;
                }
                // stop case
                case 8: {
                    motors[0]->setVelocity(0);
                    motors[1]->setVelocity(0);
                    motors[2]->setVelocity(0);
                    motors[3]->setVelocity(0);
                    motors[0]->setPosition(0);
                    motors[1]->setPosition(0);
                    motors[2]->setPosition(0);
                    motors[3]->setPosition(0);
                    Maze_Switch_word = 10;
                    break;

                }
                // use camera detect landmark
                case 9: {
                    //detect floor
                    const CameraRecognitionObject *object0 = camera[0]->getRecognitionObjects();
                    const CameraRecognitionObject *object1 = camera[1]->getRecognitionObjects();
                    const CameraRecognitionObject *object2 = camera[2]->getRecognitionObjects();
                    const CameraRecognitionObject *object3 = camera[3]->getRecognitionObjects();

                    if (camera[0]->getRecognitionNumberOfObjects() == 1) {
                        if (object0[0].position[0] + object0[0].position[1] + object0[0].position[2] > -0.20 &&
                            object0[0].position[0] + object0[0].position[1] + object0[0].position[2] < -0.021
                                ) {
                            detect_floor = 0;
                        }
                    }

                    if (camera[1]->getRecognitionNumberOfObjects() == 1) {
                        if (object1[0].position[0] + object1[0].position[1] + object1[0].position[2] > -0.20 &&
                            object1[0].position[0] + object1[0].position[1] + object1[0].position[2] < -0.021) {
                            detect_floor = 1;
                        }
                    }

                    if (camera[2]->getRecognitionNumberOfObjects() == 1) {
                        if (object2[0].position[0] + object2[0].position[1] + object2[0].position[2] > -0.20 &&
                            object2[0].position[0] + object2[0].position[1] + object2[0].position[2] < -0.021) {
                            detect_floor = 2;
                        }
                    }
                    if (camera[3]->getRecognitionNumberOfObjects() == 1) {
                        if (object3[0].position[0] + object3[0].position[1] + object3[0].position[2] > -0.20 &&
                            object3[0].position[0] + object3[0].position[1] + object3[0].position[2] < -0.021) {
                            detect_floor = 3;
                        }
                    }
                    
                    if (detect_floor == 9) {
                        Maze_Switch_word = 0;
                        break;
                    } else if (detect_floor == 0) {
                        Maze_Switch_word = 4;
                        break;
                    } else if (detect_floor == 1) {
                        Maze_Switch_word = 7;
                        break;
                    } else if (detect_floor == 2) {
                        Maze_Switch_word = 6;
                        break;
                    } else if (detect_floor == 3) {
                        Maze_Switch_word = 5;
                        break;
                    }

                }
            }
        }
    }
    motors[0]->setVelocity(0);
    motors[1]->setVelocity(0);
    motors[2]->setVelocity(0);
    motors[3]->setVelocity(0);
    motors[0]->setPosition(0);
    motors[1]->setPosition(0);
    motors[2]->setPosition(0);
    motors[3]->setPosition(0);
}


int main() {
    MazeRobot *controller = new MazeRobot();
    controller->Maze_control();
    delete controller;
    return 0;
}

