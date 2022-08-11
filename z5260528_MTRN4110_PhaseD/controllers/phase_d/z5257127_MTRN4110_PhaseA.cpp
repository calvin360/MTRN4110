#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <fstream>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>

constexpr double maxMotorSpeed = 6.28;  // rad/s
constexpr double wheel_radius = 0.0205; // m
constexpr int num_distance_sensors = 3;
constexpr double obstacleDistance = 90.0;
#define TIME_STEP 64

const std::array<std::string, num_distance_sensors> distanceSensorNames = {"dsL", "dsF", "dsR"};

// Helper function to check for obstacles.

int mod(int a, int b)
{
    return (a % b + b) % b;
}

auto executeMove(char command, double leftMotor, double rightMotor, char bearing, int row, int col)
{
    // returns position for motors and updates rows, cols and heading
    int bearingINT;

    switch (bearing)
    {
    case 'N':
        bearingINT = 0;
        break;
    case 'E':
        bearingINT = 1;
        break;
    case 'S':
        bearingINT = 2;
        break;
    case 'W':
        bearingINT = 3;
        break;
    }
    // forward command
    if (command == 'F')
    {
        leftMotor = leftMotor + 2.62586 * M_PI;
        rightMotor = rightMotor + 2.62586 * M_PI;
        switch (bearing)
        {
        case 'N':
            row = row - 1;
            break;
        case 'E':
            col = col + 1;
            break;
        case 'S':
            row = row + 1;
            break;
        case 'W':
            col = col - 1;
            break;
        }
    }
    // left command
    else if (command == 'L')
    {
        leftMotor = leftMotor - 0.711175 * M_PI;
        rightMotor = rightMotor + 0.711175 * M_PI;
        bearingINT = mod((bearingINT - 1), 4);
    }
    // right command
    else if (command == 'R')
    {
        leftMotor = leftMotor + 0.711175 * M_PI;
        rightMotor = rightMotor - 0.711175 * M_PI;
        bearingINT = mod((bearingINT + 1), 4);
    }
    switch (bearingINT)
    {
    case 0:
        bearing = 'N';
        break;
    case 1:
        bearing = 'E';
        break;
    case 2:
        bearing = 'S';
        break;
    case 3:
        bearing = 'W';
        break;
    }
    struct motorPos
    {
        double leftMotor;
        double rightMotor;
        int row;
        int col;
        char bearing;
    };

    return motorPos{leftMotor, rightMotor, row, col, bearing};
}

bool checkWall(double distanceValue)
{
    if (distanceValue < 600)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void writeToFile(int step, int row, int col, char heading, bool left, bool front, bool right)
{
    std::string leftWall = "N";
    std::string rightWall = "N";
    std::string frontWall = "N";
    if (left == true)
    {
        leftWall = "Y";
    }
    if (right == true)
    {
        rightWall = "Y";
    }
    if (front == true)
    {
        frontWall = "Y";
    }

    std::ofstream file("..\\..\\MotionExecution.csv", std::ios::app);
    if (!file.is_open())
    {
        std::cerr << "Cannot write to file";
    }
    else
    {
        file << step << "," << row << "," << col << "," << heading << "," << leftWall << "," << frontWall << "," << rightWall << "\n";
        file.close();
    }

    std::cout << "[z5257127_MTRN4110_PhaseA] "
              << "Step: " << std::setw(3) << std::setfill('0') << step << ","
              << " Row: " << row << ","
              << " Column: " << col << ","
              << " Heading: "
              << heading << ","
              << " Left Wall: " << leftWall << ","
              << " Front Wall: " << frontWall << ","
              << " Right Wall: " << rightWall << std::endl;
    return;
}

// auto deadRec(int row, int col, char heading, double diffR, double diffL, char command) {
// transfer row col to cartesian
// double x = col * 16.5;
//  double y = -row * 16.5; //units in mm
//
// std::cout << "2: "<<currPoseR << std::endl;
// diffR = currPoseR - prevPosR;
// diffL = currPoseL - prevPosL;
// std::cout <<"3: "<< diffR << std::endl;
// changeS = ((0.020001509812471*diffL)/2) + ((0.020001509812471*diffR)/2);
// changeZ = ((0.020001509812471*diffL)/0.056888894253924) + ((0.020001509812471*diffR)/0.056888894253924);

//  x = x + changeS*cos(z+(changeZ/2));
// y = y + changeS*sin(z+(changeZ/2));
// z = z + changeZ;
// z = normalize(z) * (M_PI/180);
//

//
//}

int run()
{
    // Initialise robot.
    webots::Robot robot;
    // Get pointers to our robot's motors.
    std::unique_ptr<webots::Motor> leftMotor{robot.getMotor("left wheel motor")};
    std::unique_ptr<webots::Motor> rightMotor{robot.getMotor("right wheel motor")};
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    // from workshop code
    std::array<std::unique_ptr<webots::DistanceSensor>, num_distance_sensors> distanceSensors;
    for (int i = 0; i < num_distance_sensors; i++)
    {
        distanceSensors[i] = std::make_unique<webots::DistanceSensor>(*robot.getDistanceSensor(distanceSensorNames[i]));
        distanceSensors[i]->enable(TIME_STEP);
    }

    // Our control loop.
    // double leftSpeed, rightSpeed, leftPosition, rightPosition;
    // leftPosition = 0;
    // rightPosition = 0;

    //  int Bearing = 2;

    // int X = 0;
    // int Y = 0;

    // int step = 0;
    // int prevstep = -1;
    // Open command file and save to string variable
    std::string moveCommands;
    std::ifstream file("..\\..\\MotionPlan.txt");
    std::cout << "[z5257127_MTRN4110_PhaseA] Reading in motion plan from ../../MotionPlan.txt..." << std::endl;

    if (file.is_open())
    {
        while (std::getline(file, moveCommands))
        {
            // Output the text from the file
            std::cout << "[z5257127_MTRN4110_PhaseA] Motion Plan: " << moveCommands << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Cannot read file";
    }

    std::ofstream file2("..\\..\\MotionExecution.csv", std::ios::trunc);
    if (!file2.is_open())
    {
        std::cerr << "Cannot write to file";
    }
    else
    {
        file2 << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
        file2.close();
    }

    // 15.74311 for cells

    std::cout << "[z5257127_MTRN4110_PhaseA] Motion plan read in!" << std::endl;
    // Initial data from move file
    int numMoves = moveCommands.length();
    int step = 0;
    int row = moveCommands[0] - '0';
    int col = moveCommands[1] - '0';
    char heading = moveCommands[2];
    int moveCounter = 3;
    double currTime = 0;
    double prevTime = 0;
    double leftPos = 0;
    double rightPos = 0;
    std::cout << "[z5257127_MTRN4110_PhaseA] Executing motion plan..." << std::endl;

    while (robot.step(TIME_STEP) != -1)
    {
        // std::cout << robot.getTime() << std::endl;
        // Get average values for distance sensors
        int aveLeft = 0;
        int aveRight = 0;
        int aveFront = 0;
        int sum = 0;
        currTime = robot.getTime();
        robot.step(1);
        while (robot.getTime() < currTime + 3)
        {
            robot.step(1);
            aveLeft = distanceSensors[0]->getValue() + aveLeft;
            aveFront = distanceSensors[1]->getValue() + aveFront;
            aveRight = distanceSensors[2]->getValue() + aveRight;
            sum++;
        }
        aveLeft = aveLeft / sum;
        aveFront = aveFront / sum;
        aveRight = aveRight / sum;
        // write to file
        writeToFile(step, row, col, heading, checkWall(aveLeft), checkWall(aveFront), checkWall(aveRight));

        if (moveCounter < numMoves)
        {

            auto motorPos = executeMove(moveCommands[moveCounter], leftPos, rightPos, heading, row, col);
            leftMotor->setVelocity(0.15 * maxMotorSpeed);
            rightMotor->setVelocity(0.15 * maxMotorSpeed);

            leftMotor->setPosition(motorPos.leftMotor);
            rightMotor->setPosition(motorPos.rightMotor);
            prevTime = robot.getTime();
            robot.step(1);
            while (robot.getTime() < prevTime + 8)
            {
                robot.step(1);
            }
            leftPos = motorPos.leftMotor;
            rightPos = motorPos.rightMotor;
            row = motorPos.row;
            col = motorPos.col;
            heading = motorPos.bearing;
            step++;
            moveCounter++;
        }
        else
        {
            std::cout << "[z5257127_MTRN4110_PhaseA] Motion plan executed!" << std::endl;
            exit(0);
        }
    }

    return 0;
}