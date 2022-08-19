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
#include "helperA.hpp"

constexpr double maxMotorSpeed = 6.28;  // rad/s
constexpr double wheel_radius = 0.0205; // m
const double oneSquare = 2.62586;
const double quarter = 0.711175;
constexpr int num_distance_sensors = 3;
constexpr double obstacleDistance = 90.0;
const std::string MOTION_PLAN_FILE_NAME = "../../PathPlan.txt";
// const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";
const int TIME_STEP = 64;

const std::array<std::string, num_distance_sensors>
    distanceSensorNames = {"dsL", "dsF", "dsR"};

class myRobot : public webots::Robot
{
public:
    myRobot();
    webots::Motor *leftMotor;
    webots::Motor *rightMotor;
    std::string moveCommands;
    std::array<webots::DistanceSensor *, num_distance_sensors> distanceSensors;
    int numMoves = moveCommands.length();
    int stepC;
    int row;
    int col;
    char heading;
    int moveCounter;
    double currTime;
    double prevTime;
    double leftPos;
    double rightPos;

private:
};

myRobot::myRobot()
{
    // Get pointers to our robot's motors.
    leftMotor = getMotor("left wheel motor");
    rightMotor = getMotor("right wheel motor");
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    // from workshop code
    for (int i = 0; i < num_distance_sensors; i++)
    {
        distanceSensors[i] = getDistanceSensor(distanceSensorNames[i]);
        distanceSensors[i]->enable(TIME_STEP);
    }

    std::ifstream file(MOTION_PLAN_FILE_NAME, std::ios::in);
    std::cout << "RoboBot Reading in motion plan from ../../MotionPlan.txt..." << std::endl;

    if (file.is_open())
    {
        while (std::getline(file, moveCommands))
        {
            // Output the text from the file
            std::cout << "RoboBot Motion Plan: " << moveCommands << std::endl;
        }
        file.close();
    }
    else
    {
        std::cerr << "Cannot read file";
    }

    std::ofstream file2(MOTION_EXECUTION_FILE_NAME, std::ios::trunc);
    if (file2.is_open())
    {
        file2 << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall\n";
        file2.close();
    }
    else
    {
        std::cerr << "Cannot write to file";
    }

    numMoves = moveCommands.length();
    stepC = 0;
    row = moveCommands[0] - '0';
    col = moveCommands[1] - '0';
    heading = moveCommands[2];
    moveCounter = 3;
    currTime = 0;
    prevTime = 0;
    leftPos = 0;
    rightPos = 0;
    std::cout << "RoboBot Executing motion plan..." << std::endl;
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
        leftMotor = leftMotor + oneSquare * M_PI;
        rightMotor = rightMotor + oneSquare * M_PI;
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
        leftMotor = leftMotor - quarter * M_PI;
        rightMotor = rightMotor + quarter * M_PI;
        bearingINT = mod((bearingINT - 1), 4);
    }
    // right command
    else if (command == 'R')
    {
        leftMotor = leftMotor + quarter * M_PI;
        rightMotor = rightMotor - quarter * M_PI;
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

void runA()
{
    myRobot myRobot;
    while (myRobot.step(TIME_STEP) != -1)
    {
        // Get average values for distance sensors
        int aveLeft = 0;
        int aveRight = 0;
        int aveFront = 0;
        int sum = 0;
        myRobot.currTime = myRobot.getTime();
        myRobot.step(TIME_STEP);
        while (myRobot.getTime() < myRobot.currTime + 3)
        {
            myRobot.step(TIME_STEP);
            aveLeft = myRobot.distanceSensors[0]->getValue() + aveLeft;
            aveFront = myRobot.distanceSensors[1]->getValue() + aveFront;
            aveRight = myRobot.distanceSensors[2]->getValue() + aveRight;
            sum++;
        }
        aveLeft = aveLeft / sum;
        aveFront = aveFront / sum;
        aveRight = aveRight / sum;
        // write to file
        writeToFile(myRobot.stepC, myRobot.row, myRobot.col, myRobot.heading, checkWall(aveLeft), checkWall(aveFront), checkWall(aveRight));

        if (myRobot.moveCounter < myRobot.numMoves)
        {
            auto motorPos = executeMove(myRobot.moveCommands[myRobot.moveCounter], myRobot.leftPos, myRobot.rightPos, myRobot.heading, myRobot.row, myRobot.col);

            myRobot.leftMotor->setVelocity(0.17 * maxMotorSpeed);
            myRobot.rightMotor->setVelocity(0.17 * maxMotorSpeed);
            myRobot.leftMotor->setPosition(motorPos.leftMotor);
            myRobot.rightMotor->setPosition(motorPos.rightMotor);

            myRobot.prevTime = myRobot.getTime();
            // myRobot.step(TIME_STEP);
            while (myRobot.getTime() < myRobot.prevTime + 6)
            {
                myRobot.step(TIME_STEP);
            }
            myRobot.leftPos = motorPos.leftMotor;
            myRobot.rightPos = motorPos.rightMotor;
            myRobot.row = motorPos.row;
            myRobot.col = motorPos.col;
            myRobot.heading = motorPos.bearing;
            myRobot.stepC++;
            myRobot.moveCounter++;
        }
        else
        {
            std::cout << "[RoboBot] Motion plan executed!" << std::endl;
            return;
        }
    }

    return;
}