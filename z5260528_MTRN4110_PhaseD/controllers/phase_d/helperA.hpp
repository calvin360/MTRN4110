#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <fstream>

int mod(int a, int b)
{
    return (a % b + b) % b;
}

bool checkWall(double distanceValue)
{
    if (distanceValue < 850)
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

    std::ofstream file("../../MotionExecution.csv", std::ios::app);
    if (!file.is_open())
    {
        std::cerr << "Cannot write to file";
    }
    else
    {
        file << step << "," << row << "," << col << "," << heading << "," << leftWall << "," << frontWall << "," << rightWall << "\n";
        file.close();
    }

    std::cout << "[RoboBot] "
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