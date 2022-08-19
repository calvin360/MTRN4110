/*
File: z5260528_MTRN4110_PhaseA.cpp
Date: 15/06/2022
Description: controller of e-puck for phase a
Author: Calvin Lau
Modifications: N/A
Platform: windows
Notes: using csv processor from mtrn2500
*/
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <fstream>
#include <limits>
#include <stdlib.h> 
#include <iomanip>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Gyro.hpp>
#include <webots/InertialUnit.hpp>

constexpr double maxMotorSpeed = 6.28;  // rad/s
constexpr double wheel_radius = 0.02;  // m
constexpr int num_distance_sensors = 8;
constexpr double obstacleDistance = 90.0;
const std::array<std::string, num_distance_sensors> distanceSensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
const std::array<std::string, 3> turretDsNames={"dsL","dsF","dsR"};
const std::string MOTION_PLAN_FILE_NAME="../../MotionPlan.txt";
const std::string MOTION_EXECUTION_FILE_NAME="../../MotionExecution.csv";
const double oneSquare=2.626056561016273;// og 2.626056561016273
const double pi=3.14159265359;
const double quart=0.7645;//full 3.114 og 0.7715
double leftPos=0;
double rightPos=0;
int poseX;
int poseY;
char heading;
const std::array<char,4>headings={'N','W','S','E'};
int headI;
const char delim=',';

class myRobot :public webots::Robot{
public:
    myRobot();
    std::array<webots::DistanceSensor*, num_distance_sensors> distanceSensors;
    std::array<webots::DistanceSensor*, 3> turretDs;
    int timeStep;
    webots::Motor* leftMotor;
    webots::Motor* rightMotor;
    webots::PositionSensor* leftMotorPos;
    webots::PositionSensor* rightMotorPos;
    webots::Gyro* gyro;
    webots::InertialUnit* imu;
    std::string planstr;
    std::vector<char> plan;
    std::vector<double> Lsum;
    std::vector<double> Fsum;
    std::vector<double> Rsum;
    std::vector<double> proxSumL;
    std::vector<double> proxSumR;
private:
};

myRobot::myRobot(){
    // timeStep=static_cast<int>(getBasicTimeStep());
    timeStep=64;
    //getting motor pointers and initialising
    leftMotor=getMotor("left wheel motor");
    rightMotor=getMotor("right wheel motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);
    //getting position sensors pointers
    leftMotorPos=getPositionSensor("left wheel sensor");
    rightMotorPos=getPositionSensor("right wheel sensor");
    leftMotorPos->enable(timeStep);
    rightMotorPos->enable(timeStep);
    //getting gyro pointer (unused for phase A)
    gyro=getGyro("gyro");
    gyro->enable(timeStep);
    //getting imu pointer (unused for phase A)
    imu=getInertialUnit("IMU");
    imu->enable(timeStep);
    //getting proximity sensor pointers  (unused for phase A)
    for (int i = 0; i < num_distance_sensors; i++) {
        distanceSensors[i] = getDistanceSensor(distanceSensorNames[i]);
        distanceSensors[i]->enable(timeStep);
    }
    //getting distance sensor pointers (from turret slot)
    for (int i = 0; i < 3; i++) {
        turretDs[i] = getDistanceSensor(turretDsNames[i]);
        turretDs[i]->enable(timeStep);
    }

    //get motion plan
    std::cout<<"[z5260528_MTRN4110_PhaseA] Reading in motion plan from "<<MOTION_PLAN_FILE_NAME<<"..."<<std::endl;
    std::ifstream inFile{MOTION_PLAN_FILE_NAME,std::ios::in};
    if(inFile.is_open()){
        std::getline(inFile,planstr);
    }
    inFile.close();
    std::cout<<"[z5260528_MTRN4110_PhaseA] Motion Plan: "<<planstr<<"\n"<<"[z5260528_MTRN4110_PhaseA] Motion plan read in!\n"<<"[z5260528_MTRN4110_PhaseA] Executing motion plan..."<<std::endl;   
    //print motion plan
    for(int i=0;i<int(planstr.length());i++){
        plan.push_back(planstr[i]);
    }
    std::cout<<std::endl;
    //get initial pose
    poseX=plan[0]-'0';
    poseY=plan[1]-'0';
    heading=plan[2];
    //get index for inital heading
    for (int i=0;i<4;i++){
        if(heading==headings[i]){
            headI=i;
            break;
        }
    }    
}

// Helper function to check for obstacles.
// bool checkObstacle(double dist1, double dist2, double dist3){
    // return dist1 > obstacleDistance || dist2 > obstacleDistance || dist3 > obstacleDistance;
// }
//compute running average
const char* getAvg(std::vector<double> vect){
    double sum=0;
    for(auto i:vect){
        sum+=i;
    }
    if((sum/3)<=800){
        return "Y";
    }
    return "N";
}
//write to csv (append to end)
static void writeLineToCsv(int j,std::vector<double> Lsum,std::vector<double> Fsum,std::vector<double> Rsum){
    std::ofstream fout{ MOTION_EXECUTION_FILE_NAME, std::ios::out | std::ios::app };
    if (fout.is_open()) {
        fout<<j-2<<delim<<poseX<<delim<<poseY<<delim<<char(heading)<<delim<<getAvg(Lsum)<<delim<<getAvg(Fsum)<<delim<<getAvg(Rsum)<<std::endl;
    }
}
//write to csv (wipe file)
static void writeLineToCsvTrunc(){
    std::ofstream fout{ MOTION_EXECUTION_FILE_NAME, std::ios::out | std::ios::trunc };
    if (fout.is_open()) { 
        fout<<"Step"<<delim<<"Row"<<delim<<"Column"<<delim<<"Heading"<<delim<<"Left Wall"<<delim<<"Front Wall"<<delim<<"Right Wall"<<std::endl;
    }
}
//forward move command
static void forward(webots::Motor* leftMotor, webots::Motor* rightMotor,int step,double leftpos,double rightpos,std::vector<double> Lsum,std::vector<double> Rsum){
    double L=0;
    double R=0;
    double leftSpeed,rightSpeed;
    //compute running average  again
    for (int i=0;i<3;i++){
        L+=Lsum[i];
        R+=Rsum[i];
    }
    L=L/3;
    R=R/3;
    //using raw sensor values (not converted to metres)
    if(L<300||((R>700)&&(R<880))){
        leftSpeed=0.315*maxMotorSpeed;
        rightSpeed=0.3*maxMotorSpeed;
    }else if(R<300||((L>700)&&(L<880))){
        leftSpeed=0.3*maxMotorSpeed;
        rightSpeed=0.315*maxMotorSpeed;
    }else{
        leftSpeed=0.3*maxMotorSpeed;
        rightSpeed=0.3*maxMotorSpeed;

    }
    leftPos+=oneSquare*pi;
    rightPos+=oneSquare*pi;
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    leftMotor->setPosition(leftPos);
    rightMotor->setPosition(rightPos);
}
//left turn command
static void left(webots::Motor* leftMotor, webots::Motor* rightMotor,int step,double leftpos,double rightpos){
    double leftSpeed=0.5*maxMotorSpeed;
    double rightSpeed=0.5*maxMotorSpeed;
    leftPos+=-quart*pi;
    rightPos+=quart*pi;
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    leftMotor->setPosition(leftPos);
    rightMotor->setPosition(rightPos);
}
//right turn command
static void right(webots::Motor* leftMotor, webots::Motor* rightMotor,int step,double leftpos,double rightpos){
    double leftSpeed=0.5*maxMotorSpeed;
    double rightSpeed=0.5*maxMotorSpeed;
    leftPos+=quart*pi;
    rightPos+=-quart*pi;
    leftMotor->setVelocity(leftSpeed);
    rightMotor->setVelocity(rightSpeed);
    leftMotor->setPosition(leftPos);
    rightMotor->setPosition(rightPos);
}

int main() {
    // Initialise robot.
    myRobot myRobot;
    bool run=false;
    int i=0;
    int j=3;
    int stuck = 0;
    double prevL = -1;
    double prevR = -1;
    while (myRobot.step(myRobot.timeStep) != -1) {
        if(i<3){
            //getting running average for step 0 wall sensing
            myRobot.Lsum.insert(myRobot.Lsum.begin(),myRobot.turretDs[0]->getValue());
            myRobot.Fsum.insert(myRobot.Fsum.begin(),myRobot.turretDs[1]->getValue());
            myRobot.Rsum.insert(myRobot.Rsum.begin(),myRobot.turretDs[2]->getValue());
        }
        if(i==3){
            std::cout<<"[z5260528_MTRN4110_PhaseA] Step:"<<std::setw(3)<<std::setfill('0')<<j-3<<",Row:"<<poseX<<",Column:"<<poseY<<",Heading:"<<heading<<",Left Wall:"<<getAvg(myRobot.Lsum)<<",Front Wall:"<<getAvg(myRobot.Fsum)<<",Right Wall:"<<getAvg(myRobot.Rsum)<<std::endl;
            writeLineToCsvTrunc();
            writeLineToCsv(2,myRobot.Lsum,myRobot.Fsum,myRobot.Rsum);
        }
        if(i>3){
            //only keeping the 3 latest values for wall sensing
            myRobot.Lsum.pop_back();
            myRobot.Fsum.pop_back();
            myRobot.Rsum.pop_back();
            myRobot.Lsum.insert(myRobot.Lsum.begin(),myRobot.turretDs[0]->getValue());
            myRobot.Fsum.insert(myRobot.Fsum.begin(),myRobot.turretDs[1]->getValue());
            myRobot.Rsum.insert(myRobot.Rsum.begin(),myRobot.turretDs[2]->getValue());
        }
        //processing motion plan steps 
        if(!run){
            if(myRobot.plan[j]=='F'){
                forward(myRobot.leftMotor,myRobot.rightMotor,j,leftPos,rightPos,myRobot.Lsum,myRobot.Rsum);
                run=true;
            }
            else if(myRobot.plan[j]=='L'){
                left(myRobot.leftMotor,myRobot.rightMotor,j,leftPos,rightPos);
                run=true;
            }
            else if(myRobot.plan[j]=='R'){
                right(myRobot.leftMotor,myRobot.rightMotor,j,leftPos,rightPos);
                run=true;
            }
        }
        //getting 3 readings to filter out noise (not using moving average since value is always changing)
        double lTemp=0;
        double rTemp=0;
        for(int n=0;n<3;n++){
            lTemp+=myRobot.leftMotorPos->getValue();
            rTemp+=myRobot.rightMotorPos->getValue();
            myRobot.step(myRobot.timeStep);
        }
        lTemp=lTemp/3;
        rTemp=rTemp/3;
        if ((prevL == lTemp)){
            stuck++;
        }
        prevL = lTemp;
        prevR = rTemp;
        std::cout<<lTemp<<std::endl;
        std::cout<<rTemp<<std::endl;
        //check if bot is in the right spot
        if ((((0.997 * leftPos <= lTemp) && (lTemp <= 1.003 * leftPos) && (0.997 * rightPos <= rTemp) && (rTemp <= 1.003 * rightPos)) || (stuck == 10)) && run){
            //update pose if movement is complete
            if(myRobot.plan[j]=='F'){
                switch(heading){
                    case 'N':
                        poseX+=-1;
                        break;
                    case 'S':
                        poseX+=1;
                        break;
                    case 'E':
                        poseY+=1;
                        break;
                    case 'W':
                        poseY+=-1;
                        break;
                }
            }else if(myRobot.plan[j]=='L'){
                (headI==3)?(headI=0):(headI+=1);
                heading=headings[headI];
            }else if(myRobot.plan[j]=='R'){
                (headI==0)?(headI=3):(headI+=-1);
                heading=headings[headI];
            }
            std::cout<<"[z5260528_MTRN4110_PhaseA] Step:"<<std::setw(3)<<std::setfill('0')<<j-2<<",Row:"<<poseX<<",Column:"<<poseY<<",Heading:"<<heading<<",Left Wall:"<<getAvg(myRobot.Lsum)<<",Front Wall:"<<getAvg(myRobot.Fsum)<<",Right Wall:"<<getAvg(myRobot.Rsum)<<std::endl;
            writeLineToCsv(j,myRobot.Lsum,myRobot.Fsum,myRobot.Rsum);
            run=false;
            j++;
            //break if plan is done
            if(j==int(myRobot.planstr.length())){
                std::cout<<"[z5260528_MTRN4110_PhaseA] Motion plan executed!"<<std::endl;
                break;
            }
        }
        i++;
    }
}