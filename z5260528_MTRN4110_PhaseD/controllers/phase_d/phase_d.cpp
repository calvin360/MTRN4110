#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <fstream>
// #include <include/Python.h>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>
#include "phaseA.hpp"
#include "phaseB.hpp"

const std::string PHASEC_FILE_NAME = "phaseC.py";

int main()
{
    // Py_SetProgramName(PHASEC_FILE_NAME);
    // Py_Initalize();
    // FILE *file = _Py_fopen(PHASEC_FILE_NAME, "r+");
    // PyRun_SimpleFile(file, PHASEC_FILE_NAME);
    // Py_FinalizeEx();

    // const char* name = "phaseC.py";
    // Py_Initalize();
    // FILE *file = _Py_fopen(name, "r+");
    // PyRun_SimpleFile(file, name);
    // Py_Finalize();

    runB();
    runA();
    return 0;
}