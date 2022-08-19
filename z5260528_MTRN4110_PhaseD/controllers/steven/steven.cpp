// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>

// All the webots classes are defined in the "webots" namespace
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <utility>
#include <map>
#include <queue>
#include <limits>
#include <cstring>
#include <chrono>

//#define NV 45
#define ROWS 5
#define COLS 9
const std::string MAP_FILE_NAME = "../../Map0.txt";
const std::string PATH_PLAN_FILE_NAME = "../../PathPlan.txt";
const std::string OUTPUT_FILE_NAME = "../../Output.txt";

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

void addEdge(std::vector<int> adj[],int u, int v) { 
    adj[u].push_back(v);
    adj[v].push_back(u);
}



int getRow(int nV) {
    int row = 0;
    if (nV < 9) {
      row = 0;
    }
    else if (nV < 18) {
      row = 1;
    }
    else if (nV < 27) {
      row = 2;
    }
    else if (nV < 36) {
      row = 3;
    } else {
      row = 4;
    }
    
    return row;
}

int getCol(int nV) {
    int col = 0;
    
    col = nV % 9;
    
    
    return col;
}

bool checkStart(char input) {
  switch (input) {
    case '^':
        return true;
    case '>':
        return true;
    case 'v':
        return true;
    case '<':
        return true;
    default:
        return false;
  }

}


int getStartpos(char input) {
  switch (input) {
    case '^':
        return 0;
    case '>':
        return 1;
    case 'v':
        return 2;
    case '<':
        return 3;
    default:
        return -1;
  }
  

}

std::string getStartstring(int input) {
  switch (input) {
    case 0:
        return "N";
    case 1:
        return "E";
    case 2:
        return "S";
    case 3:
        return "W";
    default:
        return "FAILED TO GET STARTING DIRECTION PLEASE RESTART";
  }
  

}


int getDirection(int difference) {
   switch (difference) {
      case -9: //^
          return 0;
      case 1: //>
          return 1;
      case 9: //v
          return 2;
      case -1: // <
          return 3;
      default:
          return -1;
    }
  
}

std::string getPP(int curr_heading, int next_heading) {
    switch (curr_heading) {
        case 0:
            if (next_heading == 3) return "L";
            if (next_heading == 1) return "R";
            break;
        case 1:
            if (next_heading == 0) return "L";
            if (next_heading == 2) return "R";
            break;
        case 2:
            if (next_heading == 1) return "L";
            if (next_heading == 3) return "R";
            break;
        case 3:
            if (next_heading == 2) return "L";
            if (next_heading == 0) return "R";
            break;
    }
    return "LL";

}
 
/***************************************************************************************************
* The following function, which uses BFS to find all shortest paths, was adapted from:
* Title: find_paths source code
* Author: shobhitgupta907 (@shobhitgupta907), sanjeev2552
* Last Updated : 03 Feb, 2022
* Available at https://www.geeksforgeeks.org/print-all-shortest-paths-between-given-source-and-destination-in-an-undirected-graph/
***************************************************************************************************/

void find_paths(std::vector<std::vector<int> >& paths,
                std::vector<int>& path,
                std::vector<int> parent[],
                int n, int u)
{
    // Base Case
    if (u == -1) {
        paths.push_back(path);
        return;
    }
 
    // Loop for all the parents
    // of the given vertex
    for (int par : parent[u]) {
 
        // Insert the current
        // vertex in path
        path.push_back(u);
 
        // Recursive call for its parent
        find_paths(paths, path, parent,
                   n, par);
 
        // Remove the current vertex
        path.pop_back();
    }

}
 
// Function which performs bfs
// from the given source vertex
void bfs(std::vector<int> adj[],
         std::vector<int> parent[],
         int n, int start)
{
    // dist will contain shortest distance
    // from start to every other vertex
    std::vector<int> dist(n, INT_MAX);
 
    std::queue<int> q;
 
    // Insert source vertex in queue and make
    // its parent -1 and distance 0
    q.push(start);
    parent[start] = { -1 };
    dist[start] = 0;
 
    // Until Queue is empty
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int v : adj[u]) {
            if (dist[v] > dist[u] + 1) {
 
                // A shorter distance is found
                // So erase all the previous parents
                // and insert new parent u in parent[v]
                dist[v] = dist[u] + 1;
                q.push(v);
                parent[v].clear();
                parent[v].push_back(u);
            }
            else if (dist[v] == dist[u] + 1) {
 
                // Another candidate parent for
                // shortes path found
                parent[v].push_back(u);
            }
        }
    }
}
 
 

 
// Function which prints all the paths
// from start to end
void print_paths(std::vector<int> adj[], int n, int start, int end, std::string saveLine[], int startingPos){
    std::vector<std::vector<int> > paths;
    std::vector<int> path;
    std::vector<int> parent[n];
 
    // Function call to bfs
    bfs(adj, parent, n, start);
 
    // Function call to find_paths
    find_paths(paths, path, parent, n, end);
    int totalPaths = paths.size();
    //int pathLength = paths[0].size() - 2;
    int nV = 0;
    
    std::ofstream file2(OUTPUT_FILE_NAME,std::ios::app);
    if (!file2.is_open()) {
      std::cerr << "Cannot write to file";
    }
    std::cout << "[z5257127_MTRN4110_PhaseB] Finding shortest paths..." << std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Finding shortest paths..." << "\n";
    int nPaths = 1;
    int turns;
    int minTurn = INT_MAX;
    int dir = startingPos;
    int newDir = -1;
    int temp = start;
    int pathOfleastTurns = 0;
    std::string startCoords = std::to_string(getRow(start)) + std::to_string(getCol(start)) + getStartstring(startingPos);
    std::string pathPlan;
    std::vector<int>::iterator index;
    for (auto v : paths) {
 
        // Since paths contain each
        // path in reverse order,
        // so reverse it
        //reverse(v.begin(), v.end());
        nV = 0;
        // Print node for the current path
          std::cout << "[z5257127_MTRN4110_PhaseB] Path - " << nPaths << ":" << std::endl;
          file2 << "[z5257127_MTRN4110_PhaseB] Path - " << nPaths << ":" <<"\n";

          std::cout << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << std::endl;
          file2 << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << "\n";

          for(int j = 0; j < 9; j++) {
          std::cout << "[z5257127_MTRN4110_PhaseB] ";
          file2 << "[z5257127_MTRN4110_PhaseB] ";
            for(int c = 0; c < 37; c++) {
                if (c % 2 == 0 && c != 0 && c % 4 != 0 && j % 2 == 0 ) {
                    index = std::find(v.begin(), v.end(), nV);
                    if (nV != start &&  index != v.end() ) {
                      std::cout << std::distance(v.begin(), index);
                      file2 << std::distance(v.begin(), index);
                      if (std::distance(v.begin(), index) > 9) {
                        c++;
                      }
                    } else {
                      std::cout << saveLine[j][c];
                      file2 << saveLine[j][c];
                    }
                    nV++;
                } else { 
                    std::cout << saveLine[j][c];
                    file2 << saveLine[j][c];
                }
              }
              std::cout << std::endl;
              file2 << "\n";

           }
            
          std::cout << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << std::endl;
          file2 << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << "\n";
          turns = 0;
          temp = start;
          reverse(v.begin(), v.end());
          dir = startingPos;
          for (int u : v) {
                if (u - temp != 0) {
                  newDir = getDirection((u - temp));
                  if (newDir != dir) {
                      if (abs(dir-newDir) == 1 ||abs(dir-newDir) == 3 ) {
                        turns++;
                      }
                      if (abs(dir-newDir) == 2) {
                        turns = turns + 2;
                      }
                      dir = newDir;
                  }
                  
                }
                temp = u;
         }

         if (turns < minTurn) {
           pathOfleastTurns = nPaths - 1;
           minTurn = turns;
         }
         reverse(v.begin(), v.end());

         //std::cout << "("<< getCol(u) << ","<< getRow(u) <<")" << " ";
       
        std::cout << std::endl;
        nPaths++;

    }
    std::cout << "[z5257127_MTRN4110_PhaseB] " << totalPaths << " shortest paths found!"<< std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] " << totalPaths << " shortest paths found!"<< "\n";

    std::cout << "[z5257127_MTRN4110_PhaseB] Finding shortest path with least turns..."<< std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Finding shortest path with least turns..."<< "\n";
    nV = 0;
    auto v = paths[pathOfleastTurns];
    std::cout << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << "\n";

          for(int j = 0; j < 9; j++) {
          std::cout << "[z5257127_MTRN4110_PhaseB] ";
          file2 << "[z5257127_MTRN4110_PhaseB] ";
            for(int c = 0; c < 37; c++) {
                if (c % 2 == 0 && c != 0 && c % 4 != 0 && j % 2 == 0 ) {
                    index = std::find(v.begin(), v.end(), nV);
                    if (nV != start &&  index != v.end() ) {
                      std::cout << std::distance(v.begin(), index);
                      file2 << std::distance(v.begin(), index);
                      if (std::distance(v.begin(), index) > 9) {
                        c++;
                      }
                    } else {
                      std::cout << saveLine[j][c];
                      file2 << saveLine[j][c];

                    }
                    nV++;
                } else { 
                    std::cout << saveLine[j][c];  
                    file2 << saveLine[j][c];
                }
              }
              std::cout << std::endl;
              file2 << "\n";


           }
    reverse(v.begin(), v.end());
    dir = startingPos;       
    for (auto ptr = v.begin(); ptr < v.end() - 1; ptr++){
        newDir = getDirection(*(ptr + 1) - *ptr);
        if (dir != newDir) {
          pathPlan = pathPlan + getPP(dir,newDir);
          dir = newDir;
        } 
        pathPlan = pathPlan + "F";
        
    }
    
    std::cout << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB]  --- --- --- --- --- --- --- --- --- " << "\n";
    std::cout << "[z5257127_MTRN4110_PhaseB] Shortest path with least turns found!"<< std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Shortest path with least turns found!"<< "\n";
    std::cout << "[z5257127_MTRN4110_PhaseB] Path Plan ("<< pathPlan.length() <<" steps): " << startCoords << pathPlan <<std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Path Plan ("<< pathPlan.length() <<" steps): " << startCoords << pathPlan <<"\n";
    std::cout << "[z5257127_MTRN4110_PhaseB] Writing path plan to "<<PATH_PLAN_FILE_NAME <<"..." << std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Writing path plan to "<<PATH_PLAN_FILE_NAME <<"..." << "\n";
    std::cout << "[z5257127_MTRN4110_PhaseB] Path plan written to "<<PATH_PLAN_FILE_NAME <<"!"<< std::endl;
    file2 << "[z5257127_MTRN4110_PhaseB] Path plan written to "<<PATH_PLAN_FILE_NAME <<"!"<< "\n";
    file2.close();

   std::ofstream file(PATH_PLAN_FILE_NAME,std::ios::trunc);
    if (!file.is_open()) {
      std::cerr << "Cannot write to file";
    } else {
      file << startCoords << pathPlan;
      file.close();
    }
  

    //print lowest turn
    
    
    
}





int main(int argc, char **argv) {
    auto start = std::chrono::high_resolution_clock::now();
    int n = 45;
    std::vector<int> adj[n];
    int src = 0;
    int dest = 22;
    
    int nV = 0;
    int nV_2 = 0;
    std::string Map;
    std::ifstream file(MAP_FILE_NAME);
    std::ofstream file2(OUTPUT_FILE_NAME,std::ios::trunc);

    int row = 0;
    int startPos = -1;
    std::string saveLine[9];
 
    if (file.is_open()) {
      if (!file2.is_open()) {
        std::cerr << "Cannot write to file";
      } 
      std::cout << "[z5257127_MTRN4110_PhaseB] Reading in map from "<< MAP_FILE_NAME << "..."<<std::endl;
      file2 << "[z5257127_MTRN4110_PhaseB] Reading in map from "<< MAP_FILE_NAME << "..."<< "\n";
      while (std::getline(file, Map)) {
        //::cout << nV << "  "<< nV_2 <<std::endl;

      // Output the text from the file
        std::cout << "[z5257127_MTRN4110_PhaseB] " << Map << std::endl;
        file2  << "[z5257127_MTRN4110_PhaseB] " << Map << "\n";
        if (row != 0 && row != 10){
            saveLine[row-1] = Map;
        }
        if(row % 2 != 0) {
          //std::cout << row <<std::endl;
          for (std::size_t j = 0; j < Map.length(); j++) {
             if (j % 4 == 0 && j != 0) {
               if(Map[j] != '|') {
                 addEdge(adj,nV,nV + 1);

               }
               nV++;
               
             }
             if (checkStart(Map[j])) {
               src = nV;
               startPos = getStartpos(Map[j]);

             }
             if(Map[j] == 'x') {
               dest = nV;
             }
           }
        }
        else if (row != 0 && row != 10) {
         for (std::size_t c = 0; c < Map.length(); c++) {
           if (c % 2 == 0 && c != 0 && c % 4 != 0)  {
              if(Map[c] != '-') {
                addEdge(adj,nV_2,nV_2 + 9);
              }
              nV_2++;
           } 
         }
       
       }
       /* else if(row != 0 && row != 10) {
          for (std::size_t j = 0; j < Map.length(); j++) {
             
            if (j % 4 == 0 && j != 0 ) {
              if(Map[j-1] != '-') {
                addEdge(adj,nV_2,nV_2 + 9);
                 std::cout << "("<< nV_2 << "," << nV_2 + 9<< ")";
                j = j + 3;
              } 
               nV_2++;
             }
          }
        }  */     
        row++;
      }
      std::cout << "[z5257127_MTRN4110_PhaseB] Map read in!"<<std::endl;
      file2 << "[z5257127_MTRN4110_PhaseB] Map read in!" << "\n";
      file.close();
      file2.close();
    } else {
      std::cerr << "Cannot read file";
    }




    // Function Call
    print_paths(adj, n, src, dest,saveLine,startPos);
    auto end = std::chrono::high_resolution_clock::now();
    auto ms_int = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::chrono::duration<double, std::milli> ms_double = end - start;
    std::cout << ms_int.count() << "ms\n";
    std::cout << ms_double.count() << "ms\n";
  return 0;
}
