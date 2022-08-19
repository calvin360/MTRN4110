// File:          z5260528_MTRN4110_PhaseB.cpp
// Date:          07/07/2022
// Description: controller of e-puck for phase b
// Author: Calvin Lau
// Modifications: N/A
// Platform: windows
// Notes: borrowing some graph generation code from the lab and floodfill psuedo code from lecture

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream>
#include <string>

const std::string MAP_FILE_NAME="../../Map0.txt";
const std::string OUTPUT_FILE_NAME="../../Output.txt";
const std::string PATHPLAN_FILE_NAME="../../PathPlan.txt";
int src=-1;
int dest=-1;
char heading;
int paths=1;

void addEdge(std::vector<std::vector<int>> &adj, int vertex_a, int vertex_b){
  adj[vertex_a].push_back(vertex_b);
  adj[vertex_b].push_back(vertex_a);
}

std::vector<std::vector<char>> readMap(){
  std::string mapstr;
  std::vector<std::vector<char>> map;
  std::ofstream outFile{OUTPUT_FILE_NAME,std::ios::trunc};

  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] "<<"Reading in map from "<<MAP_FILE_NAME<<"..."<<"\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] "<<"Reading in map from "<<MAP_FILE_NAME<<"..."<<std::endl; 
  std::ifstream inFile{MAP_FILE_NAME,std::ios::in};
  if(inFile.is_open()){
    while(!inFile.eof()){
      std::getline(inFile,mapstr);
      std::vector<char> temp;
      for (int i=0;i<int(mapstr.length());i++){
        temp.push_back(mapstr[i]);
      }
      map.push_back(temp);
      if(outFile.is_open()){
        outFile<<"[z5260528_MTRN4110_PhaseB] "<<mapstr<<"\n";
      }
      std::cout<<"[z5260528_MTRN4110_PhaseB] "<<mapstr<<std::endl;   
    }
  }
  inFile.close();
  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] "<<"Map read in!"<<"\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] "<<"Map read in!"<<std::endl;

  return map;
}

void makeGraph(std::vector<std::vector<char>> map,std::vector<std::vector<int>> &adj){
  int row=1;
  bool srcF=false;
  bool destF=false;

  for (int i=1;i<10;i+=2){
      for (int j=1;j<35;j++){
        //empty space left
        if(map[i][j]==' '&&(map[i][j+1]=='v'||map[i][j+1]=='^'||map[i][j+1]=='<'||map[i][j+1]=='>'||map[i][j+1]==' '||map[i][j+1]=='x')&&map[i][j+2]==' '){
          //looking for up down edges
          if(i<8&&map[i+1][j]==' '){
            if(map[i+2][j]==' '&&(map[i+2][j+1]=='v'||map[i+2][j+1]=='^'||map[i+2][j+1]=='<'||map[i+2][j+1]=='>'||map[i+2][j+1]==' '||map[i+2][j+1]=='x')&&map[i+2][j+2]==' '){
              addEdge(adj,9*(i-row)+(j+3)/4-1,9*(i-row+1)+(j+3)/4-1);
            }
          }
          //src in first col
          if(!srcF&&(map[i][j+1]=='v'||map[i][j+1]=='^'||map[i][j+1]=='<'||map[i][j+1]=='>')){
            src=9*(i-row)+(j+3)/4-1;
            switch(map[i][j+1]){
              case '^':
                heading='N';
                break;
              case 'v':
                heading='S';
                break;
              case '>':
                heading='E';
                break;
              case '<':
                heading='W';
                break;
            }
            srcF=true;
          //dest in first col
          }else if (!destF&&map[i][j+1]=='x'){
            dest=9*(i-row)+(j+3)/4-1;
            destF=true;
          }
          //vert wall found
          if(map[i][j+3]=='|'){
            j+=3; 
          //no wall
          }else if(j<31&&map[i][j+3]==' '){  
            if(map[i][j+4]==' '&&(map[i][j+5]=='v'||map[i][j+5]=='^'||map[i][j+5]=='<'||map[i][j+5]=='>'||map[i][j+5]==' '||map[i][j+5]=='x')&&map[i][j+6]==' '){
              if(!srcF&&(map[i][j+5]=='v'||map[i][j+5]=='^'||map[i][j+5]=='<'||map[i][j+5]=='>')){
                src=9*(i-row)+(j+7)/4-1;
                switch(map[i][j+5]){
                  case '^':
                    heading='N';
                    break;
                  case 'v':
                    heading='S';
                    break;
                  case '>':
                    heading='E';
                    break;
                  case '<':
                    heading='W';
                    break;
                }
                srcF=true;
              }else if (!destF&&map[i][j+1]=='x'){
                dest=9*(i-row)+(j+7)/4-1;
                destF=true;
              }
              addEdge(adj,9*(i-row)+(j+3)/4-1,9*(i-row)+(j+7)/4-1);
              j+=3;
            }
          }
        }else{
          j+=3;
        }
    } 
    row++;
  }
}

// utility function to form edge between two vertices
// vertex_a and vertex_b
void printAdjTable(std::vector<std::vector<int>> &adj){
  int counter{0};

  for (auto vector : adj){
    std::cout<< "Node "<<counter<<" : ";
    for(auto elem : vector){
      std::cout<<elem<<", ";
    }
    std::cout<<std::endl;
    counter++;
  }
}

void floodFill(std::vector<std::vector<int>> &adj, int src, int dest, int v, std::vector<int> &pred, std::vector<int> &cost){
  cost[dest]=0;
  int currVal=0;
  bool changed=true;

  //up+9,down-9
  while (changed){
    changed=false;
    for (int i=0;i<45;i++){
      if(cost[i]==currVal){
        for (auto val:adj[i]){
          if(cost[val]==INT_MAX){
            cost[val]=currVal+1;
          }
          changed=true;
        }
      }
    }
    currVal+=1;
  }
}

void find(int curr,int dest,std::vector<int> &cost,std::vector<std::vector<int>> &adj,std::vector<int> &temp,std::vector<std::vector<int>> &path){
  while (curr!=dest){
    std::vector<int> min;
    min.push_back(curr);
    //for all adjacent squares
    for (auto i:adj[curr]){
      if (cost[i]<cost[min[0]]){
        min.clear();
        min.push_back(i);
      }else if(cost[i]==cost[min[0]]){
        min.push_back(i);
      }
    }
    if(min.size()==1){
      temp.push_back(min[0]);
      curr=min[0];
    }else{
      for (int i=min.size()-1;i>0;i--){
        paths++;
        std::vector<int> v=temp;
        v.push_back(min[i]);
        find(min[i],dest,cost,adj,v,path);
        min.pop_back();
      }
      temp.push_back(min[0]);
      curr=min[0];
    }
  }
  path.push_back(temp);
  return;
}

std::vector<std::vector<int>> findPath(int src,int dest,std::vector<int> &cost,std::vector<std::vector<int>> &adj){
  int curr=src;
  std::vector<std::vector<int>> path;
  std::vector<int> temp;
  std::ofstream outFile{OUTPUT_FILE_NAME,std::ios::app};

  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Finding shortest paths..."<<"\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Finding shortest paths..."<<std::endl;
  temp.push_back(curr);
  find(curr,dest,cost,adj,temp,path);
  
  return path;
}

void print(std::vector<std::vector<int>> &path,int i, int j,std::vector<int> &cost,std::vector<std::vector<char>> &map,int row){
  int c=cost[path[i][j]];
  int dig,dig1;

  if(c>9){
    dig=c/10%10;
    dig1=c%10;
    if(row==1){
      map[row][(path[i][j]+1)*4-2]=char(48+dig);
      map[row][(path[i][j]+1)*4-1]=char(48+dig1);
    }else if(row==3){
      map[row][(path[i][j]+1-9)*4-2]=char(48+dig);
      map[row][(path[i][j]+1-9)*4-1]=char(48+dig1);
    }else if(row==5){
      map[row][(path[i][j]+1-9*2)*4-2]=char(48+dig);
      map[row][(path[i][j]+1-9*2)*4-1]=char(48+dig1);
    }else if(row==7){
      map[row][(path[i][j]+1-9*3)*4-2]=char(48+dig);
      map[row][(path[i][j]+1-9*3)*4-1]=char(48+dig1);
    }else if(row==9){
      map[row][(path[i][j]+1-9*4)*4-2]=char(48+dig);
      map[row][(path[i][j]+1-9*4)*4-1]=char(48+dig1);
    }
  }else{
    if(row==1){
      map[row][(path[i][j]+1)*4-2]=char(48+c);
    }else if(row==3){
      map[row][(path[i][j]+1-9)*4-2]=char(48+c);
    }else if(row==5){
      map[row][(path[i][j]+1-9*2)*4-2]=char(48+c);
    }else if(row==7){
      map[row][(path[i][j]+1-9*3)*4-2]=char(48+c);
    }else if(row==9){
      map[row][(path[i][j]+1-9*4)*4-2]=char(48+c);
    }
  }
}

void printPaths(std::vector<std::vector<int>> &path,int src,int dest,std::vector<std::vector<char>> map,std::vector<int> &cost,int i){
    std::ofstream outFile{OUTPUT_FILE_NAME,std::ios::app};

    for (int j=1;j<int(path[i].size());j++){
        if(path[i][j]<9){  
          print(path,i,j,cost,map,1);
         }else if(8<path[i][j]&&path[i][j]<18){
          print(path,i,j,cost,map,3);
        }else if(17<path[i][j]&&path[i][j]<27){
          print(path,i,j,cost,map,5);
        }else if(26<path[i][j]&&path[i][j]<36){
          print(path,i,j,cost,map,7);
        }else if(35<path[i][j]&&path[i][j]<45){
          print(path,i,j,cost,map,9);
        }
    }
    for (int i=0;i<11;i++){
      if(outFile.is_open()){
        outFile<<"[z5260528_MTRN4110_PhaseB] ";
      }
      std::cout<<"[z5260528_MTRN4110_PhaseB] ";
      for (int j=0;j<37;j++){
        if(outFile.is_open()){
          outFile<<map[i][j];
        }
        std::cout<<map[i][j];
      }
      if(outFile.is_open()){
        outFile<<"\n";
      }
      std::cout<<std::endl;
    } 
}

int findShort(std::vector<std::vector<int>> &path,int src,int dest,std::vector<std::vector<char>> map,std::vector<int> &cost){
  std::vector<char>opt;
  int turns=INT_MAX;
  std::vector<char>temp;
  int col=src%9;
  int row=src/9;
  int best=-1;
  std::ofstream outFile{OUTPUT_FILE_NAME,std::ios::app};

  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Finding shortest path with least turns..."<<std::endl;
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Finding shortest path with least turns..."<<std::endl;

  temp.push_back(48+row);
  temp.push_back(48+col);
  temp.push_back(heading);
  std::vector<char> base=temp;

  for (int i=0;i<paths;i++){
    int t=0;
    char h=heading;
    for (int j=0;j<int(path[i].size()-1);j++){
      //right
      if(path[i][j]-path[i][j+1]==-1){
        switch(h){
          case 'S':
            t++;
            temp.push_back('L');
            temp.push_back('F');
            h='E';
            break;
          case 'E':
            temp.push_back('F');
            break;
          case 'W':
            t+=2;
            temp.push_back('L');
            temp.push_back('L');
            temp.push_back('F');
            h='E';
            break;
          case 'N':
            t++;
            temp.push_back('R');
            temp.push_back('F');
            h='E';
            break;
        }
      //left
      }else if(path[i][j]-path[i][j+1]==1){
        switch(h){
          case 'N':
            t++;
            temp.push_back('L');
            temp.push_back('F');
            h='W';
            break;
          case 'E':
            t+=2;
            temp.push_back('L');
            temp.push_back('L');
            temp.push_back('F');
            h='W';
            break;
          case 'W':
            temp.push_back('F');
            break;
          case 'S':
            t++;
            temp.push_back('R');
            temp.push_back('F');
            h='W';
            break;
        }
      //down
      }else if(path[i][j]-path[i][j+1]==-9){
        switch(h){
          case 'N':
            t+=2;
            temp.push_back('L');
            temp.push_back('L');
            temp.push_back('F');
            h='S';
            break;
          case 'S':
            temp.push_back('F');
            break;
          case 'W':
            t++;
            temp.push_back('L');
            temp.push_back('F');
            h='S';
            break;
          case 'E':
            t++;
            temp.push_back('R');
            temp.push_back('F');
            h='S';
            break;
        }
      //up
      }else if(path[i][j]-path[i][j+1]==9){
        switch(h){
          case 'N':
            temp.push_back('F');
            break;
          case 'S':
            t+=2;
            temp.push_back('L');
            temp.push_back('L');
            temp.push_back('F');
            h='N';
            break;
          case 'E':
            t++;
            temp.push_back('L');
            temp.push_back('F');
            h='N';
            break;
          case 'W':
            t++;
            temp.push_back('R');
            temp.push_back('F');
            h='N';
            break;
        }
      }
    }
    if(t<turns){
      turns=t;
      opt=temp;
      best=i;
    }
    temp=base;
  }
  printPaths(path,src,dest,map,cost,best);
  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Shortest path with least turns found!\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Shortest path with least turns found!"<<std::endl;
  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Path Plan ("<<opt.size()-3<<" steps): ";
    for(auto i:opt){
      outFile<<i;
    }
    outFile<<"\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Path Plan ("<<opt.size()-3<<" steps): ";
  for(auto i:opt){
      std::cout<<i;
    }
  std::cout<<std::endl;

  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Writing path plan to "<<PATHPLAN_FILE_NAME<<"...\n";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Writing path plan to "<<PATHPLAN_FILE_NAME<<"..."<<std::endl;

  std::ofstream move{PATHPLAN_FILE_NAME,std::ios::trunc};
  if(move.is_open()){
    for(auto i:opt){
      move<<i;
    }
  }
  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] Path plan written to "<<PATHPLAN_FILE_NAME<<"!";
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] Path plan written to "<<PATHPLAN_FILE_NAME<<"!"<<std::endl;

  return best;
}

int main() {
  std::vector<std::vector<char>> map=readMap();
  int v=45;
  std::vector<std::vector<int>> adj1(v);
  makeGraph(map,adj1);
  std::vector<std::vector<int>> adj(v);
  std::vector<int> cost(v, INT_MAX);

  // addEdge(adj,0,1);
  // addEdge(adj,0,9);
  // addEdge(adj,1,2);
  // addEdge(adj,2,3);
  // addEdge(adj,2,11);
  // addEdge(adj,3,12);
  // addEdge(adj,4,5);
  // addEdge(adj,4,13);
  // addEdge(adj,5,6);
  // addEdge(adj,5,14);
  // addEdge(adj,6,7);
  // addEdge(adj,7,8);
  // addEdge(adj,7,16);
  // addEdge(adj,8,17);
  // addEdge(adj,9,10);
  // addEdge(adj,10,11);
  // addEdge(adj,10,19);
  // addEdge(adj,11,20);
  // addEdge(adj,12,13);
  // addEdge(adj,12,21);
  // addEdge(adj,13,14);
  // addEdge(adj,15,24);
  // addEdge(adj,16,17);
  // addEdge(adj,17,26);
  // addEdge(adj,18,19);
  // addEdge(adj,18,27);
  // addEdge(adj,19,28);
  // addEdge(adj,21,30);
  // addEdge(adj,22,23);
  // addEdge(adj,23,24);
  // addEdge(adj,24,25);
  // addEdge(adj,25,26);
  // addEdge(adj,25,34);
  // addEdge(adj,26,35);
  // addEdge(adj,27,36);
  // addEdge(adj,28,29);
  // addEdge(adj,28,37);
  // addEdge(adj,29,30);
  // addEdge(adj,30,31);
  // addEdge(adj,30,39);
  // addEdge(adj,31,32);
  // addEdge(adj,31,40);
  // addEdge(adj,33,34);
  // addEdge(adj,33,42);
  // addEdge(adj,34,35);
  // addEdge(adj,35,44);
  // addEdge(adj,37,38);
  // addEdge(adj,38,39);
  // addEdge(adj,40,41);
  // addEdge(adj,41,42);
  // addEdge(adj,43,44);

  // printAdjTable(adj);
  // printAdjTable(adj1);

  std::vector<int> pred(v);
  floodFill(adj1,src,dest,v,pred,cost);
  std::vector<std::vector<int>> path=findPath(src,dest,cost,adj1);
  std::ofstream outFile{OUTPUT_FILE_NAME,std::ios::app};
  for(int i=0;i<paths;i++){
    if(outFile.is_open()){
      outFile<<"[z5260528_MTRN4110_PhaseB] Path - "<<i+1<<":"<<std::endl;
    }
    std::cout<<"[z5260528_MTRN4110_PhaseB] Path - "<<i+1<<":"<<std::endl;
    printPaths(path,src,dest,map,cost,i);
  }
  if(outFile.is_open()){
    outFile<<"[z5260528_MTRN4110_PhaseB] "<<paths<<" shortest paths found!"<<std::endl;
  }
  std::cout<<"[z5260528_MTRN4110_PhaseB] "<<paths<<" shortest paths found!"<<std::endl;
  findShort(path,src,dest,map,cost);

  return 0;
}
