#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream>
#include <string>

/**
Encodes a single digit of a POSTNET "A" bar code.
@param digit the single digit to encode.
@return a bar code of the digit using "|" as the long bar
and "," as the half bar.
*/
void addEdge(std::vector<int> adj[], int u, int v)
{
    adj[u].push_back(v);
    adj[v].push_back(u);
}

int getRow(int nV)
{
    int row = 0;
    if (nV < 9)
    {
        row = 0;
    }
    else if (nV < 18)
    {
        row = 1;
    }
    else if (nV < 27)
    {
        row = 2;
    }
    else if (nV < 36)
    {
        row = 3;
    }
    else
    {
        row = 4;
    }

    return row;
}

int getCol(int nV)
{
    int col = 0;

    col = nV % 9;

    return col;
}

bool checkStart(char input)
{
    switch (input)
    {
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

int getStartpos(char input)
{
    switch (input)
    {
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

std::string getStartstring(int input)
{
    switch (input)
    {
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

int getDirection(int difference)
{
    switch (difference)
    {
    case -9: //^
        return 0;
    case 1: //>
        return 1;
    case 9: // v
        return 2;
    case -1: // <
        return 3;
    default:
        return -1;
    }
}

std::string getPP(int curr_heading, int next_heading)
{
    switch (curr_heading)
    {
    case 0:
        if (next_heading == 3)
            return "L";
        if (next_heading == 1)
            return "R";
        break;
    case 1:
        if (next_heading == 0)
            return "L";
        if (next_heading == 2)
            return "R";
        break;
    case 2:
        if (next_heading == 1)
            return "L";
        if (next_heading == 3)
            return "R";
        break;
    case 3:
        if (next_heading == 2)
            return "L";
        if (next_heading == 0)
            return "R";
        break;
    }
    return "LL";
}