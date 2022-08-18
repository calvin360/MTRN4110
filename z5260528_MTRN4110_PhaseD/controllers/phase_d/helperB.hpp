#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <fstream>
#include <string>

void addEdge(std::vector<std::vector<int>> &adj, int vertex_a, int vertex_b)
{
    adj[vertex_a].push_back(vertex_b);
    adj[vertex_b].push_back(vertex_a);
}