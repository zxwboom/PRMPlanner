#ifndef _PRM_H_
#define _PRM_H_
#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <queue>
struct Pose2f
{
  float x;
  float y;
  
};
struct Edge {  
    int from;  
    int to;  
    double cost; 
};
float distance(const Pose2f &p1, const Pose2f &p2) {
  return std::sqrt(std::pow(p1.x -p2.x, 2) + std::pow(p1.y - p2.y, 2));
}
// 单个点是否为障碍物 
bool isObstacle(const Pose2f& c) {  
    // 根据实际情况定义障碍物区域  
    return c.x < 0 || c.x > 10 || c.y < 0 || c.y > 10;  
}
// 两点一线是否为障碍物
bool isObstacle(const Pose2f& c1, const Pose2f& c2) {
  return false;
}
// 欧几里得距离  
double heuristic(const Pose2f& a, const Pose2f& b) {  
    return distance(a, b);  
}
int findVertexIndex(const std::vector<Pose2f>& vertices, const Pose2f& pose) {
    for (size_t i = 0; i < vertices.size(); ++i) {  
        if (vertices[i] == pose) {  
            return i;  
        }  
    }  
    return -1; // 如果未找到，则返回-1  
}

#endif