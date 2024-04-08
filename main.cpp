/*
 * @Author: zhangxiaowei zhangxiaowei145@joyoung.com
 * @Date: 2024-04-08 17:13:17
 * @LastEditors: zhangxiaowei zhangxiaowei145@joyoung.com
 * @LastEditTime: 2024-04-08 20:33:01
 * @FilePath: /day_zero_project/jybot/pp/tracker/dwa_local_planner/prm.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <queue>
#include <unordered_map>
 


std::vector<Pose2f> astar(const std::vector<Pose2f>& vertices,  
                                  const std::vector<Edge>& edges,  
                                  const Pose2f& start, 
                                  const Pose2f& goal) {  
    std::unordered_map<int, double> g_score; // 到达每个顶点的实际代价  
    std::unordered_map<int, double> h_score; // 启发式代价  
    std::unordered_map<int, int> parent; // 记录每个顶点的父顶点  
    std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> open_set;  
  
    for (size_t i = 0; i < vertices.size(); ++i) {  
        g_score[i] = std::numeric_limits<double>::infinity();  
        h_score[i] = heuristic(vertices[i], goal);  
    }  
  
    g_score[findVertexIndex(vertices, start)] = 0;  
    open_set.push({0, findVertexIndex(vertices, start)});
  
    while (!open_set.empty()) {  
        auto current = open_set.top();  
        open_set.pop();  
  
        int current_index = current.second;  
        if (g_score[current_index] > current.first) continue; // 如果已经找到更好的路径，则跳过  
  
        if (vertices[current_index] == goal) {  
            // 构建路径  
            std::vector<Pose2f> path;  
            while (parent.find(current_index) != parent.end()) {  
                path.push_back(vertices[current_index]);  
                current_index = parent[current_index];  
            }  
            std::reverse(path.begin(), path.end()); // 反转路径，使其从起点到终点  
            return path;  
        }  
  
        for (const auto& edge : edges) {  
            if (edge.from == current_index) {  
                int neighbor_index = edge.to;  
                double tentative_g_score = g_score[current_index] + edge.cost;  
  
                if (tentative_g_score < g_score[neighbor_index]) {  
                    parent[neighbor_index] = current_index;  
                    g_score[neighbor_index] = tentative_g_score;  
                    double f_score = g_score[neighbor_index] + h_score[neighbor_index];  
                    open_set.push({f_score, neighbor_index});  
                }  
            }  
        }  
    }  
  
    // 如果没有找到路径  
    return {};  
}  
class prm
{
private:
  int numSamples;
  float connectionRadius;
  int width;
  int heigh;
  /* data */
public:
  prm(/* args */) {};
  ~prm() {};
  std::vector<Pose2f> prmPlan(const Pose2f& start, const Pose2f& end) {
    std::vector<Pose2f> ret_path;
    generateRandomPose(ret_path);
    std::set<std::pair<int, int>> edges; // 使用set来存储边，以自动去重
    generateEdge(ret_path, edges);
    
  }
private:
  void generateRandomPose(std::vector<Pose2f> &poseList) {
    poseList.clear();
    Pose2f tem_pose;
    for (size_t i = 0; i < numSamples; ++i) {
      tem_pose.x = (float)rand() / RAND_MAX * width;
      tem_pose.y = (float)rand() / RAND_MAX * heigh;
      if (!isObstacle(tem_pose)) poseList.emplace_back(tem_pose);
    }
    return;
  }
  void generateEdge(std::vector<Pose2f> &poseList, std::set<std::pair<int, int>> &edges) {
    for (size_t i = 0; i < poseList.size() - 1; ++i) {
      for (size_t j = i + 1; j < poseList.size(); ++j) {
        if (distance(poseList[i], poseList[j]) <= connectionRadius && !isObstacle(poseList[i], poseList[j])) edges.insert({i, j});
      }
    }
  }
  
};
