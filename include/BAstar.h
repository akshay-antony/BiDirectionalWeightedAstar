#pragma once
#include <iostream>
#include <vector>
#include <utility>
#include <bits/stdc++.h>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <cmath>
#include <queue>
#include <limits>
#include <cfloat>
#include <chrono>
#include <thread>
#include <mutex>

namespace gp{
    float height = 2000;
    float width = 2000;
    float w = 1.1;
    std::pair<float,float> start_point = {1, 1};
    std::pair<float,float> goal_point = {999, 999};
    std::set<std::pair<float,float>> visited_fwd;
    std::set<std::pair<float,float>> visited_bwd;
    bool found = false;
    std::pair<float,float> meet_point;
    typedef std::priority_queue<std::vector<float>, std::vector<std::vector<float>>, std::greater<std::vector<float>>> pq;
}

bool is_valid(const float x, const float y);
bool is_not_obstacle(const float x, const float y, std::map<std::pair<float,float>,std::vector<float>>& all_points);
bool is_in_closed_set(const float x, const float y, const std::set<std::pair<float,float>>& closed_points);
void make_graph(std::vector<std::pair<int,int>>& input_obstacles,
                   std::map<std::pair<float,float>,std::vector<float>>& all_points);
float find_h(const float x, const float y, const bool forward);
void expand_cell(float x, float y, std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points,
    std::set<std::pair<float,float>>& closed_points, std::pair<float,float>& parent, float distance, bool forward);
void plan(std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points, 
    std::set<std::pair<float,float>>& closed_points, bool forward);
void trace_path(std::map<std::pair<float,float>,std::vector<float>>& all_points_fwd, 
    std::map<std::pair<float,float>,std::vector<float>>& all_points_bwd);
