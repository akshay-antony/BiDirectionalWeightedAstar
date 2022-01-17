#include "BAstar.h"

// check if point is inside the map
bool is_valid(const float x, const float y){
    if(x < gp::height && y < gp::width && x >= 0 && y >= 0)
        return true; 
    return false;
}
// check if the point is an obstacle
bool is_not_obstacle(const float x, const float y, std::map<std::pair<float,float>,std::vector<float>>& all_points){
    if(all_points[{x,y}][5] == 0)
        return true;
    return false;
}

// check if the point is already explored
bool is_in_closed_set(const float x, const float y, const std::set<std::pair<float,float>>& closed_points){
    bool res = (closed_points.find({x,y}) != closed_points.end())?true:false;
    return res;
}

// function builds the whole map with parent_x, parent_y, f, g, h, is_obstacle values 
void make_graph(std::vector<std::pair<int,int>>& input_obstacles,
                   std::map<std::pair<float,float>,std::vector<float>>& all_points){
    for(int i=0;i<gp::height;++i){
        for(int j=0;j<gp::width;++j){
            all_points[{i,j}] = {-1, -1, FLT_MAX, FLT_MAX, FLT_MAX,0};
        }
    }
    for(auto x: input_obstacles){
        all_points[x][5] = 1;
    }
}

// function to find heuristics with eucledian distance
float find_h(const float x, const float y,const bool forward){
    if(forward)
        return sqrt(pow(gp::goal_point.first - x, 2) + pow(gp::goal_point.second - y, 2));
    else
        return sqrt(pow(gp::start_point.first - x, 2) + pow(gp::start_point.second - y, 2));
}

std::mutex m;

// explores the current cell and adds it open_list if req
void expand_cell(float x, float y, std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points,
    std::set<std::pair<float,float>>& closed_points, std::pair<float,float>& parent, float distance, bool forward){
    if(is_valid(x,y) && is_not_obstacle(x, y, all_points) && !is_in_closed_set(x, y, closed_points)){
        float curr_h = find_h(x,y,forward);
        float curr_g = all_points[parent][4] + distance;
        //weighted a star part
        float curr_f = (curr_g <= curr_h)?curr_g+curr_h:curr_g + ((2*gp::w-1)*curr_h)/gp::w;
        //float curr_f = curr_g + curr_h;
        if(curr_f < all_points[{x,y}][2]){   
            all_points[{x,y}] = {parent.first, parent.second, curr_f, curr_h, curr_g, 0};
            open_points.push({curr_f, x, y});
            //m.lock();
            if(forward)
                gp::visited_fwd.insert({x,y});
            else
                gp::visited_bwd.insert({x,y});
            //m.unlock();
        }
    }
}

// Astar planning part
void plan(std::map<std::pair<float,float>,std::vector<float>>& all_points, gp::pq& open_points, 
    std::set<std::pair<float,float>>& closed_points, bool forward){
    while(!open_points.empty()){
        std::vector<float> curr = open_points.top();
        open_points.pop();
        std::pair<float, float> curr_point = {curr[1],curr[2]};

        //m.lock();
        if(gp::found)
            return;
        if((gp::visited_fwd.find(curr_point) != gp::visited_fwd.end() && 
            gp::visited_bwd.find(curr_point) != gp::visited_bwd.end())){
            std::cout<<"Path Found"<<std::endl; // and explored: "<<closed_points.size()+open_points.size()<<" "<<forward<<std::endl;
            //std::cout<<"Meet point "<<curr[1]<<" "<<curr[2]<<std::endl;
            gp::meet_point = curr_point;
            gp::found = true;
            break;
        }
        if(is_in_closed_set(curr[1], curr[2], closed_points))
            continue;
        closed_points.insert(curr_point);

        // explores all the 8 neighbours
        expand_cell(curr[1]+1, curr[2], all_points, open_points, closed_points, curr_point, 1., forward);
        expand_cell(curr[1]+1, curr[2]-1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
        expand_cell(curr[1]+1, curr[2]+1, all_points, open_points, closed_points, curr_point, 1.414, forward);
        expand_cell(curr[1], curr[2]+1, all_points, open_points, closed_points, curr_point, 1., forward);
        expand_cell(curr[1], curr[2]-1, all_points, open_points, closed_points, curr_point, 1., forward);
        expand_cell(curr[1]-1, curr[2], all_points, open_points, closed_points, curr_point, 1., forward);
        expand_cell(curr[1]-1, curr[2]-1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
        expand_cell(curr[1]-1, curr[2]+1, all_points, open_points, closed_points, curr_point, 1.414f, forward);
    }
}

// trace back the path from the intersection point
void trace_path(std::map<std::pair<float,float>,std::vector<float>>& all_points_fwd, 
    std::map<std::pair<float,float>,std::vector<float>>& all_points_bwd){
        std::vector<std::pair<float,float>> path;
        float x = gp::meet_point.first;
        float y = gp::meet_point.second;
        path.push_back({x,y});
        std::cout<<"Path length: "<<all_points_fwd[{x,y}][4]+all_points_bwd[{x,y}][4]<<std::endl;
        while(x != gp::start_point.first || y != gp::start_point.second){
            float p1 = all_points_fwd[{x,y}][0];
            float p2 = all_points_fwd[{x,y}][1];
            x = p1;
            y = p2;
            path.push_back({x,y});
        }
        reverse(path.begin(),path.end());

        x = gp::meet_point.first;
        y = gp::meet_point.second;
        while(x != gp::goal_point.first || y != gp::goal_point.second){
            float p1 = all_points_bwd[{x,y}][0];
            float p2 = all_points_bwd[{x,y}][1];
            x = p1;
            y = p2;
            path.push_back({x,y});
        }
        // for(auto x: path)
        //     std::cout<<x.first<<" "<<x.second<<std::endl;
    }

int main(int argc, char* argv[]){
    // gp::start_point.first = std::atof(argv[1]);
    // gp::start_point.second = std::atof(argv[2]);
    if(argc == 5){
        gp::goal_point.first = std::atof(argv[1]);
        gp::goal_point.second = std::atof(argv[2]);
        gp::start_point.first = std::atof(argv[3]);
        gp::start_point.second = std::atof(argv[4]);
        gp::height = std::atof(argv[5]);
        gp::width = std::atof(argv[6]);
        gp::w = std::atof(argv[7]);
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::pair<int,int>> input_obstacles;
    input_obstacles.push_back({2, 2});
    input_obstacles.push_back({2, 3});
    // the value of the map is {x_parent, y_parent, f, h, g, is_obs}
    std::map<std::pair<float,float>,std::vector<float>> all_points_fwd, all_points_bwd;
    gp::pq open_points_fwd, open_points_bwd;
    std::set<std::pair<float,float>> closed_points_fwd, closed_points_bwd;
    bool found = false;
    make_graph(input_obstacles, all_points_fwd);
    all_points_bwd = all_points_fwd;

    if(!is_valid(gp::start_point.first, gp::start_point.second) || !is_valid(gp::goal_point.first, gp::goal_point.second)){
        std::cout<<"\n Start or Goal Point out of bounds...";
        return 0;
    }
    else if(!is_not_obstacle(gp::start_point.first, gp::start_point.second, all_points_fwd) 
            || !is_not_obstacle(gp::goal_point.first,gp::goal_point.second, all_points_fwd)){
        std::cout<<"\n Start or Goal Point is an obstacle...";
    }

    all_points_fwd[gp::start_point] = {gp::start_point.first, gp::start_point.second, 0., 0., 0., 0.};
    open_points_fwd.push({all_points_fwd[gp::start_point][2], gp::start_point.first, gp::start_point.second});
    all_points_bwd[gp::goal_point] = {gp::goal_point.first, gp::goal_point.second, 0., 0., 0., 0.};
    open_points_bwd.push({all_points_bwd[gp::goal_point][2], gp::goal_point.first, gp::goal_point.second});
    
    std::thread t1(plan, std::ref(all_points_fwd), std::ref(open_points_fwd), std::ref(closed_points_fwd), true);
    std::thread t2(plan, std::ref(all_points_bwd), std::ref(open_points_bwd), std::ref(closed_points_bwd), false);
    t1.join();
    t2.join();

    if(gp::found)
        trace_path(all_points_fwd, all_points_bwd);
    else
        std::cout<<"No path Found"<<std::endl;

    auto stop_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time-start_time);
    std::cout<<"Time Taken in milliseconds: "<<duration.count()<<std::endl;
    return 0;
}