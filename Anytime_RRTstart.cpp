#define _USE_MATH_DEFINES
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <exception>
#include <stdlib.h>
#include <ctime>
#include <stdio.h>
#include <limits>

using namespace std;

class Point{
    public:
        float x,y;

        Point(){}
        Point(float x_in, float y_in):x(x_in),y(y_in){}

        Point operator + (Point p){
            Point p_out(x+p.x, y+p.y);
            return p_out;
        }

        Point operator - (Point p){
            Point p_out(x-p.x, y-p.y);
            return p_out;
        }

        Point operator * (float scale){
            Point p_out(x*scale, y*scale);
            return p_out;
        }

        float distance_to(Point p){
            return sqrt(pow(x - p.x,2) + pow(y - p.y,2));
        }
};

class Node {
    // Paths are constructed using the tree data structure
    public:
        Point coord;
        Node* parent;
        vector<Node*> children;

        Node():parent(NULL){}
        Node(Point p){
            coord.x = p.x;
            coord.y = p.y;
            parent = NULL;
        }

};

class Obstacle{
    // Obstacles are assumed to be ellipses, where
    // a indicates the width in x-axis
    // b indicates the width in y-axis
    public:
        Point center;
        float a,b;

        Obstacle(float x, float y, float a_in, float b_in){
            center.x = x;
            center.y = y;
            a = a_in;
            b = b_in;
        }

        bool is_collided(Point p){
            float r = (pow(p.x-center.x,2)/pow(a,2)) + (pow(p.y-center.y,2)/pow(b,2)); 

            if (r <= 1){
                return true;
            }
            else{
                return false;
            }
        }
};

vector<float> linspace(float start, float end, int num){
    vector<float> out;
    out.clear();

    float step = (end-start)/(num-1);

    for (int i = 0; i < num; i++){
        out.push_back(start + i*step);
    }

    return out;
}

struct BoundException : public exception {
    const char * what () const throw () {
        return "Invalid Bounds!";
    }
};

struct StartException : public exception {
    const char * what () const throw () {
        return "Invalid Start Node!";
    }
};

struct GoalException : public exception {
    const char * what () const throw () {
        return "Invalid Goal Node!";
    }
};

class RRT{
    private:
        Point p_start;
        Point p_goal;
        Point p_sampled;
        Point p_new;

        Node* node_start;
        Node* node_goal;
        Node* node_nearest;
        Node* node_shortest_to_root_distance;
        vector<Node*> nodes_near;
        Node* node_new;

        float x_lb,x_ub,y_lb,y_ub;
        vector<Obstacle> obstacles;
        vector<Point> path;

        bool is_collided;
        float distance_step = 0.1;
        float r_group = 0.15;
        float r_goal = 0.1;
        int node_count = 0;
        int max_node_count = 20000;

        void SetParent(Node*& node, Node*& parent){
            if (node->parent != NULL){
                auto it_find = find((node->parent->children).begin(),(node->parent->children).end(),node);
                (node->parent->children).erase(it_find);
            }

            node->parent = parent;
            (parent->children).push_back(node);
        }

        void TracePath(){
            Node* node = node_goal;
            path.clear();
            while (node != NULL){
                path.insert(path.begin(),node->coord);
                node = node->parent;
            }
        }

        float DistanceToRoot(Node* node){
            float dis = 0;
            while (node != node_start){
                dis += node->coord.distance_to(node->parent->coord);
                node = node->parent;
            }
            return dis;
        }


        void Sampler(){
            float x_sampled , y_sampled;
            x_sampled = x_lb + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)/(x_ub-x_lb));
            y_sampled = y_lb + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX)/(y_ub-y_lb));
            p_sampled.x = x_sampled;
            p_sampled.y = y_sampled;
        }

        void FindNearestNode(){
            float distance;
            float distance_min = numeric_limits<float>::max();
            vector<Node*> current_layer;
            vector<Node*> next_layer;

            current_layer.clear();
            current_layer.push_back(node_start);

            while(current_layer.size() != 0){
                next_layer.clear();

                for (auto it = current_layer.begin(); it != current_layer.end(); it++){
                    distance = (*it)->coord.distance_to(p_sampled);
                    if (distance < distance_min){
                        node_nearest = (*it);
                        distance_min = distance;
                    }
                    next_layer.insert(next_layer.end(),((*it)->children).begin(),((*it)->children).end());
                }

                current_layer = next_layer;
            }
        }

        void FindNearNodes(){
            nodes_near.clear();
            vector<Node*> current_layer;
            vector<Node*> next_layer;

            current_layer.clear();
            current_layer.push_back(node_start);

            while(current_layer.size() != 0){
                next_layer.clear();

                for (auto it = current_layer.begin(); it != current_layer.end(); it++){
                    if ((*it)->coord.distance_to(p_new) <= r_group){
                        nodes_near.push_back(*it);
                    }
                    next_layer.insert(next_layer.end(),((*it)->children).begin(),((*it)->children).end());
                }

                current_layer = next_layer;
            }
        }

        

    public:
        RRT(Point start, Point goal, vector<Obstacle> obs, float x_b, float y_b):
        p_start(start),p_goal(goal),obstacles(obs),
        x_lb(-x_b),x_ub(x_b),
        y_lb(-y_b),y_ub(y_b)
        {
        
            if (x_lb > x_ub || y_lb > y_ub ||
                p_start.x > x_ub || p_start.x < x_lb ||
                p_goal.y > y_ub  || p_goal.y < y_lb){
                throw BoundException();
            }

            for (auto it = obstacles.begin(); it != obstacles.end(); it++){
                if ((*it).is_collided(p_start)){
                    throw StartException();
                }

                if ((*it).is_collided(p_goal)){
                    throw GoalException();
                }
            }

            srand (static_cast <unsigned> (time(0)));
            node_start = new Node(p_start);
            node_goal = new Node(p_goal);
             
        }

        vector<Point> output_path(){
            if (path.size() == 0){
                // Run RRT algorithm if path is initally empty
                while(node_count < max_node_count){
                    Sampler();
                    FindNearestNode();
                    if (node_nearest->coord.distance_to(p_sampled) <= distance_step){
                        p_new = p_sampled;
                    }
                    else{
                        p_new = node_nearest->coord + (p_sampled - node_nearest->coord)*(distance_step/node_nearest->coord.distance_to(p_sampled));
                    }

                    is_collided = false;
                    for (auto it = obstacles.begin(); it != obstacles.end(); it++){
                        if ((*it).is_collided(p_new)){
                            is_collided = true;
                            break;
                        }
                    }

                    if (!is_collided){
                        node_count += 1;
                        node_new = new Node(p_new);
                        SetParent(node_new,node_nearest);

                        if (p_new.distance_to(p_goal) <= r_goal){
                            SetParent(node_goal,node_new);
                            TracePath();
                            break;
                        }
                    }
                }
                return path;
            }
            else{
                // Return the currently found path 
                return path;
            }
        }

        void improve_path(int node_count_improve){
            int node_count_target = node_count + node_count_improve;
            float shortest_distance_to_root;
            float distance_to_root;
            float original_distance_to_root;
            float distance_to_root_via_new;

            vector<Node*>::iterator it_shortest_distance_to_root;
            
            if (path.size() != 0){
                // Run RRT* algorithm to improve the previously found path
                while(node_count < max_node_count){
            
                    Sampler();
                    FindNearestNode();
       
                    if (node_nearest->coord.distance_to(p_sampled) <= distance_step){
                        p_new = p_sampled;
                    }
                    else{
                        p_new = node_nearest->coord + (p_sampled - node_nearest->coord)*(distance_step/node_nearest->coord.distance_to(p_sampled));
                    }

                    is_collided = false;
                    for (auto it = obstacles.begin(); it != obstacles.end(); it++){
                        if ((*it).is_collided(p_new)){
                            is_collided = true;
                            break;
                        }
                    }
                    if (!is_collided){
                        node_count += 1;
                        node_new = new Node(p_new);

                        FindNearNodes();

                        shortest_distance_to_root = numeric_limits<float>::max();
                        for (auto it = nodes_near.begin(); it != nodes_near.end(); it++){
                            distance_to_root = p_new.distance_to((*it)->coord) + DistanceToRoot(*it);
                            if (distance_to_root < shortest_distance_to_root){
                                shortest_distance_to_root = distance_to_root;
                                node_shortest_to_root_distance = *it;
                                it_shortest_distance_to_root = it;
                            }
                            
                        }
            
                        SetParent(node_new,node_shortest_to_root_distance);
                        nodes_near.erase(it_shortest_distance_to_root);
   
                        for (auto it = nodes_near.begin(); it != nodes_near.end(); it++){
                            original_distance_to_root = DistanceToRoot(*it);
                            distance_to_root_via_new = (*it)->coord.distance_to(p_new) + DistanceToRoot(node_new);
                            if (distance_to_root_via_new < original_distance_to_root){
                                SetParent(*it,node_new);
                            }
                        }

        
                        
                        if (node_count == node_count_target){
                            break;
                        }
                        
                    }
                }

                TracePath();
                return;
            }
            else{
                return;
            }
        }


};


int main(){
    // Define output files for storing data from the planner and environment
    FILE* path_vanilla;
    FILE* path_improved;
    FILE* obstacles_data;
    FILE* point_start;
    FILE* point_goal;

    float x_bound  = 5;
    float y_bound = 5;
    vector<Point> path;

    // Define start and goal points
    Point p_start(5,-5);
    Point p_goal(0,0);

    // Define obstacles in the environment
    Obstacle obstacle1(0,4,4,0.5);
    Obstacle obstacle2(4,0,0.5,4);
    Obstacle obstacle3(0,-4,4,0.5);
    Obstacle obstacle4(-4,-1,0.5,3);
    Obstacle obstacle5(-1,2,3,0.5);
    Obstacle obstacle6(2,0,0.5,2);
    Obstacle obstacle7(0,-2,2,0.5);
    Obstacle obstacle8(-2,-1,0.5,1);
    vector<Obstacle> obstacles = {obstacle1,obstacle2,obstacle3,obstacle4,obstacle5,obstacle6,obstacle7,obstacle8};

    // Define necessary variables for plotting obstacles
    float r,a,b,theta,x,y,x_center,y_center;
    vector<float> theta_vec = linspace(0,2*M_PI,200);

    // vvvvvvvvvvvvvvvvv Anytime RRT* vvvvvvvvvvvvvvvvv
    RRT robot(p_start,p_goal,obstacles,x_bound,y_bound);

    // Obtaining a vanilla path
    path = robot.output_path();
    path_vanilla = fopen("path_vanilla.tmp","w");
    cout << path.size() << endl;
    for (auto it = path.begin(); it != path.end(); it++){
        fprintf(path_vanilla, "%f %f\n",(*it).x ,(*it).y);
    }
    fclose(path_vanilla);

    // Improving the path
    robot.improve_path(15000);
    path = robot.output_path();
    cout << path.size() << endl;
    path_improved = fopen("path_improved.tmp","w");
    for (auto it = path.begin(); it != path.end(); it++){
        fprintf(path_improved, "%f %f\n",(*it).x ,(*it).y);
    }
    fclose(path_improved);

    // ^^^^^^^^^^^^^^^^^ Anytime RRT* ^^^^^^^^^^^^^^^^^

    // Getting obstacle geometry for plotting
    obstacles_data = fopen("obstacles_data.tmp","w");
    for (auto obstacle_it = obstacles.begin(); obstacle_it != obstacles.end(); obstacle_it++){
        a = (*obstacle_it).a;
        b = (*obstacle_it).b;
        x_center = (*obstacle_it).center.x;
        y_center = (*obstacle_it).center.y;
        for (auto theta_it = theta_vec.begin(); theta_it != theta_vec.end(); theta_it++){
            theta = *theta_it;
            r = (a*b)/sqrt(pow(b*cos(theta),2) + pow(a*sin(theta),2));
            x = r*cos(theta) + x_center;
            y = r*sin(theta) + y_center;
            fprintf(obstacles_data, "%f %f\n",x ,y);
        }
        fprintf(obstacles_data, "\n");
    }
    fclose(obstacles_data);

    // Getting goal and end points locations for plotting
    point_start = fopen("point_start.tmp","w");
    point_goal  = fopen("point_goal.tmp","w");

    fprintf(point_start, "%f %f\n",p_start.x,p_start.y);
    fprintf(point_goal, "%f %f\n",p_goal.x,p_goal.y);

    fclose(point_start);
    fclose(point_goal);
    
    return 0;
}