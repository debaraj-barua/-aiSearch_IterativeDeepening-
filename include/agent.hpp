/*
 * Daniel Vazquez
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
 *
 * agent.hpp
 * */
 
#ifndef AGENT_HPP
#define AGENT_HPP

#include <vector>
#include <utility>
#include <string>

using namespace std;

class Agent
{
public:
    Agent(vector<vector<string> >, const pair<int,int>, int);
    ~Agent();
    void run();

private:
    vector<vector<string>> map;       //Holds a working copy of the map
    vector<vector<string>> empty_map; //Holds a fresh copy of the map
    const pair<int, int> initial_pos; //Inital position of the robot
    int number_of_goals;              
    int max_number_of_stored_nodes;    //Some metrics
    int number_of_visited_nodes;
    int deepest_level; 
    int total_of_stored_nodes;
    int max_limit;                     //Holds the allowed depth limit
    void print_map(vector<vector<string>>& a_map);
    
    /* These methods are based on the ones provided by the book*/
    bool recursive_dls(pair<int, int> current_node, int goal, int current_level,
                       int limit, vector<pair<int,int>> current_path);
    bool depth_limited_seach(int limit);
    void iterative_deepening_search();
    
    void print_final_results();
    void backtrack_path(vector<pair<int, int> > current_path);
};

#endif
