/*
 * Daniel Vazquez
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
 *
 * agent.cpp
 * */

#include "agent.hpp"

#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>

#define map_rows 25
#define map_cols 141

using namespace std;

Agent::Agent
    (
    vector<vector<string>> selected_map,
    const pair<int,int> initial_pos,
    int number_of_goals
    ):
    empty_map(selected_map),
    map(selected_map),
    initial_pos(initial_pos),
    number_of_goals(number_of_goals),
    max_number_of_stored_nodes(0),
    number_of_visited_nodes(0),
    total_of_stored_nodes(0),
    deepest_level(0)
{
    max_limit = map_rows * map_cols;
    print_map(empty_map);
}

Agent::~Agent()
{
}

void Agent::run()
{
    cout << "Running IDFS " << endl;
    cout << "Number of goals " << number_of_goals << endl;
    sleep(1);
    //iterative_deepening_search();
}

void Agent::print_map(vector<vector<string>>& a_map)
{
    system("clear");
    int row = initial_pos.first;
    int col = initial_pos.second;

    for(int row = 0; row < map_rows; row++)
    {
        for(int col = 0; col < map_cols; col++)
        {
            cout << a_map[row][col];
        }    
        cout << endl;
    }
    this_thread::sleep_for(chrono::milliseconds(20));
}


bool Agent::recursive_dls(pair<int,int> current_node, int goal, int current_level,
                          int limit, vector<pair<int,int>> current_path)
{
    //TODO
    
    //Notes: 
    //Backtrack from here once you have found a goal.
    //If you have found a goal, do not forget to get a fresh copy of the map.
    //Stop searching if you have found a goal or reached the depth limit.
    //Only return true if a goal has been found.
    
    return true;
}

bool Agent::depth_limited_seach(int limit)
{
    vector<pair<int,int>> current_path; 
    //TODO
    return true;
}

void Agent::iterative_deepening_search()
{
    //TODO
}

void Agent::print_final_results()
{
    cout << "Deepest level reached: " << deepest_level  << endl;
    cout << "Total of stored nodes: " << total_of_stored_nodes << endl;
    cout << "Total of visited nodes: " << number_of_visited_nodes << endl;
}

void Agent::backtrack_path(vector<pair<int,int>> current_path)
{
    //use the original map to backtrace
    vector<vector<string>> local_map = empty_map;
    
    pair<int,int> current_data; 
    //TODO
    //Backtrace. Use the current path vector to set the path on the map. 
    
    //print backtraced path
    print_map(local_map);
}
