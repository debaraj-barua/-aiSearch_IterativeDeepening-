/*
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
 *
 * agent.cpp
 * */

#include "agent.hpp"
#include <algorithm>
#include <iostream>
#include <string>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <stack>
#define map_rows 25
#define map_cols 141

using namespace std;

vector<vector<int>> depth_count_map; 	//--Used to check the last depth at which node was accessed.
										//To get optimum path.
pair<int,int> initial_pos_set;			//--Used to set the initial position
int goal_set;							//--Used to set goal

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
	sleep(1);
	initial_pos_set=initial_pos;
	goal_set=0;
	number_of_visited_nodes=0;

	//Loop until all goals are found
    for (int i=0; i <number_of_goals;i++)
    {
    	goal_set=i+1;
    	cout<<"\n Looking for Goal Number: "<<goal_set<<"\n";
		iterative_deepening_search();
		print_final_results();
		cout << "Press Enter to Continue to next goal" << endl;
		cin.get();
		cin.get();

    }

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
	//TO DO

    //Notes: 
    //Backtrack from here once you have found a goal.
    //If you have found a goal, do not forget to get a fresh copy of the map.
    //Stop searching if you have found a goal or reached the depth limit.
    //Only return true if a goal has been found.

	std::string goal1=std::to_string(goal);
    int max_rows = map.size();
    int max_cols = map[0].size();
    bool result=false;
	int row = current_node.first;		//stores row index of current node
	int col = current_node.second;		//stores column index of current node



    depth_count_map[row][col] = current_level;


	if(map[row][col] != "-" and (map[row][col]!="=") and (map[row][col]!="|"))
	{
		number_of_visited_nodes++;		//increase size of visited node as one node is being explored
	}

	if(current_level>limit)
	    {
	    	return false;
	    }

		/*If Goal is found*/
		if  (map[row][col]==goal1)
		{
			initial_pos_set=make_pair(row,col);     //Reset Initial position as current goal position
			current_path.push_back(current_node);
			deepest_level=current_path.size();

			backtrack_path(current_path);			//Print path to solution
			return true;
		}

		 map[row][col] = "-";
		 current_path.push_back(current_node);
		 deepest_level=current_path.size();

		/*********************************************************************
		 * "If" statements below are used to find the children of current node
		 * Child nodes which are legal and unexplored function is recursively called
		 *********************************************************************/

		if (((row+1)<max_rows and (row+1)>0) and ((map[row+1][col]!="=") and (map[row+1][col]!="|"))
			 and result!=true)
		{

			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row+1][col] > current_level + 1)
			{
			result=recursive_dls(make_pair(row+1, col), goal, current_level+1, limit, current_path);
			}
		}

		if (((row-1)<max_rows and (row-1)>0)and ((map[row-1][col]!="=") and (map[row-1][col]!="|"))
				 and result!=true)
		{

			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row-1][col] > current_level + 1)
			{
			result=recursive_dls(make_pair(row-1, col), goal, current_level+1, limit, current_path);
			}

		}

		if (((col+1)<max_cols and (col+1)>0)and ((map[row][col+1]!="=") and (map[row][col+1]!="|") )
				 and result!=true)
		{
			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row][col+1] > current_level + 1)
			{
			result=recursive_dls(make_pair(row, col+1), goal, current_level+1, limit, current_path);
			}


		}

		if (((col-1)<max_cols and (col-1)>0)and ((map[row][col-1]!="=") and (map[row][col-1]!="|") )
				 and result!=true)
		{
			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row][col-1] > current_level + 1)
			{
			result=recursive_dls(make_pair(row, col-1), goal, current_level+1, limit, current_path);
			}
		}

	if (result==true)
	{
		return true;
	}
	else
	{
		return false;
	}

}

bool Agent::depth_limited_seach(int limit)
{
	map = empty_map;					//reset map to empty map
    vector<pair<int,int>> current_path;
    pair<int,int> current_node;
	int goal, current_level;
	number_of_visited_nodes=0;
	deepest_level=0;
	current_level=0;

	current_node=make_pair(initial_pos_set.first, initial_pos_set.second);
    current_path.push_back(current_node);

    goal=goal_set;

    /***************************************************************************************
     *@ depth_count_map::
     *
     * This vector is used to find the optimum solution. This vector is of the size of the map.
     * Each element here stores a large value.
     * When we encounter a new node, we will check if that node has been visited earlier by a shorter path,
     * if yes, we do not expand it again, else we expand the node as current path is more optimal than earlier
     *****************************************************************************************/
    depth_count_map = vector<vector<int>>(map.size(), vector<int>(map[0].size(), 5000));

	bool result=recursive_dls(current_node, goal, current_level+1, limit, current_path);
	if (result==true)
		return true;
	else
		return false;
}

void Agent::iterative_deepening_search()
{
    //TO DO
	int i=0;
	bool result=false;

	while(result ==false )
	{
		result=depth_limited_seach(i+1);
		/*
		 *If all unique nodes in the map have already been checked at this depth,
		 *and solution was still not found, we break out of loop as goal is unreachable.
		 */
		if(number_of_visited_nodes > max_limit)
		{

			cout <<"Goal number " << goal_set << " not reachable \n";
			break;
		}
		i++;

	}
}

void Agent::print_final_results()
{
    cout << "Deepest level reached: " << deepest_level  << endl;
    //cout << "Total of stored nodes: " << total_of_stored_nodes << endl;
    cout << "Total of visited nodes: " << number_of_visited_nodes << endl;
}

void Agent::backtrack_path(vector<pair<int,int>> current_path)
{
    //use the original map to backtrace
    vector<vector<string>> local_map = empty_map;
    
    pair<int,int> current_data; 
    //TO DO
    //Backtrace. Use the current path vector to set the path on the map.

    for (auto i = current_path.begin(); i != current_path.end(); i++)
    {

    	current_data=*i;
		if (local_map[current_data.first][current_data.second]==" ")
			local_map[current_data.first][current_data.second]="-";
    	print_map(local_map);
    }

}
