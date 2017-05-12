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
bool flag_check=true;					//Used to check for unsuccessful searches

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

	flag_check=true;
    cout << "Running IDFS " << endl;
	sleep(1);
	initial_pos_set=initial_pos;
	goal_set=0;
	number_of_visited_nodes=0;

	//Loop until all goals are found
    for (int i=0; i <number_of_goals;i++)
    {
    	goal_set=goal_set+1;
    	cout<<"\n Looking for Goal Number: "<<goal_set<<"\n";
    	print_map(map);
    	iterative_deepening_search();
    	if (flag_check==false)
    	{
			i=i-1; //set repeat for same initial node if last node was not found
			flag_check=true;
			continue;
    	}
    	else
    	{
    	for (int x=0; x<map.size();x++)
    	{
    		for (int y=0;y<map[0].size();y++)
    		{
    			if (map[x][y]==std::to_string(goal_set))
    				initial_pos_set=make_pair(x, y); //set initial position as the current goal position

    		}
    	}
       	print_final_results();
		cout << "Press Enter to Continue" << endl;
		cin.ignore();
		cin.get();
    	}
    }

//	iterative_deepening_search();

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


    depth_count_map[current_node.first][current_node.second] = current_level;
	number_of_visited_nodes++;			//increase size of visited node as one node is being explored

	int row = current_node.first;		//stores row index of current node
	int col = current_node.second;		//stores column index of current node

	if(current_level>limit)
	    {
	    	return false;
	    }


		if  (map[row][col]==goal1)
		{

			current_path.push_back(current_node);
			deepest_level=current_path.size();
			backtrack_path(current_path);
			return true;
		}

		/*********************************************************************
		 * "If" statements below are used to find the children of current node
		 * Child nodes which are legal and unexplored are added to stack
		 *********************************************************************/
		current_path.push_back(current_node);
		if (((row+1)<max_rows and (row+1)>0) and ((map[row+1][col]!="=") and (map[row+1][col]!="|"))
			 and result!=true)
		{

			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached
			if(depth_count_map[row+1][col] > current_level + 1)
			{
			result=recursive_dls(make_pair(row+1, col), goal, current_level+1, limit-1, current_path);
			}
		}

		if (((row-1)<max_rows and (row-1)>0)and ((map[row-1][col]!="=") and (map[row-1][col]!="|"))
				 and result!=true)
		{

			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached
			if(depth_count_map[row-1][col] > current_level + 1)
			{
			result=recursive_dls(make_pair(row-1, col), goal, current_level+1, limit-1, current_path);
			}

		}

		if (((col+1)<max_cols and (col+1)>0)and ((map[row][col+1]!="=") and (map[row][col+1]!="|") )
				 and result!=true)
		{
			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row][col+1] > current_level + 1)
			{
			result=recursive_dls(make_pair(row, col+1), goal, current_level+1, limit-1, current_path);
			}


		}

		if (((col-1)<max_cols and (col-1)>0)and ((map[row][col-1]!="=") and (map[row][col-1]!="|") )
				 and result!=true)
		{
			//Explore nodes which are already explored if the current depth is less
			//than the previous depth at which this node was reached

			if(depth_count_map[row][col-1] > current_level + 1)
			{
			result=recursive_dls(make_pair(row, col-1), goal, current_level+1, limit-1, current_path);
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
    vector<pair<int,int>> current_path;
    pair<int,int> current_node;
	int goal, current_level;
	number_of_visited_nodes=0;
	deepest_level=0;

	current_node=make_pair(initial_pos_set.first, initial_pos_set.second);
    current_path.push_back(current_node);
	goal=goal_set;

	current_level=0;
	depth_count_map = vector<vector<int>>(map.size(), vector<int>(map[0].size(), max_limit));

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
	bool result;

	for (i=0; i<max_limit; i++)
	{
		result=depth_limited_seach(i+1);

		if (result==true)
		{
			cout<<"Found!";
			break;
		}
		else
		{
			continue;
		}


	}
	if (i==max_limit)
	{
		cout<<"\n Goal "<<goal_set<<" not found";
		flag_check=false;

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

    //print backtraced path
    //print_map(local_map);
}
