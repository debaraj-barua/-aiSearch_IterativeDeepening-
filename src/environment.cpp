/*
 * Daniel Vazquez
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
 *
 * environment.cpp
 * */

#include "environment.hpp"
#include "agent.hpp"

Environment::Environment():
    map_dir("./maps/"),
    map(map_rows, vector<string>(map_cols,"")),
    start_sym("s"),
    map_number_of_goals(0)
{
	map_names[0] = "map1.txt";
    map_names[1] = "map2.txt";
    map_names[2] = "map3.txt";
}

void Environment::run()
{
    int map_number = 0;
    string input;
    int op = 0; //option to let user select if program should run again.
    do
    {
		do
		{
			cout << "Please select the map : " << endl;
			cout << "(1) Map1 (2) Map2 (3) Map3" << endl;
			cin >> input;
            //this converts char to int using ascii encoding
            map_number = (int) input[0] - 48;
		}
        while(cin.fail() || map_number < 1 || map_number > 3);

		//load map, create and run agent
		load_map(map_number - 1);
		Agent anAgent(map, initial_pos, map_number_of_goals);
		anAgent.run();

		do
		{
			cout << "Search again ?: " << endl;
			cout << "(1) Yes  (2) No " << endl;
			cin >> input;
            op = (int) input[0] - 48;
			if (op < 1 || op > 2)
            {
				cout << "Wrong option, please select again" << endl;
            }
		}
        while(op < 1 || op > 2);
	}
    while (op == 1);
}

//create the map based on the selected file
int Environment::load_map(int map_index)
{
    initialize_map();
    string line;
    ifstream aMap((map_dir + map_names[map_index]).c_str());

    int row = 0;

    if(aMap.is_open())
    {
        while(getline(aMap, line))
        {
            for(int col = 0; col < line.length(); col++)
            {
                map[row][col] = line[col];
              
                //Store the initial possition 
                if(line[col] ==  's')
                {
                    initial_pos.first = row;
                    initial_pos.second = col;
                    cout << "Initial Pos: " << initial_pos.first << ", "
                         << initial_pos.second << endl;
                }
                else if (isdigit(line[col]))
                { //get the number of goals in the given map
                    map_number_of_goals++;
                }
            }
            row++;
        }
        
        cout << "The selected map has been loaded" << endl;
        cout << "Number of goals in the map: " << map_number_of_goals << endl;
        aMap.close();
        //print_map();
        return 1;
    }
    return 0;
}

//Initialize the array to ""
void Environment::initialize_map()
{
    map_number_of_goals = 0;

    for(int row = 0; row < map_rows; row++)
    {
        for(int col = 0; col < map_cols; col++)
        {
            map[row][col] = "";
        }
    }
}

//print map
void Environment::print_map()
{
    for(int row = 0; row < map_rows; row++)
    {
        for(int col = 0; col < map_cols; col++)
        {
            cout << map[row][col];
        }
        cout << endl;
    }
}
