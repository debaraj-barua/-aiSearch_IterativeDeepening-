/*
 * Daniel Vazquez
 * Aritificial Intelligence for Robotics
 * SS 2016
 * Assignment 5
 *
 * environment.hpp
 * */

#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#define map_rows 25
#define map_cols 141



using namespace std;

class Environment
{
public:
    Environment();
    void run();
    int load_map(int map_index);
    void initialize_map();
    void print_map();

private:
    string map_names[3];
    string map_dir;
    string start_sym;
    int map_number_of_goals;
    vector<vector<string>> map;
    pair<int,int> initial_pos;
};

#endif
