#include <iostream>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <cmath>
#include <float.h>
#include <opencv2/opencv.hpp>
#include "map_util.h"
using namespace std;

//maze properties
int vertical_lines = 10;
int horisontal_lines = 6;

int maze_width = vertical_lines-1;
int maze_height = horisontal_lines-1;
int numberOfActions = 4;

int numberOfKeyCombinations = 2;
int key_xpos = 8;
int key_ypos = 2;

int terminal_state_ypos = 0;
int terminal_state_xpos = 8;

int objects_found;



vector<vector<int>> xy_actions = {{0, -1},{0, 1},{1,0},{-1,0}};

vector<vector<vector<bool>>> valid_actions(maze_width, vector<vector<bool>>(maze_height, vector<bool>(numberOfActions, false)));
vector<vector<vector<vector<double>>>> Q_matrix(maze_width, vector<vector<vector<double>>>(maze_height, vector<vector<double>>(numberOfKeyCombinations, vector<double>(numberOfActions, -1.0))));

double get_reward(vector<int> state_vec, vector<int> next_state_vec) //int pos_x, int pos_y)
{
    if(next_state_vec[0] == key_xpos && next_state_vec[1] == key_ypos && state_vec[2] == 0) //keyNotfound
    {
        cout << "key found" << endl;
        objects_found = objects_found +1;
        return 3.0;
    }
    else if(next_state_vec[0] == terminal_state_xpos && next_state_vec[1] == terminal_state_ypos)
    {
        if(state_vec[2] == 0)
        {
           cout << "object found" << endl;
           objects_found = objects_found +1;
           return 5.0;
        }
        else
        {
            cout << "object found" << endl;
            objects_found = objects_found +1;
            return 25.0;
        }

    }
   return 0.0;
}


bool terminate_episode(vector<int> state_vec)
{
    if(state_vec[0] == terminal_state_xpos && state_vec[1] == terminal_state_ypos)
    {
        return true;
    }

    return false;
}

int get_maximal_action(vector<int> state_vec)
{
    vector<double> action_values = Q_matrix[state_vec[0]][state_vec[1]][state_vec[2]];

    int max_action_index = 0;
    double max_value = DBL_MIN;

    for(unsigned int i = 0; i < action_values.size(); i++)
    {
        if(action_values[i] > max_value && valid_actions[state_vec[0]][state_vec[1]][i] == true)
        {
            max_value = action_values[i];
            max_action_index = i;
        }
    }


    return max_action_index;
}

vector<int> get_next_state(vector<int> state_vec, int action)
{
    vector<int> output_vec;

    //check if next state is valid
    if(valid_actions[state_vec[0]][state_vec[1]][action] == true)
    {
        int next_x_pos = state_vec[0] + xy_actions[action][0];
        int next_y_pos = state_vec[1] + xy_actions[action][1];

        output_vec.push_back(next_x_pos);
        output_vec.push_back(next_y_pos);

        if(next_x_pos == key_xpos && next_y_pos == key_ypos)
        {
            output_vec.push_back(1);
        }
        else
        {
            output_vec.push_back(state_vec[2]);
        }

    }
    else
    {
        cout << "not valid next state" << endl;
        output_vec = state_vec;
        output_vec[2] = action;
    }

    return output_vec;

}

int get_random_action(vector<int> state_vec)
{
    int random_index =  rand() % 4;
    while(valid_actions[state_vec[0]][state_vec[1]][random_index] != true )
    {
        random_index = rand() % 4;
    }

    return random_index;

}

void create_Q_matrix()
{
    default_random_engine generator(0);
    uniform_real_distribution<double> distribution(0.0, 1.0);

    for(int x_coor = 0; x_coor < maze_width; x_coor++)
    {
        for(int y_coor = 0; y_coor < maze_height; y_coor++)
        {
            for(int action = 0; action < numberOfActions; action++)
            {
                bool validAction = valid_actions[x_coor][y_coor][action];
                if(validAction == true)
                {
                    for(int i = 0; i < numberOfKeyCombinations; i++)
                    {
                        if(valid_actions[x_coor][y_coor][action] == true)
                        {
                            Q_matrix[x_coor][y_coor][i][action] = distribution(generator); //fix random number
                        }
                    }
                }

            }
        }
    }

}


void print_Q_matrix()
{
    vector<string> pos_actions = {"UP", "DOWN", "RIGHT", "LEFT"};
    cout << "printing Q-matrix" << endl;

    for(int key_config = 0; key_config < numberOfKeyCombinations; key_config++)
    {
        for(int action = 0; action < numberOfActions; action++)
        {
            cout << "Q_matrix number:  " << key_config * numberOfActions + action + 1 << endl;
            cout << "Key Configuration : " << pos_actions[key_config] << endl;
            cout << "Action:         : " << pos_actions[action] << endl;

            for(int y = 0; y < maze_height; y++)
            {
                for(int x = 0; x < maze_width; x++)
                {
                    cout << Q_matrix[x][y][key_config][action] << "     ";
                }

                cout << "\n\n";
            }
        }

    }

}

void update_valid_action_vector(std::vector<std::vector<std::vector<bool>>> grid_actions)
{
    for(unsigned int y = 0; y <  grid_actions.size(); y++)
    {
        for(unsigned int x = 0; x < grid_actions[1].size(); x++)
        {
            for(int i = 0; i < numberOfActions; i++)
            {
                valid_actions[x][y][i] = grid_actions[y][x][i];
            }
        }
    }

}



int main()
{
    cv::Mat img_floor_plan =cv::imread("floor_plan.png", cv::IMREAD_COLOR) ;
    cv::imshow("Floor plan", img_floor_plan);

    cv::Mat reduced_waupoints = img_floor_plan.clone();
    cv::Mat gray_map;
    cv::cvtColor(img_floor_plan, gray_map, cv::COLOR_BGR2GRAY);

    cv::Mat only_grid;
    cv::Mat img_floor_plan_grid;

    // Create the grid for the map with the chosen grid size

    createGridMap(img_floor_plan, vertical_lines, horisontal_lines, only_grid, img_floor_plan_grid);

    std::vector<std::vector<std::vector<bool>>> grid_actions(maze_height, std::vector<std::vector<bool>>(maze_width, std::vector<bool>(4, false)));
    std::vector<cv::Point2i> visual_actions;
    getValidGridActions(only_grid, vertical_lines, horisontal_lines, gray_map, visual_actions, grid_actions);
    update_valid_action_vector(grid_actions);
    //print_valid_state_matrix();
    //cv::imshow("test", img_floor_plan);

    cv::waitKey(0);

    int max_steps_episodes = 15;
    int max_episodes = 100;
    int print_every_x_episode = 5;


    vector<double> score_vector;
    std::ofstream optimal_path_file;
    std::ofstream score_file;
    optimal_path_file.open("optimal_path.csv");
    score_file.open("score.csv");

    double epsilon = 0.7;
    double lr_rate = 0.2;
    double discount_rate = 0.90;

    default_random_engine generator;
    uniform_real_distribution<double> distribution(0.0, 1.0);


    create_Q_matrix();
    cout << "Initial Q-Matrix " << endl;
    print_Q_matrix();

    for(int episode = 0; episode < max_episodes; episode++)
    {
        cout << "episode has begun" << endl;
        epsilon = epsilon * ((double) max_episodes - (double) episode)/ (double) max_episodes;

        if(epsilon < 0.2)
        {
            epsilon = 0.2;
        }


        double score = 0;
        objects_found = 0;
        int steps = 0;
        vector<int> state = {4,2,0};
        vector<int> state_ = state;
        int action; //current_action
        int action_; //next action
        double reward;  //


        // Run a episode

        while(steps < max_steps_episodes && terminate_episode(state) == false)
        {
            state = state_;
            cout << "state : " << state[0] << "  -  " << state[1] << "  -  " << state[2] << endl;

            double random_number = distribution(generator);

            if(random_number < epsilon && episode % print_every_x_episode != 0)
            {
                action = get_random_action(state);
            }
            else
            {
                action = get_maximal_action(state);
            }

            state_ = get_next_state(state, action);
            reward = get_reward(state, state_);
            action_ = get_maximal_action(state_);


            double new_Qvalue = Q_matrix[state[0]][state[1]][state[2]][action] + lr_rate * (reward + discount_rate * Q_matrix[state_[0]][state_[1]][state_[2]][action_] - Q_matrix[state[0]][state[1]][state[2]][action]);
            Q_matrix[state[0]][state[1]][state[2]][action] = new_Qvalue;

            if(episode % print_every_x_episode == 0)
            {
                //optimal_path_file << "state - X: " << state[0] << " Y: " <<  state[1] <<endl;

            }


            //state = state_;
            score = score + pow(discount_rate, (double) steps)*reward;
            steps++;


        }

        if(episode % print_every_x_episode == 0)
        {
            optimal_path_file << "Episode " << episode << ": " << score << " Objects found: " << objects_found << endl;
        }
        else
        {
            score_vector.push_back(score);
            score_file << score << ", ";
        }


    cout << "Episode ended " << endl;

    }
    cout << "Hello World!" << endl;


    cout << "Terminal Q-matrix" << endl;
    print_Q_matrix();
    optimal_path_file.close();
    score_file.close();

    return 0;
}
