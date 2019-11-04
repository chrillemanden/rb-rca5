#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>


using namespace std;

vector<float> init_means(int k, int max, int min)
{
    vector<float> means;
    for(
        int i = 0; i < k; i++)
    {
        means.push_back(min + i * (max-min)/(k-1));
    }

    return means;
}

float get_distance(int dataPoint, float mean)
{
    float distance = abs((float) dataPoint - mean);

    return distance;
}

vector<float> update_mean(vector<float> means, vector<vector<int>> clustered_data)
{
    for(unsigned int i = 0; i < means.size(); i++)
    {
        float sum = std::accumulate(clustered_data[i].begin(), clustered_data[i].end(), 0);
        cout << "sum: " << sum;
        if(clustered_data.size() > 0)
        {
            means[i] = sum/(float)clustered_data[i].size();
        }

    }


    return means;
}

vector<vector<int>> assign_to_mean(vector<float> means, vector<int> data)
{
    vector<int> empty_vec(0,0);
    vector<vector<int>> output_vec(means.size(), empty_vec);

    for(unsigned int i = 0; i < data.size(); i++)
    {
        int index = 0;
        float distance;
        float min_distance = get_distance(data[i], means[0]);
        for(unsigned int j = 1; j < means.size(); j++)
        {
            distance = get_distance(data[i], means[j]);

            if(distance < min_distance) //add compare;
            {
                index = j;
                min_distance = distance;
            }
        }

        output_vec[index].push_back(data[i]);
    }

    return output_vec;
}

vector<float> KMeansClustering(vector<int> data, int k, int iterations)
{
    int min_list = *std::min_element(data.begin(), data.end());
    int max_list = *std::max_element(data.begin(), data.end());

    //Initialize clusters*/
    vector<float> means = init_means(k, max_list, min_list);



    //create histogram
    vector<int> hist_vec((max_list - min_list+1), 0);

    for(unsigned int i = 0;i < data.size(); i++)
    {
        hist_vec[data[i]-min_list]++;
    }

    //print histogram

    for(unsigned int i = 0; i < hist_vec.size(); i++)
    {
        cout << hist_vec[i] << endl;
    }

    cout << "before iteration 0 " << endl;

    for(unsigned int j = 0; j < means.size(); j++)
    {
        cout << means[j] << endl;
    }


    for(int i = 0; i < iterations; i++)
    {
        vector<vector<int>> clustered_data = assign_to_mean(means, data);
        means = update_mean(means, clustered_data);

        /*cout << "iteration: " << i << endl;
        for(unsigned int j = 0; j < means.size(); j++)
        {
            cout << means[j] << endl;
        }*/
    }

    //round means
    for(int i = 0; i < means.size(); i++)
    {
        means[i] = cvRound(means[i]);
    }

    cout << "finished clustering" << endl;

    return means;
}
