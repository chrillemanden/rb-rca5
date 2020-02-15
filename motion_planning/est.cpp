#include "est.h"

#include <algorithm>


std::default_random_engine generator;

void showImage(std::string image_name, cv::Mat mat)
{
    cv::namedWindow(image_name, cv::WINDOW_NORMAL);
    imshow(image_name, mat);
}


// Helper function
int pi_t(std::vector<vertex> vertices)
{
    float sum = 0;

    for (auto &vertex : vertices)
    {
        sum += vertex.weight;
    }

    float w_index = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/sum));

    int n = 0;
    for (auto &vertex : vertices)
    {
        w_index -= vertex.weight;
        if (w_index < 0.0)
            return n;
        n++;
    }
    return 0;
}



EST::EST()
{

}

EST::EST(vertex q_start, cv::Mat map) : map(map), h(map.rows), w(map.cols)
{
    // Build the tree with the initial configuration
    vertices.push_back(q_start);
}

vertex EST::extendEST(cv::Mat map_show)
{
    // Loop until the tree has been extended with a valid new configuration
    while (true)
    {
        // Sample a random vertex from the vertex vector with a higher probability for choosing unexplored vertices
        int q_index = pi_t(vertices);
        vertex q = vertices[q_index];

        // Build the random generators for generating new vertices
        std::uniform_int_distribution<int> move_y(-h/10, h/10);
        std::uniform_int_distribution<int> move_x(-w/10, w/10);

        // find qnew close to q
        vertex q_new;
        q_new.x_pos = q.x_pos + move_x(generator);
        q_new.y_pos = q.y_pos + move_y(generator);

        // Check if there is a collision free path from q to qnew
        if (collFreePath(q_new,q,map_show))
        {
            number_edges++;

            // Update the newly found vertex and add it to the the tree
            q_new.n_conn += 1;
            q_new.weight = (double)number_edges / (double)q_new.n_conn;
            q_new.origin_indx = q_index;
            vertices.push_back(q_new);

            // Update the sampled vertex
            vertices[q_index].n_conn += 1;
            vertices[q_index].weight = (double)number_edges / (double)vertices[q_index].n_conn;

            // Graphics
            cv::circle(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
            cv::line(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), cv::Point2i(q.x_pos,q.y_pos), cv::Scalar(0,127,127), 2);
            return q_new;
        }
    }
}

vertex EST::getClosest(vertex q_new)
{
    // Find the maximum distance in the map
    double dist = sqrt(h*h+w*w);

    vertex r;

    for (auto &ver : vertices)
    {
        int dist_x = q_new.x_pos - ver.x_pos;
        int dist_y = q_new.y_pos - ver.y_pos;
        if (sqrt(dist_x*dist_x + dist_y*dist_y) < dist)
        {
            r = ver;
            dist = sqrt(dist_x*dist_x + dist_y*dist_y);
        }
    }
    return r;
}

bool collFreePath(vertex q1, vertex q2, cv::Mat map)
{
    // Iterate through the line
    cv::LineIterator line_it(map, cv::Point2i(q1.x_pos, q1.y_pos), cv::Point2i(q2.x_pos, q2.y_pos), 8);
    for(int k = 0; k < line_it.count; line_it++, k++)
    {
        //check if a wall pixel is encountered
        if(map.at<cv::Vec3b>(line_it.pos()) == cv::Vec3b(0,0,0))
            return false;
    }
    return true;
}


void shortenPath(std::vector<vertex> &vertices, cv::Mat map)
{
    cv::Mat map_show = map.clone();

    // Graphics - show the path without the rest of the tree

    for (int i = 0; i < vertices.size() - 1; i++)
    {
    cv::circle(map_show, cv::Point2i(vertices[i].x_pos, vertices[i].y_pos), 4, cv::Scalar(0,0,255), 2, 8);
    cv::line(map_show, cv::Point2i(vertices[i].x_pos, vertices[i].y_pos), cv::Point2i(vertices[i+1].x_pos,vertices[i+1].y_pos), cv::Scalar(0,127,127), 2);
    }
    cv::circle(map_show, cv::Point2i(vertices[0].x_pos,vertices[0].y_pos), 7, cv::Scalar(0,255,0), 7, 8);
    cv::circle(map_show, cv::Point2i(vertices[vertices.size()-1].x_pos,vertices[vertices.size()-1].y_pos), 7, cv::Scalar(255,0,0), 7, 8);
    showImage("Shortest Path", map_show);
    cv::waitKey(2000);

    // Initialise a list for storing the path and save the first configuration
    std::vector<vertex> path;
    vertex q = vertices[0];
    path.push_back(q);

    // Graphics - first point
    cv::circle(map_show, cv::Point2i(q.x_pos, q.y_pos), 6, cv::Scalar(127,0,200), 3, 8);

    // Loop until the configuration before the end configuration is encountered
    int q_indx = 0;
    while(q_indx < vertices.size()-1)
    {
        // Loop from the current configuration to the end configuration
        for (int i = vertices.size()-1; i > q_indx; i--)
        {
            // If path from configurations is free store the configuration and break out of loop
            if (collFreePath(q, vertices[i], map))
            {
                // Graphics - show the shortened path
                cv::circle(map_show, cv::Point2i(vertices[i].x_pos, vertices[i].y_pos), 6, cv::Scalar(127,0,200), 3, 8);
                cv::line(map_show, cv::Point2i(q.x_pos, q.y_pos), cv::Point2i(vertices[i].x_pos,vertices[i].y_pos), cv::Scalar(127,0,200), 4);
                showImage("Shortest Path", map_show);
                cv::waitKey(400);

                q = vertices[i];
                path.push_back(q);
                q_indx = i;
                break;
            }
        }
    }
    vertices = path;
}

std::vector<vertex> isolatePath(EST start_tree, EST goal_tree, vertex q1, vertex q2, cv::Mat map, cv::Mat map_show)
{
    //cv::Mat map_show = map.clone();
    std::vector<vertex> path;
    vertex q;

    // Find the path from the start tree
    q = q1;
    while(q.origin_indx >= 0)
    {
        path.insert(path.begin(), q);
        q = start_tree.vertices[q.origin_indx];
    }
    path.insert(path.begin(), q);

    // Find the path from the goal tree
    q = q2;
    while(q.origin_indx >= 0)
    {
        path.push_back(q);
        q = goal_tree.vertices[q.origin_indx];
    }
    path.push_back(q);

    // Graphics - highlight found path as an overlay over the trees
    for (int i = 0; i < path.size() - 1; i++)
    {
        cv::circle(map_show, cv::Point2i(path[i].x_pos, path[i].y_pos), 6, cv::Scalar(0,0,255), 3, 8);
        cv::line(map_show, cv::Point2i(path[i].x_pos, path[i].y_pos), cv::Point2i(path[i+1].x_pos,path[i+1].y_pos), cv::Scalar(0,0,255), 4);
        showImage("Path", map_show);
        cv::waitKey(400);
    }
    cv::circle(map_show, cv::Point2i(path[-1].x_pos, path[-1].y_pos), 6, cv::Scalar(0,0,255), 3, 8);
    showImage("Path", map_show);
    cv::waitKey(2000);
    //showImage("Shortest Path", map);

    // Do post-processing on the path
    shortenPath(path, map);

    return path;
}

std::vector<vertex> ESTquery(vertex q_start, vertex q_goal, int n, cv::Mat map)
{
    cv::Mat map_show = map.clone();

    // Build the trees that will be used in the query
    EST start_tree(q_start, map);
    EST goal_tree(q_goal, map);

    // Graphics - show start and end configuration on the map
    cv::circle(map_show, cv::Point2i(q_start.x_pos,q_start.y_pos), 6, cv::Scalar(0,255,0), 3, 8);
    cv::circle(map_show, cv::Point2i(q_goal.x_pos,q_goal.y_pos), 6, cv::Scalar(255,0,0), 3, 8);
    showImage("Trees", map_show);
    cv::waitKey(2000);

    vertex q1;
    vertex q2;

    // Loop until max iterations for the algorithm has been reached
    for (int i = 0; i < n; i++)
    {
        // Extend the start tree with a new configuration and check if a connection to the goal tree exists
        q1 = start_tree.extendEST(map_show);
        q2 = goal_tree.getClosest(q1);

        // Graphics - highlight the new configuration and the configuration in the opposite tree closest too
        cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 4, cv::Scalar(255,80,255), 2, 8);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,0), 2, 8);
        showImage("Trees", map_show);
        cv::waitKey(30);

        if (collFreePath(q1, q2, map))
        {
            std::cout << "A path has been found in " << i << " steps!" << std::endl;

            // Graphics - connect the two trees
            cv::line(map_show, cv::Point2i(q1.x_pos,q1.y_pos), cv::Point2i(q2.x_pos,q2.y_pos), cv::Scalar(200,64,127), 2);
            showImage("Trees", map_show);
            cv::waitKey(2000);

            return isolatePath(start_tree,goal_tree,q1,q2, map, map_show);;
        }

        // Graphics revert highlighting
        cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,255), 2, 8);

        // Extend the goal tree with a new configuration and check if a connection to the start tree exists
        q1 = goal_tree.extendEST(map_show);
        q2 = start_tree.getClosest(q1);

        // Graphics - highlight the new configuration and the configuration in the opposite tree closest too
        cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 4, cv::Scalar(255,80,255), 2, 8);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,0), 2, 8);
        showImage("Trees", map_show);
        cv::waitKey(30);

        if (collFreePath(q1, q2, map))
        {
            std::cout << "A path has been found in " << i << " steps!" << std::endl;

            // Graphics - connect the two trees
            cv::line(map_show, cv::Point2i(q1.x_pos,q1.y_pos), cv::Point2i(q2.x_pos,q2.y_pos), cv::Scalar(200,64,127), 2);
            showImage("Trees", map_show);
            cv::waitKey(2000);

            return isolatePath(start_tree,goal_tree,q2,q1, map, map_show);
        }

        // Graphics - revert highlighting
        cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
    }

    // Path not found within max iterations, return empty list!
    std::vector<vertex> empty_path;
    return empty_path;
}


void buildSingleEST(vertex q_start, int n, cv::Mat &map)
{
    // Show the first point
    cv::Mat map_show = map.clone();
    cv::circle(map_show, cv::Point2i(q_start.x_pos,q_start.y_pos), 4, cv::Scalar(0,128,0), 2, 8);

    // Initialise lists for storing vertices and edges
    std::vector<vertex> vertices;
    vertices.push_back(q_start);


    // Build the random generators for generating new vertices
    std::uniform_int_distribution<int> move_y(-100, 100);
    std::uniform_int_distribution<int> move_x(-100, 100);
    //std::normal_distribution<int> move_y(10.0, 2.0);
    //std::normal_distribution<int> move_x(10.0, 2.0);

    int number_edges = 0;

    for (int i = 1; i <= n; i++)
    {

        // Sample a random vertex from the vertex vector with a higher probability for choosing unexplored vertices
        int q_index = pi_t(vertices);
        vertex q = vertices[q_index];


        // Extend the tree
        // find qnew close to q
        vertex q_new;
        q_new.x_pos = q.x_pos + move_x(generator);
        q_new.y_pos = q.y_pos + move_y(generator);


        /* Check if there is a collision free path from q to qnew */

        //Walk along the length of the line between the two points
        cv::LineIterator line_it(map, cv::Point2i(q_new.x_pos, q_new.y_pos), cv::Point2i(q.x_pos, q.y_pos), 8);

        for(int k = 0; k < line_it.count; line_it++, k++)
        {
            cv::Vec3b pixel_val = map.at<cv::Vec3b>(line_it.pos());
            //std::cout << "pixel_val" << pixel_val << std::endl;

            //check if a wall pixel is encountered
            if(pixel_val == cv::Vec3b(0,0,0))
            {
                //std::cout << "I: "<< k << "black pixel found" << std::endl;
                break;
            }

            //store the connection if no wall pixels is encountered along the length of line
            if(k + 1 == line_it.count)
            {
                number_edges++;
                q_new.n_conn += 1;
                //q_new.weight = ((double)vertices.size() + 1.0) / (double)q_new.n_conn;
                q_new.weight = (double)number_edges / (double)q_new.n_conn;
                //q_new.conn.push_back(*vertices[q_index]);
                vertices.push_back(q_new);
                //vertices[q_index].conn.push_back(*vertices[vertices.size()-1]);
                vertices[q_index].n_conn += 1;
                //vertices[q_index].weight = (double)vertices.size() / (double)vertices[q_index].n_conn;
                vertices[q_index].weight = (double)number_edges / (double)vertices[q_index].n_conn;

                cv::circle(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
                cv::line(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), cv::Point2i(q.x_pos,q.y_pos), cv::Scalar(0,127,127), 2);

                break;
            }
        }

        showImage("EST", map_show);

        cv::waitKey(1);

    }
}
