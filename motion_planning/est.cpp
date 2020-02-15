#include "est.h"


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
    vertices.push_back(q_start);
}

vertex EST::extendEST(cv::Mat &map_show)
{
    // Sample a random vertex from the vertex vector with a higher probability for choosing unexplored vertices
    while (true)
    {
        bool found_new_q = false;
        int q_index = pi_t(vertices);
        vertex q = vertices[q_index];

        // Build the random generators for generating new vertices
        std::uniform_int_distribution<int> move_y(-h/10, h/10);
        std::uniform_int_distribution<int> move_x(-w/10, w/10);

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
                q_new.origin_indx = q_index;
                //q_new.conn.push_back(*vertices[q_index]);
                vertices.push_back(q_new);
                //vertices[q_index].conn.push_back(*vertices[vertices.size()-1]);
                vertices[q_index].n_conn += 1;
                //vertices[q_index].weight = (double)vertices.size() / (double)vertices[q_index].n_conn;
                vertices[q_index].weight = (double)number_edges / (double)vertices[q_index].n_conn;

                cv::circle(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
                cv::line(map_show, cv::Point2i(q_new.x_pos,q_new.y_pos), cv::Point2i(q.x_pos,q.y_pos), cv::Scalar(0,127,127), 2);
                found_new_q = true;
                break;
            }
        }
        if (found_new_q)
            return q_new;
    }

}

vertex EST::getClosest(vertex q_new)
{
    double dist = sqrt(h*h+w*w);
    //std::cout << dist << std::endl;
    vertex r;
    for (auto &ver : vertices)
    {
        int dist_x = q_new.x_pos - ver.x_pos;
        int dist_y = q_new.y_pos - ver.y_pos;
        if (sqrt(dist_x*dist_x + dist_y*dist_y) < dist)
        {
            r = ver;
            dist = sqrt(dist_x*dist_x + dist_y*dist_y);
            std::cout << "new least distance is: " << dist << std::endl;
        }
    }
    return r;
}

bool collFreePath(vertex q1, vertex q2, cv::Mat map)
{
    cv::LineIterator line_it(map, cv::Point2i(q1.x_pos, q1.y_pos), cv::Point2i(q2.x_pos, q2.y_pos), 8);

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
            return true;
        }
    }

    return false;
}


void shortenPath(std::vector<vertex> vertices, cv::Mat map)
{
    std::vector<vertex> path;
    path.push_back(vertices[0]);

    vertex q = vertices[0];

    int index = 0;

    while(index < vertices.size()-1)
    {
        for (int i = vertices.size(); i > index; i--)
        {
            if (collFreePath(q, vertices[i], map))
            {
                cv::circle(map, cv::Point2i(q.x_pos, q.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
                cv::line(map, cv::Point2i(q.x_pos, q.y_pos), cv::Point2i(vertices[i].x_pos,vertices[i].y_pos), cv::Scalar(127,0,200), 4);
                showImage("Shortest Path", map);
                cv::waitKey(300);

                q = vertices[i];
                path.push_back(q);
                index = i;
                break;
            }
        }
    }
    std::cout << "I came here to the end" << std::endl;
    cv::waitKey(0);
}

void isolatePath(EST start_tree, EST goal_tree, vertex q1, vertex q2, cv::Mat map)
{
    std::vector<vertex> path;
    vertex q;
    cv::Mat map_show = map.clone();

    // Find the path from the start tree
    q = q1;

    while(q.origin_indx >= 0)
    {
        path.insert(path.begin(), q);
        q = start_tree.vertices[q.origin_indx];
    }

    // Find the path from the goal tree
    q = q2;

    while(q.origin_indx >= 0)
    {
        path.push_back(q);
        q = goal_tree.vertices[q.origin_indx];
    }

    // Connect the path visually on the map
    for (int i = 0; i < path.size() - 1; i++)
    {

        cv::circle(map_show, cv::Point2i(path[i].x_pos, path[i].y_pos), 4, cv::Scalar(0,0,255), 2, 8);
        cv::line(map_show, cv::Point2i(path[i].x_pos, path[i].y_pos), cv::Point2i(path[i+1].x_pos,path[i+1].y_pos), cv::Scalar(0,127,127), 2);
        showImage("Path", map_show);
        cv::waitKey(300);
    }
    cv::circle(map_show, cv::Point2i(path[-1].x_pos, path[-1].y_pos), 4, cv::Scalar(0,0,255), 2, 8);
    showImage("Path", map_show);
    cv::waitKey(0);
    shortenPath(path, map_show);
}

void ESTquery(vertex q_start, vertex q_goal, int n, cv::Mat map)
{
    cv::Mat map_show = map.clone();

    EST start_tree(q_start, map);
    EST goal_tree(q_goal, map);

    vertex q1;
    vertex q2;

    for (int i = 0; i < n; i++)
    {
        q1 = start_tree.extendEST(map_show);
        q2 = goal_tree.getClosest(q1);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(255,0,255), 2, 8);

        if (collFreePath(q1, q2, map))
        {
            cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 8, cv::Scalar(0,255,255), 2, 8);
            cv::line(map_show, cv::Point2i(q1.x_pos,q1.y_pos), cv::Point2i(q2.x_pos,q2.y_pos), cv::Scalar(200,64,127), 2);
            std::cout << "A path has been found in " << i << " steps!" << std::endl;
            showImage("Path", map_show);
            std::cout << "Path found from start to goal" << std::endl;

            cv::waitKey(0);

            isolatePath(start_tree,goal_tree,q1,q2, map);

            return;
        }

        showImage("Trees", map_show);

        cv::waitKey(10);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,255), 2, 8);

        q1 = goal_tree.extendEST(map_show);
        q2 = start_tree.getClosest(q1);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(255,0,255), 2, 8);

        if (collFreePath(q1, q2, map))
        {
            vertex q_temp = q1;
            cv::circle(map_show, cv::Point2i(q1.x_pos,q1.y_pos), 8, cv::Scalar(0,255,255), 2, 8);
            cv::line(map_show, cv::Point2i(q1.x_pos,q1.y_pos), cv::Point2i(q2.x_pos,q2.y_pos), cv::Scalar(200,64,127), 2);
            std::cout << "A path has been found in " << i << " steps!" << std::endl;
            std::cout << "Path found from goal to start" << std::endl;
            showImage("Path", map_show);

            cv::waitKey(0);

            isolatePath(start_tree,goal_tree,q2,q1, map);

            return;
        }


        showImage("Trees", map_show);

        cv::waitKey(10);
        cv::circle(map_show, cv::Point2i(q2.x_pos,q2.y_pos), 4, cv::Scalar(0,0,255), 2, 8);
    }

    return;
}
