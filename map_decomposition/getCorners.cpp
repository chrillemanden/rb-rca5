void getCorners(cv::Mat &mat, std::vector<cv::Point2i> &corners)
{
	for (int col = 0; col < mat.cols; col++)
	{
		for (int row = 0; row < mat.rows; row++)
		{
			int curr_pixel = (int)mat.at<uchar>(row, col);

			//std::cout << (int)mat.at<uchar>(row, col) << " ";

			/* If white then not a corner */
			if (curr_pixel)
				continue;

			int rows = mat.rows - 1;
			int cols = mat.cols - 1;

			/* Special border corner cases */
			if ((col == 0 && row == 0) || (col == 0 && row == rows) || (col == cols && row == 0) || (col == cols && row == rows))
			{
				// These corners don't matter for the algorithm
				continue;
			}

			/* Special border cases */
			// Left side
			if (col == 0)
			{
				if (!(int)mat.at<uchar>(row, col + 1))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Top side 
			else if (row == 0)
			{
				if (!(int)mat.at<uchar>(row + 1, col))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Right side
			else if (col == cols)
			{
				if (!(int)mat.at<uchar>(row, col - 1))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}
			// Bottom side
			else if (row == rows)
			{
				if (!(int)mat.at<uchar>(row - 1, col))
				{
					corners.push_back(cv::Point2i(row, col));
					continue;
				}
				continue;
			}


			/* Rules for all other pixels */

			// Adjacent pixels
			int l_pixel = (int)mat.at<uchar>(row, col - 1);
			int r_pixel = (int)mat.at<uchar>(row, col + 1);
			int u_pixel = (int)mat.at<uchar>(row - 1, col);
			int d_pixel = (int)mat.at<uchar>(row + 1, col);
			// Diagonal pixels
			int ul_pixel = (int)mat.at<uchar>(row - 1, col - 1);
			int ur_pixel = (int)mat.at<uchar>(row - 1, col + 1);
			int dr_pixel = (int)mat.at<uchar>(row + 1, col + 1);
			int dl_pixel = (int)mat.at<uchar>(row + 1, col - 1);

			// Special rules for walls that are thicker than 1 pixel
			if ((!l_pixel && !r_pixel) && ((!ul_pixel && !ur_pixel) || (!dl_pixel && !dr_pixel)))
			{
				continue;
			}
			else if ((!u_pixel && !d_pixel) && ((!ul_pixel && !dl_pixel) || (!ur_pixel && !dr_pixel)))
			{
				continue;
			}
			
			// Finds the rest of corners
			if ((l_pixel ^ r_pixel) || (u_pixel ^ d_pixel))
			{
				corners.push_back(cv::Point2i(row, col));
				continue;
			}


		}

	}
}