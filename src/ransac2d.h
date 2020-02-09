/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting
#ifndef RANSAC_H
#define RANSAC_H

#include "kdtree.h"



// using templates for processPointClouds so also include .cpp to help linker


std::vector<float> getLineEq(pcl::PointXYZ first, pcl::PointXYZ second)
{

	std::vector<float> result;
	result.push_back(first.y - second.y);
	result.push_back(second.x - first.x);
	result.push_back(first.x * second.y - second.x * first.y);
	return result;
}
float distanceBtwLinePoint(std::vector<float> &line, pcl::PointXYZ point_l)
{
	float dist = 0;
	dist = std::fabs(line[0] * point_l.x + line[1] * point_l.y + line[2]) / (std::sqrt((line[0] * line[0]) + (line[1] * line[1])));
	return dist;
}

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol, int &one, int &two)
{
	std::unordered_set<int> *inliersResult_res = new std::unordered_set<int>();
	std::unordered_set<int> *inliersResult_temp = new std::unordered_set<int>();
	// Time segmentation process
	auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

	// TODO: Fill in this function

	// For max iterations

	while (maxIterations--)
	{
		auto startTime = std::chrono::steady_clock::now();
		int ind = 0;
		int ind_i = 0;
		while (ind == ind_i)
		{
			ind = std::rand() % cloud->points.size();
			ind_i = std::rand() % cloud->points.size();
		}

		std::vector<float> line = getLineEq(cloud->points[ind], cloud->points[ind_i]);
		inliersResult_temp->insert(ind);
		inliersResult_temp->insert(ind_i);
		for (int ind_j = 0; ind_j < cloud->points.size() && ind_j != ind_i && ind != ind_j; ind_j++)
		{
			float dist = distanceBtwLinePoint(line, cloud->points[ind_j]);
			if (dist <= distanceTol)
			{
				// std::cout<<"distance is "<<dist<<std::endl;
				inliersResult_temp->insert(ind_j);
			}
			else
			{
				; //std::cout<<"Invalid distance is "<<dist<<std::endl;
			}
		}
		if (inliersResult_temp->size() > inliersResult_res->size())
		{
			// std::cout<<"Res size is "<<inliersResult_res->size()<<std::endl;
			// std::cout<<"Temp size is "<<inliersResult_temp->size()<<std::endl;
			one = ind;
			two = ind_i;
			*inliersResult_res = *inliersResult_temp;
		}
		inliersResult_temp->clear();
		// std::cout<<"One Iteration Done================================ "<<std::endl;
	}

	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
	// Return indicies of inliers from fitted line with most inliers
	// std::cout<<"Final size is "<<inliersResult_res->size()<<std::endl;
	return *inliersResult_res;
}

template<typename PointT>
float getPlaneDistance(std::vector<float> plane, PointT p)
{
	float dist = std::fabs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) / std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	return dist;
}

template<typename PointT>
std::vector<float> getPlane(PointT first, PointT second, PointT third)
{
	std::vector<float> res;
	int x1 = first.x, y1 = first.y, z1 = first.z;
	int x2 = second.x, y2 = second.y, z2 = second.z;
	int x3 = third.x, y3 = third.y, z3 = third.z;
	float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
	float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
	float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
	float D = -(A * x1 + B * y1 + C * z1);

	res.push_back(A);
	res.push_back(B);
	res.push_back(C);
	res.push_back(D);

	return res;
}


template<typename PointT>
std::vector<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::vector<int> inliersResult_res;

	auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

	while (maxIterations--)
	{
		int ind_i = 0;
		int ind_j = 0;
		int ind_k = 0;
		
		std::vector<int> inliersResult_temp ;

		ind_i = std::rand() % cloud->points.size();
		do
		{
			ind_j = std::rand() % cloud->points.size();
		} while (ind_i == ind_j);

		do
		{
			ind_k = std::rand() % cloud->points.size();
		} while (ind_k == ind_j || ind_k == ind_i);

		std::vector<float> plane = getPlane(cloud->points[ind_i], cloud->points[ind_j], cloud->points[ind_k]);
		for(int ind = 0; ind < cloud->points.size(); ind++)
		{
			/*get distance*/
			float dist = getPlaneDistance(plane, cloud->points[ind]);
			if(dist <= distanceTol)
			{
				inliersResult_temp.push_back(ind);

			}
		}
		if(inliersResult_temp.size() > inliersResult_res.size())
		{
			inliersResult_res = inliersResult_temp;
		}
	}
	std::cout<<"The final size is: "<<inliersResult_res.size()<<std::endl;
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	return inliersResult_res;
}

#endif