/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "render/render.h"
#include <unordered_set>
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	// Add inliers
	float scatter = 0.6;
	for (int i = -5; i < 5; i++)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = i + scatter * rx;
		point.y = i + scatter * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	// Add outliers
	int numOutliers = 10;
	while (numOutliers--)
	{
		double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
		pcl::PointXYZ point;
		point.x = 5 * rx;
		point.y = 5 * ry;
		point.z = 0;

		cloud->points.push_back(point);
	}
	cloud->width = cloud->points.size();
	cloud->height = 1;

	return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("2D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();
	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
	viewer->addCoordinateSystem(1.0);
	return viewer;
}
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

float getPlaneDistance(std::vector<float> plane, pcl::PointXYZ p)
{
	float dist = std::fabs(plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) / std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
	return dist;
}

std::vector<float> getPlane(pcl::PointXYZ first, pcl::PointXYZ second, pcl::PointXYZ third)
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
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult_res ; //= new std::unordered_set<int>();
	// std::unordered_set<int> inliersResult_temp ; //= new std::unordered_set<int>();
	auto startTime = std::chrono::steady_clock::now();
	srand(time(NULL));

	while (maxIterations--)
	{
		int ind_i = 0;
		int ind_j = 0;
		int ind_k = 0;

		std::unordered_set<int> inliersResult_temp ;

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
				inliersResult_temp.insert(ind);

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

// int main()
// {

// 	// Create viewer
// 	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

// 	// Create data
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
// 	int one, two;
// 	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
// 	// std::unordered_set<int> inliers = RansacLine(cloud, 100, 2.0, one, two);
// 	std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);

// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

// 	for (int index = 0; index < cloud->points.size(); index++)
// 	{
// 		pcl::PointXYZ point = cloud->points[index];
// 		if (inliers.count(index))
// 			cloudInliers->points.push_back(point);
// 		else
// 			cloudOutliers->points.push_back(point);
// 	}
// 	//Print the line points in blue
// 	// pcl::PointCloud<pcl::PointXYZ>::Ptr linePoints(new pcl::PointCloud<pcl::PointXYZ>());
// 	// linePoints->points.push_back(cloud->points[one]);
// 	// linePoints->points.push_back(cloud->points[two]);
// 	// Render 2D point cloud with inliers and outliers
// 	if (inliers.size())
// 	{
// 		renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
// 		renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
// 		// renderPointCloud(viewer, linePoints, "linepoints", Color(0, 0, 1));
// 	}
// 	else
// 	{
// 		renderPointCloud(viewer, cloud, "data");
// 	}

// 	while (!viewer->wasStopped())
// 	{
// 		viewer->spinOnce();
// 	}
// }
