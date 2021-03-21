// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>
#include "KDTree.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr obstaclesCloud (new pcl::PointCloud<PointT> ());
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

  for(int index: inliers->indices)
    planeCloud->points.push_back(cloud->points[index]);

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstaclesCloud);
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, planeCloud);
  return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Replace for my function
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;
  	srand(time(NULL));

  	// TODO: Fill in this function
  	while(maxIterations--)
  	{
  		std::unordered_set<int> inliers;
  		while(inliers.size()<3)
  			inliers.insert(rand()%(cloud->points.size()));

  		float x1,y1,z1,x2,y2,z2,x3,y3,z3;

  		auto itr = inliers.begin();
  		x1 = cloud->points[*itr].x;
  		y1 = cloud->points[*itr].y;
  		z1 = cloud->points[*itr].z;
  		itr++;
  		x2 = cloud->points[*itr].x;
  		y2 = cloud->points[*itr].y;
  		z2 = cloud->points[*itr].z;
  		itr++;
  		x3 = cloud->points[*itr].x;
  		y3 = cloud->points[*itr].y;
  		z3 = cloud->points[*itr].z;

  		std::vector<float> v1(3);
  		std::vector<float> v2(3);
  		// initializing vector 1 (v1) -- point1 -> point2
  		v1.push_back(x2-x1);
  		v1.push_back(y2-y1);
  		v1.push_back(z2-z1);
  		// initializing vector 2 (v2) -- point1 -> point3
  		v1.push_back(x3-x1);
  		v1.push_back(y3-y1);
  		v1.push_back(z3-z1);
  		// Computing cross product v1xv2
  		float a = ((y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1)); // i
  		float b = - ((x2 - x1)*(z3 - z1) - (z2 - z1)*(x3 - x1)); // j
  		float c = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1)); // k
  		float d = - ( a*x1 + b*y1 + c*z1);

  		for(int index=0; index < cloud->points.size(); index++)
  		{
  			if(inliers.count(index)>0)
  				continue;

  			pcl::PointXYZ point = cloud->points[index];
  			float x4 = point.x;
  			float y4 = point.y;
  			float z4 = point.z;

  			float d = fabs(a*x4 + b*y4 + c*z4 + d)/sqrt(a*a + b*b + c*c); // fabs - float abs

  			if (d < distanceThreshold)
  				inliers.insert(index);

  		}

  		if(inliers.size()>inliersResult.size()){
  			inliersResult = inliers;
  		}

  	}

    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
  	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

  	for(int index = 0; index < cloud->points.size(); index++)
  	{
  		pcl::PointXYZ point = cloud->points[index];
  		if(inliersResult.count(index))
  			cloudInliers->points.push_back(point);
  		else
  			cloudOutliers->points.push_back(point);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = cloudOutliers; // "ObstaclesCloud"
    segResult.second = cloudInliers; // "PlaneCloud"
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(int index, const std::vector<std::vector<float>> points, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol) {
	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> nearest = tree->search(points[index],distanceTol);

	for(int id: nearest) {
		if(!processed[id]){
			proximity(id,points, cluster,processed, tree,distanceTol);
		}
	}
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(points.size(),false);

	int i = 0;
	while (i < points.size()) {
		if (processed[i]) {
			i++;
			continue;
		}

		std::vector<int> cluster;
		proximity(i,points,cluster,processed,tree,distanceTol);
		clusters.push_back(cluster);
		i++;
	}

	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Building tree
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i < cloud->points.size(); i++) {
      std::vector<float> point;
      point.push_back(cloud->points[i].x);
      point.push_back(cloud->points[i].y);
      point.push_back(cloud->points[i].z);
      points.push_back(point);
      tree->insert(point,i);
    }

    // Computing euclidean clusters
    std::vector<std::vector<int>> euClusters = euclideanCluster(points,tree,clusterTolerance);

    // Building clusters
    for(const auto& indexes: euClusters) {

      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

      for (const auto index: indexes){
        cloudCluster->points.push_back(cloud->points[index]);
      }
      cloudCluster->width = cloudCluster->points.size();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      if (cloudCluster->width >= minSize && cloudCluster->width <= maxSize){
        clusters.push_back(cloudCluster);
      }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
