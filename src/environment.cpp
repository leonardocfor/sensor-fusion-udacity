/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "KDTree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


// void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer, bool render_clusters, bool render_box)
// {
//     // ----------------------------------------------------
//     // -----Open 3D viewer and display simple highway -----
//     // ----------------------------------------------------
//
//     // RENDER OPTIONS
//     bool renderScene = true;
//     std::vector<Car> cars = initHighway(renderScene, viewer);
//
//     // TODO:: Create lidar sensor
//     Lidar* lidar = new Lidar(cars,0);
//     pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
//     //renderRays(viewer,lidar->position,inputCloud);
//     renderPointCloud(viewer,inputCloud,"inputCloud");
//     // TODO:: Create point processor
//     ProcessPointClouds<pcl::PointXYZ> pointProcessor;
//     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud,500,0.25);
//     renderPointCloud(viewer,segmentCloud.first,"ObstaclesCloud",Color(1,0,0));
//     renderPointCloud(viewer,segmentCloud.second,"PlaneCloud",Color(0,1,0));
//
//     // Clustering
//     std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first,1.0,3,30);
//
//     int clusterId = 0;
//     std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
//     for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
//     {
//       if(render_clusters)
//       {
//         std::cout << "Cluster size ";
//         pointProcessor.numPoints(cluster);
//         renderPointCloud(viewer,cluster,"obstaclesCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
//       }
//       if(render_box) {
//         Box box = pointProcessor.BoundingBox(cluster);
//         renderBox(viewer,box,clusterId);
//       }
//
//         ++clusterId;
//     }
// }


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI> pointProcessor,pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
  {
  //bool renderScene = true;
  //std::vector<Car> cars = initHighway(renderScene, viewer);

  // Filter cloud Parameters to play
  float filterRes = 0.3;
  Eigen::Vector4f minPoint = Eigen::Vector4f(-10,-5,-2,1);
  Eigen::Vector4f maxPoint = Eigen::Vector4f(30,8,1,1);
  inputCloud = pointProcessor.FilterCloud(inputCloud,filterRes,minPoint,maxPoint);
  renderPointCloud(viewer,inputCloud,"inputCloud");
  ///////////////////////////////

  // Getting the plane and obstacles clouds
  int iterations = 25;
  float distanceThreshold = 0.3;
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud,iterations,distanceThreshold);
  renderPointCloud(viewer,segmentCloud.first,"ObstaclesCloud",Color(1,0,0));
  renderPointCloud(viewer,segmentCloud.second,"PlaneCloud",Color(0,1,0));
  ///////////////////////////////

  // Clustering
  float cloudTolerance = 0.53;
  int minSize = 10;
  int maxSize = 500;
  std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first,cloudTolerance,minSize,maxSize);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};
  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
    std::cout << "Cluster size ";
    pointProcessor.numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstaclesCloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);

    Box box = pointProcessor.BoundingBox(cluster);
    renderBox(viewer,box,clusterId);

    ++clusterId;
  }
  ///////////////////////////////


}


int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  //simpleHighway(viewer, true,true);

  ProcessPointClouds<pcl::PointXYZI> pointProcessor;
  std::vector<boost::filesystem::path> stream = pointProcessor.streamPcd("../src/sensors/data/pcd/data_1");
  auto streamIterator = stream.begin();
  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;

  while (!viewer->wasStopped ())
  {
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      inputCloud = pointProcessor.loadPcd((*streamIterator).string());
      cityBlock(viewer,pointProcessor,inputCloud);

      streamIterator++;
      if(streamIterator == stream.end()) {
        streamIterator = stream.begin();
      }

      viewer->spinOnce ();
  }
}
