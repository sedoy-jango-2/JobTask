#include <iostream>
#include <cstdlib>
#include <stdlib.h>
#include <ctime>
#include <conio.h>
#include <math.h>
#include <string>
#include <algorithm>
#include <random>

#include <pcl/io/io.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/centroid.h>

#pragma once

using namespace pcl;
using namespace std;

#ifndef CLOUD_H
#define CLOUD_H

class Cloud
{
public:
	void setCloud();
	PointCloud<PointXYZ>::Ptr getCloud();
	Cloud(const Cloud &copywrite);
	Cloud();
	~Cloud();

	PointCloud<pcl::PointNormal>::Ptr cloudWithNorGen(pcl::PointCloud<PointXYZ>::Ptr cloud);
	PolygonMesh triangleGeneration(PointCloud<pcl::PointNormal>::Ptr cloudWithNorGen);

	PolygonMesh * meshAnimationArr();
private:
	PointCloud<PointXYZ>::Ptr cloud;
	float discriminant(float quadratic[3]);
	float* vectorComp(PointXYZ point1, PointXYZ point2);
	float* quadraticComp(float vector[3]);
	float* intersectionComp(float root, float vector[3], PointXYZ centroid);
	float* stepComp(float point[3], PointXYZ point_from_cloud);
	PointXYZ getCentroid();
};

#endif // CLOUD_H