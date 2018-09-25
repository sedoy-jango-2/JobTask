#include "Cloud.h"

void Cloud::setCloud()
{
	float HI = 25.0;
	float LO = -25.0;
	bool flag = false;
	PointCloud<PointXYZ>::Ptr cloud_test(new PointCloud<PointXYZ>);
	PointXYZ point;

	random_device rd;
	mt19937 gen(rd());

	int expValue = -5 + gen() % 5;
	int dispercion = 4 + gen() % 4;

	std::default_random_engine generator;
	std::normal_distribution<float> distribution(expValue, dispercion);

	for (int i = 0; i < 5000; i++)
	{
		point.x = distribution(generator);
		point.y = distribution(generator);
		point.z = distribution(generator);
		cloud_test->push_back(point);
	}

	cloud_test->width = (int)cloud_test->points.size();
	cloud_test->height = 1;
	cloud = cloud_test;
}

PointCloud<PointXYZ>::Ptr Cloud::getCloud()
{
	return this->cloud;
}

Cloud::Cloud(const Cloud & copywrite)
{
	cloud = copywrite.cloud;
}

Cloud::Cloud()
{
	Cloud::setCloud();
}

Cloud::~Cloud()
{
}

PointCloud<pcl::PointNormal>::Ptr Cloud::cloudWithNorGen(pcl::PointCloud<PointXYZ>::Ptr cloud1)
{
	pcl::PointCloud<PointXYZ>::Ptr cloud = cloud1;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	tree->setInputCloud(cloud);
	normal.setInputCloud(cloud);
	normal.setSearchMethod(tree);
	normal.setKSearch(20);
	normal.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

	return cloud_with_normals;
}

PolygonMesh Cloud::triangleGeneration(PointCloud<pcl::PointNormal>::Ptr cloudWithNorGen)
{
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
	
	tree->setInputCloud(cloudWithNorGen);

	gp3.setSearchRadius(15);

	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(10);
	gp3.setMaximumSurfaceAngle(M_PI / 4);
	gp3.setMinimumAngle(M_PI / 18);
	gp3.setMaximumAngle(2 * M_PI / 3);
	gp3.setNormalConsistency(false);
	
	gp3.setInputCloud(cloudWithNorGen);
	gp3.setSearchMethod(tree);
	gp3.reconstruct(triangles);

	return triangles;
}

PolygonMesh * Cloud::meshAnimationArr()
{
	float *vector, quadratic_equation, *point_intersection, *stepArr, *quad, *disc;
	float array_of_steps[5000][3];
	float root;
	PolygonMesh *mesh_arr = new PolygonMesh[5];
	PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
	pcl::PointXYZ point;
	pcl::PolygonMesh mesh;
	cout << "Making steps array..." << endl;
	for (int i = 0; i < this->cloud->size(); i++)
	{
		vector = this->vectorComp(this->cloud->points[i], this->getCentroid());
		quad = this->quadraticComp(vector);
		root = this->discriminant(quad);
		point_intersection = this->intersectionComp(root, vector, this->getCentroid());
		stepArr = this->stepComp(point_intersection, this->cloud->points[i]);
		array_of_steps[i][0] = stepArr[0];
		array_of_steps[i][1] = stepArr[1];
		array_of_steps[i][2] = stepArr[2];
		if (i % 100 == 0)
			cout << i << " vectors are rendered." << endl;
	}
	cout << "Done!" << endl;
	cout << "Making meshes array..." << endl;
	for (int i = 0; i < 5; i++)
	{
		for (int j = 0; j < this->cloud->size(); j++)
		{
			point.x = this->cloud->points[j].x + array_of_steps[j][0] * i;
			point.y = this->cloud->points[j].y + array_of_steps[j][1] * i;
			point.z = this->cloud->points[j].z + array_of_steps[j][2] * i;
			cloud->push_back(point);
		}
		mesh_arr[i] = triangleGeneration(cloudWithNorGen(cloud));
		cout << i+1 << " meshes are rendered." << endl;
		cloud->clear();
	}
	cout << "Done!" << endl;
	return mesh_arr;
}

PointXYZ Cloud::getCentroid()
{
	PointXYZ point;
	CentroidPoint<pcl::PointXYZ> centroid;
	for (int i = 0; i < this->cloud->size(); i++)
	{
		centroid.add(this->cloud->points[i]);
	}
	centroid.get(point);
	return point;
}

float Cloud::discriminant(float quadratic[3])
{
	float X = sqrt(pow(quadratic[2], 2)/ pow(quadratic[0], 2));
	return X;
}

float * Cloud::vectorComp(PointXYZ point1, PointXYZ point2)
{
	float *arr = new float [3];
	arr[0] = point1.x - point2.x;
	arr[1] = point1.y - point2.y;
	arr[2] = point1.z - point2.z;
	return arr;
}

float * Cloud::quadraticComp(float vector[3])
{
	float *arr = new float[3];
	arr[0] = pow(vector[0], 2) + pow(vector[1], 2) + pow(vector[2], 2);
	arr[1] = 0;
	arr[2] = -100;
	return arr;
}

float * Cloud::intersectionComp(float root, float vector[3], PointXYZ centroid)
{
	float *arr = new float[3];
	arr[0] = centroid.x + vector[0] * root;
	arr[1] = centroid.y + vector[1] * root;
	arr[2] = centroid.z + vector[2] * root;
	return arr;
}

float * Cloud::stepComp(float point[3], PointXYZ point_from_cloud)
{
	float *arr = new float[3];
	arr[0] = (point[0] - point_from_cloud.x) / 5;
	arr[1] = (point[1] - point_from_cloud.y) / 5;
	arr[2] = (point[2] - point_from_cloud.z) / 5;
	return arr;
}
