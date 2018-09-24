#include <iostream>
#include <cstdlib>
#include "Cloud.h"

using namespace std;

int user_data = 0;
Cloud cloud;
PolygonMesh *arr_animation = cloud.meshAnimationArr();

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0);
	viewer.setCameraPosition(75, 75, 75, 0, 0, 0);

}

void showCloud(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.spinOnce();
	std::stringstream ss;
	ss << "Sample cloud of 5000 points.";
	viewer.removePolygonMesh();
	viewer.removePointCloud();
	viewer.addPointCloud(cloud.getCloud());
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);
	viewer.spinOnce();

}

void showMesh(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.spinOnce();
	std::stringstream ss;
	ss << "Sample mesh.";
	viewer.removePolygonMesh();
	viewer.removePointCloud();
	viewer.addPolygonMesh(cloud.triangleGeneration(cloud.cloudWithNorGen(cloud.getCloud())));
	viewer.removeShape("text", 0);
	viewer.addText(ss.str(), 200, 300, "text", 0);
	viewer.spinOnce();

}

void viewerPsycho(pcl::visualization::PCLVisualizer& viewer)
{
	if (user_data < 5 && user_data >= 0)
	{
		std::stringstream ss;
		ss << "Sample mesh.";
		viewer.removePolygonMesh();
		viewer.removePointCloud();
		viewer.addPolygonMesh(arr_animation[user_data]);
		viewer.removeShape("text", 0);
		viewer.addText(ss.str(), 200, 300, "text", 0);
		Sleep(500);
		viewer.spinOnce();
	}
	else
	{
		user_data = 0;
		std::stringstream ss;
		ss << "Sample mesh.";
		viewer.removePolygonMesh();
		viewer.removePointCloud();
		viewer.addPolygonMesh(arr_animation[user_data]);
		viewer.removeShape("text", 0);
		viewer.addText(ss.str(), 200, 300, "text", 0);
		Sleep(500);
		viewer.spinOnce();
	}
	user_data++;

}

int main() {
	char op = 's';
	float time = 0, fuel_rate = 0, start = 0, end = 0, pace = 0;
	visualization::CloudViewer viewer("Cloud Viewer");

	viewer.runOnVisualizationThreadOnce(showCloud);
	viewer.runOnVisualizationThreadOnce(viewerOneOff);

	while (!viewer.wasStopped())
	{
		cout << "Please select: " << endl;
		cout << "1 ---> 1'st task" << endl;
		cout << "2 ---> 2'nd task" << endl;
		cout << "3 ---> 3'rd task" << endl;
		cout << "x ---> Exit" << endl;

		cin >> op;

		switch (op) {
		case '1':
			cout << "Sample cloud of 5000 points." << endl << endl;
			viewer.removeVisualizationCallable();
			viewer.runOnVisualizationThreadOnce(showCloud);
			break;
		case '2':
			cout << "Sample mesh." << endl << endl;
			viewer.removeVisualizationCallable();
			viewer.runOnVisualizationThreadOnce(showMesh);
			break;
		case '3':
			cout << "Animation" << endl << endl;
			viewer.runOnVisualizationThread(viewerPsycho);
			break;
		case 'x':
			return 0;
		default:
			continue;
		}
	}
	return 0;
}