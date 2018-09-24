#include <iostream>
#include <cstdlib>
#include "Cloud.h"

using namespace std;

Cloud cloud;

void showCloud(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(0, 0, 0);
	viewer.setCameraPosition(75, 75, 75, 0, 0, 0);
	viewer.addPointCloud(cloud.getCloud());
	viewer.spinOnce();

}

int main()
{
	visualization::CloudViewer viewer("Cloud Viewer");
	viewer.runOnVisualizationThreadOnce(showCloud);
	while (!viewer.wasStopped())
	{
	}
	return 0;
}