/*** INCLUDE FILES ***/
//#include <pcl_server/pcl_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iterator>
#include <math.h>
#include <string>
#include <boost/thread/mutex.hpp>
#include <pcl/surface/concave_hull.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Point.h"
#include "pcl_server/ComputeConvexHull.h"



bool computeConvexHull ( pcl_server::ComputeConvexHullRequest &req,
                         pcl_server::ComputeConvexHullResponse &resp ) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::ConvexHull<pcl::PointXYZ> chull;
    std::vector< pcl::Vertices > polygons;
    double vol ( 0.0 );
    geometry_msgs::Point p;

    for ( int var = 0; var < req.vertices.size(); ++var ) {
        pcl::PointXYZ p ( req.vertices[var].x,
                          req.vertices[var].y,
                          req.vertices[var].z );
        cloud_projected->points.push_back ( p );
    }


    try {
        chull.setComputeAreaVolume ( req.compute_volume );
        chull.setInputCloud ( cloud_projected );
        chull.reconstruct ( *cloud_hull,polygons );
    } catch ( ... ) {
        ROS_ERROR ( "qhull error" );
        resp.success=false;
        return false;
    }

    resp.volume=chull.getTotalVolume();


    if ( ! ( cloud_hull->points.empty() ) ) {

        std::vector<geometry_msgs::Point> points;
        points.clear();

        resp.mesh.vertices.resize ( cloud_hull->points.size() );
        for ( int var = 0; var < cloud_hull->points.size(); ++var ) {
            resp.mesh.vertices[var].x=cloud_hull->points[var].x;
            resp.mesh.vertices[var].y=cloud_hull->points[var].y;
            resp.mesh.vertices[var].z=cloud_hull->points[var].z;
        }

        // Polygons is a vector of triangles represented by 3 indices
        // The indices correspond to points in cloud_hull
        resp.mesh.triangles.resize ( polygons.size() );
        for ( int tri = 0; tri<polygons.size(); ++tri ) {
            pcl::Vertices triangle=polygons[tri];
            for ( int var = 0; var<3; ++var ) {
                resp.mesh.triangles[tri].vertex_indices[var]=triangle.vertices[var];
            }
        }
    } else {
        ROS_WARN ( "Empty Hull" );
    }
    resp.success=true;
    return true;
}



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "pcl_server" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ros::ServiceServer service = nh.advertiseService ( "compute_convex_hull", computeConvexHull );
    ROS_INFO ( "PCL SERVER RUNNING." );
    ros::spin();
    return 0;
}
