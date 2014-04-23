#include <ros/ros.h>

// Flann
#include <float.h>
#include <flann/flann.hpp>
#include <flann/io/hdf5.h>

using namespace flann;

std::string getFolderName()
{
  std::string folder_key;
  folder_key = "f4025675e127122e084d959288e4555d/";
  std::ostringstream folder_name;
  folder_name << getenv("HOME") << "/.openrave/robot." << folder_key;
  return folder_name.str();
}

std::string getFilename(const std::string& database)
{
  std::string file_key;
  file_key = ".27d697e7d8a999dfc3b0a3305edb1ee6.pp";
  std::ostringstream filename;
  filename << getFolderName() << database << file_key;
  return filename.str();
}

int main(int argc, char** argv)
{
  // Start the node
  ros::init (argc, argv, "build_metrics_index");
  ros::NodeHandle nh_private ("~");
  // Get parameters from the server
  std::string database_name;
  nh_private.param(std::string("metrics_database"), database_name, std::string("ik_metrics"));
  if (!nh_private.hasParam("metrics_database"))
    ROS_WARN_STREAM("Parameter [~metrics_database] not found, using default: " << database_name);

  std::string filename;
  filename = getFilename(database_name);
  ROS_INFO_STREAM("Loading [metrics database] from:\n" << filename);
  ros::Time flann_start_time = ros::Time::now();
  Matrix<float> positions_mat;
  try {
    load_from_file(positions_mat, filename, "positions");
  }
  catch (...) {
    ROS_ERROR_STREAM("Failed loading [metrics database] from:\n" << filename);
    ros::shutdown();
    return -1;
  }
  // construct an randomized kd-tree index using 4 kd-trees
  Index<L2<float> > index(positions_mat, flann::KDTreeIndexParams(4));
  index.buildIndex();
  double elapsed_time = (ros::Time::now() - flann_start_time).toSec();
  ROS_INFO("[metrics database] successfully loaded in %.2f seconds", elapsed_time);
  
  // example of a query
  int nn = 100; 
  flann::Matrix<float> query(new float[3], 1, 3);
  query[0][0] = 0.075;
  query[0][1] = 0.4;
  query[0][2] = 0.6;
  Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
  Matrix<float> dists(new float[query.rows*nn], query.rows, nn);
  // do a knn search, using 128 checks
  index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));
  ROS_INFO("Example query: [%.3f, %.1f, %.1f] found [%d] nearest_neighbors", query[0][0], query[0][1], query[0][2], (int)indices.cols);
  
  // save the index
  std::ostringstream index_file;
  index_file << getFolderName() << "ik_metrics_index.dat";
  index.save(index_file.str());
  ROS_INFO_STREAM("Saved [metrics index] to:\n" << index_file.str());

  delete[] positions_mat.ptr();
  delete[] query.ptr();
  delete[] indices.ptr();
  delete[] dists.ptr();
  
  return 0;
}
