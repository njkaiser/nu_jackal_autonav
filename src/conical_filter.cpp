#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// field offsets for velodyne_points::PointXYZIR data type
// see message prototype (fields section) for offset values
#define XSTART 0 // x starts at beginning of each message
#define YSTART 1 //* 4 bytes for a float =  4 byte offset
#define ZSTART 2 //* 4 bytes for a float =  8 byte offset
#define ISTART 4 //* 4 bytes for a float = 16 byte offset
#define RSTART 5 //* 4 bytes for a float = 20 byte offset
#define PT_LEN 8 //* 4 bytes for a float = 32 bytes in message total

// don't waste CPU calculating angle each time
// #define TAN1 0.01745506492
#define TAN1_5 0.02618592156
// #define TAN2 0.03492076949

#define ZOFFSET 0.338 // fixed offset from Velodyne frame to ground


inline float radial_dist(float & x, float & y)
  { return sqrt(pow(x,2) + pow(y, 2)); }


class ConicalFilter
{
private:
  // set up ROS
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;

  int width; // width of input cloud (# points)
  int s; // size of each point
  float * current_address; // to keep track of current position in data stream
  int new_width; // width of output cloud (# points)

  float x; // store x value of each data point
  float y; // store y value of each data point
  float z; // store z value of each data point

public:
  ConicalFilter()
  {
    std::cout << "instance of ConicalFilter class instantiated" << std::endl;

    // create publisher for output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points/for_costmap", 1);

    // subscribe to input point cloud and pass data to filter
    sub = nh.subscribe("/velodyne_points/voxeled", 1, &ConicalFilter::conical_filter, this);
  }


private:
  void conical_filter (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // create new cloud to fill with points I care about
    sensor_msgs::PointCloud2Ptr cloud_filtered = boost::make_shared<sensor_msgs::PointCloud2>();

    // grab necessary info for processing
    width = input->width;
    s = input->point_step; // store as variable since we use this later
    // std::cout << "number of points in  input cloud: " << width << std::endl;

    // fill new pointcloud with matching data from old
    cloud_filtered->header = input->header;
    cloud_filtered->height = input->height;
    cloud_filtered->fields = input->fields;
    cloud_filtered->is_bigendian = input->is_bigendian;
    cloud_filtered->point_step = s;
    cloud_filtered->is_dense = input->is_dense;


    // BEGIN HACKIEST SHIT EVER
    float * start_address = (float *)(&(input->data[0]));
    // std::cout << "start address of data: " << start_address << std::endl;

    new_width = 0; // keep track of this to export with output data
    for(int i = 0; i < width; ++i) // iterate through each point in input cloud
    {
      current_address = start_address + i*PT_LEN;
      x = *(current_address + XSTART);
      y = *(current_address + YSTART);
      z = *(current_address + ZSTART) + ZOFFSET;

      if(z > radial_dist(x, y) * TAN1_5)
      {
        for(int j = 0; j < PT_LEN; ++j)
        {
          // NESTED HACKS FTW!!!
          // this just converts the 4 byte floats into 4 x 1 byte
          // uint8_t's to keep the PointCloud2 data structure happy
          uint8_t * hack = reinterpret_cast<uint8_t*>((current_address + j));
          cloud_filtered->data.push_back(hack[0]);
          cloud_filtered->data.push_back(hack[1]);
          cloud_filtered->data.push_back(hack[2]);
          cloud_filtered->data.push_back(hack[3]);
        }
        ++new_width; // increment # of new data points
      }
    }
    // std::cout << "number of points in output cloud: " << new_width << std::endl;
    cloud_filtered->width = new_width;
    cloud_filtered->row_step = new_width * s; // # points * length of point

    // and, finally, output pointcloud data for costmap
    pub.publish(*cloud_filtered);
  }
}; // END OF ConicalFilter CLASS



int main(int argc, char** argv)
{
  ros::init(argc, argv, "conical_filter");
  ConicalFilter CF;
  ros::spin();
}
