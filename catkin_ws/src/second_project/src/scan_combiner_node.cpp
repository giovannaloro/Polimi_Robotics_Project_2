/*
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>

class ScanCombiner {
public:
  ScanCombiner(ros::NodeHandle& nh)
    : nh_(nh),
      front_sub_(nh_, "/scan_front", 10),
      back_sub_(nh_, "/scan_back", 10),
      sync_(SyncPolicy(10), front_sub_, back_sub_),
      tf_listener_(tf_buffer_)
  {
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
    sync_.registerCallback(boost::bind(&ScanCombiner::callback, this, _1, _2));
  }

private:
  void callback(const sensor_msgs::LaserScanConstPtr& front,
                const sensor_msgs::LaserScanConstPtr& back)
  {
    // Parametri della nuova scansione
    sensor_msgs::LaserScan output;
    output.header.stamp = front->header.stamp;
    output.header.frame_id = "base_link";
    output.angle_min = -M_PI;         // copertura 360°
    output.angle_max = M_PI;
    output.angle_increment = front->angle_increment;  // assumiamo stessa risoluzione
    output.time_increment = front->time_increment;
    output.scan_time = front->scan_time;
    output.range_min = std::min(front->range_min, back->range_min);
    output.range_max = std::max(front->range_max, back->range_max);

    int n_points = std::round((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(n_points, output.range_max + 1.0); // inizializza con valori grandi (infinito)

    // Funzione helper per inserire punti nello scan comabinato
    auto insert_point = [&](float angle, float range){
      // filtro distanza minima per evitare robot
      if(range < 0.3 || range > output.range_max) return;
      // angolo normalizzato da -pi a +pi
      while(angle < output.angle_min) angle += 2*M_PI;
      while(angle > output.angle_max) angle -= 2*M_PI;

      int index = (int)((angle - output.angle_min) / output.angle_increment);
      if(index < 0 || index >= n_points) return;

      if(range < output.ranges[index])
        output.ranges[index] = range;
    };

    // Processa i due LaserScan
    processScan(front, insert_point);
    processScan(back, insert_point);

    pub_scan_.publish(output);
  }

  void processScan(const sensor_msgs::LaserScanConstPtr& scan,
                   std::function<void(float,float)> insert_point)
  {
    for(size_t i=0; i < scan->ranges.size(); ++i)
    {
      float r = scan->ranges[i];
      if(std::isnan(r) || std::isinf(r)) continue;

      float angle = scan->angle_min + i * scan->angle_increment;

      // Converti al frame base_link se serve (qui assumiamo scansione già in base_link o zero offset)
      // Se serve, aggiungi trasformazione usando tf2

      insert_point(angle, r);
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_scan_;

  message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> back_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_combiner");
  ros::NodeHandle nh;

  ScanCombiner combiner(nh);

  ros::spin();
  return 0;
}
  */

  /*
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>  // <-- new
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>
#include <functional>

class ScanCombiner {
public:
  ScanCombiner(ros::NodeHandle& nh)
    : nh_(nh),
      front_sub_(nh_, "/scan_front", 10),
      back_sub_(nh_, "/scan_back", 10),
      sync_(SyncPolicy(10), front_sub_, back_sub_),
      tf_listener_(tf_buffer_)
  {
    tf_buffer_.canTransform("base_link", "sick_front", ros::Time(0), ros::Duration(1.0));
    tf_buffer_.canTransform("base_link",  "sick_back", ros::Time(0), ros::Duration(1.0));
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);
    sync_.registerCallback(boost::bind(&ScanCombiner::callback, this, _1, _2));
  }

private:
  void callback(const sensor_msgs::LaserScanConstPtr& front,
                const sensor_msgs::LaserScanConstPtr& back)
  {
    // Prepare output scan
    sensor_msgs::LaserScan output;
    output.header.stamp = front->header.stamp;
    output.header.frame_id = "base_link";
    output.angle_min = -M_PI;
    output.angle_max =  M_PI;
    output.angle_increment = front->angle_increment;
    output.time_increment  = front->time_increment;
    output.scan_time       = front->scan_time;
    output.range_min       = std::min(front->range_min, back->range_min);
    output.range_max       = std::max(front->range_max, back->range_max);

    int n_points = std::round((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(n_points, output.range_max + 1.0);

    auto insert_point = [&](float angle, float range) {
      if (range < output.range_min || range > output.range_max) return;
      // normalize angle
      while (angle < output.angle_min) angle += 2*M_PI;
      while (angle > output.angle_max) angle -= 2*M_PI;
      int idx = int((angle - output.angle_min) / output.angle_increment);
      if (idx>=0 && idx<n_points && range < output.ranges[idx]) {
        output.ranges[idx] = range;
      }
    };

    // Process each scan, transforming into base_link
    processScan(front, insert_point);
    processScan(back,  insert_point);

    pub_scan_.publish(output);
  }

  void processScan(const sensor_msgs::LaserScanConstPtr& scan,
                   std::function<void(float,float)> insert_point)
  {
    const std::string target_frame = "base_link";
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      if (std::isnan(r) || std::isinf(r)) continue;

      // original angle in sensor frame
      float angle_sensor = scan->angle_min + i * scan->angle_increment;

      // build point in sensor frame
      geometry_msgs::PointStamped pin;
      pin.header        = scan->header;
      pin.header.frame_id = scan->header.frame_id;
      pin.point.x       = r * std::cos(angle_sensor);
      pin.point.y       = r * std::sin(angle_sensor);
      pin.point.z       = 0.0;

      // transform into base_link
      geometry_msgs::PointStamped pout;
      try {
        tf_buffer_.transform(pin, pout, target_frame, ros::Duration(0.1));
      } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "TF error: %s", ex.what());
        continue;
      }

      // back to polar in base_link
      float r_base   = std::hypot(pout.point.x, pout.point.y);
      float ang_base = std::atan2(pout.point.y, pout.point.x);

      insert_point(ang_base, r_base);
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_scan_;

  message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_, back_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_combiner");
  ros::NodeHandle nh;
  ScanCombiner combiner(nh);
  ros::spin();
  return 0;
}
*/

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>
#include <vector>
#include <functional>

class ScanCombiner {
public:
  ScanCombiner(ros::NodeHandle& nh)
    : nh_(nh),
      front_sub_(nh_, "/scan_front", 10),
      back_sub_(nh_,  "/scan_back",  10),
      sync_(SyncPolicy(10), front_sub_, back_sub_),
      tf_listener_(tf_buffer_)
  {
    // Load parameters
    nh_.param("body_filter_radius", body_filter_radius_, 0.18);

    // Ensure TF availability
    tf_buffer_.canTransform("base_link", "sick_front", ros::Time(0), ros::Duration(1.0));
    tf_buffer_.canTransform("base_link",  "sick_back",  ros::Time(0), ros::Duration(1.0));

    // Publisher for combined scan
    pub_scan_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 10);

    // Register synchronized callback
    sync_.registerCallback(boost::bind(&ScanCombiner::callback, this, _1, _2));
  }

private:
  void callback(const sensor_msgs::LaserScanConstPtr& front,
                const sensor_msgs::LaserScanConstPtr& back)
  {
    // Prepare merged scan in base_link frame
    sensor_msgs::LaserScan output;
    output.header.stamp = front->header.stamp;
    output.header.frame_id = "base_link";
    output.angle_min       = -M_PI;
    output.angle_max       =  M_PI;
    output.angle_increment = front->angle_increment;
    output.time_increment  = front->time_increment;
    output.scan_time       = front->scan_time;
    output.range_min       = std::min(front->range_min, back->range_min);
    output.range_max       = std::max(front->range_max, back->range_max);

    int n_points = std::round((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(n_points, output.range_max + 1.0);

    auto insert_point = [&](float angle, float range) {
      if (range < output.range_min || range > output.range_max) return;
      // normalize angle
      while (angle < output.angle_min) angle += 2*M_PI;
      while (angle > output.angle_max) angle -= 2*M_PI;
      int idx = int((angle - output.angle_min) / output.angle_increment);
      if (idx >= 0 && idx < n_points && range < output.ranges[idx]) {
        output.ranges[idx] = range;
      }
    };

    // Combine front and back scans
    processScan(front, insert_point);
    processScan(back,  insert_point);

    // Publish filtered, combined scan
    pub_scan_.publish(output);
  }

  void processScan(const sensor_msgs::LaserScanConstPtr& scan,
                   std::function<void(float,float)> insert_point)
  {
    const std::string target_frame = "base_link";
    for (size_t i = 0; i < scan->ranges.size(); ++i) {
      float r = scan->ranges[i];
      // Skip invalid measurements
      if (std::isnan(r) || std::isinf(r)) continue;

      // Polar to Cartesian in sensor frame
      float angle_sensor = scan->angle_min + i * scan->angle_increment;
      geometry_msgs::PointStamped pin;
      pin.header        = scan->header;
      pin.header.frame_id = scan->header.frame_id;
      pin.point.x       = r * std::cos(angle_sensor);
      pin.point.y       = r * std::sin(angle_sensor);
      pin.point.z       = 0.0;

      // Transform point into base_link
      geometry_msgs::PointStamped pout;
      try {
        tf_buffer_.transform(pin, pout, target_frame, ros::Duration(0.1));
      } catch (tf2::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "TF error: %s", ex.what());
        continue;
      }

      // Cartesian to polar in base_link
      float r_base   = std::hypot(pout.point.x, pout.point.y);
      float ang_base = std::atan2(pout.point.y, pout.point.x);

      // Filter out points hitting robot body
      if (r_base < body_filter_radius_) continue;

      // Insert valid point
      insert_point(ang_base, r_base);
    }
  }

  ros::NodeHandle nh_;
  ros::Publisher pub_scan_;

  message_filters::Subscriber<sensor_msgs::LaserScan> front_sub_, back_sub_;
  typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::LaserScan, sensor_msgs::LaserScan> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  double body_filter_radius_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_combiner");
  ros::NodeHandle nh;
  ScanCombiner combiner(nh);
  ros::spin();
  return 0;
}
