#include<ros/ros.h>
#include<math.h>
#include<sensor_msgs/PointCloud.h>

using namespace std;

class Prediction{
public:
  Prediction(){
    association_obstacle_sub = nh.subscribe("assocition_obstacle", 10, &Prediction::association_obstacleCallBack, this);
    path_prediction_pub = nh.advertise<sensor_msgs::PointCloud>("path_prediction", 1000);
  }
  void association_obstacleCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    int count;
    callback = true;
    cluster_num_max = 0;
    prediction_cost.points.clear();
    prediction_cost.channels.clear();

    timestamp = ros::Time::now().toSec();
    prediction_cost.header.stamp = ros::Time::now();

    for(int i = 0; i < (int)msg->points.size(); i++){
      if(cluster_num_max < msg->channels[1].values[i] + 1 ){
        cluster_num_max = msg->channels[1].values[i] + 1;
      }
    }

    cluster_x.clear();
    cluster_y.clear();
    cluster_x.resize(cluster_num_max);
    cluster_y.resize(cluster_num_max);
    for(int i = 0; i < cluster_num_max; i++){
      count = 0;
      for(int j = 0; j < (int)msg->points.size(); j++){
        if(i == msg->channels[1].values[j]){
          cluster_x[i] += msg->points[j].x;
          cluster_y[i] += msg->points[j].y;
          count += 1;

        }
      }
      cluster_x[i] = cluster_x[i] / count;
      cluster_y[i] = cluster_y[i] / count;
    }
  }

  void calc(){
    int count, now;
    float diff_x, diff_y, distance;

    for(int i = 0; i < cluster_num_max; i++){
      if(cluster_x[i] != 0 && cluster_y[i] != 0 && cluster_old_x[i] != 0 && cluster_old_y[i] != 0){
        diff_x = cluster_x[i] - cluster_old_x[i];
        diff_y = cluster_y[i] - cluster_old_y[i];
        count += 1;
      }
      //進んだ距離
      distance = hypotf(diff_x, diff_y);
      
      //速度
      velocity[i] = distance / (timestamp - timestamp_old);

      //角度
      theta[i] = atan2(diff_y, diff_x);
    }
  }

  void run(){
    int old = false;
    callback = false;
    while(ros::ok()){
      if(callback == true){
        if(old == true){
          old = false;
          calc();
        }
        for(int i = 0; i < cluster_num_max; i++){
          if(cluster_x[i] != 0 && cluster_y[i] != 0){
            cluster_old_x[i] = cluster_x[i];
            cluster_old_y[i] = cluster_y[i];
            timestamp_old = timestamp;
            old = true;
          }
        }
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber association_obstacle_sub;
  ros::Publisher path_prediction_pub;
  sensor_msgs::PointCloud prediction_cost;
  vector<float> cluster_x, cluster_y, cluster_old_x, cluster_old_y, velocity, theta;
  int callback, cluster_num_max;
  float timestamp, timestamp_old;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "prediction");
  Prediction path_prediction;
  path_prediction.run();
  return 0;
}
