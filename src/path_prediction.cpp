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

    objects.points.clear();
    objects.channels.clear();
    objects.points.resize((int)msg->points.size());
    objects.channels.resize((int)msg->points.size());
    objects.channels[0].values.resize((int)msg->channels[0].values.size());
    objects.header.frame_id = msg->header.frame_id;
    objects.header.stamp = ros::Time::now();
    objects.channels[0].name = msg->channels[0].name;
    for(int i = 0; i < (int)msg->points.size(); i++){
      objects.points[i].x = msg->points[i].x;
      objects.points[i].y = msg->points[i].y;
      objects.channels[0].values[i] = msg->channels[0].values[i];
    }
  }

  void calc(){
    int check;
    float diff_x, diff_y, distance;

    for(int i = 0; i < (int)objects.points.size(); i++){
      check = false;
      for(int j = 0; j < (int)objects.points.size(); i++){
        if(objects.channels[0].values[i] == objects_old_number[j]){
          diff_x = objects.points[i].x - objects_old_x[i];
          diff_y = objects.points[i].y - objects_old_y[i];
          //進んだ距離
          distance = hypotf(diff_x, diff_y);
          //速度
          velocity[i] = distance / (objects.header.stamp.toSec() - stamp_old);
          //角度
          theta[i] = atan2(diff_y, diff_x);

          check = true;
        }
      }
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

        objects_old_x.clear();
        objects_old_y.clear();
        objects_old_number.clear();
        objects_old_x.resize((int)objects.points.size());
        objects_old_y.resize((int)objects.points.size());
        objects_old_number.resize((int)objects.points.size());
        for(int i = 0; i < (int)objects.points.size(); i++){
          objects_old_x[i] = objects.points[i].x;
          objects_old_y[i] = objects.points[i].y;
          objects_old_number[i] = objects.channels[0].values[i];
        }
        objects_old_num = (int)objects.points.size();
        stamp_old = objects.header.stamp.toSec();
        old = true;
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber association_obstacle_sub;
  ros::Publisher path_prediction_pub;
  sensor_msgs::PointCloud objects;
  sensor_msgs::PointCloud prediction_cost;
  vector<float> objects_old_x, objects_old_y, objects_old_number, velocity, theta;
  int callback, objects_old_num;
  float stamp_old;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "prediction");
  Prediction path_prediction;
  path_prediction.run();
  return 0;
}
