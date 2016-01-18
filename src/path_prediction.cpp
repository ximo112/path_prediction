#include<ros/ros.h>
#include<math.h>
#include<sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#define elapsed_time 2
#define interval 0.05
#define acceptable_acceleration 5

using namespace std;

class Prediction{
public:
  Prediction(){
    associatied_obstacle_sub = nh.subscribe("associatied_obstacle", 10, &Prediction::associatied_obstacleCallBack, this);
    path_prediction_pub = nh.advertise<sensor_msgs::PointCloud>("path_prediction", 1000);
  }
  void associatied_obstacleCallBack(const sensor_msgs::PointCloud::ConstPtr& msg){
    callback = true;

    objects.points.clear();
    objects.channels.clear();
    objects.points = msg->points;
    objects.channels = msg->channels;
    objects.header.frame_id = msg->header.frame_id;
    objects.header.stamp = ros::Time::now();
  }

  void calc(){
    int points_num[(int)objects.points.size()];
    float diff_x, diff_y, distance;

    velocity.clear();
    theta.clear();
    velocity.resize((int)objects.points.size());
    theta.resize((int)objects.points.size());
    forecast_x_all.clear();
    forecast_y_all.clear();
    points_num_all = 0;
    for(int i = 0; i < (int)objects.points.size(); i++){
      for(int j = 0; j < objects_old_num_max; j++){
        if(objects.channels[0].values[i] == objects_old_number[j]){
          diff_x = objects.points[i].x - objects_old_x[j];
          diff_y = objects.points[i].y - objects_old_y[j];
          //距離
          distance = hypotf(diff_x, diff_y);
          //速度
          velocity[i] = distance / (objects.header.stamp.toSec() - stamp_old);
          ROS_INFO("velo:%f", velocity[i]);///
          //角度
          theta[i] = atan2(diff_y, diff_x);

          ROS_INFO("diff:%f, time:%f, num:%f, now_x:%f",distance, objects.header.stamp.toSec() - stamp_old, objects.channels[0].values[i], objects.points[i].x);
        }
      }
      length = velocity[i] * elapsed_time;
      points_num[i] = abs(length / interval);
      ROS_WARN("points_num;%d, num:%f", points_num[i], objects.channels[0].values[i]);
      float forecast_x[points_num[i]];
      float forecast_y[points_num[i]];
      for(int j = 0; j < points_num[i]; j++){
        forecast_x[j] = cos(theta[i]) * (interval * j) + objects.points[i].x;
        forecast_y[j] = sin(theta[i]) * (interval * j) + objects.points[i].y;
      }
      points_num_all += points_num[i];
      forecast_x_all.resize(points_num_all);
      forecast_y_all.resize(points_num_all);
      int k = 0;
      for(int j = points_num_all - points_num[i]; j < points_num_all; j++){
        forecast_x_all[j] = forecast_x[k];
        forecast_y_all[j] = forecast_y[k];
        k += 1;
      }
    }
/*    if(check == 0){
      velocity_ave0.clear();
      association_num_ave0.clear();
      velocity_ave0.resize((int)objects.points.size());
      association_num_ave0.resize((int)objects.points.size());
      objects_num_ave0 = (int)objects.points.size();
      for(int i = 0; i < (int)objects.points.size(); i++){
        velocity_ave0[i] = velocity[i];
        association_num_ave0[i] = objects.channels[0].values[i];
      }
    }else if(check == 1){
      velocity_ave1.clear();
      association_num_ave1.clear();
      velocity_ave1.resize((int)objects.points.size());
      association_num_ave1.resize((int)objects.points.size());
      objects_num_ave1 = (int)objects.points.size();
      for(int i = 0; i < (int)objects.points.size(); i++){
        velocity_ave1[i] = velocity[i];
        association_num_ave1[i] = objects.channels[0].values[i];
      }
    }else if(check == 2){
      check = 0;
      velocity_ave.clear();
      velocity_ave.resize((int)objects.points.size());
      float velocity_decision[3], velocity_ave[(int)objects.points.size()];
      for(int i = 0; i < (int)objects.points.size(); i++){
        for(int j = 0; j < objects_num_ave1; j++){
          for(int k = 0; k < objects_num_ave0; k++){
            if(objects.channels[0].values[i] == association_num_ave1[j] && objects.channels[0].values[i] == association_num_ave0[k]){
              float velocity_diff0 = abs(velocity[i] - velocity_ave1[j]);
              if(velocity_diff0 > acceptable_acceleration){
                velocity_decision[2] += 1;
                velocity_decision[1] += 1;
              }
              float velocity_diff1 = abs(velocity[i] - velocity_ave0[k]);
              if(velocity_diff1 > acceptable_acceleration){
                velocity_decision[2] += 1;
                velocity_decision[0] += 1;
              }
              float velocity_diff2 = abs(velocity_ave1[j] - velocity_ave0[k]);
              if(velocity_diff2 > acceptable_acceleration){
                velocity_decision[1] += 1;
                velocity_decision[0] += 1;
              }
              for(int l = 0; l < 3; l++){
                if(velocity_decision[l] == 2){
                  if(l == 0){
                    velocity_ave[i] = (velocity[i] + velocity_ave1[j]) / 2;
                  }else if(l == 1){
                    velocity_ave[i] = (velocity[i] + velocity_ave0[k]) / 2;
                  }else if(l == 2){
                    velocity_ave[i] = (velocity_ave1[j] + velocity_ave0[k]) / 2;
                  }
                }else if(l >= 2){
                  velocity_ave[i] = (velocity[i] + velocity_ave1[j] + velocity_ave0[k]) / 3;
                }
              }
            }
          }
        }
        length = velocity_ave[i] * elapsed_time;
        points_num[i] = abs(length / interval);
        ROS_WARN("points_num;%d, num:%f", points_num[i], objects.channels[0].values[i]);
        float forecast_x[points_num[i]];
        float forecast_y[points_num[i]];
        for(int j = 0; j < points_num[i]; j++){
          forecast_x[j] = cos(theta[i]) * (interval * j) + objects.points[i].x;
          forecast_y[j] = sin(theta[i]) * (interval * j) + objects.points[i].y;
        }
        points_num_all += points_num[i];
        forecast_x_all.resize(points_num_all);
        forecast_y_all.resize(points_num_all);
        int k = 0;
        for(int j = points_num_all - points_num[i]; j < points_num_all; j++){
          forecast_x_all[j] = forecast_x[k];
          forecast_y_all[j] = forecast_y[k];
          k += 1;
        }
      }
    }*/
  }

  void run(){
    int old = false, stamp_adjustment = 0;
    callback = false;
    check = 0;
    while(ros::ok()){
      if(callback == true){
        callback = false;
        if(old == true){
          old = false;
          int count = 0;
          for(int i = 0; i < (int)objects.points.size(); i++){
            for(int j = 0; j < objects_old_num_max; j++){
              if(objects.channels[0].values[i] == objects_old_number[j]){
                if(hypotf(objects.points[i].x - objects_old_x[j], objects.points[i].y - objects_old_y[j]) == 0){
                  count += 1;
                }
              }
            }
          }
          if(count == (int)objects.points.size()){
            stamp_adjustment += objects.header.stamp.toSec() - stamp_old;
          }else{
            stamp_old = stamp_old - stamp_adjustment;
            stamp_adjustment = 0;

            calc();
            check += 1;

            prediction_cost.points.clear();
            prediction_cost.points.resize(points_num_all);
            for(int i = 0; i < points_num_all; i++){
              prediction_cost.points[i].x = forecast_x_all[i];
              prediction_cost.points[i].y = forecast_y_all[i];
            }
            prediction_cost.header.frame_id = objects.header.frame_id;
            prediction_cost.header.stamp = ros::Time::now();

            if(!listener.waitForTransform(prediction_cost.header.frame_id, "base_link", ros::Time(0), ros::Duration(10.0))){
              return;
            }
            listener.transformPointCloud("base_link", prediction_cost, prediction_cost);
            path_prediction_pub.publish(prediction_cost);
            ROS_INFO("----");//
          }
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
        objects_old_num_max = (int)objects.points.size();
        stamp_old = objects.header.stamp.toSec();
        old = true;
      }
      ros::spinOnce();
    }
  }
private:
  ros::NodeHandle nh;
  ros::Subscriber associatied_obstacle_sub;
  ros::Publisher path_prediction_pub;
  sensor_msgs::PointCloud objects;
  sensor_msgs::PointCloud prediction_cost;
  tf::TransformListener listener;
  vector<float> objects_old_x, objects_old_y, objects_old_number, velocity, theta, forecast_x_all, forecast_y_all, velocity_ave0, velocity_ave1, association_num_ave0, association_num_ave1, velocity_ave;
  int callback, objects_old_num_max, points_num_all, check, objects_num_ave0, objects_num_ave1;
  float length;
  double stamp_old;

};

int main(int argc, char **argv){
  ros::init(argc, argv, "prediction");
  Prediction path_prediction;
  path_prediction.run();
  return 0;
}
