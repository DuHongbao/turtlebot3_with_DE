#include<iostream>
#include <math.h>
#include<ctime>
#include<ros/ros.h>
#include<tf/tf.h>
#include<tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include<tf/transform_datatypes.h>
#include<geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include<vector>
#include<fstream>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include<unistd.h>

using namespace std;


Eigen::Vector3d obs;
double obs_d[10][3];
vector<Eigen::Vector3d>  obs_n;
nav_msgs::Odometry odom;
Eigen::Vector3d target(10.0, -5.0, 0.0 );

double Tao(  int j, geometry_msgs::Point mbot_pose_1){
  double tao(1);
  tao = (pow(obs_n[j][0] - mbot_pose_1.x,2) +  pow(obs_n[j][1] - mbot_pose_1.y,2))/0.5; 

  return tao;
}




Eigen::Matrix2d modulation_Matrix(vector<Eigen::Vector3d> obs_nn ,geometry_msgs::Point mbot_pose ){
  Eigen::Matrix2d modu,E_matrix,D_martix;
  vector<double> w(obs_nn.size(),1);
  modu<<1,0,0,1;
  for(int i = 0; i<obs_nn.size();i++){
    E_matrix<<(mbot_pose.x - obs_nn[i][0]), -(mbot_pose.y - obs_nn[i][1]), 
                        (mbot_pose.y - obs_nn[i][1]), mbot_pose.x - obs_nn[i][0];
    E_matrix = (2.0/0.25)*E_matrix; 

  for(int j = 0;    j< obs_nn.size();    j++    ){
        if(j!=i){
            w[i] *= (Tao(j, mbot_pose) - 1.0)/(  (Tao(i, mbot_pose) - 1.0) + (Tao(j, mbot_pose) - 1.0)   );
        }
  }
    D_martix<< 1.0 - w[i]/pow(Tao(i,mbot_pose), 0.5 ),  0.0, 
                            0.0,  1.0 + w[i]/pow(Tao(i,mbot_pose), 0.5);

    modu *= E_matrix * D_martix * E_matrix.inverse();  
    //cout<<"D_matrix is:"<<D_martix<<endl;
    //cout<<"mbot_pose  is:"<<mbot_pose<<endl;
  }

  return modu;
}



void odomCallbck(const nav_msgs::Odometry& msg) {
odom= msg;
//cout<<"The odom odom.pose.pose.position.x is"<<odom.pose.pose.position.x<<endl;
}


int main(int argc, char** argv){
  ros::init(argc,argv,"modulation_vel");
  ros::NodeHandle nh("~");
  sleep(10);
  ros::Publisher m_cmd_pub;
  ros::Subscriber odom_sub  = nh.subscribe("/burger/odom", 1, odomCallbck);
  m_cmd_pub = nh.advertise<geometry_msgs::Twist>("/burger/cmd_vel",10);
  geometry_msgs::Twist twist_1;
  double roll, pitch, yaw;
  double mbot_velocity(0.5);
  Eigen::Vector2d v_origin(0.1, 0.1), v_modu(0.1 ,0.1);

  //Eigen::AngleAxisd v_orientation = odom.pose.pose.orientation;
  //Eigen::Matrix3d t_R = t_V.matrix();
  //Eigen::Quaterniond q_orientation;



  ifstream obsFile("/home/derek/turtlebot3_ws/turtlebot3/src/modulation_velocity/src/obstacles.txt");
	if (!obsFile.is_open())
	{
		cout << "can not open this file" << endl;
	}


  twist_1.linear.x = 0.6;
  twist_1.linear.y = 0.0;
  twist_1.linear.z = 0.0;
  twist_1.angular.x = 0.0;
  twist_1.angular.y = 0.0;
  twist_1.angular.z = 0.0;




  for(int i = 0; i<10; i++){
    for(int j = 0; j<3; j++){
      obsFile>>obs_d[i][j];
    }
  }
  for(int i = 0; i<10; i++ ){
    obs << obs_d[i][0],  obs_d[i][1],  obs_d[i][2];
    obs_n.push_back(obs);
  }
  for(int i = 0; i<10; i++ ){
    //cout<<"obs"<<i<<":"<<obs_n[i][0]<<"   "<<obs_n[i][1]<<"  "<<obs_n[i][2]<<endl;
  }
cout<<"---------hello modulation velocity--------"<<endl;


  //cout<<twist_1.linear.x<<" "twist_1.linear.y<<" "<<twist_1.linear.z<<endl;
    // 设置循环的频率
ros::Rate loop_rate(100);
while(ros::ok()){
    clock_t time_stt = clock();
    ros::spinOnce();

    tf2::Quaternion q_orientation(0,0,0,1);
    q_orientation.normalize();
    tf2::convert(odom.pose.pose.orientation, q_orientation);
    //tf2::quaternionMsgToTF(odom.pose.pose.orientation, q_orientation);
    target[1]+=0.002;
    double yaw_d = atan( (odom.pose.pose.position.y - target[1])/(odom.pose.pose.position.x - target[0]) );
    v_origin<< mbot_velocity*cos(yaw_d), mbot_velocity*sin(yaw_d);
    Eigen::Matrix2d modu = modulation_Matrix(obs_n, odom.pose.pose.position);
    v_modu = modu*v_origin;
    yaw_d = atan(v_modu[1]/v_modu[0]);
    tf2::Matrix3x3(q_orientation).getRPY(roll,pitch,yaw); 
    cout <<"time use in Modulation  is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
//while(abs(yaw-yaw_d)>0.01){
    //cout<<"yaw not equal yaw_d!!!"<<endl;
    if (yaw<yaw_d)
      twist_1.angular.z = 2.8*(yaw_d-yaw);
    else 
      twist_1.angular.z = -2.8*(yaw-yaw_d);
    //twist_1.linear.x -= 0.001;
    //if(odom.pose.pose.orientation )
    //twist_1.angular.z = atan((odom.pose.pose.position.x - target[0]) / (odom.pose.pose.position.y - target[1]) );
    m_cmd_pub.publish(twist_1);

//}

    //cout<<"turtlebot3_No1:"<<twist_1<<endl;
    //cout<<"the roll is:"<<  roll   <<endl;
    //cout<<"the pitch is:"<<   pitch   <<endl;
    //cout<<"the yaw is:"<<   yaw   <<endl;
    //cout<<"the yaw_d is:"<<yaw_d;
    //ros::Duration(1.0).sleep();
    // 按照循环频率延时
    loop_rate.sleep();
   }


  return 0;
}
