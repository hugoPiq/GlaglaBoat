#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <cmath>
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include <cstdlib>
#include "std_msgs/Float64.h"

float vx,vy,vz;
float x,y,z;
float R = 0.5;
float wR,wL;
float rho = 1000;
float D = 0.005; //Diam hélice

float Archimede()
{
	float g = 9.81;
	float V = std::M_PI*pow(h,2)*(3*R-2*R)/3;
	float fA = -rho*V*g;
	return fA;
}

float Traction(float n)
{
	float Ct = 0.2; //coef de traction à déterminer
	float Ft = rho*Ct*pow(n,2)*pow(D,4);
	return Ft;
}

float Trainee(float n)
{
	float alpha = 1000; //à affiner
	float Ftrain = alpha*n;
	return Ftrain;
}

float frottement(float v)
{
	float beta = 1;
	float Fr = -1*v*beta;
	return Fr;
}

void getspeed(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  vx = msg->pose.position.x;
  vy = msg->pose.position.y;
  vz = msg->pose.position.z;
}
void getpose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  x = msg->pose.position.x;
  y = msg->pose.position.y;
  z = msg->pose.position.z;
}

void getwR(const std_msgs::Float64::ConstPtr& msg)
{
	wR = msg->data; //unité à préciser
}

void getwR(const std_msgs::Float64::ConstPtr& msg)
{
	wL = msg->data; //unité à préciser
}
int main(int argc, char **argv)
{
	ros::init(argc,argv,"Boat");
	ros::NodeHandle n;
	ros::Publisher arch_pub = n.advertise<std_msgs::Float64>("archimede_force",1000);
	ros::Publisher frot_pub = n.advertise<geometry_msgs::PoseStamped>("frixion_force",1000);
	ros::Publisher hd_pub = n.advertise<geometry_msgs::PoseStamped>("hdroite_forces",1000);
	ros::Publisher hg_pub = n.advertise<geometry_msgs::PoseStamped>("hgauche_forces",1000);
	ros::Subscriber speed_sub = n.subscribe("Speed",1000,getspeed); //référenciel map (déplacement du bateau dans le repère absolu)
	ros::Subscriber pose_sub = n.subscribe("Pose",1000,getpose);//reférenciel map

	ros::Subscriber wL_sub = n.subscribe("RotSpeedLeft",1000,getwL); //vitesse de rotation hélice gauche
	ros::Subscriber wR_sub = n.subscribe("RotSpeedRight",1000,getwR);//vitesse de rotation hélice droite
	

	ros::Rate loop_rate(25);
	while(ros::ok())
	{	//Poussée d'achimède
		std_msgs::Float64 msgarch;
		msgarch.data = Archimede();
		arch_pub.publish(msgarch);

		//Frotement contre l'eau
		geometry_msgs::PoseStamped msgfrot;
		msgfrot.pose.position.x = frottement(vx);
		msgfrot.pose.position.y = frottement(vy);
		msgfrot.pose.position.z = frottement(vz);
		msgfrot.header.frame_id = "Boat"; //à faire évoluer éventuelment
		msgfrot.header.stamp = ros::Time::now();
		frot_pub.publish(msgfrot);

		//Traction et trainée des hélices
		geometry_msgs::PoseStamped msghdroite;
		msghdroite.pose.position.x = Traction(wR); //traction
		msghdroite.pose.position.y = Trainee(wR); //trainée
		msghdroite.header.frame_id = "Boat";
		msghdroite.header.stamp = ros::Time::now();
		hd_pub.publish(msghdroite);

		geometry_msgs::PoseStamped msghgauche;
		msghgauche.pose.position.x = Traction(wL); //traction
		msghgauche.pose.position.y = Trainee(wL); //trainée
		msghgauche.header.frame_id = "Boat";
		msghgauche.header.stamp = ros::Time::now();
		hd_pub.publish(msghgauche);

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}