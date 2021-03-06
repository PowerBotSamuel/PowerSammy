#include <ros/ros.h>
//#include <string>
#include <sensor_msgs/LaserScan.h>
//#include <cstdlib>
#include <stdlib.h>
#include <math.h>
#include <signal.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>

void qrMessReceived(std_msgs::String);
void posMessReceived(geometry_msgs::PoseStamped msg);

ros::NodeHandle *node;
ros::Publisher velocityPub;
ros::Publisher cameraPub;
ros::Subscriber poseSub;
ros::Subscriber qrMessSub;
ros::Subscriber qrPosSub;
ros::Subscriber cameraPosSub;
ros::Subscriber laserSub;

// glowa kreci do wykrycia
// powerbot obrot do glowy
// laser sprawdza w ktore strone ma krecic, ZAPISZ KIERUNEK
// powerbot kreci do prostopadle do sciany (az lasery +-15 rowne w stronie KIERUNEK)
// powerbot 90 stopnie w przeciwna niz KIERUNEK
// glowa 90 stopni w strone KIERUNEK
// do przodu az KOD lub lasry mniej niz 10cm


int status = 0;

double distToMove = 0;      // o ile ma sie ruszyc (z kodu qr, metry)
double initX = 0;           // poczatkowa wspolrzedna x przy ruchu
double initY = 0;           // pocatkowa wspolrzedna y przy ruchu

double angToMove = 0;       // o jaki kat ma sie obrocic na poczatku (z kodu qr, stopnie)
double angToMoveFinal = 0;  // o jaki kat am sie obrocic na koncu (z kodu qr, stopnie)
double targetAng = 0;     // docelowy kat przy obracaniu sie (stopnie)
double initAng = 0;         // poczatkowy kat przy obracaniu sie (stopnie)

double cameraPos = 2.79;
double cameraVel = 0;
double cameraTriggerPos = 0;

double rangeM15 = -100;
double range0 = 0;
double rangeP15 = 1;

int kierunek = 0;

ros::Timer timeoutTimer;

// FUNKCJA ZATRZYMUJACA RUCH I RESETUJACA STATUS DO WSADZENIA POD TIMER
void timeoutStop(const ros::TimerEvent& e)
{
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	velocityPub.publish(vel);
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("!!! TIMEOUT !!!");
	ROS_INFO_STREAM("");
	status = 0;
}

// FUNKCJA ZATRZYMUJACA RUCH DO WYWOLANIA
void stopMove()
{
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
    vel.angular.x = vel.angular.y = vel.angular.z = 0;
	velocityPub.publish(vel);
}

void laserDataReceived(sensor_msgs::LaserScan msg)
{
    rangeM15 = msg.ranges[75];
	range0 = msg.ranges[90];
	rangeP15 = msg.ranges[105];
}

// modul liczby
double mabs(double arg){ return (arg>0)?arg:arg*-1; }

void cameraPosReceived(dynamixel_msgs::JointState msg)
{
	cameraPos = msg.current_pos;
	cameraVel = msg.velocity;
	//ROS_INFO_STREAM(cameraPos);
}

void poseDataReceived(nav_msgs::Odometry msg)
{
	// przelicz aktualny kat i pozycje
	tf::Quaternion quat(msg.pose.pose.orientation.y, msg.pose.pose.orientation.y,
						msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Matrix3x3 mat(quat);
	double r, p, y;
	mat.getRPY(r, p, y);
	// aktualne wspolrzedne
	double currAng = y*360/(2*3.14);
	double currX = msg.pose.pose.position.x;
	double currY = msg.pose.pose.position.y;


	if(status == 1) // obrot do pozycji glowy i glowy w bok
	{
		// zapisz poczatkowe wspolrzedne
		initX = msg.pose.pose.position.x;
		initY = msg.pose.pose.position.y;
		initAng = currAng;

		// oblicz docelowy kat, sprowadz do zakresu (-180 : 180)
		targetAng  = initAng - cameraTriggerPos;
	    while(mabs(targetAng) > 180)
			targetAng += 360*((targetAng>0)?-1:1);

		std_msgs::Float64 cameraTarget;
		cameraTarget.data = (cameraTriggerPos > 0)?3.665:1.2217;
		cameraPub.publish(cameraTarget);

		// nastaw timer, 5s + 10s na każde 90 stopni obrotu
		double maxRotationTime = 5 + mabs(angToMove)*10/90;
		timeoutTimer.stop();
		timeoutTimer.setPeriod(ros::Duration(maxRotationTime));
		timeoutTimer.start();
		status = 2;
	}
	else if(status == 2) // obrot do pozycji glowy
	{
		ROS_INFO_STREAM("CURR:");
		ROS_INFO_STREAM(currAng);
		ROS_INFO_STREAM("TARGET:");
		ROS_INFO_STREAM(targetAng);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(mabs(targetAng - currAng));
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if(mabs(targetAng - currAng) > 1) // dopoki roznica > 1 stopnien
		{
			vel.angular.z = 0.5;
			if(cameraTriggerPos > 0) vel.angular.z = -1*vel.angular.z;
			if(mabs(targetAng - currAng) < 30)
				vel.angular.z = vel.angular.z*(mabs(targetAng - currAng)/30);
			ROS_INFO_STREAM("VEL:");
			ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			stopMove();

			// resetuj i nastaw timer, 5s + 5s na kazdy metr do przejechania
			//double maxMoveTime = 5 + 5*distToMove;
			//timeoutTimer.stop();
			//timeoutTimer.setPeriod(ros::Duration(maxMoveTime));
			//timeoutTimer.start();
			
			status = 3;
		}
		
		
	}
	else if(status == 3) // obrot do prostopadlosci sciany
	{
		double diff = mabs(rangeP15) - mabs(rangeM15);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(diff);
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if( mabs(diff) > 0.05 )
		{
			vel.angular.z = 0.2;
			if(rangeM15 < rangeP15) vel.angular.z = -1*vel.angular.z;
			//if(diff < 0.15)
			//	vel.angular.z = vel.angular.z*(diff*6.66);
			//ROS_INFO_STREAM("VEL:");
			//ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			ROS_INFO_STREAM("OK");
			stopMove();

			// resetuj i nastaw timer, 5s + 5s na kazdy metr do przejechania
			//double maxMoveTime = 5 + 5*distToMove;
			//timeoutTimer.stop();
			//timeoutTimer.setPeriod(ros::Duration(maxMoveTime));
			//timeoutTimer.start();
			
			status = 4;
		}
		
		
	}
	
	else if(status == 4) // obrot 90 stopni
	{
		// zapisz poczatkowe wspolrzedne
		initX = msg.pose.pose.position.x;
		initY = msg.pose.pose.position.y;
		initAng = currAng;
		// oblicz docelowy kat, sprowadz do zakresu (-180 : 180)
		targetAng  = initAng + (90*kierunek);
	    while(mabs(targetAng) > 180)
			targetAng += 360*((targetAng>0)?-1:1);

		
		status = 5;
	}
	else if(status == 5) // obrot 90 stopni
	{
		ROS_INFO_STREAM(status);
    	ROS_INFO_STREAM("CURR:");
		ROS_INFO_STREAM(currAng);
		ROS_INFO_STREAM("TARGET:");
		ROS_INFO_STREAM(targetAng);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(mabs(targetAng - currAng));
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if(mabs(targetAng - currAng) > 1) // dopoki roznica > 1 stopnien
		{
			vel.angular.z = 0.5;
			if(kierunek == 1) vel.angular.z = -1*vel.angular.z;
			if(mabs(targetAng - currAng) < 30)
				vel.angular.z = vel.angular.z*(mabs(targetAng - currAng)/30);
			ROS_INFO_STREAM("VEL:");
			ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			stopMove();			
			status = 6;
		}
		
		
	}
	else if(status == 6) // do przodu
	{
	    geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		vel.linear.x = 0.1;
		velocityPub.publish(vel);
		status = 7;
	}
	/*
	if(status == 1)
	{
		double diff = mabs(rangeM15) - mabs(rangeP15);
		
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(diff);
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if(diff > 0.05) // dopoki roznica > 1 stopnien
		{
			vel.angular.z = 0.5;
			if((rangeM15 - rangeP15) < 0) vel.angular.z = -1*vel.angular.z;
			if(diff < 0.15)
				vel.angular.z = vel.angular.z*(diff*6.66);
			ROS_INFO_STREAM("VEL:");
			ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			stopMove();
			status = 0;
		}
		}*/

	
	if(status == 8) // ruch QR: inicjalizacja
	{
		// zapisz poczatkowe wspolrzedne
		initX = msg.pose.pose.position.x;
		initY = msg.pose.pose.position.y;
		initAng = currAng;

		// oblicz docelowy kat, sprowadz do zakresu (-180 : 180)
		targetAng  = initAng + angToMove;
	    while(mabs(targetAng) > 180)
			targetAng += 360*((targetAng>0)?-1:1);

		// nastaw timer, 5s + 10s na każde 90 stopni obrotu
		double maxRotationTime = 5 + mabs(angToMove)*10/90;
		timeoutTimer.stop();
		timeoutTimer.setPeriod(ros::Duration(maxRotationTime));
		timeoutTimer.start();
		status = 9;
	}
	else if(status == 9) // ruch QR: obrot do zadanego kata
	{
		ROS_INFO_STREAM("CURR:");
		ROS_INFO_STREAM(currAng);
		ROS_INFO_STREAM("TARGET:");
		ROS_INFO_STREAM(targetAng);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(mabs(targetAng - currAng));
		geometry_msgs::Twist vel;
	    vel.linear.x = vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;
		
		if(mabs(targetAng - currAng) > 1) // dopoki roznica > 1 stopnien
		{
			vel.angular.z = 0.5;
			if(angToMove < 0) vel.angular.z = -1*vel.angular.z;
			if(mabs(targetAng - currAng) < 30)
				vel.angular.z = vel.angular.z*(mabs(targetAng - currAng)/30);
			ROS_INFO_STREAM("VEL:");
			ROS_INFO_STREAM(vel.angular.z);
			velocityPub.publish(vel);
		}
		else
		{
			stopMove();

			// resetuj i nastaw timer, 5s + 5s na kazdy metr do przejechania
			double maxMoveTime = 5 + 5*distToMove;
			timeoutTimer.stop();
			timeoutTimer.setPeriod(ros::Duration(maxMoveTime));
			timeoutTimer.start();
			
			status = 10;
		}
		
	}
	else if(status == 10) // ruch QR: do przodu
	{
		// oblicz ile aktualnie przejechane
		double distX = initX - currX;
		double distY = initY - currY;
		double currDist = sqrt(pow(distX,2) + pow(distY,2));
		
		ROS_INFO_STREAM("CURR DIST:");
		ROS_INFO_STREAM(currDist);
		ROS_INFO_STREAM("TARGET DIST:");
		ROS_INFO_STREAM(distToMove);
		ROS_INFO_STREAM("DIFF:");
		ROS_INFO_STREAM(distToMove - currDist);

		geometry_msgs::Twist vel;
	    vel.linear.y = vel.linear.z = 0;
		vel.angular.x = vel.angular.y = vel.angular.z = 0;

		if( ( mabs(distToMove - currDist) > 0.05 ) && ( distToMove != 0 ) )
		{
			vel.linear.x = 0.3 * kierunek;
			if(mabs(distToMove - currDist) < 0.4)
				vel.linear.x = vel.linear.x*((distToMove - currDist)/0.4); // bylo 0.4
			//if((distToMove - currDist) < 0)
			//vel.linear.x = vel.linear.x * -1;
			velocityPub.publish(vel);
			
		}
		else
		{
			timeoutTimer.stop();
			//timeoutTimer.setPeriod(ros::Duration(0.5));
			//timeoutTimer.start();
			
			if(angToMoveFinal == 0) // jesli nie bylo zadanego koncowego obrotu
			{
				stopMove();
				status = 0;
			}
			else
			{
				angToMove = angToMoveFinal;
				angToMoveFinal = 0;
				distToMove = 0;
				status = 8;
				}
			/*stopMove();
			double data = 2.79253 + angToMoveFinal*360/(2*3.14);
			std_msgs::Float64 msg;
			msg.data = data;
			//cameraPub.publish(msg);
			status = 6;*/
	    
		}
		
	}
	
}

/*void qrPosReceived(geometry_msgs::PoseStamped msg)
{
	tf::Quaternion quat(msg.pose.orientation.y, msg.pose.orientation.y,
						msg.pose.orientation.z, msg.pose.orientation.w);
	tf::Matrix3x3 mat(quat);
	double r, p, y;
	mat.getRPY(r, p, y);
	ROS_INFO_STREAM("");
	ROS_INFO_STREAM("X:");
	ROS_INFO_STREAM(msg.pose.position.x);
	ROS_INFO_STREAM("Y:");
	ROS_INFO_STREAM(msg.pose.position.y);
	ROS_INFO_STREAM("Z:");
	ROS_INFO_STREAM(msg.pose.position.z);
	ROS_INFO_STREAM("R:");
	ROS_INFO_STREAM(r*360/(2*3.14));
	ROS_INFO_STREAM("P:");
	ROS_INFO_STREAM(p*360/(2*3.14));
	ROS_INFO_STREAM("Y:");
	ROS_INFO_STREAM(y*360/(2*3.14));
}*/

void qrDataReceived(std_msgs::String msg)
{
	timeoutTimer.stop();
	
	if(status == 0)
	{
		ROS_INFO_STREAM(msg.data);
		std::string qr_msg;
		if(msg.data == std::string(""))
		{
			ROS_INFO_STREAM(cameraPos);
			ROS_INFO_STREAM(cameraVel);
			static int dir = 1;
			if(cameraPos > 4.25) dir = -1;
			else if(cameraPos < 1.35) dir = 1;

			double targetPos;
			if(mabs(cameraVel) < 0.05)
			{
				targetPos = (dir == 1)?4.5:1;
				std_msgs::Float64 posMsg;
				posMsg.data = targetPos;
				cameraPub.publish(posMsg);
			}
		}
		else
		{

			cameraTriggerPos = (cameraPos - 2.79)*360/(2*3.14);
			kierunek = (cameraTriggerPos < 0)?1:-1;
			ROS_INFO_STREAM("OBROT KAMERY NA QR");
			ROS_INFO_STREAM(cameraTriggerPos);
			ROS_INFO_STREAM(kierunek);
			std_msgs::Float64 posMsg;			
			//posMsg.data = cameraPos;
			posMsg.data = 2.79;
			cameraPub.publish(posMsg);
			status = 1;
			
			/*
			*/
		}
	}
	else if(status == 7)
	{
		cameraTriggerPos = (cameraPos - 2.79)*360/(2*3.14);
		
		if(msg.data == std::string("D"))
		{ 
			distToMove = 2.15;
			angToMove = -120;
			angToMoveFinal = 30;	
		}
		else if(msg.data == std::string("B"))
		{ 
			distToMove = 2;
			angToMove = 180;
			angToMoveFinal = 45;
		}
		else if(msg.data == std::string("C"))
		{ 
			distToMove = 1;
			angToMove = -135;
			angToMoveFinal = 0;
		}
		else if(msg.data == std::string("A"))
		{
			distToMove = 1;
			angToMove = -90;
			angToMoveFinal = 0;
		}
		else
		{ 
			distToMove = 0;
			angToMove = 0;
			angToMoveFinal = 0;
   
		}
		if((distToMove != 0)&&(angToMove!=0))
		{
			ROS_INFO_STREAM("ZADANE");
			ROS_INFO_STREAM(angToMove);
			angToMove = angToMove - cameraTriggerPos;
			ROS_INFO_STREAM("NOWE");
			ROS_INFO_STREAM(angToMove);
			status = 8;
		}
	}
}

void sigHandler(int sig)
{
	ROS_INFO_STREAM("SIG");
	stopMove();
	ros::shutdown();
}

int main (int argc, char** argv)
{
	signal(SIGINT, &sigHandler);
	ros::init(argc, argv, "qr_lms", ros::init_options::NoSigintHandler);
	node = new ros::NodeHandle;
	ros::Rate rate(30);

	velocityPub = node->advertise<geometry_msgs::Twist>("/rosaria/cmd_vel", 100);
	cameraPub = node->advertise<std_msgs::Float64>("/nie_controller/command", 100);
	poseSub = node->subscribe("/rosaria/pose", 100, &poseDataReceived);
	qrMessSub = node->subscribe("/visp_auto_tracker/code_message", 100, &qrDataReceived);
	//qrPosSub = node->subscribe("visp_auto_tracker/object_position", 100, &qrPosReceived);
	cameraPosSub = node->subscribe("/nie_controller/state", 100, &cameraPosReceived);
	laserSub = node->subscribe("/scan", 100, &laserDataReceived);

	std_msgs::Float64 initMsg;
	initMsg.data = 2.79;
	cameraPub.publish(initMsg);
	
	timeoutTimer = node->createTimer(ros::Duration(10), timeoutStop, true);
	timeoutTimer.stop();
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	
	ros::spin();
	return 0;
}
