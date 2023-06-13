#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

using namespace std;
    
    float a[5][3]={
                {0.0, 0.0, 5.0},
                {0.0, 5.0, 5.0},
                {0.0, 2.5, 8.0},
                {0.0, 0.0, 5.0},
                {0.0, 0.0, 0.0} };  

geometry_msgs::PoseStamped pose;    // drone_ +

double velocity_horizontal_max, velocity_vertical_max; // максимальные скорости по осям
double Kp_horizontal, Ki_horizontal, Kd_horizontal, Kp_vertical, Ki_vertical, Kd_vertical;//коэфиценты для регуляторов
double drone_velocity_y, drone_velocity_z, drone_velocity_x;// запрашиваемая скорость дрона
double Ix, Iy, Iz; //предыдущее значение интегрального регулятора
double err_last_x ,err_last_y,err_last_z;//ошибка на предыдушем такте
double drone_x,drone_y,drone_z; //текущее положение дрона
double distancedd;// пастояние до целефой точки
double kx,ky,kz; //выравнивающие коэфиуенты по осям
double goal_point_x, goal_point_y, goal_point_z; // целевая точка в глобальных координатах 


void receiveCb(const geometry_msgs::PoseStamped::ConstPtr& movement)
{
    pose = *movement;
}


int main(int argc, char **argv)
{     

     ros::init(argc, argv, "regspeed");     

     ros::NodeHandle n;     

     ros::Publisher goal_velocity_pub = n.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1000);     // goal_velocity_
     
     ros::Subscriber position_sub = n.subscribe("/mavros/local_position/pose", 1000, receiveCb); // 

    // ros::Rate loop_rate(20);   
    

    velocity_horizontal_max = 3;  //  Максималная скорость горизонт   
    velocity_vertical_max = 3; // Максималная скорость drone_z

    //h горизонтальные коэфиценты
    Kp_horizontal = 1;//пропорциональный коэфицент
    Ki_horizontal =0 ;//итегральный коэфицент
    Kd_horizontal = 0;//2дифференциальный коэфицент

    //v вертикальные коэфиценты
    Kp_vertical = 1;//пропорциональный коэфицент
    Ki_vertical = 0;//итегральный коэфицент
    Kd_vertical = 0;//2дифференциальный коэфицент

    //сервисные переменные

    Ix = 0;//предудушее значение интегрального регулятора // + last_
    Iy = 0;//предудушее значение интегрального регулятора
    Iz = 0;//предудушее значение интегрального регулятора

    err_last_x = 0;//предудушее значение ошибки // err_last_x
    err_last_y = 0;//предудушее значение ошибки
    err_last_z = 0;//предудушее значение ошибки


     //twist.leaner.drone_x


    drone_x = pose.pose.position.x; // x, drone_y, drone_z -  Какие????
    drone_y = pose.pose.position.y;
    drone_z = pose.pose.position.z;
    
    /*
    int i = 0;
    int count;
    */

    int count=0;
        

        while  (ros::ok())
        {
            geometry_msgs::TwistStamped twist;
            goal_point_x = a[count][0];//целевая точка goal_point_x
             goal_point_y = a[count][1];//целевая точка
              goal_point_z = a[count][2];//целевая точка
            distancedd = std::sqrt(std::pow(goal_point_x-drone_x,2)+std::pow(goal_point_y-drone_y,2)+std::pow(goal_point_z-drone_z,2));
            kx = std::abs((goal_point_x-drone_x)/distancedd);  // ЧТО ЗА k????????????//
            ky = std::abs((goal_point_y-drone_y)/distancedd);
            kz = std::abs((goal_point_z-drone_z)/distancedd);
            drone_velocity_x= Kp_horizontal*kx*(goal_point_x-drone_x)+Ix+Ki_horizontal*(goal_point_x-drone_x)+Kd_horizontal*((goal_point_x-drone_x)-err_last_x);
            drone_velocity_y = Kp_horizontal*ky*(goal_point_y-drone_y)+Iy+Ki_horizontal*(goal_point_y-drone_y)+Kd_horizontal*((goal_point_y-drone_y)-err_last_y);
            drone_velocity_z = Kp_vertical*kz*(goal_point_z-drone_z)+Iz+Ki_vertical*(goal_point_z-drone_z)+Kd_vertical*((goal_point_z-drone_z)-err_last_z);

            if (drone_velocity_x> velocity_horizontal_max * kx){ // drone_velocity_x
                drone_velocity_x= velocity_horizontal_max * kx;
            }
            if (drone_velocity_x< velocity_horizontal_max * kx * (-1)){   // -, _*_
                drone_velocity_x= velocity_horizontal_max * kx * (-1);
            }
            if (drone_velocity_y > velocity_horizontal_max * ky){
                drone_velocity_y = velocity_horizontal_max * ky;
            }
            if (drone_velocity_y < velocity_horizontal_max * ky*(-1)){
                drone_velocity_y = velocity_horizontal_max * ky*(-1);
            }

            if (drone_velocity_z > velocity_vertical_max){
                drone_velocity_z = velocity_vertical_max;
            }
            if (drone_velocity_z < velocity_vertical_max * (-1)){
                drone_velocity_z = velocity_vertical_max * (-1);
            }

            Ix = Ki_horizontal * (goal_point_x-drone_x);
            Iy = Ki_horizontal * ( goal_point_y-drone_y);
            Iz = Ki_vertical * (goal_point_z-drone_z);

            err_last_x = goal_point_x-drone_x;
            err_last_y = goal_point_y-drone_y;
            err_last_z = goal_point_z-drone_z;

            /*
            
            */

            twist.twist.linear.x = drone_velocity_x;
            twist.twist.linear.y = drone_velocity_y;
            twist.twist.linear.z = drone_velocity_z ;
            goal_velocity_pub.publish(twist);

            drone_x = pose.pose.position.x;
            drone_y = pose.pose.position.y;
            drone_z = pose.pose.position.z;


            if(std::sqrt(std::pow(goal_point_x - drone_x,2)+std::pow(goal_point_y - drone_y,2)+std::pow(goal_point_z - drone_z,2)) <= 0.2){
                count++;
                

            }


            ros::spinOnce();

        }
    return 0;      
}     




// perepisat maks speed



