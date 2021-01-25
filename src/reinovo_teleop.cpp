#include "reinovo_teleop.h"
using namespace std;

ReinovoTeleop::ReinovoTeleop(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Form)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
        ros::console::notifyLoggerLevelsChanged();
    }
    ui->setupUi(this);
    ROS_INFO("ReinovoQt");

    //速度使能
    connect(ui->enable, SIGNAL(clicked()), this, SLOT(fspeed_enable()));   //速度使能
    //保存目标点
    connect(ui->save, SIGNAL(clicked()), this, SLOT(fsave())); 
    //删除目标点
    connect(ui->remove, SIGNAL(clicked()), this, SLOT(fremove())); 
    //目标点列表
    // connect(ui->goal_list, SIGNAL(currentIndexChanged(QString)), this, SLOT(fgoal_list(QString)));         //发布正vx

    //单点导航
    connect(ui->goto_2, SIGNAL(clicked()), this, SLOT(fnav_goto()));         //发布正vx

    //单点导航
    connect(ui->cruise, SIGNAL(clicked()), this, SLOT(fcruise()));         //发布正vx

    ui->vxvalue->setSuffix("m/s");       //设置后缀
    ui->vyvalue->setSuffix("m/s");       //设置后缀
    ui->vthvalue->setSuffix("rad/s");       //设置后缀

    pose_sub = nh_.subscribe("/amcl_pose",100,&ReinovoTeleop::pose_callback,this);

    navclient = new MoveBaseClient ("move_base", true);
	while(!navclient->waitForServer(ros::Duration(5.0))){
  		ROS_INFO("Waiting for the move_base action server to come up");
	}
    fthread = 0;
}


void ReinovoTeleop::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    ui->posex->setValue(msg.pose.pose.position.x);  
    ui->posey->setValue(msg.pose.pose.position.y);

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
    ui->poseth->setValue(yaw);
}

void ReinovoTeleop::nav_thread()
{
    fthread = 1;

    if (!(v_goal.size() > 0))
    {
        cout << "错误: 没有点" << endl;
        fthread = 0;
        return;
    }

    move_base_msgs::MoveBaseGoal goal;  //目标点
    int index = ui->goal_list->currentIndex();

    cout << "开始去 : " << v_goalname[index] << endl;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x =  v_goal[index].x;
    goal.target_pose.pose.position.y =  v_goal[index].y;

    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw( v_goal[index].theta);
    goal.target_pose.pose.orientation.x = orientation.x;
    goal.target_pose.pose.orientation.y = orientation.y;
    goal.target_pose.pose.orientation.z = orientation.z;
    goal.target_pose.pose.orientation.w = orientation.w;

	ROS_INFO("Sending goal");
    navclient->sendGoal(goal);
  	navclient->waitForResult();
    if(navclient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        cout << "已到达" << endl;
    }
    else
    {
        cout << "错误" << endl;
    }
    fthread = 0;
}

void ReinovoTeleop::nav_thread1()
{
    fthread = 1;
    if (!(v_goal.size() > 0))
    {
        cout << "错误: 没有点" << endl;
        fthread = 0;
        return;
    }

    for (size_t i = 0; i < v_goal.size(); i++)
    {
        move_base_msgs::MoveBaseGoal goal;  //目标点

        cout << "开始去 : " << v_goalname[i] << endl;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x =  v_goal[i].x;
        goal.target_pose.pose.position.y =  v_goal[i].y;

        geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw( v_goal[i].theta);
        goal.target_pose.pose.orientation.x = orientation.x;
        goal.target_pose.pose.orientation.y = orientation.y;
        goal.target_pose.pose.orientation.z = orientation.z;
        goal.target_pose.pose.orientation.w = orientation.w;

        ROS_INFO("Sending goal");
        navclient->sendGoal(goal);
        navclient->waitForResult();
        if(navclient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            cout << "已到达" << endl;
        }
        else
        {
            cout << "错误" << endl;
        }
    }
    fthread = 0;
}

void ReinovoTeleop::fnav_goto()
{
    cout << "nav go to" << endl;
    if (fthread == 0)
    {
        navthread = boost::thread(boost::bind(&ReinovoTeleop::nav_thread, this));
    }else{
        cout << "正在导航中!" << endl;
    }
}

void ReinovoTeleop::fcruise()
{
    cout << "nav go to" << endl;
    if (fthread == 0)
    {
        navthread = boost::thread(boost::bind(&ReinovoTeleop::nav_thread1, this));
    }else{
        cout << "正在导航中!" << endl;
    }
}



void ReinovoTeleop::fremove()
{
    if (v_goal.size() > 0)
    {
        if (v_goal.size() == 1)
        {
            disconnect(ui->goal_list, SIGNAL(currentIndexChanged(QString)), this, SLOT(fgoal_list(QString)));         //发布正vx
            ui->goal_list->clear();
            v_goal.clear();
            v_goalname.clear();
        }else{
            int index = ui->goal_list->currentIndex();
            ui->goal_list->removeItem(index);
            v_goal.erase(v_goal.begin()+index);
            v_goalname.erase(v_goalname.begin()+index);
        }
    }else
        cout << "错误：目标点删除完了！" << endl;
}


void ReinovoTeleop::fsave()
{
    string text=ui->goalname->text().toStdString();
    if (text==""){
        cout << "错误：请设置目标点名称！" << endl;
    }else{
        if (v_goal.size() > 0)
        {
            for (size_t i = 0; i < v_goal.size(); i++)
            {
                if (v_goalname[i] == text)
                {
                    cout << "与已有导航点重名" << endl;
                    return;
                }
            }
        }
        if (v_goal.size() == 0)
            connect(ui->goal_list, SIGNAL(currentIndexChanged(QString)), this, SLOT(fgoal_list(QString)));
        v_goalname.push_back(text);
        geometry_msgs::Pose2D pose;
        pose.x = ui->posex->value();
        pose.y = ui->posey->value();
        pose.theta = ui->poseth->value();
        v_goal.push_back(pose);
        //添加
        ui->goal_list->addItem(ui->goalname->text());
        //删除
        // ui->goal_list->removeItem(curIndex);
    }
}

void ReinovoTeleop::fgoal_list(QString)
{
    int index = ui->goal_list->currentIndex();
    cout << "当前目标点 :　" << v_goalname[index] << endl;

    cout << "x: " << v_goal[index].x << endl;
    cout << "y: " << v_goal[index].y << endl;
    cout << "th: " << v_goal[index].theta << endl;
}


void ReinovoTeleop::fspeed_enable(){
    //如果被勾选了
    if (ui->enable->isChecked()){
        string text=ui->topic->text().toStdString();
        if (text==""){
            cout << "错误：请设置速度话题名称！" << endl;
        }else{
            connect(ui->vxplus, SIGNAL(clicked()), this, SLOT(fpub_vxplus()));         //发布正vx
            connect(ui->vxneg, SIGNAL(clicked()), this, SLOT(fpub_vxminus()));         //发布负vx
            connect(ui->vyplus, SIGNAL(clicked()), this, SLOT(fpub_vyplus()));         //发布正vy
            connect(ui->vyneg, SIGNAL(clicked()), this, SLOT(fpub_vyminus()));         //发布负vy
            connect(ui->vthplus, SIGNAL(clicked()), this, SLOT(fpub_vthplus()));         //发布正vx
            connect(ui->vthneg, SIGNAL(clicked()), this, SLOT(fpub_vthminus()));         //发布正vx
            // connect(ui->vel_stop, SIGNAL(clicked()), this, SLOT(fvel_stop()));         //发布正vx
            vel_pub = nh_.advertise<geometry_msgs::Twist>(text, 1 );    //发布速度信息

            cout << "速度已使能！" << endl;
        }
    }else{
            disconnect(ui->vxplus, SIGNAL(clicked()), this, SLOT(fpub_vxplus()));         //发布正vx
            disconnect(ui->vxneg, SIGNAL(clicked()), this, SLOT(fpub_vxminus()));         //发布负vx
            disconnect(ui->vyplus, SIGNAL(clicked()), this, SLOT(fpub_vyplus()));         //发布正vy
            disconnect(ui->vyneg, SIGNAL(clicked()), this, SLOT(fpub_vyminus()));         //发布负vy
            disconnect(ui->vthplus, SIGNAL(clicked()), this, SLOT(fpub_vthplus()));         //发布正vx
            disconnect(ui->vthneg, SIGNAL(clicked()), this, SLOT(fpub_vthminus()));         //发布正vx
            cout << "取消发布速度！" << endl;
            vel_pub.shutdown();
    }
}

void ReinovoTeleop::fpub_vxplus()
{
    geometry_msgs::Twist msg;
    msg.linear.x = ui->vxvalue->value();
    vel_pub.publish(msg);
}

void ReinovoTeleop::fpub_vxminus()
{
    geometry_msgs::Twist msg;
    msg.linear.x = -(ui->vxvalue->value());
    vel_pub.publish(msg);
}
void ReinovoTeleop::fpub_vyplus()
{
    geometry_msgs::Twist msg;
    msg.linear.y = ui->vyvalue->value();
    vel_pub.publish(msg);
}
void ReinovoTeleop::fpub_vyminus()
{
    geometry_msgs::Twist msg;
    msg.linear.y = -(ui->vyvalue->value());
    vel_pub.publish(msg);
}

void ReinovoTeleop::fpub_vthplus()
{
    geometry_msgs::Twist msg;
    msg.angular.z = (ui->vthvalue->value());
    vel_pub.publish(msg);
}
void ReinovoTeleop::fpub_vthminus()
{
    geometry_msgs::Twist msg;
    msg.angular.z = -(ui->vthvalue->value());
    vel_pub.publish(msg);
}



ReinovoTeleop::~ReinovoTeleop()
{
    delete ui;
    ROS_DEBUG_STREAM("reinovo qt已关闭");
}


#include "pluginlib/class_list_macros.h"
PLUGINLIB_EXPORT_CLASS(ReinovoTeleop, rviz::Panel)