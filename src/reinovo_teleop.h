#ifndef QREINOVO_TELEOP_H
#define QREINOVO_TELEOP_H
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf/transform_datatypes.h"//转换函数头文件
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Pose2D.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


#include <ui_reinovoqt.h>
#ifndef Q_MOC_RUN
#include "rviz/panel.h"
#include "QTimer"
#include "QMessageBox"
#include "QDialog"
#endif

#include "boost/thread.hpp"
#include "boost/bind.hpp"
#include <string>
#include <sstream>
#include "vector"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


namespace Ui {
class Form;
}

class ReinovoTeleop : public rviz::Panel //public QMainWindow,
{
    Q_OBJECT
public:
    ReinovoTeleop(QWidget *parent = 0);
    virtual ~ReinovoTeleop();

//槽函数
public slots:
    void fspeed_enable();   //速度使能
    void fpub_vxplus();    //发布+vx指令
    void fpub_vxminus();    //发布-vx指令
    void fpub_vyplus();    //发布+vy指令
    void fpub_vyminus();    //发布-vy指令
    void fpub_vthplus();    //发布+vth指令
    void fpub_vthminus();    //发布-vth指令
    void fvel_stop();    //发布stop指令

    void fsave();       //保存导航点
    void fremove();     //删除

    void fgoal_list(QString);

    void fnav_goto();//单点导航
    void fcruise();//多点导航
//nav_goto
//cruise
public:
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    void nav_thread();
    void nav_thread1();

private:
    Ui::Form *ui;

    ros::NodeHandle nh_;
    ros::Publisher vel_pub;
	ros::Subscriber pose_sub;

    vector<string> v_goalname;
    vector<geometry_msgs::Pose2D> v_goal;

    MoveBaseClient* navclient;

    bool fthread;
    boost::thread navthread;

};

#endif // REINOVOQT_H