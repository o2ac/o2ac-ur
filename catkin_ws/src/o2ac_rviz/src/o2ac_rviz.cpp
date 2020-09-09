#include <rviz/panel.h>
#include <ui_panel.h>   // generated from "panel.ui" using CMAKE_AUTOMOC
#include <ros/ros.h>
#include <atomic>

#include <ur_msgs/IOStates.h>
#include "o2ac_skills/o2ac_skill_server.h"

namespace o2ac_rviz {

class O2ACSetupPanel : public rviz::Panel {
    Q_OBJECT
public:
    O2ACSetupPanel(QWidget* parent = nullptr);

// slots
private slots:
    // auto-connected slots: on_<object_name>_<signal_name>(<signal parameters>)

    // buttons
    void on_button_activate_ros_control_clicked();
    void on_button_robots_home_clicked();

    void on_button_home_a_bot_clicked();
    void on_button_open_gripper_a_clicked();
    void on_button_close_gripper_a_clicked();

    void on_button_home_b_bot_clicked();
    void on_button_open_gripper_b_clicked();
    void on_button_close_gripper_b_clicked();

    void on_button_open_base_fixation_clicked();
    void on_button_close_base_fixation_clicked();

    void on_button_a_bot_m3_equip_clicked();
    void on_button_a_bot_m3_unequip_clicked();
    void on_button_a_bot_m3_pick_screw_clicked();

    void on_button_b_bot_m4_equip_clicked();
    void on_button_b_bot_m4_unequip_clicked();
    void on_button_b_bot_m4_pick_screw_clicked();

    void on_button_a_bot_nut_equip_clicked();
    void on_button_a_bot_nut_unequip_clicked();

    void on_button_b_bot_m2s_equip_clicked();
    void on_button_b_bot_m2s_unequip_clicked();
    
    void on_button_tighten_m3_5s_clicked();
    void on_button_tighten_m4_5s_clicked();
    void on_button_loosen_m3_1s_clicked();
    void on_button_loosen_m4_1s_clicked();
    void on_button_tighten_nut_5s_clicked();
    void on_button_tighten_m2s_5s_clicked();
    void on_button_loosen_nut_1s_clicked();
    void on_button_loosen_m2s_1s_clicked();

    void on_button_vacuum_m3_clicked();
    void on_button_vacuum_m4_clicked();

// methods
private:
    void set_button_active(QPushButton *button, const bool active);
    
    void io_state_callback(const ur_msgs::IOStates &states);
    void update_status(const ros::TimerEvent&);

    // The below 2 functions are taken from "planning_scene_display.h" *(MoveIt)
    
    // TODO: Add queue for background jobs (or just live with the buttons not being perfect)
    // /** Queue this function call for execution within the background thread
    // All jobs are queued and processed in order by a single background thread. */
    // void addBackgroundJob(const boost::function<void()>& job, const std::string& name);

    /** Directly spawn a (detached) background thread for execution of this function call
         Should be used, when order of processing is not relevant / job can run in parallel.
        Must be used, when job will be blocking. Using addBackgroundJob() in this case will block other queued jobs as
        well */
    void spawnBackgroundJob(const boost::function<void()>& job);

    void robots_go_home();

    void open_gripper_wrapper(std::string robot);
    void close_gripper_wrapper(std::string robot);
    void go_named_pose_wrapper(std::string robot_name, std::string pose);
    void equip_unequip_wrapper(std::string robot_name, std::string tool, std::string equip_or_unequip);

// members
private:
    Ui::O2ACSetupPanel ui;

    SkillServer ss_;
    ros::NodeHandle n;
    
    ros::Subscriber sub_io_states;
    ros::Timer update_status_timer_;

    actionlib::SimpleActionClient<o2ac_msgs::changeToolAction> changeToolActionClient_; // Only used for heartbeat

    QString red_label_style = "QLabel { background-color : red; color : black; }";
    QString yellow_label_style = "QLabel { background-color : yellow; color : yellow; }";
    QString green_label_style = "QLabel { background-color : green; color : green; }";
    QString grey_label_style = "QLabel { background-color : grey; color : black; }";

    QString red_button_style = "QPushButton { background-color : red; color : black; }";
    QString yellow_button_style = "QPushButton { background-color : yellow; color : black; }";
    QString green_button_style = "QPushButton { background-color : green; color : green; }";
    QString grey_button_style = "QPushButton { background-color : grey; color : black; }";

    bool m3_suction_on, m4_suction_on;
};

O2ACSetupPanel::O2ACSetupPanel(QWidget* parent) : rviz::Panel(parent), changeToolActionClient_("/o2ac_skills/change_tool", true)
{
    // setup ROS (subscribers, publishers, action clients)
    // pub_command = n.advertise<RQ3Fout>("Robotiq3FGripperRobotOutput", 1);
    sub_io_states = n.subscribe("/b_bot/ur_hardware_interface/io_states", 1, &O2ACSetupPanel::io_state_callback, this);

    // load ui form and auto-connect slots
    ui.setupUi(this);

    // Default status is RED
    ui.label_robots_online->setStyleSheet(red_label_style);
    ui.label_skill_server_online->setStyleSheet(red_label_style);
    ui.label_tools_online->setStyleSheet(red_label_style);
    ui.label_tools_online_2->setStyleSheet(red_label_style);

    update_status_timer_ = n.createTimer(ros::Duration(0.5), &O2ACSetupPanel::update_status, this);

    //TODO: Add buttons for vacuum
}


// void O2ACSetupPanel::addBackgroundJob(const boost::function<void()>& job, const std::string& name)
// {
    
// }
void O2ACSetupPanel::spawnBackgroundJob(const boost::function<void()>& job)
{
  boost::thread t(job);
}

void O2ACSetupPanel::update_status(const ros::TimerEvent& event)
{
    // Check tools
    if (ss_.fastening_tool_client.waitForServer(ros::Duration(0.2)))
    {
        ui.label_tools_online->setStyleSheet(green_label_style);
        ui.label_tools_online_2->setStyleSheet(green_label_style);
    }
    else
    {
        ui.label_tools_online->setStyleSheet(red_label_style);
        ui.label_tools_online_2->setStyleSheet(red_label_style);
    }

    // Check skill server
    if (changeToolActionClient_.waitForServer(ros::Duration(0.2)))
        ui.label_skill_server_online->setStyleSheet(green_label_style);
    else
        ui.label_skill_server_online->setStyleSheet(red_label_style);
    
    if (ss_.a_bot_ros_control_active_ && ss_.b_bot_ros_control_active_)
        ui.label_robots_online->setStyleSheet(green_label_style);
    else
        ui.label_robots_online->setStyleSheet(red_label_style);
}

void O2ACSetupPanel::io_state_callback(const ur_msgs::IOStates &states) {
    // First check output (suction on/off)
    if (states.digital_out_states[0].state) // suction m3
    {
        m3_suction_on = true;
        ui.button_vacuum_m3->setText("O");
    }
    else
    {
        m3_suction_on = false;
        ui.button_vacuum_m3->setText("-");
    }
        
    if (states.digital_out_states[1].state) // suction m4
    {
        m4_suction_on = true;
        ui.button_vacuum_m4->setText("O");
    }
    else
    {
        m4_suction_on = false;
        ui.button_vacuum_m4->setText("-");
    }

    // Then check vacuum (suction seal)
    if (states.digital_in_states[0].state) // vacuum m3
    {
        ui.button_vacuum_m3->setStyleSheet(green_button_style);
    }
    else 
    {
        if (m3_suction_on)
            ui.button_vacuum_m3->setStyleSheet(yellow_button_style);
        else
            ui.button_vacuum_m3->setStyleSheet(grey_button_style);
        
    }
    
    if (states.digital_in_states[1].state) // vacuum m4
    {
        ui.button_vacuum_m4->setStyleSheet(green_button_style);
    }
    else 
    {
        if (m4_suction_on)
            ui.button_vacuum_m4->setStyleSheet(yellow_button_style);
        else
            ui.button_vacuum_m4->setStyleSheet(grey_button_style);
    }
}

void O2ACSetupPanel::on_button_activate_ros_control_clicked()
{
    ss_.activateROSControlOnUR("a_bot");
    ss_.activateROSControlOnUR("b_bot");
}

void O2ACSetupPanel::on_button_robots_home_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::robots_go_home, this));  // Required for background processing
}
void O2ACSetupPanel::robots_go_home()
{
    ss_.goToNamedPose("home", "a_bot", 0.2, 0.2);
    ss_.goToNamedPose("home", "b_bot", 0.2, 0.2);
}

void O2ACSetupPanel::on_button_home_a_bot_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::go_named_pose_wrapper, this, "a_bot", "home"));  // Required for background processing
}
void O2ACSetupPanel::on_button_open_gripper_a_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::open_gripper_wrapper, this, "a_bot"));
}
void O2ACSetupPanel::on_button_close_gripper_a_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::close_gripper_wrapper, this, "a_bot"));
}

void O2ACSetupPanel::on_button_home_b_bot_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::go_named_pose_wrapper, this, "b_bot", "home"));  // Required for background processing
}
void O2ACSetupPanel::on_button_open_gripper_b_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::open_gripper_wrapper, this, "b_bot"));
}
void O2ACSetupPanel::on_button_close_gripper_b_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::close_gripper_wrapper, this, "b_bot"));
}

void O2ACSetupPanel::on_button_open_base_fixation_clicked()
{
    ss_.setSuctionEjection("base_plate_lock", false);
    ss_.setSuctionEjection("base_plate_release", true);
}
void O2ACSetupPanel::on_button_close_base_fixation_clicked()
{
    ss_.setSuctionEjection("base_plate_release", false);
    ss_.setSuctionEjection("base_plate_lock", true);
}

// Equip commands

void O2ACSetupPanel::on_button_a_bot_m3_equip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "a_bot", "screw_tool_m3", "equip"));
}
void O2ACSetupPanel::on_button_a_bot_m3_unequip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "a_bot", "screw_tool_m3", "unequip"));
}
void O2ACSetupPanel::on_button_a_bot_m3_pick_screw_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "a_bot", "screw_tool_m3", "equip"));
}

void O2ACSetupPanel::on_button_b_bot_m4_equip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "b_bot", "screw_tool_m4", "equip"));
}
void O2ACSetupPanel::on_button_b_bot_m4_unequip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "b_bot", "screw_tool_m4", "unequip"));
}
void O2ACSetupPanel::on_button_b_bot_m4_pick_screw_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "b_bot", "screw_tool_m4", "equip"));
}

void O2ACSetupPanel::on_button_a_bot_nut_equip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "a_bot", "nut_tool_m6", "equip"));
}
void O2ACSetupPanel::on_button_a_bot_nut_unequip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "a_bot", "nut_tool_m6", "unequip"));
}

void O2ACSetupPanel::on_button_b_bot_m2s_equip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "b_bot", "set_screw_tool", "equip"));
}
void O2ACSetupPanel::on_button_b_bot_m2s_unequip_clicked()
{
    spawnBackgroundJob(boost::bind(&O2ACSetupPanel::equip_unequip_wrapper, this, "b_bot", "set_screw_tool", "unequip"));
}


// === Tool commands

void O2ACSetupPanel::on_button_tighten_m3_5s_clicked()
{
    ss_.sendFasteningToolCommand("screw_tool_m3", "tighten", false, 5.0);
}

void O2ACSetupPanel::on_button_tighten_m4_5s_clicked()
{
    ss_.sendFasteningToolCommand("screw_tool_m4", "tighten", false, 5.0);
}

void O2ACSetupPanel::on_button_loosen_m3_1s_clicked()
{
    ss_.sendFasteningToolCommand("screw_tool_m3", "loosen", false, 1.0);
}

void O2ACSetupPanel::on_button_loosen_m4_1s_clicked()
{
    ss_.sendFasteningToolCommand("screw_tool_m4", "loosen", false, 1.0);
}

void O2ACSetupPanel::on_button_tighten_nut_5s_clicked()
{
    ss_.sendFasteningToolCommand("nut_tool_m6", "tighten", false, 5.0);
}

void O2ACSetupPanel::on_button_tighten_m2s_5s_clicked()
{
    ss_.sendFasteningToolCommand("set_screw_tool", "tighten", false, 5.0);
}

void O2ACSetupPanel::on_button_loosen_nut_1s_clicked()
{
    ss_.sendFasteningToolCommand("nut_tool_m6", "loosen", false, 1.0);
}

void O2ACSetupPanel::on_button_loosen_m2s_1s_clicked()
{
    ss_.sendFasteningToolCommand("set_screw_tool", "loosen", false, 1.0);
}

void O2ACSetupPanel::on_button_vacuum_m3_clicked()
{
    ss_.setSuctionEjection("screw_tool_m3", !m3_suction_on);
}
void O2ACSetupPanel::on_button_vacuum_m4_clicked()
{
    ss_.setSuctionEjection("screw_tool_m4", !m4_suction_on);
}


// Wrappers to allow background processing
void O2ACSetupPanel::open_gripper_wrapper(std::string robot)
{
    ss_.openGripper(robot);
}
void O2ACSetupPanel::close_gripper_wrapper(std::string robot)
{
    ss_.closeGripper(robot);
}
void O2ACSetupPanel::go_named_pose_wrapper(std::string robot_name, std::string pose)
{
    ss_.goToNamedPose(pose, robot_name, 0.2, 0.2);
}
void O2ACSetupPanel::equip_unequip_wrapper(std::string robot_name, std::string tool, std::string equip_or_unequip)
{
    ss_.equipUnequipScrewTool(robot_name, tool, equip_or_unequip);
}

// --- 

void O2ACSetupPanel::set_button_active(QPushButton *button, const bool active) {
    const QString style = active ? "QPushButton {color: red;}" : "QPushButton {}";
    button->setStyleSheet(style);
}

}

#include "o2ac_rviz.moc"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(o2ac_rviz::O2ACSetupPanel, rviz::Panel )
