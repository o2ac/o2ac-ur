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

// methods
private:
    void set_button_active(QPushButton *button, const bool active);
    void io_state_callback(const ur_msgs::IOStates &states);

    void update_status(const ros::TimerEvent&);

// members
private:
    Ui::O2ACSetupPanel ui;

    SkillServer ss_;
    ros::NodeHandle n;
    
    ros::Subscriber sub_io_states;
    ros::Timer update_status_timer_;

    actionlib::SimpleActionClient<o2ac_msgs::changeToolAction> changeToolActionClient_; // Never used, just checked

    QString red_style = "QLabel { background-color : red; color : black; }";
    QString yellow_style = "QLabel { background-color : yellow; color : yellow; }";
    QString green_style = "QLabel { background-color : green; color : green; }";
    QString grey_style = "QLabel { background-color : grey; color : black; }";
};

O2ACSetupPanel::O2ACSetupPanel(QWidget* parent) : rviz::Panel(parent), changeToolActionClient_("/o2ac_skills/change_tool", true)
{
    // setup ROS (subscribers, publishers, action clients)
    // pub_command = n.advertise<RQ3Fout>("Robotiq3FGripperRobotOutput", 1);
    sub_io_states = n.subscribe("/b_bot/ur_hardware_interface/io_states", 1, &O2ACSetupPanel::io_state_callback, this);

    // load ui form and auto-connect slots
    ui.setupUi(this);

    // Default status is RED
    ui.label_tool_m3_vac->setStyleSheet(grey_style);
    ui.label_tool_m4_vac->setStyleSheet(grey_style);
    ui.label_robots_online->setStyleSheet(red_style);
    ui.label_skill_server_online->setStyleSheet(red_style);
    ui.label_tools_online->setStyleSheet(red_style);
    ui.label_tools_online_2->setStyleSheet(red_style);

    update_status_timer_ = n.createTimer(ros::Duration(0.5), &O2ACSetupPanel::update_status, this);

    // Disabling buttons that don't work yet. 
    // TODO: Figure out why those commands freeze (move() does not return for some reason)
    ui.button_robots_home->setDisabled(true);
    ui.button_home_a_bot->setDisabled(true);
    ui.button_home_b_bot->setDisabled(true);

    ui.button_a_bot_m3_equip->setDisabled(true);
    ui.button_a_bot_m3_unequip->setDisabled(true);
    ui.button_a_bot_m3_pick_screw->setDisabled(true);

    ui.button_b_bot_m4_equip->setDisabled(true);
    ui.button_b_bot_m4_unequip->setDisabled(true);
    ui.button_b_bot_m4_pick_screw->setDisabled(true);

    ui.button_a_bot_nut_equip->setDisabled(true);
    ui.button_a_bot_nut_unequip->setDisabled(true);

    ui.button_b_bot_m2s_equip->setDisabled(true);
    ui.button_b_bot_m2s_unequip->setDisabled(true);

    //TODO: Add buttons for vacuum
}

void O2ACSetupPanel::update_status(const ros::TimerEvent& event)
{
    // Check tools
    if (ss_.fastening_tool_client.waitForServer(ros::Duration(0.2)))
    {
        ui.label_tools_online->setStyleSheet(green_style);
        ui.label_tools_online_2->setStyleSheet(green_style);
    }
    else
    {
        ui.label_tools_online->setStyleSheet(red_style);
        ui.label_tools_online_2->setStyleSheet(red_style);
    }

    // Check skill server
    if (changeToolActionClient_.waitForServer(ros::Duration(0.2)))
        ui.label_skill_server_online->setStyleSheet(green_style);
    else
        ui.label_skill_server_online->setStyleSheet(red_style);
    
    if (ss_.a_bot_ros_control_active_ && ss_.b_bot_ros_control_active_)
        ui.label_robots_online->setStyleSheet(green_style);
    else
        ui.label_robots_online->setStyleSheet(red_style);
}

void O2ACSetupPanel::io_state_callback(const ur_msgs::IOStates &states) {
    if (states.digital_in_states[0].state) // suction m3
    {
        ui.label_tool_m3_vac->setText("O");
        ui.label_tool_m3_vac->setStyleSheet(green_style);
    }
    else 
    {
        ui.label_tool_m3_vac->setText("");
        ui.label_tool_m3_vac->setStyleSheet(grey_style);
    }
    
    if (states.digital_in_states[1].state) // suction m4
    {
        ui.label_tool_m4_vac->setText("O");
        ui.label_tool_m4_vac->setStyleSheet(green_style);
    }
    else 
    {
        ui.label_tool_m4_vac->setText("");
        ui.label_tool_m4_vac->setStyleSheet(grey_style);
    }
}

void O2ACSetupPanel::on_button_activate_ros_control_clicked()
{
    ss_.activateROSControlOnUR("a_bot");
    ss_.activateROSControlOnUR("b_bot");
}

void O2ACSetupPanel::on_button_robots_home_clicked()
{
    ss_.goToNamedPose("home", "a_bot", 0.2, 0.2);
    ss_.goToNamedPose("home", "b_bot", 0.2, 0.2);
}


void O2ACSetupPanel::on_button_home_a_bot_clicked()
{
    ss_.goToNamedPose("home", "a_bot", 0.5, 0.5);
}
void O2ACSetupPanel::on_button_open_gripper_a_clicked()
{
    ss_.openGripper("a_bot");
}
void O2ACSetupPanel::on_button_close_gripper_a_clicked()
{
    ss_.closeGripper("a_bot");
}

void O2ACSetupPanel::on_button_home_b_bot_clicked()
{
    ss_.goToNamedPose("home", "b_bot", 0.5, 0.5);
}
void O2ACSetupPanel::on_button_open_gripper_b_clicked()
{
    ss_.openGripper("b_bot");
}
void O2ACSetupPanel::on_button_close_gripper_b_clicked()
{
    ss_.closeGripper("b_bot");
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
    ss_.equipUnequipScrewTool("a_bot", "screw_tool_m3", "equip");
}
void O2ACSetupPanel::on_button_a_bot_m3_unequip_clicked()
{
    ss_.equipUnequipScrewTool("a_bot", "screw_tool_m3", "unequip");
}
void O2ACSetupPanel::on_button_a_bot_m3_pick_screw_clicked()
{
    ss_.equipUnequipScrewTool("a_bot", "screw_tool_m3", "equip");
}

void O2ACSetupPanel::on_button_b_bot_m4_equip_clicked()
{
    ss_.equipUnequipScrewTool("b_bot", "screw_tool_m4", "equip");
}
void O2ACSetupPanel::on_button_b_bot_m4_unequip_clicked()
{
    ss_.equipUnequipScrewTool("b_bot", "screw_tool_m4", "unequip");
}
void O2ACSetupPanel::on_button_b_bot_m4_pick_screw_clicked()
{
    ss_.equipUnequipScrewTool("b_bot", "screw_tool_m4", "equip");
}

void O2ACSetupPanel::on_button_a_bot_nut_equip_clicked()
{
    ss_.equipUnequipScrewTool("a_bot", "nut_tool_m6", "equip");
}
void O2ACSetupPanel::on_button_a_bot_nut_unequip_clicked()
{
    ss_.equipUnequipScrewTool("a_bot", "nut_tool_m6", "unequip");
}

void O2ACSetupPanel::on_button_b_bot_m2s_equip_clicked()
{
    ss_.equipUnequipScrewTool("b_bot", "set_screw_tool", "equip");
}
void O2ACSetupPanel::on_button_b_bot_m2s_unequip_clicked()
{
    ss_.equipUnequipScrewTool("b_bot", "set_screw_tool", "unequip");
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


// --- 

void O2ACSetupPanel::set_button_active(QPushButton *button, const bool active) {
    const QString style = active ? "QPushButton {color: red;}" : "QPushButton {}";
    button->setStyleSheet(style);
}

}

#include "o2ac_rviz.moc"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(o2ac_rviz::O2ACSetupPanel, rviz::Panel )
