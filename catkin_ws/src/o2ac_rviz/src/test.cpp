/********************************************************************************
** Form generated from reading UI file 'panel.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef TEST_H
#define TEST_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_O2ACSetupForm {
public:
  QVBoxLayout *verticalLayout;
  QHBoxLayout *group_activation;
  QPushButton *button_activate_ros_control;
  QSpacerItem *horizontalSpacer;
  QPushButton *button_robots_home;
  QHBoxLayout *horizontalLayout_3;
  QGroupBox *a_bot_group;
  QHBoxLayout *horizontalLayout_4;
  QVBoxLayout *verticalLayout_a_bot;
  QPushButton *button_home_a_bot;
  QPushButton *button_open_gripper_a;
  QPushButton *button_close_gripper_a;
  QGroupBox *b_bot_group;
  QHBoxLayout *horizontalLayout_5;
  QVBoxLayout *verticalLayout_b_bot;
  QPushButton *button_home_b_bot;
  QPushButton *button_open_gripper_b;
  QPushButton *button_close_gripper_b;
  QGroupBox *base_plate_fixation_group;
  QHBoxLayout *horizontalLayout_2;
  QVBoxLayout *layout_position;
  QPushButton *button_open_base_fixation;
  QPushButton *button_close_base_fixation;
  QGroupBox *tool_group_box;
  QHBoxLayout *horizontalLayout_6;
  QGridLayout *gridLayout;
  QLabel *tool_header_vac;
  QPushButton *button_a_bot_m4_pick_screw;
  QPushButton *button_b_bot_m4_pick_screw;
  QLabel *tool_row_m4;
  QPushButton *button_loosen_m4_1s;
  QPushButton *button_a_bot_m4_equip;
  QLabel *label_tool_m3_alive;
  QPushButton *button_b_bot_m4_unequip;
  QLabel *tool_header_alive;
  QLabel *tool_header_loosen;
  QLabel *tool_row_m3;
  QPushButton *button_b_bot_m4_equip;
  QPushButton *button_tighten_m4_5s;
  QPushButton *button_a_bot_m4_unequip;
  QLabel *tool_header_pick;
  QLabel *tool_header_equip;
  QLabel *label_tool_m3_vac;
  QLabel *tool_header_unequip;
  QLabel *label_tool_m4_vac;
  QPushButton *button_loosen_m3_1s;
  QPushButton *button_tighten_m3_5s;
  QLabel *label_tool_m4_alive;
  QLabel *tool_row_padless_m4;
  QLabel *tool_header_tighten;
  QLabel *tool_row_m2s;
  QLabel *label_tool_padless_m4_alive;
  QLabel *label_tool_m2s_alive;
  QPushButton *button_tighten_padless_m4_5s;
  QPushButton *button_tighten_m2s_5s;
  QPushButton *button_loosen_padless_m4_1s;
  QPushButton *button_loosen_m2s_1s;
  QPushButton *button_a_bot_padless_m4_equip;
  QPushButton *button_a_bot_m2s_unequip;
  QPushButton *button_close_b_8;
  QPushButton *button_b_bot_m2s_equip;
  QLabel *label_tool_padless_m4_vac;
  QLabel *label_tool_m2s_vac;

  void setupUi(QWidget *O2ACSetupForm) {
    if (O2ACSetupForm->objectName().isEmpty())
      O2ACSetupForm->setObjectName(QStringLiteral("O2ACSetupForm"));
    O2ACSetupForm->resize(385, 366);
    verticalLayout = new QVBoxLayout(O2ACSetupForm);
    verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
    group_activation = new QHBoxLayout();
    group_activation->setObjectName(QStringLiteral("group_activation"));
    button_activate_ros_control = new QPushButton(O2ACSetupForm);
    button_activate_ros_control->setObjectName(
        QStringLiteral("button_activate_ros_control"));

    group_activation->addWidget(button_activate_ros_control);

    horizontalSpacer =
        new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    group_activation->addItem(horizontalSpacer);

    button_robots_home = new QPushButton(O2ACSetupForm);
    button_robots_home->setObjectName(QStringLiteral("button_robots_home"));

    group_activation->addWidget(button_robots_home);

    verticalLayout->addLayout(group_activation);

    cameras_group = new QGroupBox(O2ACSetupForm);
    cameras_group->setObjectName(QStringLiteral("cameras_group"));
    cameras_group->setAlignment(Qt::AlignCenter);
    cameras_group->setFlat(false);
    camerasHorizontalLayout = new QHBoxLayout();
    camerasHorizontalLayout->setObjectName(QStringLiteral("camerasHorizontalLayout"));

    a_bot_inside_camera_btn = new QPushButton(cameras_group);
    a_bot_inside_camera_btn->setObjectName(QStringLiteral("a_bot_inside_camera_btn"));
    camerasHorizontalLayout->addWidget(a_bot_inside_camera_btn);
    
    a_bot_outside_camera_btn = new QPushButton(cameras_group);
    a_bot_outside_camera_btn->setObjectName(QStringLiteral("a_bot_outside_camera_btn"));
    camerasHorizontalLayout->addWidget(a_bot_outside_camera_btn);
    
    b_bot_inside_camera_btn = new QPushButton(cameras_group);
    b_bot_inside_camera_btn->setObjectName(QStringLiteral("b_bot_inside_camera_btn"));
    camerasHorizontalLayout->addWidget(b_bot_inside_camera_btn);
    
    b_bot_outside_camera_btn = new QPushButton(cameras_group);
    b_bot_outside_camera_btn->setObjectName(QStringLiteral("b_bot_outside_camera_btn"));
    camerasHorizontalLayout->addWidget(b_bot_outside_camera_btn);

    verticalLayout->addLayout(camerasHorizontalLayout);

    horizontalLayout_3 = new QHBoxLayout();
    horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
    a_bot_group = new QGroupBox(O2ACSetupForm);
    a_bot_group->setObjectName(QStringLiteral("a_bot_group"));
    a_bot_group->setAlignment(Qt::AlignCenter);
    a_bot_group->setFlat(false);
    horizontalLayout_4 = new QHBoxLayout(a_bot_group);
    horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
    verticalLayout_a_bot = new QVBoxLayout();
    verticalLayout_a_bot->setObjectName(QStringLiteral("verticalLayout_a_bot"));
    button_home_a_bot = new QPushButton(a_bot_group);
    button_home_a_bot->setObjectName(QStringLiteral("button_home_a_bot"));

    verticalLayout_a_bot->addWidget(button_home_a_bot);

    button_open_gripper_a = new QPushButton(a_bot_group);
    button_open_gripper_a->setObjectName(
        QStringLiteral("button_open_gripper_a"));

    verticalLayout_a_bot->addWidget(button_open_gripper_a);

    button_close_gripper_a = new QPushButton(a_bot_group);
    button_close_gripper_a->setObjectName(
        QStringLiteral("button_close_gripper_a"));

    verticalLayout_a_bot->addWidget(button_close_gripper_a);

    horizontalLayout_4->addLayout(verticalLayout_a_bot);

    horizontalLayout_3->addWidget(a_bot_group);

    b_bot_group = new QGroupBox(O2ACSetupForm);
    b_bot_group->setObjectName(QStringLiteral("b_bot_group"));
    b_bot_group->setAlignment(Qt::AlignCenter);
    b_bot_group->setFlat(false);
    horizontalLayout_5 = new QHBoxLayout(b_bot_group);
    horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
    verticalLayout_b_bot = new QVBoxLayout();
    verticalLayout_b_bot->setObjectName(QStringLiteral("verticalLayout_b_bot"));
    button_home_b_bot = new QPushButton(b_bot_group);
    button_home_b_bot->setObjectName(QStringLiteral("button_home_b_bot"));

    verticalLayout_b_bot->addWidget(button_home_b_bot);

    button_open_gripper_b = new QPushButton(b_bot_group);
    button_open_gripper_b->setObjectName(
        QStringLiteral("button_open_gripper_b"));

    verticalLayout_b_bot->addWidget(button_open_gripper_b);

    button_close_gripper_b = new QPushButton(b_bot_group);
    button_close_gripper_b->setObjectName(
        QStringLiteral("button_close_gripper_b"));

    verticalLayout_b_bot->addWidget(button_close_gripper_b);

    horizontalLayout_5->addLayout(verticalLayout_b_bot);

    horizontalLayout_3->addWidget(b_bot_group);

    base_plate_fixation_group = new QGroupBox(O2ACSetupForm);
    base_plate_fixation_group->setObjectName(
        QStringLiteral("base_plate_fixation_group"));
    base_plate_fixation_group->setAlignment(Qt::AlignCenter);
    horizontalLayout_2 = new QHBoxLayout(base_plate_fixation_group);
    horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
    layout_position = new QVBoxLayout();
    layout_position->setObjectName(QStringLiteral("layout_position"));
    button_open_base_fixation = new QPushButton(base_plate_fixation_group);
    button_open_base_fixation->setObjectName(
        QStringLiteral("button_open_base_fixation"));

    layout_position->addWidget(button_open_base_fixation);

    button_close_base_fixation = new QPushButton(base_plate_fixation_group);
    button_close_base_fixation->setObjectName(
        QStringLiteral("button_close_base_fixation"));

    layout_position->addWidget(button_close_base_fixation);

    horizontalLayout_2->addLayout(layout_position);

    horizontalLayout_3->addWidget(base_plate_fixation_group);

    verticalLayout->addLayout(horizontalLayout_3);

    tool_group_box = new QGroupBox(O2ACSetupForm);
    tool_group_box->setObjectName(QStringLiteral("tool_group_box"));
    tool_group_box->setAlignment(Qt::AlignCenter);
    tool_group_box->setFlat(false);
    horizontalLayout_6 = new QHBoxLayout(tool_group_box);
    horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
    gridLayout = new QGridLayout();
    gridLayout->setObjectName(QStringLiteral("gridLayout"));
    tool_header_vac = new QLabel(tool_group_box);
    tool_header_vac->setObjectName(QStringLiteral("tool_header_vac"));
    tool_header_vac->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_vac, 0, 4, 1, 1);

    button_a_bot_m4_pick_screw = new QPushButton(tool_group_box);
    button_a_bot_m4_pick_screw->setObjectName(
        QStringLiteral("button_a_bot_m4_pick_screw"));
    QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    sizePolicy.setHorizontalStretch(0);
    sizePolicy.setVerticalStretch(0);
    sizePolicy.setHeightForWidth(
        button_a_bot_m4_pick_screw->sizePolicy().hasHeightForWidth());
    button_a_bot_m4_pick_screw->setSizePolicy(sizePolicy);
    button_a_bot_m4_pick_screw->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_a_bot_m4_pick_screw, 1, 8, 1, 1);

    button_b_bot_m4_pick_screw = new QPushButton(tool_group_box);
    button_b_bot_m4_pick_screw->setObjectName(
        QStringLiteral("button_b_bot_m4_pick_screw"));
    button_b_bot_m4_pick_screw->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_b_bot_m4_pick_screw, 2, 8, 1, 1);

    tool_row_m4 = new QLabel(tool_group_box);
    tool_row_m4->setObjectName(QStringLiteral("tool_row_m4"));
    tool_row_m4->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_row_m4, 2, 0, 1, 1);

    button_loosen_m4_1s = new QPushButton(tool_group_box);
    button_loosen_m4_1s->setObjectName(QStringLiteral("button_loosen_m4_1s"));
    button_loosen_m4_1s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_loosen_m4_1s, 2, 3, 1, 1);

    button_a_bot_m4_equip = new QPushButton(tool_group_box);
    button_a_bot_m4_equip->setObjectName(
        QStringLiteral("button_a_bot_m4_equip"));
    sizePolicy.setHeightForWidth(
        button_a_bot_m4_equip->sizePolicy().hasHeightForWidth());
    button_a_bot_m4_equip->setSizePolicy(sizePolicy);
    button_a_bot_m4_equip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_a_bot_m4_equip, 1, 6, 1, 1);

    label_tool_m3_alive = new QLabel(tool_group_box);
    label_tool_m3_alive->setObjectName(QStringLiteral("label_tool_m3_alive"));
    label_tool_m3_alive->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m3_alive, 1, 1, 1, 1);

    button_b_bot_m4_unequip = new QPushButton(tool_group_box);
    button_b_bot_m4_unequip->setObjectName(
        QStringLiteral("button_b_bot_m4_unequip"));
    button_b_bot_m4_unequip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_b_bot_m4_unequip, 2, 7, 1, 1);

    tool_header_alive = new QLabel(tool_group_box);
    tool_header_alive->setObjectName(QStringLiteral("tool_header_alive"));
    tool_header_alive->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_alive, 0, 1, 1, 1);

    tool_header_loosen = new QLabel(tool_group_box);
    tool_header_loosen->setObjectName(QStringLiteral("tool_header_loosen"));
    tool_header_loosen->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_loosen, 0, 3, 1, 1);

    tool_row_m3 = new QLabel(tool_group_box);
    tool_row_m3->setObjectName(QStringLiteral("tool_row_m3"));
    tool_row_m3->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_row_m3, 1, 0, 1, 1);

    button_b_bot_m4_equip = new QPushButton(tool_group_box);
    button_b_bot_m4_equip->setObjectName(
        QStringLiteral("button_b_bot_m4_equip"));
    sizePolicy.setHeightForWidth(
        button_b_bot_m4_equip->sizePolicy().hasHeightForWidth());
    button_b_bot_m4_equip->setSizePolicy(sizePolicy);
    button_b_bot_m4_equip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_b_bot_m4_equip, 2, 6, 1, 1);

    button_tighten_m4_5s = new QPushButton(tool_group_box);
    button_tighten_m4_5s->setObjectName(QStringLiteral("button_tighten_m4_5s"));
    button_tighten_m4_5s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_tighten_m4_5s, 2, 2, 1, 1);

    button_a_bot_m4_unequip = new QPushButton(tool_group_box);
    button_a_bot_m4_unequip->setObjectName(
        QStringLiteral("button_a_bot_m4_unequip"));
    button_a_bot_m4_unequip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_a_bot_m4_unequip, 1, 7, 1, 1);

    tool_header_pick = new QLabel(tool_group_box);
    tool_header_pick->setObjectName(QStringLiteral("tool_header_pick"));
    tool_header_pick->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_pick, 0, 8, 1, 1);

    tool_header_equip = new QLabel(tool_group_box);
    tool_header_equip->setObjectName(QStringLiteral("tool_header_equip"));
    tool_header_equip->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_equip, 0, 6, 1, 1);

    label_tool_m3_vac = new QLabel(tool_group_box);
    label_tool_m3_vac->setObjectName(QStringLiteral("label_tool_m3_vac"));
    label_tool_m3_vac->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m3_vac, 1, 4, 1, 1);

    tool_header_unequip = new QLabel(tool_group_box);
    tool_header_unequip->setObjectName(QStringLiteral("tool_header_unequip"));
    tool_header_unequip->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_unequip, 0, 7, 1, 1);

    label_tool_m4_vac = new QLabel(tool_group_box);
    label_tool_m4_vac->setObjectName(QStringLiteral("label_tool_m4_vac"));
    label_tool_m4_vac->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m4_vac, 2, 4, 1, 1);

    button_loosen_m3_1s = new QPushButton(tool_group_box);
    button_loosen_m3_1s->setObjectName(QStringLiteral("button_loosen_m3_1s"));
    button_loosen_m3_1s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_loosen_m3_1s, 1, 3, 1, 1);

    button_tighten_m3_5s = new QPushButton(tool_group_box);
    button_tighten_m3_5s->setObjectName(QStringLiteral("button_tighten_m3_5s"));
    button_tighten_m3_5s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_tighten_m3_5s, 1, 2, 1, 1);

    label_tool_m4_alive = new QLabel(tool_group_box);
    label_tool_m4_alive->setObjectName(QStringLiteral("label_tool_m4_alive"));
    label_tool_m4_alive->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m4_alive, 2, 1, 1, 1);

    tool_row_padless_m4 = new QLabel(tool_group_box);
    tool_row_padless_m4->setObjectName(QStringLiteral("tool_row_padless_m4"));
    tool_row_padless_m4->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_row_padless_m4, 3, 0, 1, 1);

    tool_header_tighten = new QLabel(tool_group_box);
    tool_header_tighten->setObjectName(QStringLiteral("tool_header_tighten"));
    tool_header_tighten->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_header_tighten, 0, 2, 1, 1);

    tool_row_m2s = new QLabel(tool_group_box);
    tool_row_m2s->setObjectName(QStringLiteral("tool_row_m2s"));
    tool_row_m2s->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(tool_row_m2s, 4, 0, 1, 1);

    label_tool_padless_m4_alive = new QLabel(tool_group_box);
    label_tool_padless_m4_alive->setObjectName(QStringLiteral("label_tool_padless_m4_alive"));
    label_tool_padless_m4_alive->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_padless_m4_alive, 3, 1, 1, 1);

    label_tool_m2s_alive = new QLabel(tool_group_box);
    label_tool_m2s_alive->setObjectName(QStringLiteral("label_tool_m2s_alive"));
    label_tool_m2s_alive->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m2s_alive, 4, 1, 1, 1);

    button_tighten_padless_m4_5s = new QPushButton(tool_group_box);
    button_tighten_padless_m4_5s->setObjectName(
        QStringLiteral("button_tighten_padless_m4_5s"));
    button_tighten_padless_m4_5s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_tighten_padless_m4_5s, 3, 2, 1, 1);

    button_tighten_m2s_5s = new QPushButton(tool_group_box);
    button_tighten_m2s_5s->setObjectName(
        QStringLiteral("button_tighten_m2s_5s"));
    button_tighten_m2s_5s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_tighten_m2s_5s, 4, 2, 1, 1);

    button_loosen_padless_m4_1s = new QPushButton(tool_group_box);
    button_loosen_padless_m4_1s->setObjectName(QStringLiteral("button_loosen_padless_m4_1s"));
    button_loosen_padless_m4_1s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_loosen_padless_m4_1s, 3, 3, 1, 1);

    button_loosen_m2s_1s = new QPushButton(tool_group_box);
    button_loosen_m2s_1s->setObjectName(QStringLiteral("button_loosen_m2s_1s"));
    button_loosen_m2s_1s->setMinimumSize(QSize(20, 0));

    gridLayout->addWidget(button_loosen_m2s_1s, 4, 3, 1, 1);

    button_a_bot_padless_m4_equip = new QPushButton(tool_group_box);
    button_a_bot_padless_m4_equip->setObjectName(
        QStringLiteral("button_a_bot_padless_m4_equip"));
    sizePolicy.setHeightForWidth(
        button_a_bot_padless_m4_equip->sizePolicy().hasHeightForWidth());
    button_a_bot_padless_m4_equip->setSizePolicy(sizePolicy);
    button_a_bot_padless_m4_equip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_a_bot_padless_m4_equip, 3, 6, 1, 1);

    button_a_bot_m2s_unequip = new QPushButton(tool_group_box);
    button_a_bot_m2s_unequip->setObjectName(
        QStringLiteral("button_a_bot_m2s_unequip"));
    button_a_bot_m2s_unequip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_a_bot_m2s_unequip, 3, 7, 1, 1);

    button_close_b_8 = new QPushButton(tool_group_box);
    button_close_b_8->setObjectName(QStringLiteral("button_close_b_8"));
    button_close_b_8->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_close_b_8, 4, 7, 1, 1);

    button_b_bot_m2s_equip = new QPushButton(tool_group_box);
    button_b_bot_m2s_equip->setObjectName(
        QStringLiteral("button_b_bot_m2s_equip"));
    sizePolicy.setHeightForWidth(
        button_b_bot_m2s_equip->sizePolicy().hasHeightForWidth());
    button_b_bot_m2s_equip->setSizePolicy(sizePolicy);
    button_b_bot_m2s_equip->setMinimumSize(QSize(45, 0));

    gridLayout->addWidget(button_b_bot_m2s_equip, 4, 6, 1, 1);

    label_tool_padless_m4_vac = new QLabel(tool_group_box);
    label_tool_padless_m4_vac->setObjectName(QStringLiteral("label_tool_padless_m4_vac"));
    label_tool_padless_m4_vac->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_padless_m4_vac, 3, 4, 1, 1);

    label_tool_m2s_vac = new QLabel(tool_group_box);
    label_tool_m2s_vac->setObjectName(QStringLiteral("label_tool_m2s_vac"));
    label_tool_m2s_vac->setAlignment(Qt::AlignCenter);

    gridLayout->addWidget(label_tool_m2s_vac, 4, 4, 1, 1);

    horizontalLayout_6->addLayout(gridLayout);

    verticalLayout->addWidget(tool_group_box);

    retranslateUi(O2ACSetupForm);

    QMetaObject::connectSlotsByName(O2ACSetupForm);
  } // setupUi

  void retranslateUi(QWidget *O2ACSetupForm) {
    O2ACSetupForm->setWindowTitle(
        QApplication::translate("O2ACSetupForm", "Form", Q_NULLPTR));
    button_activate_ros_control->setText(QApplication::translate(
        "O2ACSetupForm", "Activate UR ROS control", Q_NULLPTR));
    button_robots_home->setText(
        QApplication::translate("O2ACSetupForm", "Robots go home", Q_NULLPTR));
    a_bot_group->setTitle(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    button_home_a_bot->setText(
        QApplication::translate("O2ACSetupForm", "Home", Q_NULLPTR));
    button_open_gripper_a->setText(
        QApplication::translate("O2ACSetupForm", "Open", Q_NULLPTR));
    button_close_gripper_a->setText(
        QApplication::translate("O2ACSetupForm", "Close", Q_NULLPTR));
    b_bot_group->setTitle(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    button_home_b_bot->setText(
        QApplication::translate("O2ACSetupForm", "Home", Q_NULLPTR));
    button_open_gripper_b->setText(
        QApplication::translate("O2ACSetupForm", "Open", Q_NULLPTR));
    button_close_gripper_b->setText(
        QApplication::translate("O2ACSetupForm", "Close", Q_NULLPTR));
    base_plate_fixation_group->setTitle(
        QApplication::translate("O2ACSetupForm", "Base plate", Q_NULLPTR));
    button_open_base_fixation->setText(
        QApplication::translate("O2ACSetupForm", "Open", Q_NULLPTR));
    button_close_base_fixation->setText(
        QApplication::translate("O2ACSetupForm", "Close", Q_NULLPTR));
    tool_group_box->setTitle(
        QApplication::translate("O2ACSetupForm", "Tools", Q_NULLPTR));
    tool_header_vac->setText(
        QApplication::translate("O2ACSetupForm", "Vac", Q_NULLPTR));
    button_a_bot_m4_pick_screw->setText(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    button_b_bot_m4_pick_screw->setText(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    tool_row_m4->setText(
        QApplication::translate("O2ACSetupForm", "M4", Q_NULLPTR));
    button_loosen_m4_1s->setText(
        QApplication::translate("O2ACSetupForm", "1s", Q_NULLPTR));
    button_a_bot_m4_equip->setText(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    label_tool_m3_alive->setText(
        QApplication::translate("O2ACSetupForm", "\342\235\214", Q_NULLPTR));
    button_b_bot_m4_unequip->setText(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    tool_header_alive->setText(
        QApplication::translate("O2ACSetupForm", "Alive", Q_NULLPTR));
    tool_header_loosen->setText(
        QApplication::translate("O2ACSetupForm", "Unscr.", Q_NULLPTR));
    tool_row_m3->setText(
        QApplication::translate("O2ACSetupForm", "M3", Q_NULLPTR));
    button_b_bot_m4_equip->setText(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    button_tighten_m4_5s->setText(
        QApplication::translate("O2ACSetupForm", "5s", Q_NULLPTR));
    button_a_bot_m4_unequip->setText(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    tool_header_pick->setText(
        QApplication::translate("O2ACSetupForm", "Pick", Q_NULLPTR));
    tool_header_equip->setText(
        QApplication::translate("O2ACSetupForm", "Equip", Q_NULLPTR));
    label_tool_m3_vac->setText(
        QApplication::translate("O2ACSetupForm", "\342\236\226", Q_NULLPTR));
    tool_header_unequip->setText(
        QApplication::translate("O2ACSetupForm", "Uneq.", Q_NULLPTR));
    label_tool_m4_vac->setText(
        QApplication::translate("O2ACSetupForm", "\342\236\226", Q_NULLPTR));
    button_loosen_m3_1s->setText(
        QApplication::translate("O2ACSetupForm", "1s", Q_NULLPTR));
    button_tighten_m3_5s->setText(
        QApplication::translate("O2ACSetupForm", "5s", Q_NULLPTR));
    label_tool_m4_alive->setText(
        QApplication::translate("O2ACSetupForm", "\342\235\214", Q_NULLPTR));
    tool_row_padless_m4->setText(
        QApplication::translate("O2ACSetupForm", "P-M4", Q_NULLPTR));
    tool_header_tighten->setText(
        QApplication::translate("O2ACSetupForm", "Screw", Q_NULLPTR));
    tool_row_m2s->setText(
        QApplication::translate("O2ACSetupForm", "M2s", Q_NULLPTR));
    label_tool_padless_m4_alive->setText(
        QApplication::translate("O2ACSetupForm", "\342\235\214", Q_NULLPTR));
    label_tool_m2s_alive->setText(
        QApplication::translate("O2ACSetupForm", "\342\235\214", Q_NULLPTR));
    button_tighten_padless_m4_5s->setText(
        QApplication::translate("O2ACSetupForm", "5s", Q_NULLPTR));
    button_tighten_m2s_5s->setText(
        QApplication::translate("O2ACSetupForm", "5s", Q_NULLPTR));
    button_loosen_padless_m4_1s->setText(
        QApplication::translate("O2ACSetupForm", "1s", Q_NULLPTR));
    button_loosen_m2s_1s->setText(
        QApplication::translate("O2ACSetupForm", "1s", Q_NULLPTR));
    button_a_bot_padless_m4_equip->setText(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    button_a_bot_m2s_unequip->setText(
        QApplication::translate("O2ACSetupForm", "a_bot", Q_NULLPTR));
    button_close_b_8->setText(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    button_b_bot_m2s_equip->setText(
        QApplication::translate("O2ACSetupForm", "b_bot", Q_NULLPTR));
    label_tool_padless_m4_vac->setText(
        QApplication::translate("O2ACSetupForm", "\342\236\226", Q_NULLPTR));
    label_tool_m2s_vac->setText(
        QApplication::translate("O2ACSetupForm", "\342\236\226", Q_NULLPTR));
  } // retranslateUi
};

namespace Ui {
class O2ACSetupForm : public Ui_O2ACSetupForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // TEST_H
