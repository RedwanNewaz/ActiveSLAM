/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTimeEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout_3;
    QFrame *line;
    QVBoxLayout *verticalLayout_6;
    QGroupBox *groupBox_2;
    QLabel *label_2;
    QVBoxLayout *verticalLayout_7;
    QLabel *image_view;
    QPlainTextEdit *textStatus;
    QHBoxLayout *horizontalLayout;
    QLabel *label_3;
    QComboBox *topics;
    QProgressBar *cpu_usage;
    QFrame *line_3;
    QTimeEdit *runTime;
    QVBoxLayout *verticalLayout_2;
    QPushButton *buttonMoveit;
    QPushButton *buttonStart;
    QPushButton *buttonStop;
    QSpacerItem *verticalSpacer_10;
    QPushButton *buttonTakeOff;
    QPushButton *buttonLand;
    QPushButton *buttonToggle;
    QPushButton *buttonGraph;
    QGroupBox *action;
    QVBoxLayout *verticalLayout_5;
    QRadioButton *simulation;
    QRadioButton *experiment;
    QRadioButton *test;
    QSpacerItem *verticalSpacer_2;
    QLabel *label;
    QLCDNumber *displayBattery;
    QGroupBox *groupBox;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_2;
    QSlider *pitchKp;
    QSlider *pitchKd;
    QWidget *layoutWidget_2;
    QGridLayout *gridLayout_4;
    QSlider *altKp;
    QSlider *altKd;
    QWidget *layoutWidget_3;
    QGridLayout *gridLayout_5;
    QSlider *YawKp;
    QSlider *YawKd;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QPushButton *pushButton;
    QLCDNumber *pdDisplay;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout;
    QSlider *rollKp;
    QSlider *rollKd;
    QLabel *notify;
    QMenuBar *menuBar;
    QMenu *menuJAIST_QUAD;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1001, 898);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        gridLayout_3 = new QGridLayout(centralWidget);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout_3->addWidget(line, 1, 4, 1, 1);

        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));

        gridLayout_3->addLayout(verticalLayout_6, 0, 0, 5, 1);

        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));

        gridLayout_3->addWidget(groupBox_2, 2, 4, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout_3->addWidget(label_2, 4, 1, 1, 1);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        image_view = new QLabel(centralWidget);
        image_view->setObjectName(QStringLiteral("image_view"));

        verticalLayout_7->addWidget(image_view);

        textStatus = new QPlainTextEdit(centralWidget);
        textStatus->setObjectName(QStringLiteral("textStatus"));
        textStatus->setMinimumSize(QSize(0, 300));

        verticalLayout_7->addWidget(textStatus);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout->addWidget(label_3);

        topics = new QComboBox(centralWidget);
        topics->setObjectName(QStringLiteral("topics"));

        horizontalLayout->addWidget(topics);


        verticalLayout_7->addLayout(horizontalLayout);

        cpu_usage = new QProgressBar(centralWidget);
        cpu_usage->setObjectName(QStringLiteral("cpu_usage"));
        cpu_usage->setValue(24);

        verticalLayout_7->addWidget(cpu_usage);


        gridLayout_3->addLayout(verticalLayout_7, 0, 2, 5, 1);

        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QStringLiteral("line_3"));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);

        gridLayout_3->addWidget(line_3, 3, 4, 1, 1);

        runTime = new QTimeEdit(centralWidget);
        runTime->setObjectName(QStringLiteral("runTime"));
        runTime->setLayoutDirection(Qt::LeftToRight);
        runTime->setAutoFillBackground(false);
        runTime->setFrame(true);

        gridLayout_3->addWidget(runTime, 2, 1, 1, 1);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        buttonMoveit = new QPushButton(centralWidget);
        buttonMoveit->setObjectName(QStringLiteral("buttonMoveit"));

        verticalLayout_2->addWidget(buttonMoveit);

        buttonStart = new QPushButton(centralWidget);
        buttonStart->setObjectName(QStringLiteral("buttonStart"));

        verticalLayout_2->addWidget(buttonStart);

        buttonStop = new QPushButton(centralWidget);
        buttonStop->setObjectName(QStringLiteral("buttonStop"));

        verticalLayout_2->addWidget(buttonStop);

        verticalSpacer_10 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_10);

        buttonTakeOff = new QPushButton(centralWidget);
        buttonTakeOff->setObjectName(QStringLiteral("buttonTakeOff"));

        verticalLayout_2->addWidget(buttonTakeOff);

        buttonLand = new QPushButton(centralWidget);
        buttonLand->setObjectName(QStringLiteral("buttonLand"));

        verticalLayout_2->addWidget(buttonLand);

        buttonToggle = new QPushButton(centralWidget);
        buttonToggle->setObjectName(QStringLiteral("buttonToggle"));

        verticalLayout_2->addWidget(buttonToggle);

        buttonGraph = new QPushButton(centralWidget);
        buttonGraph->setObjectName(QStringLiteral("buttonGraph"));

        verticalLayout_2->addWidget(buttonGraph);

        action = new QGroupBox(centralWidget);
        action->setObjectName(QStringLiteral("action"));
        verticalLayout_5 = new QVBoxLayout(action);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        simulation = new QRadioButton(action);
        simulation->setObjectName(QStringLiteral("simulation"));

        verticalLayout_5->addWidget(simulation);

        experiment = new QRadioButton(action);
        experiment->setObjectName(QStringLiteral("experiment"));

        verticalLayout_5->addWidget(experiment);

        test = new QRadioButton(action);
        test->setObjectName(QStringLiteral("test"));

        verticalLayout_5->addWidget(test);


        verticalLayout_2->addWidget(action);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer_2);

        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_2->addWidget(label);

        displayBattery = new QLCDNumber(centralWidget);
        displayBattery->setObjectName(QStringLiteral("displayBattery"));
        displayBattery->setSmallDecimalPoint(true);

        verticalLayout_2->addWidget(displayBattery);


        gridLayout_3->addLayout(verticalLayout_2, 0, 1, 1, 1);

        groupBox = new QGroupBox(centralWidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setMinimumSize(QSize(240, 0));
        layoutWidget = new QWidget(groupBox);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 160, 188, 31));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        pitchKp = new QSlider(layoutWidget);
        pitchKp->setObjectName(QStringLiteral("pitchKp"));
        pitchKp->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(pitchKp, 0, 0, 1, 1);

        pitchKd = new QSlider(layoutWidget);
        pitchKd->setObjectName(QStringLiteral("pitchKd"));
        pitchKd->setOrientation(Qt::Horizontal);

        gridLayout_2->addWidget(pitchKd, 0, 1, 1, 1);

        layoutWidget_2 = new QWidget(groupBox);
        layoutWidget_2->setObjectName(QStringLiteral("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(10, 240, 188, 31));
        gridLayout_4 = new QGridLayout(layoutWidget_2);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        altKp = new QSlider(layoutWidget_2);
        altKp->setObjectName(QStringLiteral("altKp"));
        altKp->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(altKp, 0, 0, 1, 1);

        altKd = new QSlider(layoutWidget_2);
        altKd->setObjectName(QStringLiteral("altKd"));
        altKd->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(altKd, 0, 1, 1, 1);

        layoutWidget_3 = new QWidget(groupBox);
        layoutWidget_3->setObjectName(QStringLiteral("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(10, 330, 188, 31));
        gridLayout_5 = new QGridLayout(layoutWidget_3);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        YawKp = new QSlider(layoutWidget_3);
        YawKp->setObjectName(QStringLiteral("YawKp"));
        YawKp->setOrientation(Qt::Horizontal);

        gridLayout_5->addWidget(YawKp, 0, 0, 1, 1);

        YawKd = new QSlider(layoutWidget_3);
        YawKd->setObjectName(QStringLiteral("YawKd"));
        YawKd->setOrientation(Qt::Horizontal);

        gridLayout_5->addWidget(YawKd, 0, 1, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(10, 60, 101, 17));
        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 140, 101, 17));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 220, 101, 17));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 310, 101, 17));
        pushButton = new QPushButton(groupBox);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(100, 380, 99, 27));
        pdDisplay = new QLCDNumber(groupBox);
        pdDisplay->setObjectName(QStringLiteral("pdDisplay"));
        pdDisplay->setGeometry(QRect(100, 20, 91, 41));
        pdDisplay->setSmallDecimalPoint(true);
        layoutWidget1 = new QWidget(groupBox);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(10, 80, 188, 31));
        gridLayout = new QGridLayout(layoutWidget1);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        rollKp = new QSlider(layoutWidget1);
        rollKp->setObjectName(QStringLiteral("rollKp"));
        rollKp->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rollKp, 0, 0, 1, 1);

        rollKd = new QSlider(layoutWidget1);
        rollKd->setObjectName(QStringLiteral("rollKd"));
        rollKd->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(rollKd, 0, 1, 1, 1);

        notify = new QLabel(groupBox);
        notify->setObjectName(QStringLiteral("notify"));
        notify->setGeometry(QRect(10, 420, 231, 371));
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget_2->raise();
        layoutWidget_3->raise();
        label_4->raise();
        label_5->raise();
        label_6->raise();
        label_7->raise();
        pushButton->raise();
        pdDisplay->raise();
        notify->raise();

        gridLayout_3->addWidget(groupBox, 0, 4, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1001, 25));
        menuJAIST_QUAD = new QMenu(menuBar);
        menuJAIST_QUAD->setObjectName(QStringLiteral("menuJAIST_QUAD"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::LeftToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menuJAIST_QUAD->menuAction());

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        groupBox_2->setTitle(QString());
        label_2->setText(QApplication::translate("MainWindow", "CPU Usage", 0));
        image_view->setText(QApplication::translate("MainWindow", "ImageLabel", 0));
        label_3->setText(QApplication::translate("MainWindow", "Camera frame ", 0));
        buttonMoveit->setText(QApplication::translate("MainWindow", "Initialize", 0));
        buttonStart->setText(QApplication::translate("MainWindow", "Start", 0));
        buttonStop->setText(QApplication::translate("MainWindow", "Stop", 0));
        buttonTakeOff->setText(QApplication::translate("MainWindow", "TakeOff", 0));
        buttonLand->setText(QApplication::translate("MainWindow", "Land", 0));
        buttonToggle->setText(QApplication::translate("MainWindow", "ToggleCam", 0));
        buttonGraph->setText(QApplication::translate("MainWindow", "Graph", 0));
        action->setTitle(QApplication::translate("MainWindow", "     Action", 0));
        simulation->setText(QApplication::translate("MainWindow", "Simulation", 0));
        experiment->setText(QApplication::translate("MainWindow", "Experiment", 0));
        test->setText(QApplication::translate("MainWindow", "Test", 0));
        label->setText(QApplication::translate("MainWindow", "Battery Level", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "PD TUNER", 0));
        label_4->setText(QApplication::translate("MainWindow", "Roll (kp, kd  )", 0));
        label_5->setText(QApplication::translate("MainWindow", "Pitch (kp, kd  )", 0));
        label_6->setText(QApplication::translate("MainWindow", "Altd (kp, kd  )", 0));
        label_7->setText(QApplication::translate("MainWindow", "Yaw(kp, kd  )", 0));
        pushButton->setText(QApplication::translate("MainWindow", "Set Tune", 0));
        notify->setText(QApplication::translate("MainWindow", "Robot State", 0));
        menuJAIST_QUAD->setTitle(QApplication::translate("MainWindow", "JAIST_QUAD", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
