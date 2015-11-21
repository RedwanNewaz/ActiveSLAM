#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define BUF_MAX 1024
#define MAX_CPU 128
#define RAD 57.2958
#include <QDebug>
#include "active_slam/pidgain.h"
#include "active_slam/plannertalk.h"
#include "active_slam/obstacle.h"
#include <QMainWindow>
#include <QTimer>
#include "header.h"

class ros_launch;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_buttonMoveit_clicked();

    void on_buttonStart_clicked();

    void on_buttonStop_clicked();

    void on_buttonTakeOff_clicked();

    void on_buttonLand_clicked();

    void on_pushButton_clicked();



    void on_rollKp_sliderMoved(int action);

    void on_rollKd_sliderMoved(int action);

    void on_pitchKp_sliderMoved(int action);

    void on_pitchKd_sliderMoved(int action);

    void on_altKp_sliderMoved(int action);

    void on_altKd_sliderMoved(int action);

    void on_YawKp_sliderMoved(int action);

    void on_YawKd_sliderMoved(int action);

    void timeChanged();
    void sub_write(const QImage &frame);
    void updateBattery(double status);
    void sub_debug(QString);



    void on_pushButton_2_clicked();

    void on_button_motion_clicked();

    void on_btn_test_clicked();

private:
    Ui::MainWindow *ui;
    struct pid_gain{
        double kp,kd;
        bool change;
    }rollGain,pitchGain,altdGain,yawGain;
    QTimer *timer;

    //process start variables
    bool on_fly_status,moveit_enable;
    double battery,displayCount;
    QImage *_image;
    QList<QString> pathFile,action;
    QString notification,raw_notification;
    ros_thread *drone_thread;
    ros_launch *sensor_subs;
    QStringList topicList,executionList;
    QProcess *process,*process_action;
    ros::ServiceClient client,test_obs_clinet;
    ros::ServiceClient plannerclient;
    ros::NodeHandle n;


    //-------------------------

    //cpu usage variabale
        bool cpu_usage_active;
        QVector<double>robot_x, robot_y, map_y;
        FILE *fp;
        unsigned long long int fields[10], total_tick[MAX_CPU], total_tick_old[MAX_CPU], idle[MAX_CPU], idle_old[MAX_CPU], del_total_tick[MAX_CPU], del_idle[MAX_CPU];
        int update_cycle, i, cpus, count;
        double percent_usage;
     //------------------------------

        //subordinate program
    QProcess *process_cntrl,*process_plan, *process_extra;
    QProcess *process_ukf,*process_ardrone, *process_slam;
    bool subprograms;




protected:
    void cpuInit();
    int read_fields (FILE *fp, unsigned long long int *fields);
    void cpuUsages();

    void pdLabel();
    void pdInitializer();


    void processInit();
    void processStarup();
    void notificationDisplay(QString msg);

    void cameraInit();
    void EnableSubscriber();
    void posi_display(QString);

};

#endif // MAINWINDOW_H
