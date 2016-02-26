#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDir>
#include <QFile>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->displayBattery->setPalette(Qt::red);
    ui->intensity->setPalette(Qt::blue);

    ui->robot_x->setPalette(Qt::red);
    ui->robot_y->setPalette(Qt::green);
    ui->robot_z->setPalette(Qt::blue);

    ui->robot_x_3->setPalette(Qt::black);
    ui->robot_y_3->setPalette(Qt::black);
    ui->robot_z_3->setPalette(Qt::black);


    ui->pdDisplay->setPalette(Qt::blue);
    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(timeChanged()));

    notificationDisplay("Welcome to JAIST QUAD gui!");


    //cpu usage initializer
    cpuInit();

    //process initialization
    processInit();

    //cameara initialize
    cameraInit();
    lightIntensity=0;

    //start timer
    timer->start(1000);

    ui->tc->setChecked(true);
    ui->experiment->setChecked(true);


}

MainWindow::~MainWindow()
{
    delete ui;
}

//System Button

void MainWindow::on_buttonMoveit_clicked()
{

    QStringList cntrl, planner, octomap,extra;
    cntrl<<"active_slam"<<"motion";
    planner<<"active_slam"<<"seeking";

    QDir("/home/redwan/Desktop/updateFusion");
    extra<<"mav.launch";

    process_cntrl =new QProcess(this);
    process_cntrl->start("rosrun",cntrl);


    process_plan =new QProcess(this);
    process_plan->start("rosrun",planner);

//    processStarup();
    subprograms=true;

    // initialize pd controller
    pdInitializer();

}

void MainWindow::on_buttonStart_clicked()
{
    raw_notification="Ros thread is started";
/*    moveit_enable=true;
    if (ui->experiment->isChecked())
    {
        action.removeLast();
    }
    if (ui->simulation->isChecked())
    {
        action.removeFirst();
    }

    if (moveit_enable)
    {
        if (!ui->test->isChecked() && action.size()<2)
        {
            qDebug()<<action;
            process_action =new QProcess(this);
            process_action->start("roslaunch",action);

        }
        if (ui->test->isChecked())
        {
            qDebug()<<"Testing mode enable";

            process_action =new QProcess(this);
            process_action->start("rqt_graph");

            raw_notification="/* configured for test\n";
        }
    }
//    notificationDisplay(raw_notification);
*/
    MainWindow::EnableSubscriber();
    sleep(1);


    drone_thread->start();


}

void MainWindow::on_buttonStop_clicked()
{

       qDebug()<<"Application closing request accepted";
       notificationDisplay("Application closing request accepted");
       QFile("/home/redwan/Desktop/data/log.txt").remove();

       if (timer->isActive()){
           timer->stop();
           sleep(1);
           notificationDisplay("timer is stopped");
       }

       if(cpu_usage_active){
           fclose (fp);
           sleep(1);
           MainWindow::notificationDisplay("cpu Usage is closed");
           qDebug()<<"cpu Usage closed";
           cpu_usage_active-false;
       }


       if(subprograms){
           subprograms=false;
           process_cntrl->terminate();
           process_plan->terminate();
           if(moveit_enable)
           process_slam->terminate();
       }

       notificationDisplay(raw_notification);
        moveit_enable=false;
    QProcess::startDetached(QApplication::applicationFilePath());
    exit(1);
}

// UAV MOTION BUTTON

void MainWindow::on_buttonTakeOff_clicked()
{
    if (!on_fly_status){
        notificationDisplay("Jaist Quadrotor is taking Off");
        on_fly_status=true;
        drone_thread->sendTakeoff();
    }
    else
        notificationDisplay("Quad is already in flying mode");
}

void MainWindow::on_buttonLand_clicked()
{
    if(on_fly_status){
        raw_notification="Landing...";
        on_fly_status=false;
         drone_thread->sendLand();
    }
    else
         raw_notification="Quad is already in Landing mode";
      MainWindow::notificationDisplay(raw_notification);
}

//set PID GAIN
void MainWindow::on_pushButton_clicked()
{
    active_slam::pidgain srv;

    if(rollGain.change){
        srv.request.id=1;
        srv.request.P=rollGain.kp;
        srv.request.D=rollGain.kd;
        rollGain.change=false;
    }
    if(pitchGain.change){
        srv.request.id=2;
        srv.request.P=pitchGain.kp;
        srv.request.D=pitchGain.kd;
        pitchGain.change=false;
    }

    if(altdGain.change){
        srv.request.id=3;
        srv.request.P=altdGain.kp;
        srv.request.D=altdGain.kd;
         altdGain.change=false;
    }

    if(yawGain.change){
        srv.request.id=4;
        srv.request.P=yawGain.kp;
        srv.request.D=yawGain.kd;
         yawGain.change=false;
    }

    if (client.call(srv))
        ROS_INFO_STREAM("operation "<<srv.response);
}

// PID TUNER SLIDE BAR
void MainWindow::pdInitializer(){
    QFile file("/home/redwan/Desktop/pidtune.txt");
    if (!file.open(QIODevice::ReadOnly))
           return;
    QStringList list;
    while (!file.atEnd()) {
          list.append(file.readLine());
      }
    file.close();
//    qDebug()<<list;


//    if(list.size()<8)ROS_ERROR("pid tune length errror");
    rollGain.kp=    list[0].toDouble();
    rollGain.kd=    list[1].toDouble();
    pitchGain.kp=   list[2].toDouble();
    pitchGain.kd=   list[3].toDouble();
    altdGain.kp=    list[4].toDouble();
    altdGain.kd=    list[5].toDouble();
    yawGain.kp=     list[6].toDouble();
    yawGain.kd=     list[7].toDouble();

    ui->rollKp->setValue(rollGain.kp);
    ui->pitchKp->setValue(pitchGain.kp);
    ui->altKp->setValue(altdGain.kp);
    ui->YawKp->setValue(yawGain.kp);

    ui->rollKd->setValue(rollGain.kd);
    ui->pitchKd->setValue(pitchGain.kd);
    ui->altKd->setValue(altdGain.kd);
    ui->YawKd->setValue(yawGain.kd);
}

void MainWindow::on_rollKp_sliderMoved(int action)
{
    rollGain.kp=action;
    ui->pdDisplay->display(action);
     rollGain.change=true;
}

void MainWindow::on_rollKd_sliderMoved(int action)
{
    rollGain.kd=action;
       ui->pdDisplay->display(action);
        rollGain.change=true;
}

void MainWindow::on_pitchKp_sliderMoved(int action)
{
    pitchGain.kp=action;
       ui->pdDisplay->display(action);
        pitchGain.change=true;
}

void MainWindow::on_pitchKd_sliderMoved(int action)
{
    pitchGain.kd=action;
       ui->pdDisplay->display(action);
        pitchGain.change=true;
}

void MainWindow::on_altKp_sliderMoved(int action)
{
    altdGain.kp=action;
       ui->pdDisplay->display(action);
           altdGain.change=true;
}

void MainWindow::on_altKd_sliderMoved(int action)
{
      altdGain.kd=action;
         ui->pdDisplay->display(action);
          altdGain.change=true;
}

void MainWindow::on_YawKp_sliderMoved(int action)
{
      yawGain.kp=action;
         ui->pdDisplay->display(action);
          yawGain.change=true;
}

void MainWindow::on_YawKd_sliderMoved(int action)
{
     yawGain.kd=action;
        ui->pdDisplay->display(action);
         yawGain.change=true;
}

void MainWindow::pdLabel(){
    QString note;
    note=" R_kp: "+QString::number(rollGain.kp)+"  R_kd: "+QString::number(rollGain.kd);
    note+="\n p_kp: "+QString::number(pitchGain.kp)+"  p_kd: "+QString::number(pitchGain.kd);
    note+=" \n a_kp: "+QString::number(altdGain.kp)+"  a_kd: "+QString::number(altdGain.kd);
    note+=" \n y_kp: "+QString::number(yawGain.kp)+"  y_kd: "+QString::number(yawGain.kd);

    ui->notify->setText(note);
}



// PROCESSES
void MainWindow::processInit()
{

    //thread intialization
    drone_thread=new ros_thread(this);
    sensor_subs=new ros_launch(this);

    battery=0;
    on_fly_status=false;
    moveit_enable=false;

    //enable debugging
    client=n.serviceClient<active_slam::pidgain>("pid_gains");
     test_obs_clinet=n.serviceClient<active_slam::measurement>("threshold");
     calibration_client=n.serviceClient<active_slam::measurement>("calibration");
    plannerclient=n.serviceClient<active_slam::plannertalk>("motionplan");
    connect(sensor_subs,SIGNAL(sig_main_debugger(QString)),this,SLOT(sub_debug(QString)));

    //enable image visualization
    _image=new QImage;
    connect(sensor_subs,SIGNAL(ImageQ(QImage)),this,SLOT(sub_write(QImage)));

    //    directories of launch files and convert those to processes
        QString path= "/home/redwan/Desktop/jaistQuad_launch/path.cfg";
        QFile file(path);
        if(!file.open(QIODevice::ReadOnly)) {
            notificationDisplay("Error No file at "+path);
        }
        QTextStream in(&file);
        int lineCount=0;
        while(!in.atEnd()) {
            QString line = in.readLine();
            if(lineCount<2)action.append(line);
            else pathFile.append(line);
            lineCount++;
        }
        file.close();
//        qDebug()<<"path size: "<<pathFile.size();
}

void MainWindow::EnableSubscriber()
{
    raw_notification+="\n\t\t/*image subscription is enabled";
    QString en= topicList[ui->topics->currentIndex()];
    std::string img_sub_topic=en.toUtf8().constData();
    sensor_subs->image_topic=img_sub_topic;
    sleep(1);
    MainWindow::notificationDisplay(raw_notification);
    sensor_subs->start();

}

void MainWindow::sub_write(const QImage &frame)
{
    ui->image_view->setPixmap(QPixmap::fromImage(frame).scaled(frame.width(), frame.height(),Qt::KeepAspectRatio));

}

void MainWindow::updateBattery(double status)
{
    ui->displayBattery->display(status);
}
void MainWindow::updateIntensity(double status)
{
     ui->intensity->display(int(status));
     lightIntensity=status;
}
// UTILITIES

void MainWindow::processStarup(){
    if(!moveit_enable)
       {
           moveit_enable=true;
           process_slam =new QProcess;
           qDebug()<<pathFile;
           process_slam->start("roslaunch",pathFile);
           raw_notification="Processes are configured for operation...";
       }
       else
            raw_notification="Did you shutdown the processes?";


       MainWindow::notificationDisplay(raw_notification);
       qDebug()<<"processes are "<<process_slam->state();
}

void MainWindow::cameraInit(){
    connect(sensor_subs,SIGNAL(ardrone_battery(double)),this,SLOT(updateBattery(double)));
    connect(sensor_subs,SIGNAL(light_intensity(double)),this,SLOT(updateIntensity(double)));

    topicList<<"/ORB_SLAM/Frame"<<"/camera/image_raw"<<"/ardrone/image_raw";
     ui->topics->addItems(topicList);
}

void MainWindow::notificationDisplay(QString msg)
{
    displayCount++;
    if(displayCount<2)
        notification+=QString::number(displayCount)+": "+ msg;
    else
        notification+="\n"+QString::number(displayCount)+": "+msg;

    ui->textStatus->setPlainText(notification);
}

void MainWindow::timeChanged(){
    QDateTime display_time;
    //display current time
    ui->runTime->setDateTime(display_time.currentDateTime());
    //show cpu usages
    cpuUsages();
    //show pd values as in the stausbar label
    pdLabel();


}

int MainWindow::read_fields (FILE *fp, unsigned long long int *fields)
{
  int retval;
  char buffer[BUF_MAX];


  if (!fgets (buffer, BUF_MAX, fp))
  { perror ("Error"); }
  /* line starts with c and a string. This is to handle cpu, cpu[0-9]+ */
  retval = sscanf (buffer, "c%*s %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu %Lu",
                            &fields[0],
                            &fields[1],
                            &fields[2],
                            &fields[3],
                            &fields[4],
                            &fields[5],
                            &fields[6],
                            &fields[7],
                            &fields[8],
                            &fields[9]);
  if (retval == 0)
  { return -1; }
  if (retval < 4) /* Atleast 4 fields is to be read */
  {
    fprintf (stderr, "Error reading /proc/stat cpu field\n");
    return 0;
  }
  return 1;
}

void MainWindow::cpuInit(){
    update_cycle = 0;
    cpus = 0;
    fp = fopen ("/proc/stat", "r");
    if (fp == NULL)
     {
       qDebug()<<"Error!!! cpu file can't read";
        cpu_usage_active=false;
     }
    else{
         timer->start(1000);
      cpu_usage_active=true;
     }
    while (MainWindow::read_fields (fp, fields) != -1)
     {
       for (i=0, total_tick[cpus] = 0; i<10; i++)
       { total_tick[cpus] += fields[i]; }
       idle[cpus] = fields[3]; /* idle ticks index */
       cpus++;
     }
}

void MainWindow::cpuUsages(){
    fseek (fp, 0, SEEK_SET);
             fflush (fp);
             for (count = 0; count < cpus; count++)
             {
               total_tick_old[count] = total_tick[count];
               idle_old[count] = idle[count];

               if (!MainWindow::read_fields (fp, fields))
               { return ; }

               for (i=0, total_tick[count] = 0; i<10; i++)
               { total_tick[count] += fields[i]; }
               idle[count] = fields[3];

               del_total_tick[count] = total_tick[count] - total_tick_old[count];
               del_idle[count] = idle[count] - idle_old[count];

               percent_usage = ((del_total_tick[count] - del_idle[count]) / (double) del_total_tick[count]) * 100;
               if (count == 0)
               {
                ui->cpu_usage->setValue(percent_usage);
               }
             }
             update_cycle++;



}

//Debuging
void MainWindow::posi_display(QString msg){
       QStringList pose=msg.split("\t",QString::SkipEmptyParts);

       double x,y,z,rx,ry,rz;
       if(pose.size()<7){
           ROS_ERROR("pose length is short");
       return;
       }
       x=pose[1].toDouble();y=pose[2].toDouble();z=pose[3].toDouble();
       rx=pose[4].toDouble();ry=pose[5].toDouble();rz=pose[6].toDouble();
       ui->robot_x->display(x);
       ui->robot_y->display(y);
       ui->robot_z->display(z);



       ui->robot_x_3->display(rx*RAD);
       ui->robot_y_3->display(ry*RAD);
       ui->robot_z_3->display(rz*RAD);

}

void MainWindow::sub_debug(QString msg){
     if(msg.startsWith("robot"))
         posi_display(msg);
     else
     notificationDisplay(msg);

 }

void MainWindow::on_pushButton_2_clicked()
{
    active_slam::pidgain srv;
    srv.request.id=5;
    if (client.call(srv))
        ROS_INFO_STREAM("Gain save is  "<<srv.response);
}

void MainWindow::on_button_motion_clicked()
{
    active_slam::plannertalk srv;
 //   active_slam::plannertalk srv2;
//    srv.request.id=6;
    if(ui->p2p->isChecked())
        srv.request.option=1;
    else if (ui->sb->isChecked())
        srv.request.option=2;
    else if (ui->ca->isChecked())
        srv.request.option=3;
    else if (ui->tc->isChecked())
        srv.request.option=4;

//    if (client.call(srv))
//        ROS_INFO_STREAM("Motion is selected "<<srv.response);

//    if(ui->p2p->isChecked())
//        srv2.request.option=1;
//    else if(ui->tc->isChecked())
//         srv2.request.option=2;
//    else
//         srv2.request.option=3;

    if (plannerclient.call(srv))
        ROS_INFO_STREAM("plannerMotion is changed "<<srv.response);

}

void MainWindow::on_btn_test_clicked()
{
    int x=lightIntensity-ui->threshold_box->currentIndex()-1;
        active_slam::measurement srv;
        srv.request.state=x;
        if (test_obs_clinet.call(srv))
            ROS_INFO_STREAM(" reponseded  "<<x);
}

void MainWindow::on_btn_calibration_clicked()
{
    active_slam::measurement srv;
    int option =ui->scale_cal->currentIndex();
    switch(option){
        case 0:srv.request.state=50;break;
        case 1:srv.request.state=5;break;
    }

    if (calibration_client.call(srv))
        ROS_INFO_STREAM(" reponseded  ");
}
