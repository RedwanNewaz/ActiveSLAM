#include "ros_launch.h"


//https://github.com/karlphillip/GraphicsProgramming/blob/master/cvImage/cvImage.cpp



ros_launch::ros_launch(QObject *parent) :
    QThread(parent)
{

    this->Stop=false;
    navdataCount=0;
    debugger_pub = nh_.advertise<std_msgs::String>("jaistquad/debug", 1);


}


void ros_launch::run()
{
    std::cout<< "topic in string: "  <<image_topic<<std::endl;
    ros_launch *sensor_subs;
    sensor_subs=new ros_launch(this);


    image_sub=nh_.subscribe(image_topic, 1, &ros_launch::callback_Image,sensor_subs);
    navdata_sub	   = nh_.subscribe(nh_.resolveName("ardrone/navdata"),50, &ros_launch::navdataCb, sensor_subs);
    debugger_sub=nh_.subscribe("jaistquad/debug", 1, &ros_launch::debugger_callback,sensor_subs);
    measurement_client=nh_.serviceClient<active_slam::measurement>("measurement");

    connect(sensor_subs,SIGNAL(singal_sensor_sub(QImage)),this,SLOT(slot_ros_launch(QImage)));
    connect(sensor_subs,SIGNAL(nav_battery(double)),this,SLOT(slot_nav_battery(double)));
    connect(sensor_subs,SIGNAL(sig_debugger(QString)),this,SLOT(slot_debugger(QString)));
    qDebug()<<"image subscriber enabled";
    ros::MultiThreadedSpinner();
}




void ros_launch::callback_Image(const sensor_msgs::Image::ConstPtr& msg)
{

    mutex.lock();
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
        cv::Mat conversion_mat_ = cv_ptr->image;
        QImage _img(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
        emit singal_sensor_sub(_img.rgbSwapped());
    mutex.unlock();

    mutex.lock();
        if(Stop)
        {
            qDebug()<<"terminating cv show";
            terminate();
        }
    mutex.unlock();


}

void ros_launch::navdataCb(const ardrone_autonomy::NavdataConstPtr navdataPtr)
{
    navdataCount++;

    if(navdataCount%20==0){
          emit nav_battery(navdataPtr->batteryPercent);
    }


}

void ros_launch::debugger_callback(const std_msgs::StringConstPtr msg){

    QString str = QString::fromUtf8(msg->data.c_str());
    emit sig_debugger(str);

}





//communicate to main script

void ros_launch::slot_nav_battery(double msg)
{
    emit ardrone_battery(msg);
    //send light intenisty information too
    active_slam::measurement attribute;
    double backgroundMeasurement=0;
    attribute.request.state=1;
    if(measurement_client.call(attribute))
        backgroundMeasurement=attribute.response.result;
    emit light_intensity(backgroundMeasurement);


}

void ros_launch::slot_ros_launch(const QImage &_img)
{

    emit ImageQ(_img);
}

void ros_launch::slot_debugger(QString msg){
    emit sig_main_debugger(msg);
}
