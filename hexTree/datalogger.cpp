#include "datalogger.h"
#include "ros/ros.h"
#include <QDebug>

datalogger::datalogger()
{
    path="/home/redwan/Desktop/data/";
    QString filename=path+"log.txt";

    if(QFile(filename).exists()){

        //read log file
        QString proposed=read_log_file(filename);
        //add folder name to path
        path=proposed;

    }
    else{
    //create new log file
        fileName("log");
    //create new folder
        QString proposed="EX_";
        QString folder_name=path+proposed;
        for(int i=1;QDir(folder_name).exists();i++)
            folder_name=path+proposed+QString::number(i);
        QDir().mkdir(folder_name);

    //write folder name to log file
        folder_name+="/";

        string a[1]={folder_name.toUtf8().constData()};
        addHeader(a,1);
        path=folder_name;
    }


}

void datalogger::fileName(QString proposed){

    QString filename=path+proposed;
   if(QFile(filename).exists())
    for(int i=0;QFile(filename).exists();i++)
       filename=filename+QString::number(i)+".txt";
   else
       filename+=".txt";


    name=filename;
    initialization=true;

}

void datalogger::dataWrite(float *a, float size)
{
    if(!initialization)return;
    QFile file(name);
    file.open(QIODevice::Append | QIODevice::Text);

    if(file.isOpen()){
        QTextStream outStream(&file);
        for(int i=0;i<size;i++)
            outStream<<a[i]<<"\t";
        outStream<<"\n";

    }
    file.close();

}

void datalogger::addHeader(string *v,int sizearr){
    if(!initialization)return;
    QFile file(name);
    file.open(QIODevice::Append | QIODevice::Text);

    if(file.isOpen()){
        QTextStream outStream(&file);
        for(int i=0;i<sizearr;i++)
            outStream<<QString::fromUtf8( v[i].c_str())<<"\t";
        outStream<<"\n";

    }
    file.close();

}

void datalogger::changePath(QString new_path){
    path=new_path;

}
QString datalogger::read_log_file(QString log){
    QFile file(log);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    QString data="EX";
    while(!file.isOpen())
        file.close();

        data=file.readLine();
    return data;
}
