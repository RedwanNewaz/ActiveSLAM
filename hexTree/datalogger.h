#ifndef DATALOGGER_H
#define DATALOGGER_H
#include <iostream>
#include <QtCore>
#include <QString>
#include <QFile>
using namespace std;
class datalogger
{
public:
    datalogger();
    void fileName(QString proposed);
    void dataWrite(float *a, float size);
    void addHeader(string *v,int sizearr);
    void changePath(QString);
    int read_traj(float* traj_x, float* traj_y);

private:
    QString path;
    QString name;
    bool initialization;
    vector<double> traj_x;
    vector<double>traj_y;
protected:
    QString read_log_file(QString log);
    void read_traj_file_protect(QString log);

};

#endif // DATALOGGER_H
