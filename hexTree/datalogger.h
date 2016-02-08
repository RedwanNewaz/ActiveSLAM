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

private:
    QString path;
    QString name;
    bool initialization;
protected:
    QString read_log_file(QString log);

};

#endif // DATALOGGER_H
