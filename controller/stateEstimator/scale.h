#ifndef scale_H
#define scale_H

#include "../header.h"
#define SAMPLESIZE 100
class scale
{
public:
    scale();
    bool sampling(double slam_x, double slam_y, double nav_vx, double nav_vy, double nav_ax, double nav_ay);
    double voScale();



private:
double mu,lamda,likelihood;
double sigma_x,sigma_y;
double x,y,nav_x,nav_y;
double vo_scale;
double pre_dist_x,pre_dist_y;
std::vector<double> slam_dist, nav_dist;

protected:

    double FindScale(std::vector<double> x, std::vector<double> y);
    double distance(double x, double y);
    double nav2dist(double nav_v, double nav_a);
	
	double MLE(double x, double y, double lamda, double sigma_x, double sigma_y);
	double mu_star(double x, double y, double lamda, double sigma_x, double sigma_y);
    double ss_xy(double x, double y, double sigma_x, double sigma_y);
	double lamda_star(double x, double y, double sigma_x, double sigma_y);






};

#endif // scale_H
