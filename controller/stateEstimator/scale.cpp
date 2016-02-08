#include "scale.h"
#include "../header.h"

 scale::scale()
{
		/*
		* x = SLAM scale 
		* y = NAV scale 
		*/
		pre_dist_x=pre_dist_y=vo_scale=0;
		slam_dist.clear();
		nav_dist.clear();
		nav_x=nav_y=0;


}
bool scale::sampling(double slam_x, double slam_y, double nav_vx, double nav_vy, double nav_ax, double nav_ay){
	nav_x+=nav2dist(nav_vx, nav_ax);
	nav_y+=nav2dist(nav_vy, nav_ay);

	double curr_dist_x=distance(slam_x,slam_y);
	double curr_dist_y=distance(nav_x,nav_y);
	if(abs(curr_dist_x-pre_dist_x)||abs(curr_dist_y-pre_dist_y)>0.1){
		slam_dist.push_back(curr_dist_x);
		nav_dist.push_back(curr_dist_y);
		pre_dist_x=curr_dist_x;
		pre_dist_y=curr_dist_y;

        ROS_INFO_STREAM("SCALE SAMPLE SIZE "<< slam_dist.size());

		if(slam_dist.size()>SAMPLESIZE-1){
			vo_scale=FindScale(slam_dist, nav_dist);
			return true;
		}
		else
			return false;

	}
	else 
		return false;

}
double scale::voScale(){
	return vo_scale;
}

double scale::FindScale(std::vector<double> cx, std::vector<double> cy){
// step 1: initialize the parameters
	lamda=1;
	sigma_x=sigma_y=0.3;
     x=y=0;
// step 2: compute scale 
    for (int i = 0; i < cx.size(); ++i)
	{
		// step3 : read temp distance
        double x_z= cx.at(i);
        double y_z=cy.at(i);
		 // step 4: update distance
		 x=x+x_z;
		 y=y+y_z;
		 // step 5: compute maximum likelihood
		 mu=mu_star(x,y,lamda,sigma_x,sigma_y );
		 lamda=lamda_star(x,y,sigma_x,sigma_y);
         likelihood=MLE(x,y,lamda,sigma_x,sigma_y);
		 // step 6:Eliminate noise
		 if (likelihood<0.5)
		 {
 			x=x-x_z;
		 	y=y-y_z;
		 }


	}
    lamda=1/lamda;
    // ROS_INFO_STREAM("scale size "<< lamda);
    return lamda;
}

double scale::distance(double x, double y){
	return sqrt(x*x+y*y);
}
double scale::MLE(double x, double y, double lamda, double sigma_x, double sigma_y){
    double ml1=pow(abs(x-mu*lamda),2)/pow(sigma_x,2);
    double ml2=pow(abs(y-mu),2)/pow(sigma_y,2);
    return 0.5*(ml1+ml2);
}
double scale::mu_star(double x, double y, double lamda, double sigma_x, double sigma_y){
    double mu1=lamda*pow(sigma_y,2)*x+pow(sigma_x,2)*y;
    double mu2=pow(lamda,2)*pow(sigma_y,2)+pow(sigma_x,2);
    return mu1/mu2;
}
double scale::ss_xy(double x, double y, double sigma_x, double sigma_y){
	return sigma_x*sigma_y*x*y;
}
double scale::lamda_star(double x, double y, double sigma_x, double sigma_y){
    double s_xx=ss_xy( x,x,sigma_x,sigma_x );
    double s_yy=ss_xy( y,y,sigma_y,sigma_y );
    double s_xy=ss_xy( x,y,sigma_x,sigma_y );
            
    double lamda1=s_xx-s_yy+(s_xy/fabs(s_xy))*sqrt(pow((s_xx-s_yy),2)+4*pow(s_xy,2));
 	double lamda2=2*sigma_y*s_xy/sigma_x;
 	return lamda1/lamda2;
}

double scale::nav2dist(double nav_v, double nav_a){
	double dtN=0.03;
    return nav_v*dtN+0.5*nav_a*pow(dtN,2);
}

