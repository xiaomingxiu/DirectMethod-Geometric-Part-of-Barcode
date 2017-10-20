											#define HAVE_OPENCV_FLANN
											#define HAVE_OPENCV_IMGPROC
											#define HAVE_OPENCV_VIDEO
											#define HAVE_OPENCV_OBJDETECT
											#define HAVE_OPENCV_CALIB3D
											#define HAVE_OPENCV_ML
											#define HAVE_OPENCV_HIGHGUI
											#define HAVE_OPENCV_CONTRIB
											#define HAVE_OPENCV_PHOTO
											#define HAVE_OPENCV_FEATURES2D


											#include<opencv2/opencv.hpp>
											#include<opencv2/core/core_c.h>
											#include<opencv2/core/core.hpp>


											#include<iostream>

											# include "opencv2/core/core.hpp"
											# include "opencv2/features2d/features2d.hpp"
											# include "opencv2/highgui/highgui.hpp"
											# include "opencv2/calib3d/calib3d.hpp"
											# include "opencv2/nonfree/features2d.hpp"


											#include "opencv2/opencv_modules.hpp"
#include<math.h>
											using namespace std;
											using namespace cv;
const double pi=3.1415926535898;

Mat getS(Mat& vec, Mat& R, Mat& t)
{
	return R*vec + t;
}


void solve(Mat P1_comma_vec,Mat P2_comma_vec,Mat P3_comma_vec,Mat P1_vec,Mat P2_vec,Mat P3_vec, Mat& t_result, double theta_result)
{
	
	double  k,b;
	double x1,x2,x1_c,x2_c, y1,y2, y1_c,y2_c, x3,y3,x3_c,y3_c;

	x1=P1_vec.at<double>(0,0);
	y1=P1_vec.at<double>(1,0);

	x2=P2_vec.at<double>(0,0);
	y2=P2_vec.at<double>(1,0);

	x1_c=P1_comma_vec.at<double>(0,0);
	y1_c=P1_comma_vec.at<double>(1,0);
	
	x2_c=P2_comma_vec.at<double>(0,0);
	y2_c=P2_comma_vec.at<double>(1,0);
	
	x3=P3_vec.at<double>(0,0);
	y3=P3_vec.at<double>(1,0);
	x3_c=P3_comma_vec.at<double>(0,0);
	y3_c=P3_comma_vec.at<double>(1,0);
	
//======================================	
	k= - (x1 - x2) / (y1 - y2 );

	b= 0.5*(	y1 + y2 - (1/(y1-y2)) * ( x1_c*x1_c + y1_c*y1_c -x2_c*x2_c - y2_c*y2_c - x1 * x1 + x2 * x2 )	 );

	cout<<"k="<<k<<endl;
	cout<<"b="<<b<<endl;

//======================================

double A,B,C;

A= 1+ k*k;
B= -2 * (x1 + k * (y1 -b));
C= x1 * x1 + (y1 -b)*(y1 - b) -x1_c*x1_c - y1_c*y1_c;



double Delta=B*B - 4* A*C;
double root_l= (-B+ sqrt(Delta))/(2*A);
double root_r= (-B- sqrt(Delta))/(2*A);		


cout<<"root_l="<<root_l<<endl;
cout<<"root_r="<<root_r<<endl;


double tx_l=root_l;
double tx_r=root_r;
double ty_l=k* tx_l +b;
double ty_r=k* ty_r +b;

cout<<"tx_l="<<tx_l<<"  ty_l="<<ty_l<<endl;
cout<<"tx_r="<<tx_r<<"  ty_r="<<ty_r<<endl;

double check3_l=0, check3_r=0;

check3_l=  (x3 - tx_l)*(x3 - tx_l) + (y3 -ty_l)*(y3-ty_l) - x3_c*x3_c -y3_c*y3_c;

check3_r=  (x3 - tx_r)*(x3 - tx_r) + (y3 -ty_r)*(y3-ty_r) - x3_c*x3_c -y3_c*y3_c;

cout<<"check3_l="<<check3_l<<endl;
cout<<"check3_r="<<check3_r<<endl;


double tx,ty;
if(check3_l < 0.0000001)
	{
		tx=tx_l;
		ty=ty_l;
	}
	
	else if(check3_r < 0.0000001)
		{
			tx=tx_r;
			ty=ty_r;
		}

	else

		cout<<"check unpassed! vector t is not gotten!"<<endl;



double tantheta= ( (y1-ty)*x1_c  - (x1 - tx)*y1_c  )  / (  (x1-tx)*x1_c  + (y1-ty)*y1_c );

double theta= atan2(( (y1-ty)*x1_c  - (x1 - tx)*y1_c  ) ,  (  (x1-tx)*x1_c  + (y1-ty)*y1_c ))  * 180/pi;


cout<<"theta="<<theta<<endl;



///&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

t_result.at<double>(0,0)=tx;
t_result.at<double>(1,0)=ty;

theta_result=theta;


}












											int main()
											{

double theta=30*pi/180;

Mat R= (Mat_<double>(2,2)<<cos(theta), - sin(theta), sin(theta), cos(theta));

cout<<"R="<<R<<endl;	

Mat t= (Mat_<double>(2,1)<<3,2);

Mat P1_comma_vec= (Mat_<double>(2,1)<<2.5,9.0);

Mat P2_comma_vec=(Mat_<double>(2,1)<<-2.5,9.0);

Mat P3_comma_vec=(Mat_<double>(2,1)<<-2.5,-9.0);

Mat P1_vec= getS(P1_comma_vec, R,t);
Mat P2_vec= getS(P2_comma_vec, R,t);
Mat P3_vec= getS(P3_comma_vec, R,t);


cout<<"P1_comma_vec="<<P1_vec<<endl;
cout<<"P2_comma_vec="<<P2_vec<<endl;
cout<<"P3_comma_vec="<<P3_vec<<endl;

	

//000000000000000000000000000000000000000000000000000000000000000000000000
Mat t_result;
double theta_result;

solve( P1_comma_vec, P2_comma_vec, P3_comma_vec, P1_vec, P2_vec, P3_vec, t_result,  theta_result);





return 1;
											}
