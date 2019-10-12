
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/legacy/compat.hpp>
#include <iostream>

#include <math.h>
#include <time.h>
#include <vector>

#include "BlobDetector.h"

using namespace std;
using namespace cv;


//#define _debug_view 1
bool BlobDetect(Mat GraySrc, int adaptiveThred, BlobRect &blob_rect)
{
	CV_Assert(GraySrc.channels() == 1);
	Mat  BinIMG;
	int i, j;
	{
		/*for(j=0;j<GraySrc.rows; j++){
			for(i=0; i<GraySrc.cols; i++){
				uchar gray = GraySrc.at<uchar>(i, j);
				hist[gray]++;
			}
		}*/
		int histSize[1];
		int channels[1];
		const float *ranges[1];
		float hranges[2];
		Mat hist;
		histSize[0] = 256;
		hranges[0] = 0.0;
		hranges[1] = 256.0;
		ranges[0] = hranges;
		channels[0] = 0;

		cv::calcHist(&GraySrc, 1, channels, cv::Mat(), hist,1, histSize,  ranges);

		int cout =0, ptthred = (int)(GraySrc.rows*GraySrc.cols*0.1);
		for(i=255; i>=0; i--){
			cout+=hist.at<float>(i);
			if(cout>ptthred)
				break;
		}
	}

	threshold(GraySrc, BinIMG, i/*adaptiveThred*/, 255, CV_THRESH_BINARY/*CV_THRESH_OTSU*/);
//	threshold( GraySrc, BinIMG, 0, 255, CV_THRESH_BINARY| CV_THRESH_OTSU);
	//threshold( GraySrc, BinIMG, 0, 255, CV_THRESH_OTSU);
	//adaptiveThreshold(GraySrc, BinIMG, 255, CV_ADAPTIVE_THRESH_MEAN_C,CV_THRESH_BINARY_INV,25,0);

#ifdef _debug_view
	Mat postIMG;
	resize(BinIMG, postIMG, Size(640,480), INTER_CUBIC);
	imshow("postIMG", postIMG);
	waitKey(1);
#endif

	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;

//	findContours( BinIMG, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	findContours(BinIMG, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	if( contours.size() == 0 ){
		blob_rect.bvalid = false;	
		return false;
	}

	// iterate through all the top-level contours,
	// draw each connected component with its own random color
	int idx = 0, largestComp = 0;
	double maxArea = 0;
	int maxCount = 0;

	 for( idx= 0; idx< contours.size(); idx++ )
	 {
		 size_t count = contours[idx].size();
		 const vector<Point>& c = contours[idx];
		 double area = fabs(contourArea(Mat(c)));
		 //if( area < 100 || area > (BinIMG.rows*BinIMG.cols>>2))
		 if( area < 100 || area > (BinIMG.rows*BinIMG.cols))
			 continue;

		 if( area > maxArea )
		 {
			 maxArea = area;
			 largestComp = idx;
			 maxCount = count;
		 }
	 }
	 if(maxCount < 6){
		 blob_rect.bvalid = false;
		 return false;
	 }

	 Mat pointsf;
	 Mat(contours[largestComp]).convertTo(pointsf, CV_32F);
	 RotatedRect box = fitEllipse(pointsf);

	 Size wholesize;
	 Point pt;
	 GraySrc.locateROI( wholesize, pt );

	 blob_rect.bvalid = true;
	 blob_rect.center.x = pt.x+box.center.x;
	 blob_rect.center.y = pt.y+box.center.y;
	 blob_rect.size = box.size;
	 blob_rect.angle = box.angle;

#if _debug_view
	 Mat RGBSrc;
	 cvtColor(GraySrc, RGBSrc, CV_GRAY2BGR);
	 drawContours(RGBSrc, contours, largestComp, Scalar(255,0,0), 2, 8);
	 ellipse(RGBSrc, box, Scalar(0,0,255), 4, CV_AA);
//	 ellipse(RGBSrc, box.center, box.size*0.5f, box.angle, 0, 360, Scalar(0,255,0), 4, CV_AA);

	 Point pt1,pt2,center;
	 center.x = box.center.x;
	 center.y = box.center.y;
	 pt1.x=center.x-1;pt1.y=center.y; 
	 pt2.x=center.x+1;pt2.y=center.y; 
	 line(RGBSrc, pt1, pt2, CV_RGB(255,0,0), 2, CV_AA, 0 ); 
	 pt1.x=center.x;pt1.y=center.y-1; 
	 pt2.x=center.x;pt2.y=center.y+1; 
	 line(RGBSrc, pt1, pt2, CV_RGB(255,0,0), 2, CV_AA, 0 );
	
	 Mat scaleBlob;
 	 resize(RGBSrc, scaleBlob, Size(640,480), INTER_CUBIC);
	 imshow("BlobDetect",scaleBlob);
	 waitKey(1);
#endif

	 return true;
}
