#include "LineDetector.h"

#define _USE_MATH_DEFINES

#include <iostream>
#include <string>
#include <vector>
#include <string>
#include <utility>
#include <sstream>
#include <cmath>

#include <ros/ros.h>
#include <ros/package.h>

void MyFilledCircle(Mat img, Point center);
void MyFilledCircle_stop(Mat img, Point center);

Line::Line () {

}

Line::Line(Point _p1, Point _p2) {
    p1 = _p1;
    p2 = _p2;
}

Line &Line::operator= (const Line &other) {
    if(this != &other) { // no-op on self-move-assignment (delete[]/size=0 also ok)
        p1 = other.p1;
        p2 = other.p2;
    }
    return *this;
}

void LineDetector::setDirection(int dir) {
    if(_dir == dir)
        return;
    _dir = dir;
	switch(dir) {
	case -1:
//		sorterCenter = window * 0.25;
		break;
	case 1:
//		sorterCenter = window * 0.75;
		break;
    case 0:
    default:
//        sorterCenter = window / 2;
        break;
	}
}

void LineDetector::updateRoiColStart(Mat &src) {
	switch(_dir) {
		case -1:
			roiColStart = 0;
			break;
		case 1:
			roiColStart = src.cols - window;
			break;
		case 0:
		default:
			roiColStart = (src.cols - window)/2;
			break;
	}
}

//bool LineDetector::sortByDistanceFromCenter::operator()(const Point &a, const Point &b)
//{
//	return abs(a.x - m_info->sorterCenter) < abs(b.x - m_info->sorterCenter);
//}

void LineDetector::_offsetImage(Mat &image, const cv::Scalar & bordercolour, int xoffset, int yoffset) {
	Mat temp(image.rows,image.cols,image.type(),bordercolour);
	Mat roi(temp(cvRect(std::max(0, xoffset),std::max(0, yoffset),
			image.cols-std::max(0, xoffset),image.rows-std::max(0, yoffset))));
	Mat roi2(image(cvRect(std::max(0, -xoffset),std::max(0, -yoffset),
			image.cols-std::max(0, xoffset),image.rows-std::max(0, yoffset))));
	roi2.copyTo(roi);
	image=temp.clone();
}

int LineDetector::buildLine(Mat roi, int v, vector<Line> & lines, vector<Point> & pts, vector<vector<int>> & g, vector<int> & h, int depth) {
//	Mat dd = roiSrc.clone();
	Point start = pts[v];

	//Starting and ending angles of the circle sector
	int s, e;
	//Boolean variable to show that there is a sector now
	bool going = false;
	//Sorting sectors vars
	int nearestPointXDelta = roi.cols;
	int nearestPointX = start.x;

	//The sum of the n_seg
	int sum = 0;
	//Scan angles from 20 to 160 Euler's degrees
	for(int i = 20; i <= 161; i++) {
		//Trigonometry to build the points
		int x = round(cos(i*M_PI/180)*radius) + start.x;
		int y = -round(sin(i*M_PI/180)*radius) + start.y;
		//Check the bounding
		if(x >= 0 && x < roi.cols && y >= 0 && y < roi.rows) {
			//Thresholded color check
			if((int)roi.at<unsigned char>(Point(x, y)) > 0 && i != 161) {
//				MyFilledCircle_stop(dd, Point(x,y));
				//Handle the start of the sector
				if(!going) {
					going = true;
					s = i;
				}

			} else if(going) { //Handle the end of the sector
				e = i;
				going = false;

				//Check for minimum angle to process the sector
				if (e - s >= minAng) {
					//End point is calculated as the middle of the sector
					int segAng = (e+s)/2;
					Point endp = Point(round(cos((segAng)*M_PI/180)*radius) + start.x, -round(sin((segAng)*M_PI/180)*radius) + start.y);
//					if(endp.y > roi.rows)
//						cout << endp.y << " " << roi.rows << endl;
					//Check for that the whole line placed on a black color

					if(checkForFillness(roi, Line(start, endp))) {
						lines.emplace_back(start, endp);
						pts.push_back(endp);
						int to = pts.size() - 1;
						g[v].push_back(to);
						g.emplace_back();
						h.push_back(1);

						//recursive call
						int dev_x = buildLine(roi, to, lines, pts, g, h);

						//Finding nearestPointXDelta, nearestPointX
						if(abs((dev_x + start.x)/2 - (sorterCenter + roiColStart)) < nearestPointXDelta) {
							nearestPointXDelta = abs((dev_x + start.x)/2 - (sorterCenter + roiColStart));
							nearestPointX = (dev_x + start.x)/2;
						}
					}
				}
			}
		}
	}

	for(int i = 0; i < g[v].size(); ++i)
		h[v] = std::max(h[v], h[g[v][i]]+1);

	return nearestPointX;
}

bool LineDetector::isFork(vector<vector<int>> &g, vector<int> &h) {
	for(int i = 0; i < g.size(); ++i)
		if(g[i].size() == 2 && h[g[i][0]] > 1 && h[g[i][1]] > 1)
			return true;
	return false;
//    return !g.empty() && g[0].size() > 1;
}

bool LineDetector::checkForFillness(Mat &roi, Line line) {
	const int n_check = 10;
	bool good = true;
	for(int j = 1; j <= n_check; j++) {
		Point curp = (line.p2 - line.p1) * j / (n_check + 1) + line.p1;
//						MyFilledCircle(dd, curp);
		if((int)roi.at<unsigned char>(curp) == 0)
		{
			good = false;
			break;
		}
	}

	return good;
}

LineDetectorInfo LineDetector::detectLine(Mat & src, Mat & drawing) {
	_offsetImage(src, cv::Scalar(255,255,255), imgOffset, 0);
	_offsetImage(drawing, cv::Scalar(255,255,255), imgOffset, 0);

	updateRoiColStart(src);
	//ROI where the root is found
	Rect roiRect(roiColStart, roi_row_start, window, roi_height);
	//ROI where the tree is found
	Rect bigRoiRect(0, src.rows - bigRoiHeight, src.cols, bigRoiHeight);

	Mat bigRoi = src(bigRoiRect);

	Mat thrMat;
	preprocessImage(bigRoi, thrMat);
//	imshow("thr", thrMat);

    int deviation2 = superDetection(src, thrMat);

	//extracting roi from the result
	Mat roi = thrMat(Rect(roiRect.x, roiRect.y - src.rows + bigRoiHeight, roiRect.width, roiRect.height));

	//an array where potential roots are
	vector<Point> centers;
	bool stopline = false;
	findRoots(roi, centers, stopline);

	vector<Line> lines;
	vector<Point> pts;
	vector<vector<int>> g;
	vector<int> h;
	bool fork = false;
	bool lost = true;
	//Median X of the one of branches of the tree
//	int medianX = sorterCenter + (bigRoi.cols - window)/2;
	int medianX = bigRoi.cols/2;
	if(!centers.empty()) {
		circle(drawing, Point(roiColStart + sorterCenter, roi_row_start+roi_height/2), 5, Scalar(0,0,255), 4, 8, 0);
		//showing the medianX

//		medianX = (src.cols - window) / 2 + centers[0].x;
		//building function
		int medianXb = bigRoi.cols;
		Point center;
		for(int i = 0; i < (int)centers.size(); i++) {
//			if((centers[i].x < window/3 && _dir > 0) || (centers[i].x > window*2/3cmak && _dir < 0))
//				continue;
			vector<Line> curLines;
			vector<Point> curPts;
			vector<vector<int>> curG;
			vector<int> curH;
			Point startPnt = Point(roiColStart + centers[i].x, centers[i].y - src.rows + bigRoiHeight);
			curPts.push_back(startPnt);
			curG.emplace_back();
			curH.push_back(1);

			int curMedianXb = buildLine(thrMat, 0, curLines, curPts, curG, curH);
//			curLines.clear();
//            if(curLines.empty())
//                continue;

			//sorting the potential centers and line averages by the distance from a sorting center which is set by LineDetector::setDirection(int dir = 0) function
			if(abs(curMedianXb - (sorterCenter + roiColStart)) < abs(medianXb - (sorterCenter + roiColStart)) ) {
				center = centers[i];
				medianXb = curMedianXb;
//				cout << curLines.size() << endl;
				lines = curLines;
				pts = curPts;
				g = curG;
				h = curH;
			}
		}
        lost = false;
//        lost = lines.empty();
		if(!lost) {
            circle(drawing, Point(roiColStart + center.x, roi_row_start + roi_height / 2), 5,
                   Scalar(0, 255, 0), 4, 8, 0);
//            medianX = (medianX+medianXb)/2;
            medianX = medianXb;

        }

//		cout << lines.size() << endl;
	}

	if(isFork(g, h)) {
		fork = true;
		cout << "FORK!" << endl;
	}
	const float filterCoef = 0.1f;
//	if(!lost)
//	    sorterCenter = (int)round(sorterCenter*filterCoef + (1-filterCoef)*(medianX - (bigRoi.cols - window)/2.0f));
	//forming a result
	LineDetectorInfo info;
	info.lines = lines;
	//calculating a deviation in pixels
	double pixelDeviation = medianX - bigRoi.cols/2;

	//a simple filter that reduces noise
//	pixelDeviation = old_res = 0.6*old_res + pixelDeviation*0.4;
	old_res = pixelDeviation;
	circle(drawing, Point(pixelDeviation + bigRoi.cols / 2, src.rows - bigRoiHeight + 20), 5, Scalar(0,0,255), 4, 8, 0);
	info.deviation = pixelDeviation;
	info.lines = lines;
	//empty centers array means that we lost the line
	info.lost = lost;
	info.fork = fork;
	info.stopline = stopline;
	info.deviation2 = deviation2;

	drawResult(info, drawing);

	if(_videowriting)
		writeFrame(drawing);

	return info;
}

void LineDetector::preprocessImage(Mat &src, Mat &dst) {
	//Morphology
	Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
										Size(2 * erosion_size + 1, 2 * erosion_size + 1),Point(erosion_size, erosion_size) );

	//Morphology and image color thresholding
	Mat greyMat, bluredMat, threshMat, erodedMat, dilatedMat;

	cvtColor(src, greyMat, COLOR_BGR2GRAY);
	GaussianBlur( greyMat, bluredMat, Size( 9, 9 ), 0, 0 );
	//inRange(bluredMat,Scalar(200,200,200),Scalar(255,255,255),threshMat);
	threshold(bluredMat, threshMat, 0, 100, THRESH_BINARY_INV|THRESH_OTSU);
	erode(threshMat,erodedMat,element);
	dilate(erodedMat,dilatedMat,element);
	dilate(erodedMat,dilatedMat,element);
	erode(dilatedMat,erodedMat,element);

	erodedMat.copyTo(dst);
}

void LineDetector::findRoots(Mat &src, vector<Point> &centers, bool &stopLine) {
	//finding contours
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	vector<Rect> bounds;
	findContours(src, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

	for( int j = 0; j < (int)contours.size(); j++ ) {
		//Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		Rect bound = boundingRect(contours[j]);
		//Filtering contours by the position and size and finding the stop line
		if(roi_height - bound.height <= 15 && bound.x >= 3 && bound.x + bound.width <= window - 3 && bound.width >= window / 20) {
			if(bound.width < window / 3) {
				bounds.push_back(bound);
				centers.push_back(Point(bound.x + bound.width/2, bound.y+roi_row_start+roi_height/2 + bound.height / 2));
			}
			else if(bound.x > window / 3) {
				stopLine = true;
			}
		}
		//drawContours( drawing, contours, j, Scalar(122), 2, 8, hierarchy, 0, Point() );
	}
//
//    bool going = false;
//    int s, e;
//    for(int i = 0; i < src.cols; ++i) {
//        Point curp (i, src.rows/2);
//        if((int)src.at<unsigned char>(curp) > 0 && i != src.cols-1) {
//            if(!going) {
//                going = true;
//                s = i;
//            }
//        } else if (going) {
//            going = false;
//            e = i;
//
//            int width = s - e;
//            if(width < window / 3) {
//                centers.push_back(Point((s+e)/2, src.rows/2+roi_row_start));
//			}
//			else if(window > window / 3) {
////			    stopLine = true;
//			}
//        }
//    }

}

void LineDetector::drawResult(LineDetectorInfo &info, Mat &drawing) {
	//ROI where the root is found
	Rect roiRect(roiColStart, roi_row_start, window, roi_height);
	//ROI where the tree is found
	Rect bigRoiRect(0, drawing.rows - bigRoiHeight, drawing.cols, bigRoiHeight);

	//Showing where ROIs are placed on the image
	rectangle(drawing, bigRoiRect, Scalar(0,255,0));
	rectangle(drawing, roiRect, Scalar(0,255,0));

	//displaying the tree
	for(int i = 0; i < (int)info.lines.size(); i++) {
		line(drawing, Point(info.lines[i].p1.x, info.lines[i].p1.y + drawing.rows - bigRoiHeight), Point(info.lines[i].p2.x, info.lines[i].p2.y + drawing.rows - bigRoiHeight), Scalar(0,0,255), 2);
	}

	if(info.stopline)
		rectangle(drawing, roiRect, Scalar(0,0,255), -1);

	if(info.fork)
		rectangle(drawing, roiRect, Scalar(255,0,0), -1);
}

//Some predefined drawing functions
void MyFilledCircle(Mat img, Point center) {
	circle(img,
	center,
	1,
	Scalar(0, 255, 0), //цвет круга
	FILLED,
	LINE_8);
}

void MyFilledCircle_stop(Mat img, Point center) {
	circle(img,
	center,
	1,
	Scalar(0, 0, 255),
	FILLED,
	LINE_8);
}

void LineDetector::initVideoWriting() {
	std::string path = ros::package::getPath("aist");

	outCap.open(path + "/assets/output.mpeg",
				CV_FOURCC('P','I','M','1'),
				20, cv::Size(640,480));

	if(!outCap.isOpened()) {
		cout << "Recording video to file doesn't work" << endl;
		cout << "Video writer init fail\n\n";
	}
}

void LineDetector::writeFrame(Mat &frame) {
	outCap.write(frame);
}

void LineDetector::stopVideoWriting() {
	outCap.release();
}

LineDetector::~LineDetector() {
	stopVideoWriting();
}


int LineDetector::superDetection(Mat src, Mat thrMat) {
    int sum = 0;
    int cnt = 0;
    for(int T = 0; T < superRoiRows; ++T) {
        Rect roiRect(roiColStart, roi_row_start - roi_height*T, window, roi_height);
        Mat roi = thrMat(Rect(roiRect.x, roiRect.y - src.rows + bigRoiHeight, roiRect.width, roiRect.height));

        vector<Point> centers;
        bool stopline = false;
        findRoots(roi, centers, stopline);

        if (!centers.empty()) {
            for (int i = 0; i < (int) centers.size(); i++) {
			    cnt++;
			    sum += roiColStart + centers[i].x;
            }
        }
    }

    return ((cnt > 0) ? (sum / cnt) : 0);
}