#ifndef LINE_NODE_LINEDETECTOR_H
#define LINE_NODE_LINEDETECTOR_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace std;
using namespace cv;

class Line
{
public:
    Point p1, p2;
    Line();
    Line(Point _p1, Point _p2);
    Line &operator= (const Line &other);
};

struct LineDetectorInfo
{
	bool fork = false;
	vector<Line> lines;
	int oldDeviation = 0;
	int deviation = 0;
	bool stopline = false;
	bool lost = true;
};

class LineDetector
{
public:
	explicit LineDetector(bool videowriting = false, int _bigRoiHeight = 200, int _roi_row_start = 430, int _roi_height = 20,
			int _window = 430, int _erosion_size = 3, int _minAng = 5, int _imgOffset = 40) :
		bigRoiHeight(_bigRoiHeight), roi_height(_roi_height), roi_row_start(_roi_row_start),
		erosion_size(_erosion_size), minAng(_minAng), window(_window), sorterCenter(_window / 2),
		imgOffset(_imgOffset), _videowriting(videowriting) {

		if(videowriting)
			initVideoWriting();
	}
	LineDetectorInfo detectLine(Mat & src, Mat & drawing);
	void setDirection(int dir = 0);

	~LineDetector();
private:
//	struct sortByDistanceFromCenter
//	{
//		sortByDistanceFromCenter( const LineDetector * info ) : m_info(info) { }
//		const LineDetector* m_info;
//
//		bool operator() (const Point &a, const Point &b);
//	};

	void _offsetImage(Mat &image, const cv::Scalar & bordercolour, int xoffset, int yoffset);

	void preprocessImage(Mat & src, Mat & dst);
	void findRoots(Mat & src, vector<Point> & centers, bool & stopLine);
	void drawResult(LineDetectorInfo & info, Mat & drawing);

	int buildLine(Mat roi, int v, vector<Line> & lines, vector<Point> & pts, vector<vector<int>> & g, vector<int> & h, int depth = 5);
	bool checkForFillness(Mat & roi, Line line);
	bool isFork(vector<vector<int>> & g, vector<int> & h);

	void initVideoWriting();
	void writeFrame(Mat & frame);
	void stopVideoWriting();
	cv::VideoWriter outCap;

	int _dir = 0;

	int erosion_size;
	int window;
	int imgOffset;

	int old_res = 0;

	const int radius = 60;
	int minAng;
	int roi_row_start;
	int roi_height;
	int bigRoiHeight;

	int sorterCenter;

	bool _videowriting;
};

#endif //LINE_NODE_LINEDETECTOR_H
