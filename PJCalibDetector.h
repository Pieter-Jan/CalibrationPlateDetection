#ifndef INC_PJCALIBDETECTOR_H
#define INC_PJCALIBDETECTOR_H

#include <iostream>
#include <stdio.h>

#include "cv.h"
#include "cxcore.h"
#include "highgui.h"

#include <cmath>
#include <vector>

using namespace std;
using namespace cv;

class PJCalibDetector
{
	public:
		PJCalibDetector(Mat* image);
		void FindBlueRectangle(vector<Point> &blueRectangle);
		vector<Point> OrderCorners(Mat* image, vector<Point> blueRectangle);
		
		Mat SetROIFromPolygon(Mat* inputImage, vector<Point> polygon); 
		vector<vector<Point> > GetCornerROIs(vector<Point> polygon);

	private:
		Mat *image;
		vector<Mat> rgbPlanes;
		Mat *redThreshold, *greenThreshold, *blueThreshold;
		Mat *redImage, *greenImage, *blueImage;

		/* Methods */
		bool BlueBorderFound(Point testPoint, double slope, Mat blueThreshold, double &borderWidth);				// Checks if the blue border is found in the neighbourhood of a testpoint
		vector<Vec4i> BlueBorderCheck(vector<Vec4i> Lines, Mat blueThreshold);	// Checks if the line is the edge of a blue border
		vector<Vec4i> SlopeFilter(vector<Vec4i> Lines, double parallel, double perpendicular, double merge);

		/* Corner Detectors */
		bool FindGreenCross(vector<Point> ROI);
		bool FindRedSquare(vector<Point> ROI);
		bool FindOrangeCircle(vector<Point> ROI);

		/* Intersection Check */
		vector<Point> GetLineIntersections(Vec4i line, vector<Vec4i> allLines);
		vector<Point> GetAllIntersections(vector<Vec4i> Lines);	
		bool PointOnLineSegment(Vec4i line, Point point);
		vector<Vec4i> AllLineSegmentsWithinTheirIntersections(vector<Vec4i> allLines);
};

#endif