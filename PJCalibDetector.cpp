#include "PJCalibDetector.h"

/* Constructor */
PJCalibDetector::PJCalibDetector(Mat* image)
{
	this->image = image;

	/* Split image in Red Green & Blue */
	split(*this->image, this->rgbPlanes);

	/* Threshold the blue image */
	this->blueThreshold = new Mat;
	this->greenThreshold = new Mat;
	this->redThreshold = new Mat;

	this->redImage = &this->rgbPlanes[0];
	this->greenImage = &this->rgbPlanes[1];
	this->blueImage = &this->rgbPlanes[2];

	/*Mat imageHSV, imageBlue;
	cvtColor(*this->image, imageHSV, CV_RGB2HSV);
	inRange(imageHSV, Scalar(155, 0, 0), Scalar(185, 255, 255), imageBlue);
	imshow("HSV", imageBlue);*/
	//*this->blueThreshold = imageBlue;

	threshold(this->rgbPlanes[2], *this->blueThreshold, 80, 255, 0);
}						

/* Most important function */
void PJCalibDetector::FindBlueRectangle(vector<Point> &blueRectangle)
{
	/* Apply Canny filter to find lines */
	Mat cannyDest, Blur;
	blur(*blueThreshold, Blur, Size(3,3));
	Canny(Blur, cannyDest, 5, 255, 3);

	/* Apply Hough filter to find straight lines */
	vector<Vec4i> Lines;
	HoughLinesP(cannyDest, Lines, 0.5, CV_PI/180, 5, 35.0, 4.0);

	/* Slope Filter: Only lines with at least 2 perpendiculars and 2 parallels remain */
	Lines = this->SlopeFilter(Lines, 0.3, 0.9, 10.0);		
	
	/* Check if every detected line is the edge of a blue border */
	Lines = this->BlueBorderCheck(Lines, *this->blueThreshold);

	/* Make sure that every line segment is closed in by 2 intersections */
	Lines = this->AllLineSegmentsWithinTheirIntersections(Lines);

	/* Get the perpendicular intersections between these lines */
	vector<Point> intersections = this->GetAllIntersections(Lines);

	/* Organize the intersections in 4 groups */
	vector<vector<Point> > intersectionGroups(10);
	vector<Point> barryCenters;
	barryCenters.assign(10, Point(0,0));
	for(int i=0; i < intersections.size(); i++)
	{
		for(int j=0; j < intersectionGroups.size(); j++)
		{
			if(sqrt(pow((double)intersections[i].x - (double)barryCenters[j].x, 2) + pow((double)intersections[i].y - (double)barryCenters[j].y, 2)) < 30.0 && intersectionGroups[j].size() != 0)
			{
				// Add the element to the group (sort with respect to x: small first, then big)
				int s = intersectionGroups[j].size();
				intersectionGroups[j].push_back(Point(0,0));
				while(s > 0 && intersections[i].x < intersectionGroups[j][s-1].x)
				{
					intersectionGroups[j][s] = intersectionGroups[j][s-1];
					s--;
				}
				intersectionGroups[j][s] = intersections[i];

				// Update the barry-center
				barryCenters[j].x += intersections[i].x;
				barryCenters[j].x /= 2;
				barryCenters[j].y += intersections[i].y;
				barryCenters[j].y /= 2;

				break;
			}
			else if (intersectionGroups[j].size() == 0)
			{
				// Add the element to the group
				intersectionGroups[j].push_back(intersections[i]);

				// Update the barry-center
				barryCenters[j].x += intersections[i].x;
				barryCenters[j].y += intersections[i].y;
				break;
			}
		}
	}

	if(barryCenters.size() > 4)							// TO DO: Create a filter that filters out the best 4 points
		barryCenters.erase(barryCenters.begin() + 4, barryCenters.end());

	/* If one group appears to be empty, bail */
	if(intersectionGroups[3].size() == 0 || intersectionGroups[2].size() == 0 || intersectionGroups[1].size() == 0 || intersectionGroups[0].size() == 0)
	{
		//throw exception("Not enough points are detected!");
	}

	/* Arrange barry centers */
	vector<int> xArrangement, yArrangement;
	vector<Point> mediumXArr = barryCenters, mediumYArr = barryCenters;
	int temp;
	xArrangement.assign(4,0);
	yArrangement.assign(4,0);
	Point temp2;
	for(int i=0; i<barryCenters.size(); i++)
		xArrangement[i] = yArrangement[i] = i;

	for(int i=0; i<barryCenters.size(); i++)
	{
		for(int j=(i+1); j<barryCenters.size(); j++)
		{
			if(mediumXArr[i].x > mediumXArr[j].x)
			{
				temp = xArrangement[i];
				xArrangement[i] = j;
				xArrangement[j] = temp;

				temp2 = mediumXArr[i];
				mediumXArr[i] = mediumXArr[j];
				mediumXArr[j] = temp2;
			}

			if(mediumYArr[i].y > mediumYArr[j].y)
			{
				temp = yArrangement[i];
				yArrangement[i] = j;		
				yArrangement[j] = temp;

				temp2 = mediumYArr[i];
				mediumYArr[i] = mediumYArr[j];
				mediumYArr[j] = temp2;
			}
		}
	}

	Point tempPoint;

	/* First corner (lowest x) */ 
	blueRectangle.push_back(Point(intersectionGroups[xArrangement[3]][intersectionGroups[xArrangement[3]].size()-1].x, intersectionGroups[xArrangement[3]][intersectionGroups[xArrangement[3]].size()-1].y));

	/* Second corner (highest y -> resort the group for y) */
	for(int i=0; i<intersectionGroups[yArrangement[0]].size(); i++)
	{
		for(int j=(i+1); j < intersectionGroups[yArrangement[0]].size(); j++)
		{
			if(intersectionGroups[yArrangement[0]][i].y > intersectionGroups[yArrangement[0]][j].y)
			{
				tempPoint = intersectionGroups[yArrangement[0]][i];
				intersectionGroups[yArrangement[0]][i] = intersectionGroups[yArrangement[0]][j];
				intersectionGroups[yArrangement[0]][j] = tempPoint;
			}
		}
	}
	blueRectangle.push_back(Point(intersectionGroups[yArrangement[0]][0].x, intersectionGroups[yArrangement[0]][0].y));

	/* Third corner (highest x) */
	blueRectangle.push_back(Point(intersectionGroups[xArrangement[0]][0].x, intersectionGroups[xArrangement[0]][0].y));

	/* Fourth corner (lowest y -> resort the group for y) */
	for(int i=0; i<intersectionGroups[yArrangement[3]].size(); i++)
	{
		for(int j=(i+1); j < intersectionGroups[yArrangement[3]].size(); j++)
		{
			if(intersectionGroups[yArrangement[3]][i].y > intersectionGroups[yArrangement[3]][j].y)
			{
				tempPoint = intersectionGroups[yArrangement[3]][i];
				intersectionGroups[yArrangement[3]][i] = intersectionGroups[yArrangement[3]][j];
				intersectionGroups[yArrangement[3]][j] = tempPoint;
			}
		}
	}
	blueRectangle.push_back(Point(intersectionGroups[yArrangement[3]][intersectionGroups[yArrangement[3]].size()-1].x, intersectionGroups[yArrangement[3]][intersectionGroups[yArrangement[3]].size()-1].y));
}

/* Help Functions */
vector<Vec4i> PJCalibDetector::SlopeFilter(vector<Vec4i> Lines, double parallel, double perpendicular, double merge)
{
	Vec4i combinedLine;
	Point p1, p2, p1c, p2c, midPoint;
	double m1, m2, distance1, distance2, distance3, distance4, x, y, distanceIntersectionMidPoint;
	for(int i=0; i<Lines.size(); i++)
	{
		int parallelMatches = 0;
		int perpendicularMatches = 0;

		p1 = Point(Lines[i][0], Lines[i][1]);
		p2 = Point(Lines[i][2], Lines[i][3]);

		m1 = ((double)p2.y-(double)p1.y)/((double)p2.x-(double)p1.x);

		/* Parallel check */
		for(int j=0; j<Lines.size(); j++)
		{
			if(j != i)
			{
				p1c = Point(Lines[j][0], Lines[j][1]);
				p2c = Point(Lines[j][2], Lines[j][3]);

				m2 = ((double)p2c.y-(double)p1c.y)/((double)p2c.x-(double)p1c.x);

				if(abs(m1-m2) < parallel)
				{
					parallelMatches++;	// Lines are parallel

					/* Now check if these lines have the same carrier (if they are very parallel!) */
					if(abs(m1-m2) < parallel/4.0)
					{
						x = m1*(double)p1.x - (double)p1.y - m2*double(p1c.x) + (double)p1c.y;
						x /= (m1 - m2);
						y = m1*(x-(double)p1.x) + (double)p1.y;

						distance1 = sqrt(pow((double)p1.x - (double)p1c.x, 2) + pow((double)p1.y - (double)p1c.y, 2));
						distance2 = sqrt(pow((double)p1.x - (double)p2c.x, 2) + pow((double)p1.y - (double)p2c.y, 2));
						distance3 = sqrt(pow((double)p2.x - (double)p1c.x, 2) + pow((double)p2.y - (double)p1c.y, 2));
						distance4 = sqrt(pow((double)p2.x - (double)p2c.x, 2) + pow((double)p2.y - (double)p2c.y, 2));
						if(distance1 < distance2 && distance1 < distance3 && distance1 < distance4 /*&& distance1 < 20*/)
							midPoint = Point(((double)p1.x + (double)p1c.x)/2.0, ((double)p1.y + (double)p1c.y)/2.0); 
						else if(distance2 < distance1 && distance2 < distance3 && distance2 < distance4 /*&& distance2 < 20*/) 
							midPoint = Point(((double)p1.x + (double)p2c.x)/2.0, ((double)p1.y + (double)p2c.y)/2.0); 
						else if(distance3 < distance1 && distance3 < distance2 && distance3 < distance4 /*&& distance3 < 20*/) 
							midPoint = Point(((double)p2.x + (double)p1c.x)/2.0, ((double)p2.y + (double)p1c.y)/2.0); 
						else if(distance4 < distance1 && distance4 < distance2 && distance4 < distance3 /*&& distance4 < 20*/) 
							midPoint = Point(((double)p2.x + (double)p2c.x)/2.0, ((double)p2.y + (double)p2c.y)/2.0); 

						distanceIntersectionMidPoint = sqrt(pow((double)midPoint.x - (double)x, 2) + pow((double)midPoint.y - (double)y, 2));

						if(distanceIntersectionMidPoint < merge)
						{
							if(distance1 > distance2 && distance1 > distance3 && distance1 > distance4)
								combinedLine = Vec4i(p1.x, p1.y, p1c.x, p1c.y);
							else if(distance2 > distance1 && distance2 > distance3 && distance2 > distance4) 
								combinedLine = Vec4i(p1.x, p1.y, p2c.x, p2c.y);
							else if(distance3 > distance1 && distance3 > distance2 && distance3 > distance4) 
								combinedLine = Vec4i(p2.x, p2.y, p1c.x, p1c.y);
							else if(distance4 > distance1 && distance4 > distance2 && distance4 > distance3) 
								combinedLine = Vec4i(p2.x, p2.y, p2c.x, p2c.y);

							Lines.erase(Lines.begin() + i);
							if(j > i)
								Lines.erase(Lines.begin() + (j-1));
							else
							{
								Lines.erase(Lines.begin() + j);
								i--;
							}

							Lines.push_back(combinedLine);
							i--;
							//cout << "Two lines with the same carrier where combined." << endl;
							break;			// The lines are removed, get the fuck out of here!
						}
					}
				}
				else if(abs(m1*m2+1) < perpendicular)
				{
					perpendicularMatches++;	// Lines are perpendicular
				}
			}
		}
		if(!(parallelMatches >= 2 && perpendicularMatches >=2))							// Every line needs at least one parallels and 2 perpendiculars (rectangle)
		{
			//cout << "Removed line with " << parallelMatches << " parallels and " << perpendicularMatches << " perpendiculars." << endl;
			Lines.erase(Lines.begin()+i);
			i--;
		}
	}
	return Lines;
}

bool PJCalibDetector::BlueBorderFound(Point testPoint, double slope, Mat blueThreshold, double &borderWidth)
{
	double angle = atan(slope);

	Point currentTestPoint;
	int grayValueCurrent, grayValuePrevious;
	bool transition1 = false, transition2 = false;
	int transition1Row = -100, transition2Row = -100;

	int nrCols=5, nrRows = 20;
	for(int row=0; row<nrRows; row++)
	{
		borderWidth = 0;
		grayValueCurrent = 0;
		for(int col = 0; col<nrCols; col++)
		{
			currentTestPoint = Point(testPoint.x + (row-(nrRows/2))*sin(angle) + (col-(nrCols/2))*cos(angle), testPoint.y - (row-(nrRows/2))*cos(angle) + (col-(nrCols/2))*sin(angle));
			if(testPoint.x > 0 && testPoint.x < 630 && testPoint.y > 0 && testPoint.y < 470)
			{
				grayValueCurrent += blueThreshold.at<uchar>(currentTestPoint);
			}
			else
			{
				return false;
			}

			Scalar color;
			if(transition1 && !transition2)
				color = CV_RGB(255, 0, 0);
			else
				color = CV_RGB(0, 255, 0);

			//circle(*this->image, currentTestPoint, 1, color, 1, 1);
		}
		grayValueCurrent /= nrCols;

		int value = 150;
		if(row!=0)
		{
			if((!transition1) && (grayValueCurrent - grayValuePrevious < -value))							// Going into the white border
			{
				transition1 = true;
				transition1Row = row;
			}
			else if((transition1) && (!transition2) && (grayValueCurrent - grayValuePrevious > value))		// Coming out of the white border
			{
				transition2 = true;
				transition2Row = row;
			}
		}
		grayValuePrevious = grayValueCurrent;
	}
		
	int rowValue = 1;
	if(((transition1Row-10) >= -rowValue && (transition1Row-10) <= rowValue)  || ((transition2Row-10) >= -rowValue && (transition2Row-10) <= rowValue))
		if(transition1 && transition2)
		{
			borderWidth = abs((double)transition2Row - (double)transition1Row);
			return true;
		}
		else
			return false;
	else
		return false;
}

vector<Vec4i> PJCalibDetector::BlueBorderCheck(vector<Vec4i> Lines, Mat blueThreshold)
{
	double m, borderWidth = 0, averageBorderWidth = 0;
	Point testPoint;
	double succesFrequency;
	for(int i=0; i < Lines.size(); i++)
	{
		succesFrequency = 0;
		averageBorderWidth = 0;
		m = ((double)Lines[i][1] - (double)Lines[i][3])/((double)(Lines[i][0]-(double)Lines[i][2]));
		double distance = sqrt(pow((double)Lines[i][0] - (double)Lines[i][2], 2) + pow((double)Lines[i][1] - (double)Lines[i][3], 2));
		
		int numberOfSamples = distance/5;		// 5 cols
		for(int j=0; j < numberOfSamples; j++)
		{
			testPoint = Point(Lines[i][0] + j*(distance/numberOfSamples)*cos(atan(m)), Lines[i][1] + j*(distance/numberOfSamples)*sin(atan(m)));
			if(this->BlueBorderFound(testPoint, m, blueThreshold, borderWidth))
			{
				averageBorderWidth += borderWidth;
				succesFrequency++;		
			}
		}

		averageBorderWidth /= (double)numberOfSamples;
		if(averageBorderWidth <= 1.0 || succesFrequency < numberOfSamples/5)
		{
			Lines.erase(Lines.begin() + i);
			i--;
			//cout << "Removed line because the average border width was too small." << endl;
		}
	}

	return Lines;
}

/* ROI Funtions */
Mat PJCalibDetector::SetROIFromPolygon(Mat* inputImage, vector<Point> polygon)
{
	/* ROI by creating mask for the parallelogram */
	Mat mask = cvCreateMat(480, 640, CV_8UC1);		// Create black image with the same size as the original
	for(int i=0; i<mask.cols; i++)
		for(int j=0; j<mask.rows; j++)
			mask.at<uchar>(Point(i,j)) = 0;

	vector<Point> approxedRectangle;
	approxPolyDP(polygon, approxedRectangle, 1.0, true);

	fillConvexPoly(mask, &approxedRectangle[0], approxedRectangle.size(), 255, 8, 0);

	Mat imageDest = cvCreateMat(480, 640, CV_8UC3);

	inputImage->copyTo(imageDest, mask);		

	return imageDest;
}

vector<vector<Point> > PJCalibDetector::GetCornerROIs(vector<Point> polygon)
{
	vector<vector<Point> > ROIs(4);

	/* Create parallellogram in each corner of the plate */
	int s = 0, t = 0;
	for(int i=0; i<4; i++)
	{
		(i < 3) ? s = i+1 : s = 3 - i;

		(i == 0) ? t = i+3 :t= i-1;

		ROIs[i].push_back(polygon[i]);

		double Distance = sqrt(pow(((double)polygon[i].x - (double)polygon[s].x), 2) + pow(((double)polygon[i].y - (double)polygon[s].y),2));
		double m1 = ((double)polygon[s].y - (double)polygon[i].y)/((double)polygon[s].x - (double)polygon[i].x);
		double Angle = atan(m1);

		(polygon[s].x < polygon[i].x) ? Angle += CV_PI : Angle = Angle;

		ROIs[i].push_back(Point(polygon[i].x + cos(Angle)*Distance/4.0, polygon[i].y + sin(Angle)*Distance/4.0));

		Distance = sqrt(pow(((double)polygon[i].x - (double)polygon[t].x), 2) + pow(((double)polygon[i].y - (double)polygon[t].y),2));
		double m2 = ((double)polygon[t].y - (double)polygon[i].y)/((double)polygon[t].x - (double)polygon[i].x);
		Angle = atan(m2);

		(polygon[t].x < polygon[i].x) ? Angle += CV_PI : Angle = Angle;

		ROIs[i].push_back(Point(polygon[i].x  + cos(Angle)*Distance/4.0, polygon[i].y + sin(Angle)*Distance/4.0));

		double x = m2*(double)ROIs[i][1].x - (double)ROIs[i][1].y - m1*double(ROIs[i][2].x) + (double)ROIs[i][2].y;
		x /= (m2 - m1);
		double y = m2*(x-(double)ROIs[i][1].x) + (double)ROIs[i][1].y;
		ROIs[i].push_back(Point(x, y));

		Point tempPoint = ROIs[i][1];
		ROIs[i][1] = ROIs[i][0];
		ROIs[i][0] = tempPoint;
	}

	return ROIs;
}

/* IntersectionCheck */
vector<Point> PJCalibDetector::GetLineIntersections(Vec4i line, vector<Vec4i> allLines)
{
	vector<Point> intersections;

	Point p1(line[0], line[1]);
	Point p2(line[2], line[3]);

	double x, y, m1, m2;
	Point p1c, p2c;

	m1 = ((double)p2.y-(double)p1.y)/((double)p2.x-(double)p1.x);

	for(int i=0; i < allLines.size(); i++)
	{
		x = y = 0;
		p1c = Point(allLines[i][0], allLines[i][1]);
		p2c = Point(allLines[i][2], allLines[i][3]);
		m2 = ((double)p2c.y-(double)p1c.y)/((double)p2c.x-(double)p1c.x);

		if((m1 - m2) != 0 && (m1*m2 + 1) < 1.0)		// Get only the perpendicular ones! 
		{
			x = m1*(double)p1.x - (double)p1.y - m2*double(p1c.x) + (double)p1c.y;
			x /= (m1 - m2);
			y = m1*(x-(double)p1.x) + (double)p1.y;
			intersections.push_back(Point(x, y));
		}
	}

	return intersections;
}

vector<Point> PJCalibDetector::GetAllIntersections(vector<Vec4i> Lines)
{
	/* Calculate the intersections */
	Point p1, p2, p1c, p2c;
	vector<Point> intersections;
	vector<Point> intersectionsTemp;
	for(int i=0; i < Lines.size(); i++)
	{
		intersectionsTemp = this->GetLineIntersections(Lines[i], Lines);

		for(int j=0; j < intersectionsTemp.size(); j++)
			intersections.push_back(intersectionsTemp[j]);
	}

	return intersections;
}

bool PJCalibDetector::PointOnLineSegment(Vec4i line, Point point)
{
	/* Check if point lies on the line carrier */
	double m = ((double)line[3] - (double)line[1])/((double)line[2] - (double)line[0]);
	if(abs(((double)point.y - (double)line[1])/((double)point.x - (double)line[0]) - m) > 0.05)
		return false;

	/* Check if point lies within the segment */
	if((point.x >= (double)line[0]  && point.x <= (double)line[2]) || (point.x <= (double)line[0]  && point.x >= (double)line[2]))
		return true;
	else 
		return false;
}

vector<Vec4i> PJCalibDetector::AllLineSegmentsWithinTheirIntersections(vector<Vec4i> allLines)
{
	vector<Point> lineIntersections;
	for(int s=0; s < allLines.size(); s++)
	{
		lineIntersections = this->GetLineIntersections(allLines[s], allLines);

		bool midPointOnLine;
		bool succes = false;
		Vec4i intersectionSegment; // line between the two selected intersection points
		Vec4i line = allLines[s];
		Point midPoint;
		for(int i=0; i<lineIntersections.size();i++)
		{
			for(int j=0; j<lineIntersections.size();j++)
			{
				if(i!=j)
				{
					intersectionSegment = Vec4i(lineIntersections[i].x, lineIntersections[i].y, lineIntersections[j].x, lineIntersections[j].y);
					
					midPoint = Point(((double)line[0]+(double)line[2])/2.0, ((double)line[1] + (double)line[3])/2.0);
					midPointOnLine = this->PointOnLineSegment(intersectionSegment, midPoint);
					
					if(midPointOnLine)
						succes = true;
				}
			}
		}

		if(!succes)
		{
			allLines.erase(allLines.begin() + s);
			s--;
		}
	}
	return allLines;
}

/* Order Rectangle */
vector<Point> PJCalibDetector::OrderCorners(Mat* image, vector<Point> blueRectangle)
{
	Mat imageReduced;

	vector<vector<Point> > ROIs;
	ROIs = this->GetCornerROIs(blueRectangle);

	/* Find the green cross */
	for(int i=0; i<blueRectangle.size(); i++)
	{
		// TO DO

		this->FindGreenCross(ROIs[i]);
		//this->FindRedSquare(ROIs[i]);
		//this->FindOrangeCircle(ROIs[i]);
	}

	imshow("Test", *this->image);

	return blueRectangle;
}

bool PJCalibDetector::FindGreenCross(vector<Point> ROI)
{
	Mat imageReducedThres;
	Mat imageReduced = this->SetROIFromPolygon(this->greenImage, ROI);

	double ROIarea = contourArea(ROI);
	RotatedRect rotatedRect;
	vector<vector<Point> > contours;
	double area, boxedArea, boxRatio, ROIratio, whRatio;
	vector<Point> greenCross;
	Mat superBlur;
	for(int j=45; j < 100; j = j+5)
	{
		threshold(imageReduced, imageReducedThres, j, 255, 0);		

		if(j > 70)
			blur(imageReducedThres, imageReducedThres, Size(5, 2));
		else
			blur(imageReducedThres, imageReducedThres, Size(5, 1));

		findContours(imageReducedThres, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	
		for(int i=0; i<contours.size(); i++)
		{
			approxPolyDP(contours[i], greenCross, 0.8, true);
			area = contourArea(greenCross);
			if(greenCross.size() < 35 && greenCross.size() >= 12 && area > 25.0)	// The cross has 12 vertices, the approximation will usually have more ...
			{	
				rotatedRect = minAreaRect(greenCross);
				boxedArea = rotatedRect.size.area();
		
				boxRatio = area/boxedArea;										// Ratio of the area with that of the bounding box
				ROIratio = area/ROIarea;										// Ratio with the ROI to have an idea how small/big the mark is
				whRatio = rotatedRect.size.width/rotatedRect.size.height;		// Width-Height ratio of the bounding box

				/*if(j == 50)
				{
					imshow("test", imageReducedThres);
					cout << "ROIratio: " << ROIratio << endl;
					cout << "BoxRatio: " << boxRatio << endl;
					cout << "WH: " << whRatio << endl;
				}*/

				if((boxRatio < 0.70 && boxRatio > 0.30) && (ROIratio < 0.20 && ROIratio > 0.03) && abs(whRatio-1) < 0.6)
				{
					fillConvexPoly(*this->image, &greenCross[0], greenCross.size(), CV_RGB(0, 255, 0), 8);
					return true; 	// Green cross has been found
				}
			}
		}
	}
	return false;
}

bool PJCalibDetector::FindRedSquare(vector<Point> ROI)
{
	Mat imageReduced = this->SetROIFromPolygon(this->redImage, ROI);
	threshold(imageReduced, imageReduced, 40, 255, 1); 
	
	vector<vector<Point> > contours;
	findContours(imageReduced, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);	

	// TO DO
	return false;
}

bool PJCalibDetector::FindOrangeCircle(vector<Point> ROI)
{
	// TO DO
	return false;
}