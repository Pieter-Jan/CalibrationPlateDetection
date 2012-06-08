#include "PJCalibDetector.h"
#include <ctime>

int main()
{
	/* Get the image */
	Mat *image = new Mat;
	*image = imread("TestImage5.png", 1);

	if(!image->data)
		throw exception("Unable to open image.");

	PJCalibDetector calibDetector(image);

	try
	{
		vector<Point> blueRectangle;
		clock_t start = clock();
		
		calibDetector.FindBlueRectangle(blueRectangle);
		
		clock_t ends = clock();
		cout << "Detection Time: " << (double) (ends - start) / CLOCKS_PER_SEC << endl;

		/* Display the lines */
		line(*image, blueRectangle[0], blueRectangle[1], CV_RGB(255, 0, 0), 2, 8);
		line(*image, blueRectangle[1], blueRectangle[2], CV_RGB(255, 0, 0), 2, 8);
		line(*image, blueRectangle[2], blueRectangle[3], CV_RGB(255, 0, 0), 2, 8);
		line(*image, blueRectangle[3], blueRectangle[0], CV_RGB(255, 0, 0), 2, 8);

		/* Display the corners */
		for(int i=0; i<blueRectangle.size(); i++)
		{
			circle(*image, blueRectangle[i], 1, CV_RGB(255, 255, 0), 2, 8);
		}

		//calibDetector.OrderCorners(image, blueRectangle);
		
		imshow("Test4", *image);
		waitKey(0);
	}
	catch(exception& ex)
	{
		cout << ex.what() << endl;
	}

	waitKey(0);
}