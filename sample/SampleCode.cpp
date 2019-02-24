#include "PolarizationParameters.h"

using namespace std;
using namespace cv;

int main() {
	// instantiate the object of the Polarization class
	PolPara::PolarizationParameters *objPol = new PolPara::PolarizationParameters();

	//declare variable for the image buffer and read image from file
	Mat data = imread("im.tif");

	//set the input image
	objPol->myParameters.Input.push_back(data);

	//set the Polarization type 
	objPol->setMethod(PolPara::PolarizationParameters::DIFFUSE);

	//execute the algorithm
	objPol->executePolarizationParameters();

	//delete the point to the Polarization class
	delete objPol;

	return 0;
}