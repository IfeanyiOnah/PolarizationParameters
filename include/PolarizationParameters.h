#pragma once
#ifndef POLARIZATIONPARAMETERS_H_
#define POLARIZATIONPARAMETERS_H_
#include<iostream>
#include<opencv2\core\core.hpp>
#include<opencv2\highgui\highgui.hpp>
#include<opencv2\imgproc\imgproc.hpp>

#define PolParameters_export _declspec(dllexport)
using namespace std;

namespace  PolPara {
	namespace Container
	{
		struct Parameters {
			vector<cv::Mat> Input;
			cv::Mat Intensity, AoLP, DoLP, Azimuth, Zenith;
			cv::Mat Normal_x, Normal_y, Normal_z;
		};
	}

	class PolParameters_export PolarizationParameters
	{
	public:
		PolarizationParameters();
		~PolarizationParameters();

	public:
		//data member
		Container::Parameters myParameters;

		/** \brief: existing algorithms already implemented for demosaicing */
		enum polTypes { DIFFUSE, SPECULAR };

		//member function
		int executePolarizationParameters();
		/*******************************************************************************
* @fn:	setMethod
*
* @brief:	setMethod set the user desired type of polarization method already implemented in this library
*
* @input  parameters:	input enum type for the polarization method
*/
		void setMethod(polTypes typ);

	protected:
		void  imgSplit(cv::Mat input, std::vector<cv::Mat>&output);
		void imgSave(cv::Mat img, string filepath, string format);
		void calculateAOP(cv::Mat S1, cv::Mat S2, cv::Mat& AOP);
		void calculateDOP(cv::Mat S1, cv::Mat S2, cv::Mat& DOLP);
		void calculateAzimuth(cv::Mat AOP, cv::Mat &Azimuth);
		void calcZenith(const cv::Mat DoP, cv::Mat &Zenith, double n = 1.5);
	private:

		string typ;

		void calcNormalVector2D(const cv::Mat Zenith, const cv::Mat Azimuth);
	};

}


#endif
