#include "PolarizationParameters.h"

namespace PolPara {

	PolarizationParameters::PolarizationParameters()
	{
		typ = "";
	}

	PolarizationParameters::~PolarizationParameters()
	{
	}

	void PolarizationParameters::setMethod(polTypes typ) {
		switch (typ)
		{
		case PolPara::PolarizationParameters::DIFFUSE:
			this->typ = "diffuse";
			break;
		case  PolPara::PolarizationParameters::SPECULAR:
			this->typ = "specular";
			break;
		default:
			break;
		}
	}

	int PolarizationParameters::executePolarizationParameters() {
		if (this->myParameters.Input.size() != 1 || this->myParameters.Input.size() != 4) {
			cerr << "there is no valid input size" << endl;
			return -1;
		}

		if (this->myParameters.Input.size() == 1)imgSplit(this->myParameters.Input.at(0),this->myParameters.Input);

		cv::Mat img0 = this->myParameters.Input[0].clone();
		cv::Mat img45 = this->myParameters.Input[1].clone();
		cv::Mat img90 = this->myParameters.Input[2].clone();
		cv::Mat img135 = this->myParameters.Input[3].clone();

	
		img0.convertTo(img0, CV_64FC1);
		img45.convertTo(img45, CV_64FC1);
		img90.convertTo(img90, CV_64FC1);
		img135.convertTo(img135, CV_64FC1);

		cv::Mat S1(img0.rows, img0.cols, CV_64FC1);

		cv::Mat S2 = S1.clone();
		cv::Mat S0 = S1.clone();
		cv::Mat tmp1 = S1.clone();
		cv::Mat tmp2 = S1.clone();
		cv::Mat tmp3;

		S0 = 0.5*(img0 + img45 + img90 + img135);
		S0.convertTo(this->myParameters.Intensity, CV_8UC1);

		tmp1 = (img0 - img90);
		tmp2 = (img45 - img135);

		cv::divide(tmp1, S0, S1);
		cv::divide(tmp2, S0, S2);

		//calc AoP
		calculateAOP(S1, S2, this->myParameters.AoLP);

		//calc DoP
		calculateDOP(S1, S2, this->myParameters.DoLP);

		//calc zenith
		calcZenith(this->myParameters.DoLP, this->myParameters.Zenith);

		//calc azimuth
		calculateAzimuth(this->myParameters.AoLP, this->myParameters.Azimuth);

		//calc normal vector
		calcNormalVector2D(this->myParameters.Zenith, this->myParameters.Azimuth);


	}


	void PolarizationParameters::calculateAOP(cv::Mat S1, cv::Mat S2, cv::Mat& AOP) {
		double mini, Maxim;
		AOP.release();
		AOP.create(S1.rows, S1.cols, CV_64FC1);  //convert mat to double precicion

		if (S2.rows != S1.rows || S2.cols != S1.cols) {
			cout << "dimension of s1 and s2 is not thesame" << endl;
			return;
		}

		int rows = S1.rows;
		int cols = S1.cols;

		for (int r = 0; r < rows; r++) {
			for (int c = 0; c < cols; c++) {

				double x = S1.at<double>(r, c);
				double y = S2.at<double>(r, c);

				double res = 0.5 * std::atan2(y, x);

				if (y < 0) {
					res += CV_PI;
				}
				AOP.at<double>(r, c) = res;

			}
		}


		cv::minMaxIdx(AOP, &mini, &Maxim);

		////scale for display purpose
		cv::Mat output = AOP.clone() * 60.0;

		output.convertTo(output, CV_8UC1);

		imgSave(output, "AoLP", ".png");
	}
	void PolarizationParameters::calculateDOP(cv::Mat S1, cv::Mat S2, cv::Mat& DOLP) {

		DOLP.release();
		DOLP.create(S1.rows, S1.cols, CV_64FC1);  //convert mat to double precicion

		double mini, Maxim;
		cv::Mat tmpS1, tmpS2, tmpS3, tmpS4, DOLP_tmp;
		tmpS1.convertTo(tmpS1, CV_64FC1);
		tmpS2.convertTo(tmpS2, CV_64FC1);
		tmpS3.convertTo(tmpS3, CV_64FC1);



		cv::pow(S1, 2.0, tmpS1);
		cv::pow(S2, 2.0, tmpS2);
		cv::add(tmpS1, tmpS2, tmpS3);
		cv::sqrt(tmpS3, DOLP);


		cv::minMaxIdx(DOLP, &mini, &Maxim);

		////scale for display purpose
		cv::Mat output = DOLP.clone();

		output = (output / Maxim) *800.0;
		output.convertTo(output, CV_8UC1);

		imgSave(output, "DOLP", ".png");
	}


	void PolarizationParameters::calcZenith(cv::Mat DoP, cv::Mat &Zenith, double n) {

		Zenith.release();
		Zenith.create(DoP.rows, DoP.cols, DoP.type());

		int rows = DoP.rows;
		int cols = DoP.cols;


		double aa, bb, cc, dd, retval = 0.f, t1, t2;

		//calc zenith

		if (typ == "specular") {
			for (int r = 0; r < rows; r++) {
				const double *Pzen = Zenith.ptr<double>(r);
				const double *pDoP = DoP.ptr<double>(r);
				for (int c = 0; c < cols; c++) {
					retval = 0.f;
						double d = *pDoP;
						double q = pow(n, 2);
						aa = sqrt(pow(n, 4.0) - pow(n, 2.0) *q* pow(d, 2.0));
						bb = sqrt((8 * pow(n, 4)) - (8 * pow(n, 2) * (q - 1) * pow(d, 2)) + ((q - 4) * q * pow(d, 4)) + (8 * pow(n, 2) * aa) + (8 * pow(d, 2) * aa) - (4 * q* pow(d, 2)*aa));
						cc = sqrt(8 * pow(n, 4) - 8 * pow(n, 2)*(q - 1)*pow(d, 2) + (q - 4)*q*pow(d, 4) - 8 * pow(n, 2)*aa - 8 * pow(d, 2)*aa + 4 * q*pow(d, 2)*aa);

						t1 = (pow(n, 2)*(bb / pow(d, 2) - q) - aa * (bb / pow(d, 2) + q)) / (n*q);
						t1 = 1 / t1;
						t1 = atan(t1*sqrt(2 * bb - 4 * aa - 4 * pow(n, 2) + 2 * q*pow(d, 2)));
						//if (!(0 <= t1 && t1 <= (CV_PI / 2)))t1 = 0;
						t2 = (pow(n, 2) *(cc / pow(d, 2) - q) + aa * (cc / pow(d, 2) + q)) / (n*q);
						t2 = 1 / t2;
						t2 = atan(t2 *sqrt(2 * cc + 4 * aa - 4 * pow(n, 2) + 2 * q*pow(d, 2)));
						//if (!(0 <= t2 && t2 <= (CV_PI / 2)))t2 = 0;

						//check for valid value of ref index
						//check if both are valid but t1 < t2
						if (t1 >= 0 && t1 <= (CV_PI / 2) && (!isnan(t1) && !isinf(t1)) && t2 >= 0 && t2 <= (CV_PI / 2) && (!isnan(t2) && !isinf(t2)) && t1 < t2)retval = t1;
						//check if both are valid but t1 > t2
						else if (t1 >= 0 && t1 <= (CV_PI / 2) && (!isnan(t1) && !isinf(t1)) && t2 >= 0 && t2 <= (CV_PI / 2) && (!isnan(t2) && !isinf(t2)) && t1 > t2)retval = t2;
						//check if only t1 is valid
						else if (t1 >= 0 && t1 <= (CV_PI / 2) && (!isnan(t1) && !isinf(t1)) && (isnan(t2) || isinf(t2)))retval = t1;
						//check if only t2 is valid
						else if (t2 >= 0 && t2 <= (CV_PI / 2) && (!isnan(t2) && !isinf(t2)) && (isnan(t1) || isinf(t1)))retval = t2;

						if (retval >= 0.f && retval <= (CV_PI / 2.0) && !isnan(retval) && !isinf(retval)) Zenith.at<double>(r, c) = retval;
						else Zenith.at<double>(r, c) = 0.f;
						++pDoP;
						++Pzen;
				}


			}

		}
		else {
			for (int r = 0; r < rows; r++) {
				const double *Pzen = Zenith.ptr<double>(r);
				const double *pDoP = DoP.ptr<double>(r);
				for (int c = 0; c < cols; c++) {
					retval = 0.f;
						aa = pow((n - (1 / n)), 2.0) + ((*pDoP) * pow((n + (1 / n)), 2.0));
						bb = 4 * (*pDoP) * ((n * n) + 1) * (aa - 4 * (*pDoP));
						cc = (bb * bb) + (16 * (*pDoP) * (*pDoP)) * (16 * ((*pDoP) * (*pDoP)) - (aa * aa))  * ((n * n) - 1) * ((n * n) - 1);
						dd = sqrt(((-bb - sqrt(cc)) / (2 * (16 * (*pDoP * (*pDoP)) - aa * aa))));
						retval = std::real(asin(dd));
					
						if (retval >= 0.f && retval <= (CV_PI / 2.0) && !isnan(retval) && !isinf(retval)) Zenith.at<double>(r, c) = retval;
						else Zenith.at<double>(r, c) = 0.f;
						++pDoP;
						++Pzen;
				}
			}

		}
		Zenith.convertTo(Zenith, DoP.type());
	}


	void PolarizationParameters::calculateAzimuth(cv::Mat AoP, cv::Mat &Azimuth) {
		Azimuth = AoP.clone();
		if (typ == "specular")Azimuth = AoP.clone() + (CV_PI / 2);
	}


	void PolarizationParameters::calcNormalVector2D(const cv::Mat Zenith, const cv::Mat Azimuth) {

		cv::Mat polNx, polNy, polNz;
		int rows = Zenith.rows;
		int cols = Zenith.cols;

		polNx.create(rows, cols, Zenith.type());
		polNy = polNx.clone();
		polNz = polNx.clone();

		for (int r = 0; r < rows; r++) {
			const double *PZenith = Zenith.ptr<double>(r);
			const double *PAzimuth = Azimuth.ptr<double>(r);

			double *Pgx = polNx.ptr<double>(r);
			double *Pgy = polNy.ptr<double>(r);
			double *Pgz = polNz.ptr<double>(r);

			for (int c = 0; c < cols; c++) {
				double x, y, z;
				x = std::cos(*PAzimuth) * std::sin(*PZenith);
				y = std::sin(*PAzimuth) * std::sin(*PZenith);
				z = std::cos(*PZenith);

				*Pgx = x;
				*Pgy = y;
				*Pgz = z;

				++PZenith;
				++PAzimuth;
				++Pgx;
				++Pgy;
				++Pgz;

			}
		}
		this->myParameters.Normal_x = polNx.clone();
		this->myParameters.Normal_y = polNy.clone();
		this->myParameters.Normal_z = polNz.clone();

	}

	void PolarizationParameters::imgSave(cv::Mat img, string filepath, string format) {
		string filename = filepath + format;
		cv::imwrite(filename, img);
	}

	void  PolarizationParameters::imgSplit(const cv::Mat input, std::vector<cv::Mat>&output) {
		cv::cvtColor(input, input, CV_BGR2GRAY);
		cv::Mat im0, im45, im90, im135;
		int rows = input.rows / 2;
		int cols = input.cols / 2;

		im0.release();
		im0.create(rows, cols, input.type());
		im45 = im0.clone();
		im90 = im0.clone();
		im135 = im0.clone();

		for (int r = 0; r < rows; ++r) {
			uchar *Pim0 = im0.ptr<uchar>(r);
			uchar *Pim45 = im45.ptr<uchar>(r);
			uchar *Pim90 = im90.ptr<uchar>(r);
			uchar *Pim135 = im135.ptr<uchar>(r);
			for (int c = 0; c < cols; ++c) {
				//create mask
				cv::Mat mask(2, 2, input.type());
				cv::Rect rec(c * 2, r * 2, 2, 2);

				//copy to mask
				input(rec).copyTo(mask);

				//copy to image
				*Pim0 = mask.at<uchar>(0, 1);
				*Pim45 = mask.at<uchar>(0, 0);
				*Pim90 = mask.at<uchar>(1, 0);
				*Pim135 = mask.at<uchar>(1, 1);

				++Pim0;
				++Pim45;
				++Pim90;
				++Pim135;


			}
		}

		output.clear();
		output.push_back(im0);
		output.push_back(im45);
		output.push_back(im90);
		output.push_back(im135);
	}


}

