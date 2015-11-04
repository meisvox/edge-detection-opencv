// Driver.cpp			Author: Sam Hoover
// This program takes an input image, INPUT_IMAGE_FILE, and smooths the image,
// the number of smoothing repetitions is gathered via the commandline arguments.
// The output of the smoothing is saved as "smooth.gif." After smoothing, edge
// detection is performed. The output of edge detection is saved as "edges.gif."
//
// This program utilizes Image.h and Image.lib, provided by Prof. Olson, UWB
// CSS487 Spring 2015. 
//
// Global Precondition: Image.h and Image.lib must be included.
//
#ifndef DRIVER_CPP
#define DRIVER_CPP
#include <iostream>
#include "Image.h"

using namespace std;

const int THRESHOLD = 10;
const int DEFAULT_SMOOTHING_REPS = 2;
const int MAX_SMOOTHING_REPS = 20;
const string INPUT_IMAGE_FILE = "test.gif";

// greyToFloat
// For every pixel in an Image objects, sets the floatVal value as the grey byte value.
// preconditions:	image's grey value must be set
// postconditions:	Sets the floatVal of every pixel in image to that pixel's
//					grey byte value
//
void greyToFloat(Image &image);

// floatToGrey
// For every pixel in an Image object, sets the grey byte value as floatVal.
// preconditions:	image's floatVal must be set
// postconditions:	Sets the grey byte value of every pixel in image to that 
//					pixel's floatVal value
//
void floatToGrey(Image &image);

// convolution
// Performs a convolution of two Image objects and returns a new Image object
// equal to the result. 
// preconditions:	convolution uses floatVal quantity for calculation, img and
//					kernel must have floatVal set to desired value for proper
//					output
// postconditions:	returns an Image object equal to the convolution of img and 
//					kernel
//
Image convolution(const Image &img, const Image &kernel);

// nonMaximaSuppression
// Performs non-maxima suppression on an Image object and returns a new Image
// object equal to the result. 
// preconditions:	nonMaximaSuppression uses floatVal quantity for calculation,
//					gMag, gX and gY must have floatVal set to desired value for 
//					proper output.
//					gMag must be the gradient magnitude of gX and gY.
// postconditions:	retuns an Image object equal to the non-maxima suppression 
//					of gMag.
//
Image nonMaximaSuppression(const Image &gMag, const Image &gX, const Image& gY);

// bilinearInterpolation
// Perfroms bilinear interpolation on a location in an Image object and returns
// a float equal to the result. If x or y are out of image's bounds then the 
// nearest inbound pixel is used for interpolation.
// preconditions:	bilinearInterpolation uses floatVal quantity for calculation,
//					image must have floatVal set to desired value for proper
//					output.
// postconditions:	returns a float equal to the interpolated value of (x, y)
//					in image
//
float bilinearInterpolation(double &x, double &y, const Image &image);

// nearestInboundPixel
// Finds the nearest inbound pixel to loc, with bound representing the upper
// bound of the image. The lower bound is represented by zero. If loc is inbound
// then loc is returned.
// preconditions:	bound must be equal to the number of rows or columns in a 
//					matrix for proper results. 
//					zero must be the lower bound of the matrix
// postconditions:	returns an int equal to the neaest inbound pixel to loc
//
int nearestInboundPixel(int loc, int bound);

// main
// Performs smoothing and edge dection on "test.gif" and returns the results as
// "smooth.gif" and "edges.gif" respectively. The number of smoothing repitions
// performed is equal to the value passed in through the command line. Accepted 
// values are in the ranges: 0 < x < 20. Any out of range values are set to the
// value of DEFAULT_SMOOTHING_REPS.
// preconditions:	argv must contain a numerical value, representing the number
//                  of smoothing repetitions to be performed, must be entered via
//                  the commandline arguments.
//					INPUT_IMAGE_FILE must be set to a valid image located in the
//					same file location as Driver.cpp.
// postconditions:	Two images are created in the same file location as Driver.cpp: 
//					"smooth.gif" which represents the smoothed image of "test.gif,"
//					and "edges.gif" which represents the detected edges in the 
//					smoothed "test.gif" image.
//
int main(int argc, char* argv[]) {	
	Image x_smooth(1, 3);
	x_smooth.setFloat(0, 0, 0.25);
	x_smooth.setFloat(0, 1, 0.5);
	x_smooth.setFloat(0, 2, 0.25);

	Image y_smooth(3, 1);
	y_smooth.setFloat(0, 0, 0.25);
	y_smooth.setFloat(1, 0, 0.5);
	y_smooth.setFloat(2, 0, 0.25);

	Image x_edgeDetect(1, 3);
	x_edgeDetect.setFloat(0, 0, -1.0);
	x_edgeDetect.setFloat(0, 1, 0.0);
	x_edgeDetect.setFloat(0, 2, 1.0);

	Image y_edgeDetect(3, 1);
	y_edgeDetect.setFloat(0, 0, -1.0);
	y_edgeDetect.setFloat(1, 0, 0.0);
	y_edgeDetect.setFloat(2, 0, 1.0);
	
	Image input_img(INPUT_IMAGE_FILE);
	Image output_img(input_img);
	int smoothing_repetitions;

	sscanf_s(argv[1], "%d", &smoothing_repetitions);
	if(smoothing_repetitions < 0 || smoothing_repetitions > MAX_SMOOTHING_REPS) {
		smoothing_repetitions = DEFAULT_SMOOTHING_REPS;
	}

	// set output as float
	greyToFloat(output_img);

	// smooth image
	for(int i = 0; i < smoothing_repetitions; i++) {
		output_img = convolution(output_img, x_smooth);
		output_img = convolution(output_img, y_smooth);
	}

	Image smooth(output_img);
	floatToGrey(smooth);
	smooth.writeGreyImage("smooth.gif");

	// find gradients
	Image gX = convolution(output_img, x_edgeDetect);
	Image gY = convolution(output_img, y_edgeDetect);
	Image gMag(output_img.getRows(), output_img.getCols());

	for(int i = 0; i < gMag.getCols(); i++) {
		for(int j = 0; j < gMag.getRows(); j++) {
			float x_mag2 = pow(gX.getFloat(j, i), 2);
			float y_mag2 = pow(gY.getFloat(j, i), 2);
			float mag = sqrtf((x_mag2 + y_mag2));
			gMag.setFloat(j, i, mag);
		}
	}

	// find edges using non-maxima suppression
	output_img = nonMaximaSuppression(gMag, gX, gY);

	floatToGrey(output_img);
	output_img.writeGreyImage("edge.gif");
	return(0);
}

// greyToFloat
// For every pixel in an Image objects, sets the floatVal value as the grey byte value.
// preconditions:	image's grey value must be set
// postconditions:	Sets the floatVal of every pixel in image to that pixel's
//					grey byte value
//
void greyToFloat(Image &image) {
	for(int i = 0; i < image.getCols(); i++) {
		for(int j = 0; j < image.getRows(); j++) {
			image.setFloat(j, i, image.getPixel(j, i).grey);
		}
	}
}

// floatToGrey
// For every pixel in an Image object, sets the grey byte value as floatVal.
// preconditions:	image's floatVal must be set
// postconditions:	Sets the grey byte value of every pixel in image to that 
//					pixel's floatVal value
//
void floatToGrey(Image &image) {
	for(int i = 0; i < image.getCols(); i++) {
		for(int j = 0; j < image.getRows(); j++) {
			image.setGrey(j, i, static_cast<byte>(image.getPixel(j, i).floatVal));
		}
	}
}

// convolution
// Performs a convolution of two Image objects and returns a new Image object
// equal to the result. 
// preconditions:	convolution uses floatVal quantity for calculation, img and
//					kernel must have floatVal set to desired value for proper
//					output
// postconditions:	returns an Image object equal to the convolution of img and 
//					kernel
//
Image convolution(const Image &img, const Image &kernel) {
	Image out(img.getRows(), img.getCols());
	int kernelCenter_x = kernel.getCols() / 2;
	int kernelCenter_y = kernel.getRows() / 2;
	int flippedCenter_x = kernel.getCols() - 1 - kernelCenter_x;
	int flippedCenter_y = kernel.getRows() - 1 - kernelCenter_y;

	for(int i = img.getCols() - 1; i >= 0; i--) {
		for(int j = img.getRows() - 1; j >= 0; j--) {
			float total = 0;
			for(int m = kernel.getCols() - 1; m >= 0; m--) {
				for(int n = kernel.getRows() - 1; n >= 0; n--) {
					int kernel_x = kernel.getCols() - 1 - m;
					int kernel_y = kernel.getRows() - 1 - n;
					int output_x = nearestInboundPixel(i + (m - flippedCenter_x), img.getCols());
					int output_y = nearestInboundPixel(j + (n - flippedCenter_y), img.getRows());

					// Multiply kernel value by appropriate image value
					// Add result to running total
					total += img.getPixel(output_y, output_x).floatVal * kernel.getPixel(kernel_y, kernel_x).floatVal;
				}
			}
			// Set output image pixel to value of running total	
			out.setFloat(j, i, total);
		}
	}
	return(out);
}

// nonMaximaSuppression
// Performs non-maxima suppression on an Image object and returns a new Image
// object equal to the result. 
// preconditions:	nonMaximaSuppression uses floatVal quantity for calculation,
//					gMag, gX and gY must have floatVal set to desired value for 
//					proper output.
//					gMag must be the gradient magnitude of gX and gY.
// postconditions:	retuns an Image object equal to the non-maxima suppression 
//					of gMag.
//
Image nonMaximaSuppression(const Image &gMag, const Image &gX, const Image& gY) {
	Image output(gMag.getRows(), gMag.getCols());
	for(int i = 0; i < gMag.getCols(); i++) {
		for(int j = 0; j < gMag.getRows(); j++) {
			float g_xVal = (gX.getPixel(j, i).floatVal) / (gMag.getPixel(j, i).floatVal);
			float g_yVal = (gY.getPixel(j, i).floatVal) / (gMag.getPixel(j, i).floatVal);
			double p_xVal = i - g_xVal;
			double p_yVal = j - g_yVal;
			double r_xVal = i + g_xVal;
			double r_yVal = j + g_yVal;
			float p_float = bilinearInterpolation(p_xVal, p_yVal, gMag);
			float r_float = bilinearInterpolation(r_xVal, r_yVal, gMag);

			if(gMag.getPixel(j, i).floatVal >= THRESHOLD && 
			   gMag.getPixel(j, i).floatVal > p_float && 
			   gMag.getPixel(j, i).floatVal > r_float) {
				// edge
				output.setFloat(j, i, 255);
			} else {
				// non-edge
				output.setFloat(j, i, 0);
			}
		}
	}
	return(output);
}

// bilinearInterpolation
// Perfroms bilinear interpolation on a location in an Image object and returns
// a float equal to the result. If x or y are out of image's bounds then the 
// nearest inbound pixel is used for interpolation.
// preconditions:	bilinearInterpolation uses floatVal quantity for calculation,
//					image must have floatVal set to desired value for proper
//					output.
// postconditions:	returns a float equal to the interpolated value of (x, y)
//					in image
//
float bilinearInterpolation(double &x, double &y, const Image &image) {
	float interpolatedValue = 0;
	int x1 = static_cast<int>(floor(x));
	int x2 = static_cast<int>(ceil(x));
	int y1 = static_cast<int>(floor(y));
	int y2 = static_cast<int>(ceil(y));

	if(x >= x2) {
		x2 = static_cast<int>(ceil(x + 1));
	}
	if(y >= y2) {
		y2 = static_cast<int>(ceil(y + 1));
	}

	int closest_x1 = nearestInboundPixel(x1, image.getCols());
	int closest_x2 = nearestInboundPixel(x2, image.getCols());
	int closest_y1 = nearestInboundPixel(y1, image.getRows());
	int closest_y2 = nearestInboundPixel(y2, image.getRows());

	interpolatedValue = static_cast<float>((1 - (x - x1)) * (1 - (y - y1)) * image.getPixel(closest_y1, closest_x1).floatVal +
										   (1 - (x - x1)) * (1 - (y2 - y)) * image.getPixel(closest_y2, closest_x1).floatVal +
										   (1 - (x2 - x)) * (1 - (y - y1)) * image.getPixel(closest_y1, closest_x2).floatVal +
									       (1 - (x2 - x)) * (1 - (y2 - y)) * image.getPixel(closest_y2, closest_x2).floatVal);
	
	return(interpolatedValue);
}

// nearestInboundPixel
// Finds the nearest inbound pixel to loc, with bound representing the upper
// bound of the image. The lower bound is represented by zero. If loc is inbound
// then loc is returned.
// preconditions:	bound must be equal to the number of rows or columns in a 
//					matrix for proper results. 
//					zero must be the lower bound of the matrix
// postconditions:	returns an int equal to the neaest inbound pixel to loc
//
int nearestInboundPixel(int loc, int bound) {
	if(loc < 0) {
		return(0);
	} 
	if(loc >= bound) {
		return(bound - 1);
	}
	return(loc);	
}
#endif
