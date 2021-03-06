#include "stdafx.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <time.h>
#include <ctime>
#using <System.dll>

using namespace cv;
using namespace std;

using namespace System;
using namespace System::IO::Ports;

//default capture width and height
const int FRAME_WIDTH = 640;
const int FRAME_HEIGHT = 480;

string intToString(int number) {


	std::stringstream ss;
	ss << number;
	return ss.str();
}


void drawObject(int x, int y, Mat &frame) {

	//use some of the openCV drawing functions to draw crosshairs
	//on your tracked image!

	//UPDATE:JUNE 18TH, 2013
	//added 'if' and 'else' statements to prevent
	//memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

	circle(frame, Point(x, y), 20, Scalar(0, 255, 0), 2);
	if (y - 25>0)
		line(frame, Point(x, y), Point(x, y - 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, 0), Scalar(0, 255, 0), 2);
	if (y + 25<FRAME_HEIGHT)
		line(frame, Point(x, y), Point(x, y + 25), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(x, FRAME_HEIGHT), Scalar(0, 255, 0), 2);
	if (x - 25>0)
		line(frame, Point(x, y), Point(x - 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(0, y), Scalar(0, 255, 0), 2);
	if (x + 25<FRAME_WIDTH)
		line(frame, Point(x, y), Point(x + 25, y), Scalar(0, 255, 0), 2);
	else line(frame, Point(x, y), Point(FRAME_WIDTH, y), Scalar(0, 255, 0), 2);

	putText(frame, intToString(x) + "," + intToString(y), Point(x, y + 30), 1, 1, Scalar(0, 255, 0), 2);

}


void Cam_Adjust(int x, int y)
{
	SerialPort port("COM7", 9600);
	port.Open();

	//Change H to U & L to D - After test 

	if (y < 170)
	{
		cout << "Object too high\n";
		port.Write("U");
	}
	if (y > 310)
	{
		cout << "Object too low\n";
		port.Write("D");
	}

	
	if (x < 250)
	{
		cout << "Go left\n";
		port.Write("L");
	}

	if (x > 390)
	{
		cout << "Go Right\n";
		port.Write("R");
	}
	

	else {}

}

int main(int argc, const char** argv)
{
	rs2::pipeline p;
	rs2::config cfg;

	cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
	p.start(cfg);

	while (1) {

		rs2::frameset frames = p.wait_for_frames();
		rs2::frame ir_frame = frames.first(RS2_STREAM_INFRARED);

		Mat image(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_8UC1, (void*)ir_frame.get_data());


		// prepare cascadeClassifier
		CascadeClassifier person;
		// !! Put your cascade or opencv cascede into project folder !!
		string cascadeName1 = "C:/Users/Rasmus/Desktop/cascade.xml";

		// Load cascade into CascadeClassifier
		bool loaded1 = person.load(cascadeName1);

		// Store results in these 2 vectors
		vector<Rect> human;

		// detect people, more remarks in performace section  
		person.detectMultiScale(image, human, 2, 3, 0, Size(100, 100), Size(2000,2000));

		
		if (human.size() > 0) {

			for (int gg = 0; gg < human.size(); gg++) {
				Point pt1(human[gg].x + human[gg].width/2, human[gg].y + human[gg].height/2);
				//Point pt2(human[gg].rows, human[gg].columns/2);

				//circle(original, human[gg].size()/2, 100, Scalar(0, 0, 255), 2, 8, 0);
				
				rectangle(image, human[gg].tl(), human[gg].br(), Scalar(255, 0, 0), 2, 8, 0);

				int x = pt1.x;
				int y = pt1.y;
				
				drawObject(x, y, image);
				Cam_Adjust(x, y);
			}


		}

		// Add some lines to the image:
		line(image, Point(0, FRAME_HEIGHT / 2 + 70), Point(FRAME_WIDTH, FRAME_HEIGHT / 2 + 70), Scalar(255, 255, 255), 2, 8, 0);
		line(image, Point(0, FRAME_HEIGHT / 2 - 70), Point(FRAME_WIDTH, FRAME_HEIGHT / 2 - 70), Scalar(255, 255, 255), 2, 8, 0);

		line(image, Point(FRAME_WIDTH / 2 + 70, 0), Point(FRAME_WIDTH / 2 + 70, FRAME_HEIGHT), Scalar(255, 255, 255), 2, 8, 0);
		line(image, Point(FRAME_WIDTH / 2 - 70, 0), Point(FRAME_WIDTH / 2 - 70, FRAME_HEIGHT), Scalar(255, 255, 255), 2, 8, 0);

		

		imshow("Display", image);
		waitKey(10);
	}

	return 0;
}

