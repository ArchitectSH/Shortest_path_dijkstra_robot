#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>

/*
 * 0: Raspberry Camera
 * 1: Others 
 */
#define CAM_NUM			0

/* CAP_WIDTH : CAP_HEIGHT == 4 : 3 */
#define CAP_WIDTH		640	// px
#define CAP_HEIGHT		480	// px

#define CAP_INTERVAL	200	// ms
#define CAP_FPS			5	// Frames per Second
#define WAIT_KEY		27	// ESC key

using namespace cv;
using namespace std;
using namespace zbar;

void printUsage(char *argv0){
	cout << "USAGE: " << argv0 << 
		" CAM_NUM" << 
		" CAP_WIDTH(default: " << CAP_WIDTH << ")" <<
		" CAP_HEIGHT(default: " << CAP_HEIGHT << ")" <<
		" CAP_INTERVAL(default: " << CAP_INTERVAL << ")" << endl;
	return;
}

int main(int argc, char* argv[]){
	/* check args */
	int camNum = CAM_NUM;
	int capWidth = CAP_WIDTH;
	int capHeight = CAP_HEIGHT;
	int capInterval = CAP_INTERVAL;
	int capFPS = CAP_FPS;

	switch(argc){
	case 5:
		capInterval = atoi(argv[4]);
		capFPS = (int)(1000.0 / (double)capInterval);
	case 4:
		capHeight = atoi(argv[3]);
		capWidth = atoi(argv[2]);
	case 3:
	case 2:
		 camNum = atoi(argv[1]);
		 break;
	case 1:
	default :
		printUsage(argv[0]);
		return -1;
	}

	/* confirm args */
	cout << "CAM_NUM=" << camNum << ", " <<
		"CAP_WIDTH=" << capWidth << ", " <<
		"CAP_HEIGHT=" << capHeight << ", " <<
		"CAP_INTERVAL=" << capInterval <<  ", " <<
		"CAP_FPS=" << capFPS << endl;

	/* open a video dev	 */
	cout << "Using Camera number " << camNum << endl;
	VideoCapture cap(camNum);
	if (!cap.isOpened()){
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	/* set a capturing resolution */
	cap.set(CV_CAP_PROP_FRAME_WIDTH, capWidth);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, capHeight);
	cap.set(CV_CAP_PROP_FPS, capFPS);

	/* make a QR code scanner */
	ImageScanner scanner; // zbar
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1); // zbar

	/* confirm a capturing resolution */
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "Frame Size: " << dWidth << " * " << dHeight << endl;

	/* 
	 * create a window called "MyVideo" 
	 * this fuction requires GUI
	 */
#ifdef GUI
	namedWindow("MyVideo", CV_WINDOW_AUTOSIZE);
#endif

	while (1){
		/* read a new frame from video */
		Mat frame;
		if (!cap.read(frame)){
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		/* wipe out colors from the frame */
		Mat grey;
		cvtColor(frame, grey, CV_BGR2GRAY);

		/* wrap image data */
		int width = frame.cols;
		int height = frame.rows;
		uchar *raw = (uchar *)grey.data;
		Image image(width, height, "Y800", raw, width * height);

		/* scan the image for barcodes */
		int n = scanner.scan(image); // zbar
		//cout << "scanner.scan(image) == " << n << endl;

		/* extract results */
		for(Image::SymbolIterator symbol = image.symbol_begin();  // zbar
			symbol != image.symbol_end();  // zbar
			++symbol)
		{
			/* do something useful with results */
			static int count = 0;
			count++;
			cout << "Count:\t" << count << ", ";
			/* get symbol's type and data */
			cout << "Decoded " << symbol->get_type_name() <<  // zbar
				" symbol: " << symbol->get_data() << endl; // zbar

			/* get symbol's position and angle */
			vector<Point> vp;
			int m = symbol->get_location_size(); // zbar
			//cout << "symbol->get_location_size() == " << m << endl;
			for(int i = 0; i < m; i++){
				vp.push_back(
					Point(symbol->get_location_x(i), symbol->get_location_y(i))); // zbar
			}

			RotatedRect r = minAreaRect(vp);
			Point2f pts[4];
			r.points(pts);
			for(int i = 0; i < 4; i++){
				line(frame, pts[i], pts[(i + 1) % 4], Scalar(255,0,0), 3);
			}
			cout << "Angle: " << r.angle << endl;
		}

		/* 
		 * show the frame in "MyVideo" window 
		 * this fuction requires GUI
		 */
#ifdef GUI
		imshow("MyVideo", frame); 
#endif

		/* 
		 * wait for 'esc' key press for 10ms. 
		 * If 'esc' key is pressed, break loop 
		 */
		if (waitKey(10) == WAIT_KEY){
			cout << "ESC key is pressed by user" << endl;
			break;
		}
	}
	return 0;
}
