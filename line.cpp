#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <cstdio>
#include <vector>
#include <cmath>

/*
 * 0: Raspberry Camera
 * 1: Others 
 */
#define CAM_NUM			0

/* 
 * if brightness > BRIGHTNESS_ADJ * brightness_average
 * is FLOOR(255), else LINE(0)
 */
#define BRIGHTNESS_ADJ	0.45

#define LINE			0
#define FLOOR			255

#define SMALL_WIDTH		32
#define SMALL_HEIGHT	24

/* CAP_WIDTH : CAP_HEIGHT == 4 : 3 */
#define CAP_WIDTH		640	// px
#define CAP_HEIGHT		480	// px

#define CAP_CONSTRAST	0.75
#define CAP_FPS			2.0	// Frames per Second
#define WAIT_KEY		27	// ESC key

#define PI 3.141592

using namespace cv;
using namespace std;

enum Direction{
	TOP = 0,
	BOTTOM,
	LEFT, 
	RIGHT,
	END
};

class Pattern{
public:
	int size(){ return this->n; }
	uchar *data(){ return this->array; }
	Direction direction(){ return this->dir; }

	Point* toPoint(int p){
		Point *ret = NULL;
		if(p < 0 || p >= this->n) return NULL;

		switch(this->dir){
		case TOP: 		ret = new Point(p, 0); break;
		case BOTTOM: 	ret = new Point(p, this->n / 4 * 3 - 1); break;
		case LEFT: 		ret = new Point(0, p); break;
		case RIGHT: 	ret = new Point(this->n / 3 * 4 - 1, p); break;
		default: 		ret = NULL; break;
		}
		//cout << "ret:" << (void*) ret << endl;
		return ret;
	}

	void set(uchar *array, int n, Direction dir){
		this->n = n;
		if(this->array != NULL) delete this->array;
		this->array = new uchar[n];
		for(int i = 0; i < n; i++) this->array[i] = array[i];
		this->dir = dir;
	}

	Pattern(){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
	}
	Pattern(uchar* array, int n, Direction dir){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
		this->set(array, n, dir);
	}
	Pattern(Pattern *p){
		this->n = 0;
		this->array = NULL;
		this->dir = END;
		this->set(p->data(), p->size(), p->direction());
	}
	~Pattern(){
		if(this->array != NULL) delete this->array;
	}

private:
	int n;
	uchar *array;
	Direction dir;
};

void printUsage(const char *argv0){
	cout << "USAGE: " << argv0 << 
		" CAM_NUM" << 
		" CAP_WIDTH(default: " << CAP_WIDTH << ")" <<
		" CAP_HEIGHT(default: " << CAP_HEIGHT << ")" <<
		" CAP_FPS(default: " << CAP_FPS << ")" << endl;
	return;
}

void printData(Mat m){
	int width = m.cols;
	int height = m.rows;
	uchar *data = (uchar *)m.data;
	
	cout << "w: " << width << ", " << "h: " << height << endl;
	for(int y = 0; y < height; y+=1){
		printf("%2d ", y);
		for(int x = 0; x < width; x+=1){
			int temp = data[y * width + x];
			char output;

			if(temp < 50) output = '#';
			else if(temp < 100) output = '+';
			else if(temp < 150) output = '-';
			else if(temp < 200) output = '.';
			else output = ' ';

			cout << output << " ";
		}
		cout << endl;
	}
	cout << "----------------------------------------" << endl;
	return;
}

Mat quantizeMat(Mat m, double adj = BRIGHTNESS_ADJ){
	int width = m.cols;
	int height = m.rows;
	Mat ret = m;
	uchar *data = (uchar *)ret.data;

	/* get brightness average*/
	int sum = 0;
	double avg = 0.0;
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			sum += data[i * width + j];
		}
	}
	avg = (double)sum / (height * width);
	cout << "average: " << avg << endl;

	/* get pixel which has brightness above average */
	for(int i = 0; i < height; i++){
		for(int j = 0; j < width; j++){
			if(data[i * width + j] > avg * adj)
				data[i * width + j] = FLOOR;
			else
				data[i * width + j] = LINE;
		}
	}

	return ret;
}

int findLongestPattern(Pattern *p, int *end){
	int ret = 0;
	uchar *arr = p->data();
	int n = p->size();

	int maxStart = -1;
	int maxLen = 0;
	int maxEnd = -1; 
	for(int i = 0; i < n; i++){
		/* start */
		if(maxStart == -1 && arr[i] == LINE) maxStart = i;
		/* end */
		else if(maxStart != -1 && (arr[i] == FLOOR || i == n - 1)){
			int len = i - maxStart;
			/* update maxLen */
			if(maxLen < len){
				maxLen = len;
				maxEnd = i;

				ret = maxLen;
				*end = maxEnd;
			}			
			/* ready to restart */
			maxStart = -1;
		}
		/* now is searching... */
		else if(maxStart == -1 && arr[i] == FLOOR) continue;
		/* now is measuring... */
		else if(maxStart != -1 && arr[i] == LINE) continue;
		
	}
	return ret;
}

int isConnected(vector<Point> vp, Mat m){
	if(vp.size() < 2) return -1;

	int ret = 1;
	int width = m.cols;
	int height = m.rows;
	uchar *data = (uchar *)m.data;

	Point start, end;
	start = vp.back(); vp.pop_back();
	end = vp.back(); vp.pop_back();
	int nDivide = 3;
	double dx = (end.x - start.x) / (double)nDivide;
	double dy = (end.y - start.y) / (double)nDivide;

	for(int i = 1; i < nDivide; i++){
		int idx = (int)(start.y + dy * i) * width + (int)(start.x + dx * i);
		//cout << "dy: " << idx / width << ", dx: " << idx % width << endl;
		if(data[idx] != LINE){
			ret = 0;
			break;
		}
	}

	vp.push_back(end);
	vp.push_back(start);

	return ret;
}

vector<Point> detectLine(Mat grey){
	vector<Point> ret;
	int width = grey.cols;
	int height = grey.rows;
	Mat m = quantizeMat(grey);
	uchar *data = (uchar *)m.data;
	
	/* create each edge's Pattern from frame */
	uchar *topArray		= &data[0 * width 				+ 0];
	uchar *bottomArray	= &data[(height - 1) * width	+ 0];
	uchar *leftArray	= new uchar[height];
	uchar *rightArray	= new uchar[height];
	for(int i = 0; i < height; i++){
		leftArray[i] = data[i * width + 0]; 
		rightArray[i] = data[i * width + (width - 1)]; 
	}
	Pattern *top 	= new Pattern(topArray, width, Direction(TOP));
	Pattern *bottom = new Pattern(bottomArray, width, Direction(BOTTOM));
	Pattern *left 	= new Pattern(leftArray, height, Direction(LEFT));
	Pattern *right 	= new Pattern(rightArray, height, Direction(RIGHT));
	Pattern *pn[4] = { left, right, top, bottom };

	for(int i = 0; i < 4 && ret.size() < 2; i++){
		int maxLen[2], maxEnd[2], mid[2];
		int j;
		/* find logest and continuous pattern from each edges of frame */
		maxLen[0] = findLongestPattern(pn[i], &maxEnd[0]);
		/* push back middle point of the logest and continuous pattern */
		if(maxLen[0] == 0) continue;
		mid[0] = maxEnd[0] - maxLen[0] / 2;

		for(j = i + 1; j < 4 && ret.size() < 2; j++){
			maxLen[1] = findLongestPattern(pn[j], &maxEnd[1]); 
			if(maxLen[1] == 0) continue;
			mid[1] = maxEnd[1] - maxLen[1] / 2;
			
			Point *pt[2] = { pn[i]->toPoint(mid[0]), pn[j]->toPoint(mid[1]) };
			if(pt[0] == NULL || pt[1] == NULL) break;
			
			ret.push_back(Point(pt[0]->x, pt[0]->y));
			ret.push_back(Point(pt[1]->x, pt[1]->y));
			if(!isConnected(ret, m)){
				delete pt[1]; ret.pop_back();
				delete pt[0]; ret.pop_back();
			}
		}
	}

	/* print */
	printData(m);

	/* release */
	delete leftArray;
	delete rightArray;
	delete top;
	delete bottom;
	delete left;
	delete right;
	m.release();

	return ret;
}

double calcAngle(vector<Point> vp){
	int vpSize = vp.size(); // *CAUTION* vp.size() returns a unsigned value
	double ret = 0.0;
	int count = 0;
	for(int i = 0; i < vpSize -1; i+=2){
		Point d = vp[i + 1] - vp[i];
		cout << "y:" << d.y << "x:" << d.x << endl;
		//cout << "dx: " << d.x << ", dy: " << d.y << endl;
		ret = asin(d.y / sqrt(d.x * d.x + d.y * d.y)) * 180.0 / PI;
		cout << "angle: " << ret << endl;
		count++;
	}
	cout << "count = " << count << endl;
	return ret;
}

int main(int argc, char* argv[]){
	/* check args */
	int camNum = CAM_NUM;
	int capWidth = CAP_WIDTH;
	int capHeight = CAP_HEIGHT;
	double capFPS = CAP_FPS;

	switch(argc){
	case 5:
		capFPS = (double)atoi(argv[4]);
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
		"CAP_FPS=" << capFPS << endl;

	/* open a video dev	 */
	cout << "Using Camera number " << camNum << endl;
	VideoCapture cap(camNum);
	if (!cap.isOpened()){
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	/* set a capturing resolution */
	cap.set(CV_CAP_PROP_CONTRAST, CAP_CONSTRAST);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, capWidth);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, capHeight);
	cap.set(CV_CAP_PROP_FPS, capFPS);

	/* confirm a capturing resolution */
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "Frame Size: " << dWidth << " * " << dHeight << endl;

	/* 
	 * create a window called "MyVideo" 
	 * this fuction requires GUI
	 */
#ifdef GUI
	namedWindow("INPUT", CV_WINDOW_AUTOSIZE);
	namedWindow("OUTPUT", CV_WINDOW_AUTOSIZE);
#endif

	while (1){
		/* read a new frame from video */
		Mat frame;
		if (!cap.read(frame)){
			cout << "Cannot read a frame from video stream" << endl;
			break;
		}

		/* resize and wipe out colors from the frame */
		Mat small(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC3);
		Mat grey(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC1);
		resize(frame, small, Size(SMALL_WIDTH, SMALL_HEIGHT), 0, 0, 0);
		cvtColor(small, grey, CV_BGR2GRAY);
		// small Mat will be unnecessary any more

		/* detect lines */
		vector<Point> vp = detectLine(grey);

		/* calculate angles */
		double angle = calcAngle(vp);

#ifdef GUI
		/* draw lines */
		cout << "vp.size(): " << vp.size() << endl;
		for(int i = 0; i < (int)vp.size() - 1; i+=2){
			line(small, vp[i], vp[i + 1], Scalar(0, 0, 255), 2);
		}
		
		/* 
		 * show the frame in "MyVideo" window 
		 * this fuction requires GUI
		 */
		Mat output(capWidth, capHeight, CV_8UC3);
		resize(small, output, Size(capWidth, capHeight), 0, 0, 0);
		imshow("INPUT", frame); 
		imshow("OUTPUT", output); 
		output.release();
#endif

		/* 
		 * wait for 'esc' key press for 10ms. 
		 * If 'esc' key is pressed, break loop 
		 */
		if (waitKey((int)(1.0 / capFPS * 1000.0) * 0.8) == WAIT_KEY){
			cout << "ESC key is pressed by user" << endl;
			break;
		}

		/* release */
		frame.release();
		small.release();
		grey.release();
	}

	return 0;
}
