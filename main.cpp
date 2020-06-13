#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>
#include <cmath>

#include <stdlib.h>

#include <fcntl.h>
#include <unistd.h>
#include <sys/wait.h>

#include "lmstypes.h"
#include "bytecodes.h"

#define FALSE			0
#define TRUE			1

//CONTROL///////////////////////////////////////////////////////////////////////
#define EV3_DEV_NAME "/dev/hidraw0"
#define MAX_BUF_LEN		128
#define MAGIC_KEY		0xFA

#define PORT_MOTOR_L 	1	// port A
#define PORT_MOTOR_R 	2	// port B
#define SPEED_MOTOR_MAX	100
#define SPEED_MOTOR_MIN (-100)

#define SPEED_MOTOR_SAT 12

#define CONNECTED_NODES 4
#define NODES 8

char get_key(void) {
	char key = '\0';
	char line[MAX_BUF_LEN] = {0, };
	printf(">> ");
	scanf("%s", line);
	return line[0];
}

void print_buf(const char *heading, char *p, int len) {
	int i;
	printf("%s", heading);
	for (i = 0; i < len; i++) {
		printf(" %02x", p[i]);
		if (((i+1) % 25) == 0) printf("\n%s", heading);
	}
	//printf("\n");
	printf(" (len: %d)\n", len);
}

int get_command_length(char *p, int maxlen) {
	int i;
	//for (i = 0; i < maxlen; i++) {
	for (i = 2; i < maxlen; i++) {
		//if (p[i] == opOBJECT_ENDY)
		if (p[i] == opOBJECT_END && p[i + 1] == MAGIC_KEY)
			//return i;
			return i + 1;
	}
	return 0;
}

char direct_reply_buf[MAX_BUF_LEN] = {0, };
int EV3_send_command(int fd, char *direct_command_buf, int reply_needed) {
	int n;
	n = get_command_length(direct_command_buf, MAX_BUF_LEN);
	//direct_command_buf[1] = n / 256;
	//direct_command_buf[0] = n - direct_command_buf[1] * 256;
	direct_command_buf[1] = (n - 2) / 256;
	direct_command_buf[0] = (n - 2) - direct_command_buf[1] * 256;
	//print_buf("[COMMAND]", direct_command_buf, n);
	//if(write(fd, direct_command_buf, n+1) < 0) {
	if(write(fd, direct_command_buf, n) < 0) {
		printf("write error!\n");
		return -1;
	}
	memset(direct_reply_buf, 0, sizeof(direct_reply_buf));
	if (reply_needed == FALSE)
		return 0;

	/* if reply needed */
	if((n = read(fd, direct_reply_buf, sizeof(direct_reply_buf))) < 0) {
		printf("read error!\n");
		return -1;
	}
	if (n > 0)
		n = direct_reply_buf[0] + ((int) direct_reply_buf[1]) * 256;
	print_buf("[ REPLY ]", direct_reply_buf, n + 2);
	return 1;
} 

char motor_command_buf[MAX_BUF_LEN] = {
	//0x00, 0x00, // command size
	0x3C, 0x00, // command size
	0xFF, 0xFF, // sequence counter
	0x00, 		// type of command : DIRECT_COMMAND_REPLY
	//0x01, 0x00, // message header
	0x00, 0x00, // message header

	/* byte codes */
	opOUTPUT_POWER, 
	LC0(0), 			// layer
	LC0(PORT_MOTOR_L),	// port 
	LC1(0),				// speed

	opOUTPUT_POWER, 
	LC0(0),				// layer 
	LC0(PORT_MOTOR_R), 	// port
	LC1(0),				// speed

	opOUTPUT_START, 
	LC0(0), 
	LC0(PORT_MOTOR_L + PORT_MOTOR_R),

	//opOBJECT_END
	opOBJECT_END,
	MAGIC_KEY
};

int update_motor_speed(int fd, int speed_left, int speed_right){
	int ret = TRUE;
	
	if(speed_left >= SPEED_MOTOR_MAX || speed_right >= SPEED_MOTOR_MAX 
		|| speed_left <= SPEED_MOTOR_MIN || speed_right <= SPEED_MOTOR_MIN)
		return FALSE;
	motor_command_buf[11] = speed_left;
	motor_command_buf[16] = speed_right;
	EV3_send_command(fd, motor_command_buf, FALSE);
	//printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
	
	return ret;
}

//LINE//////////////////////////////////////////////////////////////////////////
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

/* SMALL_WIDTH : SMALL_HEIGHT == 4 : 3 */
#define SMALL_WIDTH		32
#define SMALL_HEIGHT	24

/* CAP_WIDTH : CAP_HEIGHT == 4 : 3 */
#define CAP_WIDTH		640	// px
#define CAP_HEIGHT		480	// px

#define CAP_CONSTRAST	0.75
#define CAP_FPS			4.0	// Frames per Second
#define WAIT_KEY		27	// ESC key

using namespace cv;
using namespace std;
using namespace zbar;

enum Direction{
	TOP = 0,
	BOTTOM,
	LEFT, 
	RIGHT,
	END
};

///////////////////////////////////////////////////////////////////////////
VideoCapture cap(CAM_NUM);
ImageScanner scanner;

typedef struct NextNode{
	int nextAngle;
	int nextNode;
	int weight;
}_nextNode;
typedef struct ConnectedNode{
	int before;
	NextNode nextNodes[CONNECTED_NODES - 1];
}_connected;
typedef struct Node{
	int nodeNum;
	ConnectedNode connectedNodes[CONNECTED_NODES];
}_node;

int W[NODES+1][NODES+1];
#define DOCKING_SPEED_LEFT 12
#define DOCKING_SPEED_RIGHT 12
#define REVISION_SAT 15
enum STATE{
	FOLLOW_LINE = 0,
	DOCKING,
	UNDOCKING
};

typedef struct edge{
	int src;
	int dest;
}_edge;
edge F[NODES - 1];
///////////////////////////////////////////////////////////////////////////


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
	cout << endl;
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
	//cout << "average: " << avg << endl;

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

	//cout << "p->size : " << p->size() << endl;

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
				
				/* return values */
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
	if (ret < 4){
		ret = 0;
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

	//cout << "start.x : " << start.x << ", end.x : " << end.x << endl;
	//cout << "start.y : " << start.x << ", end.y : " << end.y << endl;
	//cout << "dy: " << dy << ", dx: " << dx << endl;

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
	//uchar *topArray		= &data[0 * width 				+ 0];
	//uchar *bottomArray	= &data[(height - 1) * width	+ 0];
	uchar *leftArray	= new uchar[height];
	uchar *rightArray	= new uchar[height];
	for(int i = 0; i < height; i++){
		leftArray[i] = data[i * width + 0];
		//cout << i << " left : " << (unsigned int) leftArray[i] << "  ";
		rightArray[i] = data[i * width + (width - 1)]; 
		//cout << " right : " << (unsigned int) rightArray[i] << endl;
	}
	//Pattern *top 	= new Pattern(topArray, width, Direction(TOP));
	//Pattern *bottom = new Pattern(bottomArray, width, Direction(BOTTOM));
	Pattern *left 	= new Pattern(leftArray, height, Direction(LEFT));
	Pattern *right 	= new Pattern(rightArray, height, Direction(RIGHT));
	//Pattern *pn[4] = { left, right, top, bottom };
	Pattern *pn[2] = {left, right};

	for (int i = 0; i < 2 && ret.size() < 2; i++){
		int maxLen[2], maxEnd[2], mid[2];
		int j;
		/* find logest and continuous pattern from each edges of frame */
		maxLen[0] = findLongestPattern(pn[i], &maxEnd[0]);
		/* push back middle point of the logest and continuous pattern */
		if (maxLen[0] == 0) continue;
		mid[0] = maxEnd[0] - maxLen[0] / 2;

		for (j = i + 1; j < 2 && ret.size() < 2; j++){
			maxLen[1] = findLongestPattern(pn[j], &maxEnd[1]);
			if (maxLen[1] == 0) continue;
			mid[1] = maxEnd[1] - maxLen[1] / 2;

			Point *pt[2] = { pn[i]->toPoint(mid[0]), pn[j]->toPoint(mid[1]) };
			if (pt[0] == NULL || pt[1] == NULL) break;

			ret.push_back(Point(pt[0]->x, pt[0]->y));
			ret.push_back(Point(pt[1]->x, pt[1]->y));
			if (!isConnected(ret, m)){
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
	//delete top;
	//delete bottom;
	delete left;
	delete right;
	m.release();

	return ret;
}

double calcAngle(vector<Point> vp){
	int vpSize = vp.size(); // *CAUTION* vp.size() returns a unsigned value
	double ret = 0.0;

	for(int i = 0; i < vpSize -1; i+=2){
		Point d = vp[i + 1] - vp[i];
		//cout << "dx: " << d.x << ", dy: " << d.y << endl;
		ret = asin(d.y / sqrt(d.x * d.x + d.y * d.y)) * 180.0 / CV_PI;
		//cout << "angle: " << ret << endl;
		break;
	}
	return ret;
}

Node decodeQR(string s){
	char temp[20];
	int i = 0, j = 0;
	int step = 0;
	Node node;
	void *p = 0;

	/*initialize*/
	for (int i = 0; i < CONNECTED_NODES; i++){
		ConnectedNode connectedN;
		connectedN.before = 0;
		for (int j = 0; j < CONNECTED_NODES - 1; j++){
			NextNode nextN;
			nextN.nextAngle = nextN.nextNode = nextN.weight = 0;
			connectedN.nextNodes[j] = nextN;
		}
		node.connectedNodes[i] = connectedN;
	}

	int connectedNodeCount = 0;
	int nextNodeCount = 0;
	while (1){
		int j;
		for (j = 0; s[i] != '\n' && s[i] != '\0'; i++, j++){
			temp[j] = s[i];
		}
		if (s[i] == '\0'){
			break;
		}
		i++;
		if (s[i] == '\0'){
			break;
		}
		temp[j] = '\0';
		//cout << temp << "(1)" << endl;
		switch (step){
		case(0) : node.nodeNum = atoi(temp); 
			step++;
			break;
		case(1) : node.connectedNodes[connectedNodeCount].before = atoi(temp);
			nextNodeCount = 0;
			step++;
			break;
		case(2) :
			int tried = 0;
			while (1){
				switch (tried){
				case 0:node.connectedNodes[connectedNodeCount].nextNodes[nextNodeCount].nextAngle = atoi(temp); break;
				case 1:node.connectedNodes[connectedNodeCount].nextNodes[nextNodeCount].nextNode = atoi(temp); break;
				case 2:node.connectedNodes[connectedNodeCount].nextNodes[nextNodeCount].weight = atoi(temp); nextNodeCount++;
				}
				if (tried == 2){
					if(s[i] == '\0'){
						break;
					}
					else if (s[i]!='0' && s[i + 1] == '\n' || s[i + 1] == '\0'){
						step = 1;
						connectedNodeCount++;
						break;
					}
					tried = -1;
				}
				tried++;
				for (j = 0; s[i] != '\n' && s[i] != '\0'; i++, j++){
					temp[j] = s[i];
				}
				i++;
				temp[j] = '\0';
				//cout << temp << "(2)" << endl;
			}
		}
	}
	
	/*cout << "nodeNum: " << node.nodeNum << endl;
	for (int i = 0; node.connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
		cout << "before: " << node.connectedNodes[i].before << endl;
		for (int j = 0; node.connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES-1; j++){
			cout << "nextAngle: " << node.connectedNodes[i].nextNodes[j].nextAngle << endl;
			cout << "nextNode: " << node.connectedNodes[i].nextNodes[j].nextNode << endl;
			cout << "weight: " << node.connectedNodes[i].nextNodes[j].weight << endl;
		}
	}
	cout << "///////////////////////////////////////////////////////////////////" << endl; */
	return node;
}

double calcPosition(Mat m, int state, double adj = BRIGHTNESS_ADJ){
	int width = m.cols;
	int height = m.rows;
	uchar *data = (uchar *)m.data;

	double ret=0;
	int sum = 0;
	double avg = 0.0;
	for (int i = 0; i < height; i++){
		for (int j = 0; j < width; j++){
			sum += data[i * width + j];
		}
	}
	avg = (double)sum / (height * width);

	int i;
	int maxWidth = width;
	switch (state){
		case STATE(FOLLOW_LINE): i = 0; maxWidth = width; break;
		case STATE(DOCKING): i = 0; maxWidth = width / 2;  break;
		case STATE(UNDOCKING): i = width / 2; maxWidth = width; break;
	}


	int startY =-1, lastY = -1;
	int startX = -1, lastX = -1;
	int flag = 0;
	int too = 0;

	int connected = 0;
	int maxLen = 0;
	int maxStartY = 0, maxLastY = 0;
	int maxStartX = 0, maxLastX = 0;
	int len = 0;

	int j;
	for (; i < maxWidth; i++){
		for (j = 0; j < height - 2; j++){
			if (data[j * width + i] <= avg * adj && data[(j + 1)*width + i] <= avg * adj && data[(j + 2)*width + i] <= avg * adj){
				len++;
				if (connected == 0){
					startX = i;
					startY = j + 2;
					connected = 1;
				}
				else{
					lastX = i;
					lastY = j + 2;
				}
				break;
			}
		}
		if (j == height - 2){
			connected = 0;
			len = 0;
			startY = -1;
			lastY = -1;
			startX = -1;
			lastX = -1;
		}
		if (len > maxLen && len >= 3){
			//cout << "x : " << startX <<" ";
			maxLen = len;
			maxStartY = startY;
			maxLastY = lastY;
			maxStartX = startX;
			maxLastX = lastX;
		}
	}
	//cout << endl;

	int y = maxStartY - maxLastY, x = maxLastX - maxStartX;

	cout << "maxLen : " << maxLen << " maxstartX: " << maxStartX << ", maxlastX : " << maxLastX << " maxstartY: " << maxStartY << ", maxlastY : " << maxLastY << endl;

	int mid = (maxStartY + maxLastY) / 2;
	if (maxStartY == 0 || maxLastY == 0){
		ret = 0;
	}
	else if (abs(maxStartY - maxLastY) <= 3){
		int topBoundary = height / 2 - 4;
		int bottomBoundary = height / 2 + 4;

		if (mid >= topBoundary && mid <= bottomBoundary){
			//cout << "<1>";
			ret = 0;
		}
		else if (mid < topBoundary){ //(-)shoud go left
			//cout << "<2>";
			ret = -(topBoundary - mid)*1.5;
		}
		else{ //(+)shoud go right
			//cout << "<3>";
			ret = (mid - bottomBoundary)*1.5;
		}
	}else{
		ret = asin(y / sqrt(x * x + y * y)) * 180.0 / CV_PI;
		ret = -ret;
	}


	cout << "position(" << ret << ")" << endl;


	if (ret > 20 || ret < -20 ){ //too much
		ret = 0;
	}

	return ret;
}

int reviseSpeed(Mat mat, int state){
	//- => go left, + => go right
	int position = calcPosition(mat, state);

	int speed_delta = (int)(REVISION_SAT * (position/90.0) * 5);

	return speed_delta;
}

int searchQR(Node& node){
	Mat frame;
	if (!cap.read(frame)){
		cout << "Cannot read a frame from video stream" << endl;
		return -1;
	}

	Mat grey;
	cvtColor(frame, grey, CV_BGR2GRAY);

	int width = frame.cols;
	int height = frame.rows;
	uchar *raw = (uchar *)grey.data;
	Image image(width, height, "Y800", raw, width * height);

	int n = scanner.scan(image);
	//cout << "scanner.scan(image) == " << n << endl;

	frame.release();
	grey.release();

	for (Image::SymbolIterator symbol = image.symbol_begin();  // zbar
		symbol != image.symbol_end();  // zbar
		++symbol)
	{
		cout << "found new QR code." << endl;
		node = decodeQR(symbol->get_data());
		return n;
	}
	return n;
}
Mat normalizeMat(){
	/* read a new frame from video */
	Mat frame;
	if (!cap.read(frame)){
		cout << "Cannot read a frame from video stream" << endl;
		return frame;
	}//

	/* resize and wipe out colors from the frame */
	Mat small(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC3);
	Mat grey(SMALL_WIDTH, SMALL_HEIGHT, CV_8UC1);
	resize(frame, small, Size(SMALL_WIDTH, SMALL_HEIGHT), 0, 0, 0);
	cvtColor(small, grey, CV_BGR2GRAY);
	// small Mat will be unnecessary any more
	//frame.release();
	//small.release();

	return grey;
}

void followLine(int fd){
	int speed_left, speed_right;

	cout << "follow Line........." << endl;

	double acceleration = 1;
	while (1){
		Mat grey = normalizeMat();

		/* detect lines */
		vector<Point> vp = detectLine(grey);
		int vpSize = (int)vp.size();
		
		/* starting condition */
		if (speed_left == 0 && speed_right == 0 && vpSize == 0){
			//update_motor_speed(fd, SPEED_MOTOR_SAT, SPEED_MOTOR_SAT);
			continue;
		}
		
		/* control the EV3 */
		if (vpSize != 0){
			/* calculate angles */
			double angle = calcAngle(vp);
			cout << "angle: " << angle << endl;

			/* modify heading  */
			int speed_delta = (int)(SPEED_MOTOR_SAT * (angle / 90.0) * 5);
			//cout << "speed_delta: " << speed_delta << endl;

			speed_right = SPEED_MOTOR_SAT - speed_delta;
			speed_left = SPEED_MOTOR_SAT + speed_delta;
			
			
			//cout << "calcPosition : " << calcPosition(grey, STATE(FOLLOW_LINE)) << endl;
			int position = calcPosition(grey, STATE(FOLLOW_LINE));
			if (position == 0){
				if (acceleration < 2){
					acceleration += 0.1;
				}

				speed_right *= acceleration;
				speed_left *= acceleration;
			}
			else{
				speed_delta = reviseSpeed(grey, STATE(FOLLOW_LINE));
				speed_right = SPEED_MOTOR_SAT - speed_delta;
				speed_left = SPEED_MOTOR_SAT + speed_delta;
			}

			printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
			update_motor_speed(fd, speed_left, speed_right);
		}
		else{
			break;
		}
	}
}

Node docking(int fd){
	int speed_left, speed_right;

	Node node;
	while (1){
		int n = searchQR(node);

		//cout << endl << "========================================================" << endl;
		cout << "docking...... " << endl;

		if (n){
			/*int n;
			do{
				Node node;
				update_motor_speed(fd, speed_left, speed_right);
				n = searchQR(node);
			} while (n);*/
			update_motor_speed(fd, 0, 0);

			return node;
		}
		else{
			Mat grey = normalizeMat();
			printData(grey);
			int speed_delta = reviseSpeed(grey, STATE(DOCKING));
			if (speed_delta == 0){
				speed_right = DOCKING_SPEED_RIGHT;
				speed_left = DOCKING_SPEED_LEFT;
			}
			else{
				speed_right = REVISION_SAT - speed_delta;
				speed_left = REVISION_SAT + speed_delta;
			}

			printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
			update_motor_speed(fd, speed_left, speed_right);

			grey.release();
		}
	}
}

void undocking(int fd){
	int speed_left, speed_right;

	int count = 0;
	while (1){
		Mat grey = normalizeMat();
		/* detect lines */
		vector<Point> vp = detectLine(grey);
		int vpSize = (int)vp.size();

		//cout << endl << "<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>" << endl;
		cout << "undocking......" << endl;

		if (vpSize != 0){
			count++;
			if (count > 2){
				break;
			}
		}

		int speed_delta = reviseSpeed(grey, STATE(UNDOCKING));

		if (speed_delta == 0){
			speed_right = DOCKING_SPEED_RIGHT;
			speed_left = DOCKING_SPEED_LEFT;
		}
		else{
			speed_right = REVISION_SAT - speed_delta;
			speed_left = REVISION_SAT + speed_delta;
		}
		
		printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
		update_motor_speed(fd, speed_left, speed_right);
	}
}



void goNext(int fd, NextNode next){
	int speed_right, speed_left;
	if (next.nextAngle == 90){
		/* turn right */
		speed_right = -63;
		speed_left = -23;
		for (int i = 0; i < 1500; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}

		speed_right = 20;
		speed_left = 20;
		for (int i = 0; i < 500; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}
	}
	else if(next.nextAngle==-90){
		/* turn left */
		speed_left = 28;
		speed_right = 63;
		for (int i = 0; i < 1800; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}

		speed_right = 20;
		speed_left = 20;
		for (int i = 0; i < 500; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}
	}
	else if (next.nextAngle == 180){
		/* turn back */
		speed_right = 63;
		speed_left = 26;
		for (int i = 0; i < 3000; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}

		speed_right = 20;
		speed_left = 20;
		for (int i = 0; i < 500; i++){
			update_motor_speed(fd, speed_left, speed_right);
		}
	}

	undocking(fd);
}
void calcRoutes(int src, int dest, edge* routes, int& routeCount) {
	//initialize
	for (int i = 0; i < NODES - 1; i++){
		F[i].src = F[i].dest = 0;
	}

	/* execute dijkstra*/
	int i, vnear; edge e;
	int touch[NODES + 1];
	int length[NODES + 1];
	int edges = 0;

	//cout << src << " / " << dest << endl;

	for (i = 1; i <= NODES; i++) { 
		if (i != src){
			touch[i] = src; 
			length[i] = W[src][i]; 
		}
	} 

	//cout << "init" << endl;

	int repeat = 1;
	int min;
	while (repeat < NODES) { 
		min = 999;
		for (i = 1; i <= NODES; i++) 
			if (i!=src && 0 <= length[i] && length[i] <= min) {
				min = length[i];
				vnear = i;
			}
		//cout << "vnear : " << vnear << endl;

		e.src = touch[vnear];
		e.dest = vnear;
		F[edges] = e;
		edges++;
		for (i = 1; i <= NODES; i++){
			if (i!=src && length[vnear] + W[vnear][i] < length[i]) {
				length[i] = length[vnear] + W[vnear][i]; touch[i] = vnear; 
			}
		}

		length[vnear] = -1;
		repeat++;
	}

	//cout << "dijkstra has been completed" << endl;

	/* sort the routes */
	routeCount = 0;
	for (int i = 0; i < NODES - 1 && F[i].src != 0; i++){
		if (F[i].dest == dest){
			e = F[i];
			routes[routeCount] = e;
			routeCount++;
			break;
		}
	}
	while (e.src != src){
		for (int i = 0; i < NODES - 1 && F[i].src != 0; i++){
			if (F[i].dest == e.src){
				e = F[i];
				routes[routeCount] = e;
				routeCount++;
				break;
			}
		}
	}
	/*for (int i = 0; i < routeCount; i++){
		printf("%d-%d(%d)\n", routes[i].src, routes[i].dest, W[routes[i].src][routes[i].dest]);
	}
	getchar();*/
}

int calcSkip(int before, Node* nodes, edge* routes, int routeIndex){ //if src->dest angle is 0, return 1
	edge e = routes[routeIndex];
	int ret = 0;

	//cout << " before : " << before << endl;

	if (routeIndex == -1){
		return 0;
	}

	//cout << e.src << "-" << e.dest << endl;

	for (int i = 0; nodes[e.src].connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
		if (nodes[e.src].connectedNodes[i].before == before){
			for (int j = 0; nodes[e.src].connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1; j++){
				if (nodes[e.src].connectedNodes[i].nextNodes[j].nextNode == e.dest && nodes[e.src].connectedNodes[i].nextNodes[j].nextAngle==0){
					ret = 1;
				}
			}
		}
	}

	if (ret == 0){
		return ret;
	}
	else{
		return calcSkip(e.src, nodes, routes, routeIndex - 1) + ret;
	}
}


void travelRoutes(int fd, int& beforeNode, Node* nodes, edge* routes, int routeCount){
	while (routeCount>0){
		int skipCount = calcSkip(routes[routeCount-1].src, nodes, routes, routeCount-2);
		//cout << "skipCount : " << skipCount << endl;
		//getchar();

		if (skipCount >= 1){
			int routeIndex = routeCount - 1;
			NextNode next;
			for (int i = 0; nodes[routes[routeIndex].src].connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
				if (nodes[routes[routeIndex].src].connectedNodes[i].before == beforeNode){
					for (int j = 0; nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1; j++){
						if (nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j].nextNode == routes[routeIndex].dest){
							next = nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j];
						}
					}
				}
			}

			//cout << next.nextAngle << " / " << next.nextNode << " / " << next.weight << endl;
			//update_motor_speed(fd, 0, 0);
			//getchar();
			goNext(fd, next);

			int speed_left, speed_right;
			double acceleration = 1;
			for (int i = 0; i <= skipCount; i++){
				while (1){
					Mat grey = normalizeMat();

					/* detect lines */
					vector<Point> vp = detectLine(grey);
					int vpSize = (int)vp.size();

					/* control the EV3 */
					if (vpSize != 0){
						/* calculate angles */
						double angle = calcAngle(vp);
						cout << "angle: " << angle << endl;

						/* modify heading  */
						int speed_delta = (int)(SPEED_MOTOR_SAT * (angle / 90.0) * 5);
						//cout << "speed_delta: " << speed_delta << endl;

						speed_right = SPEED_MOTOR_SAT - speed_delta;
						speed_left = SPEED_MOTOR_SAT + speed_delta;


						//cout << "calcPosition : " << calcPosition(grey, STATE(FOLLOW_LINE)) << endl;
						if (calcPosition(grey, STATE(FOLLOW_LINE)) == 0){
							if (acceleration < 2){
								acceleration += 0.1;
							}

							speed_right *= acceleration;
							speed_left *= acceleration;
						}
						else{
							speed_delta = reviseSpeed(grey, STATE(FOLLOW_LINE));
							speed_right = SPEED_MOTOR_SAT - speed_delta;
							speed_left = SPEED_MOTOR_SAT + speed_delta;
						}

						update_motor_speed(fd, speed_left, speed_right);
					}
					else{
						if (i == skipCount){
							docking(fd);
						}
						else{
							while (1){
								Mat grey = normalizeMat();
								/* detect lines */
								vector<Point> vp = detectLine(grey);
								int vpSize = (int)vp.size();
								if (vpSize != 0){
									break;
								}
								else{
									int speed = speed_left < speed_right ? speed_right : speed_left;
									if (acceleration < 2){
										acceleration += 0.1;
									}
									speed *= acceleration;
									update_motor_speed(fd, speed, speed);
								}
							}
						}
						break;
					}
				}
			}
			beforeNode = routes[routeCount-skipCount-1].src;
		} //if skip count >=1
		else{
			int routeIndex = routeCount - 1;
			//cout << "src : " << routes[routeIndex].src << " dest: " << routes[routeIndex].dest << endl;
			if (routes[routeIndex].dest == beforeNode){
				int speed_left, speed_right;
				for (int i = 0; i < 3000; i++){
					update_motor_speed(fd, 30, 72);
				}
				speed_right = 20;
				speed_left = 20;
				for (int i = 0; i < 1000; i++){
					update_motor_speed(fd, speed_left, speed_right);
				}
				undocking(fd);
			}
			else{
				NextNode next;
				for (int i = 0; nodes[routes[routeIndex].src].connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
					if (nodes[routes[routeIndex].src].connectedNodes[i].before == beforeNode){
						for (int j = 0; nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1; j++){
							if (nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j].nextNode == routes[routeIndex].dest){
								next = nodes[routes[routeIndex].src].connectedNodes[i].nextNodes[j];
							}
						}
					}
				}

				//cout << next.nextAngle << " / " << next.nextNode << " / " << next.weight << endl;
				update_motor_speed(fd, 0, 0);
				//getchar();
				goNext(fd, next);
			}

			beforeNode = routes[routeIndex].src;
			followLine(fd);

			docking(fd);
		}
		routeCount -= skipCount+1;
	}	
}


int stopFd;
void stop(int sigNo){
	update_motor_speed(stopFd, 0, 0);
	exit(0); 
}
//MAIN//////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[]) {
	char ch;
	int fd;
	int speed_left;
	int speed_right;
	int number_of_nodes=0;
	Node nodes[NODES + 1];
	int beforeNode;
	

	/* open the EV3 */
	if((fd = open(EV3_DEV_NAME, O_RDWR | O_SYNC)) < 0) {
		printf("open error!\n");
		return -1;
	}
	stopFd = fd;

	struct sigaction sa; 
	sa.sa_handler = stop;
	sigemptyset(&sa.sa_mask);
	sa.sa_flags = 0;
	sigaction(SIGINT, &sa, NULL);

	/* init EV3 motors */
	speed_left = speed_right = 0;
	update_motor_speed(fd, speed_left, speed_right);

	/* open a video dev	 */	
	if (!cap.isOpened()){
		cout << "Cannot open the video cam" << endl;
		return -1;
	}

	/* set a capturing resolution */
	cap.set(CV_CAP_PROP_CONTRAST, CAP_CONSTRAST);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, CAP_WIDTH);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, CAP_HEIGHT);
	cap.set(CV_CAP_PROP_FPS, CAP_FPS);

	/* make a QR code scanner */
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	/* confirm a capturing resolution */
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cout << "Frame Size: " << dWidth << " * " << dHeight << endl;

	/* initialize W */
	for (int i = 1; i < NODES + 1; i++){
		for (int j = 1; j < NODES + 1; j++){
			if (i == j)
				W[i][j] = 0;
			else
				W[i][j] = 9999;
		}
	}

	/* initialize nodes */
	for (int i = 1; i < NODES + 1; i++){
		nodes[i].nodeNum = 0;
	}
	
	/* capture the first QR code */
	while (1){
		Node node;
		int n = searchQR(node);

		if (n){
			nodes[node.nodeNum] = node;
			for (int i = 0; node.connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
				for (int j = 0; j < node.connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1; j++){
					W[node.nodeNum][node.connectedNodes[i].nextNodes[j].nextNode] = node.connectedNodes[i].nextNodes[j].weight;
					W[node.connectedNodes[i].nextNodes[j].nextNode][node.nodeNum] = node.connectedNodes[i].nextNodes[j].weight;
				}
			}

			beforeNode = node.nodeNum;
			number_of_nodes++;
			break;
		}
	}
	undocking(fd);

	/* explore remnant nodes */
	Node node;
	while (number_of_nodes < NODES){
		followLine(fd);

		node = docking(fd);
		
		int i = 0;
		if (nodes[node.nodeNum].nodeNum==0){ //if first visited
			nodes[node.nodeNum] = node;
			number_of_nodes++;
			for (int i = 0; node.connectedNodes[i].before != 0 && i < CONNECTED_NODES; i++){
				for (int j = 0; j < node.connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1; j++){
					W[node.nodeNum][node.connectedNodes[i].nextNodes[j].nextNode] = node.connectedNodes[i].nextNodes[j].weight;
					W[node.connectedNodes[i].nextNodes[j].nextNode][node.nodeNum] = node.connectedNodes[i].nextNodes[j].weight;
				}
			}
		}

		if (number_of_nodes < NODES){
			NextNode next;
			next.nextNode = 0;
			i = 0;
			while (node.connectedNodes[i].before != 0 && i < CONNECTED_NODES){
				if (node.connectedNodes[i].before == beforeNode){
					int j = 0;
					while (node.connectedNodes[i].nextNodes[j].nextNode != 0 && j < CONNECTED_NODES - 1){
						if (nodes[node.connectedNodes[i].nextNodes[j].nextNode].nodeNum == 0){ //if the next node is not found yet
							next = node.connectedNodes[i].nextNodes[j];
						}
						j++;
					}
				}
				i++;
			}
			if (next.nextNode == 0){ //if all nodes are found
				i = 0;
				while (node.connectedNodes[i].before != 0 && i < CONNECTED_NODES){
					if (node.connectedNodes[i].before == beforeNode){
						next = node.connectedNodes[i].nextNodes[0];
					}
					i++;
				}
			}

			//cout << next.nextAngle << " / " << next.nextNode << " / " << next.weight << endl;
			//cout << "number_of_nodes : " << number_of_nodes << endl;
			//getchar();
			
			beforeNode = node.nodeNum;

			goNext(fd, next);
		}
	}

	/*for (int i = 1; i <= NODES; i++){
		for (int j = 1; j <= NODES; j++){
			cout << W[i][j] << " ";
		}
		cout << endl;
	}*/

	cout << "input source and destination" << endl;
	int src, dest;
	cin >> src >> dest;
	getchar();

	edge routes[NODES - 1];
	int routeCount;

	// current Node -> source
	if (node.nodeNum != src){
		calcRoutes(node.nodeNum, src, routes, routeCount);
		travelRoutes(fd, beforeNode, nodes, routes, routeCount);
	}

	update_motor_speed(fd, 0, 0);
	cout << "arrived at the source" << endl;
	getchar();

	// source -> destination
	routeCount = 0;
	calcRoutes(src, dest, routes, routeCount); 
	travelRoutes(fd, beforeNode, nodes, routes, routeCount);

	/* finalize the EV3 */
	update_motor_speed(fd, 0, 0);
    close(fd);

    return 0;
}










