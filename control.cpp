#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "lmstypes.h"
#include "bytecodes.h"

#define EV3_DEV_NAME "/dev/hidraw0"

#define FALSE			0
#define TRUE			1
#define MAX_BUF_LEN		128
#define MAGIC_KEY		0xFA

/****************************************************************/
/* BEGIN : proven to work by khkim (20151112)                   */
/****************************************************************/

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
	printf(" (len: %d)\n", len);
}

int get_command_length(char *p, int maxlen) {
	int i;
	for (i = 2; i < maxlen; i++) {
		if (p[i] == opOBJECT_END && p[i + 1] == MAGIC_KEY)
			return i + 1;
	}
	return 0;
}

char direct_reply_buf[MAX_BUF_LEN] = {0, };
int EV3_send_command(int fd, char *direct_command_buf, int reply_needed) {
	int n;
	n = get_command_length(direct_command_buf, MAX_BUF_LEN);
	direct_command_buf[1] = (n - 2) / 256;
	direct_command_buf[0] = (n - 2) - direct_command_buf[1] * 256;
	print_buf("[COMMAND]", direct_command_buf, n);
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

/****************************************************************/
/* END : proven to work by khkim (20151112)                     */ 
/****************************************************************/

#define PORT_USONIC 	0	// port 1
#define TYPE_USONIC 	30
#define MODE_USONIC		0

char usonic_command_buf[MAX_BUF_LEN] = {
	0x00, 0x00, // command size
	0xFF, 0xFF, // sequence counter
	0x00, 		// type of command : DIRECT_COMMAND_REPLY
	0x01, 0x00, // message header
	/* byte codes */
	opINPUT_READ, 
	LC0(0), 	// layer
	LC0(PORT_USONIC),	// port
	LC0(TYPE_USONIC), 	// type
	LC0(MODE_USONIC), 	// mode
	GV0(0),		// return value
	//opOBJECT_END
	opOBJECT_END,
	MAGIC_KEY
};

#define PORT_LIGHT_L 	1	// port 2
#define PORT_LIGHT_R 	2	// port 3
#define TYPE_LIGHT		29	
#define MODE_LIGHT		0

char left_light_command_buf[MAX_BUF_LEN] = {
	0x00, 0x00, // command size
	0xFF, 0xFF, // sequence counter
	0x00, 		// type of command : DIRECT_COMMAND_REPLY
	0x01, 0x00, // message header
	/* byte codes */
	opINPUT_READ, 
	LC0(0), 	// layer
	LC0(PORT_LIGHT_L),	// port
	LC0(TYPE_LIGHT), 	// type
	LC0(MODE_LIGHT), 	// mode
	GV0(0),		// return value
	//opOBJECT_END
	opOBJECT_END,
	MAGIC_KEY
};

char right_light_command_buf[MAX_BUF_LEN] = {
	0x00, 0x00, // command size
	0xFF, 0xFF, // sequence counter
	0x00, 		// type of command : DIRECT_COMMAND_REPLY
	0x01, 0x00, // message header
	/* byte codes */
	opINPUT_READ, 
	LC0(0), 	// layer
	LC0(PORT_LIGHT_R),	// port
	LC0(TYPE_LIGHT), 	// type
	LC0(MODE_LIGHT), 	// mode
	GV0(0),		// return value
	//opOBJECT_END
	opOBJECT_END,
	MAGIC_KEY
};

#define PORT_MOTOR_L 	1	// port A
#define PORT_MOTOR_R 	2	// port B
#define SPEED_MOTOR_MAX	100
#define SPEED_MOTOR_MIN -100

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
	opOBJECT_END,
	MAGIC_KEY
};

int main(int argc, char* argv[]) {
	char ch;
	int fd;
	int val;
	int speed_left;
	int speed_right;

	if((fd = open(EV3_DEV_NAME, O_RDWR | O_SYNC)) < 0) {
		printf("open error!\n");
		return -1;
	}

	speed_left = 0;
	speed_right = 0;
	motor_command_buf[11] = speed_left;
	motor_command_buf[16] = speed_right;
	EV3_send_command(fd, motor_command_buf, FALSE);

	while((ch = get_key()) != 'q'){
		switch (ch) {	
		case 'u':
			EV3_send_command(fd, usonic_command_buf, TRUE);
			val = direct_reply_buf[5];
			printf("value = %d (cm)\n", val * 250/100);
			break;
		case 'l':
			EV3_send_command(fd, left_light_command_buf, TRUE);
			val = direct_reply_buf[5];
			printf("value = %d (pct)\n", val);
			break;
		case 'r':
			EV3_send_command(fd, right_light_command_buf, TRUE);
			val = direct_reply_buf[5];
			printf("value = %d (pct)\n", val);
			break;
		case 'L':
			speed_left += 10;
			motor_command_buf[11] = speed_left;
			motor_command_buf[16] = speed_right;
			EV3_send_command(fd, motor_command_buf, FALSE);
			printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
			break;
		case 'R':
			speed_right += 10;
			motor_command_buf[11] = speed_left;
			motor_command_buf[16] = speed_right;
			EV3_send_command(fd, motor_command_buf, FALSE);
			printf("motor(L,R) = (%d,%d)\n", speed_left, speed_right);
			break;
		default:
			printf("unknown key!\n");
			break;
		}
	}

	speed_left = 0;
	speed_right = 0;
	motor_command_buf[11] = speed_left;
	motor_command_buf[16] = speed_right;
	EV3_send_command(fd, motor_command_buf, FALSE);

    close(fd);
    return 0;
}
