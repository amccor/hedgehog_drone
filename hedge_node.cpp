
#include "ros/ros.h"

#include "std_msgs/String.h"

#include <sstream>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <signal.h>

#include <eigen3/Eigen/Core>

#include <hedge_node/laser.h>

#define  SAMPLE 

using namespace std;

// ---------------------------------------------------------
//	Parameters related to serial communications
// ---------------------------------------------------------
#define BUF_SIZE 	2000000

int 		  	buffer_ptr = 0;
int 			message_ptr = 0;
unsigned char 	buffer[BUF_SIZE];
unsigned char 	message_bytes[BUF_SIZE];
bool 			HeaderFound = false;
bool 			FooterFound = false;
	
ros::Time TimeTag;
ros::Time StartTime;
ros::Time StopTime;

bool new_laser = false;
hedge_node::laser custom_laser_msg;

ros::Publisher hedge_lasers; 

int 	sequence_number = 0;

FILE 	*fd_out;
FILE 	*fd_in;

char 	input_filename[80];
bool	post_processing_flag = false;

void 	ParseArguments(int argc, char **argv);
void 	ProcessSerialData(unsigned char *data, int len);
int 	kbhit(void);

int main(int argc, char **argv)
{
	ros::Time time_stamp;
	
	char filename[80];
	char pDeviceName[80];
	
	int	fd;
	int len;
	
	int cnt;
	int ii;
	
	time_t current_time;
	struct tm *aTime = NULL;
	
	struct termios options;

	current_time = time(NULL);
	aTime = localtime(&current_time);

	// Set sleep time to 1 microsecond
	struct timespec sleepDuration;
	sleepDuration.tv_sec = 0;
	sleepDuration.tv_nsec = 100000000L;	

	// ---------------------------------------------------------------
	//	Parse the command line arguments
	// ---------------------------------------------------------------
	ParseArguments(argc, argv);

	// ---------------------------------------------------------------
	//	Open output files
	// ---------------------------------------------------------------
	sprintf(filename,"data_hedge_%02d%02d%02d_%02d%02d%02d.dat", 
			aTime->tm_mday, aTime->tm_mon+1, aTime->tm_year + 1900,
			aTime->tm_hour, aTime->tm_min, aTime->tm_sec);
	
	// Open output file		 			
	if( (fd_out	= fopen(filename, "wb")) != NULL)
	{
		cout << "Successfully opened output file " << filename << "..." << endl;
	} else
	{
		cout << "Could not open output file " << filename << "..." << endl;
		exit(0);
	}

	if(post_processing_flag)
	{
		// ---------------------------------------------------------------
		//	Open input files
		// ---------------------------------------------------------------	
		if( (fd_in	= fopen(input_filename, "rb")) != NULL)
		{
			cout << "Successfully opened input file " << input_filename << "..." << endl;
		} else
		{
			cout << "Could not open input file " << input_filename << "..." << endl;
			exit(0);
		}
	}

	// ---------------------------------------------------------------
	//	Open and set up the serial device 
	// ---------------------------------------------------------------
	sprintf(pDeviceName,"/dev/hedge");
	if( (fd = open(pDeviceName, O_RDWR | O_NOCTTY | O_NONBLOCK)) == 0)
	{
		cout << "Could not open /dev/hedge device" << endl;
		exit(0);
	} else
	{
		cout << "Successfully opened /dev/hedge device" << endl;
	}

	//tcflush(fd, TCIFLUSH);
	
	// Initializ ethe options structure
	memset(&options, (int) 0, sizeof(struct termios));
	
	// Get current port options	
	tcgetattr(fd, &options);

	// Make device into a "raw" device 
	//cfmakeraw(&options);
	//options.c_iflag |= IXOFF; 
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	//options.c_cflag &= ~CNEW_RTSCTS;

	// Set the baudrate appropriately 
    cfsetispeed(&options, B57600);
	cfsetospeed(&options, B57600);
	
	options.c_cflag |= (CREAD | CLOCAL);		// turn on READ and ignore modem control lines
    options.c_cflag |= CS8; 				// Set 8 Data Bits
    
    options.c_lflag &= ~(ICANON | ECHO | ECHOE); // raw and no echo
    //options.c_iflag &= ~(IXON | IXOFF | IXANY); // raw and no echo
    
	// Set options of te current device
	tcsetattr(fd, TCSANOW, &options);
	//tcsetattr(fd, TCSAFLUSH, &options);
	
	unsigned char data[100000];
	len = 100000;

	// ---------------------------------------------------------------
	//	Start ROS handler
	// ---------------------------------------------------------------

	// Standard ROS initialization call; name of node is the 3rd argument
  	ros::init(argc, argv, "hedge_node");

  	// Handle to the node  
	ros::NodeHandle n;

	// Set up the publciher 
  	//ros::Publisher mbs_mpsm = n.advertise<nextnav_mbs_node::mpsm>("mbs/mpsm", 1000);
	hedge_lasers = n.advertise<hedge_node::laser>("hedge/lasers", 1000);

	// Set update rate: NOT necessary, only upon a newly received message
  	ros::Rate loop_rate(10);

  	unsigned char seq_counter = 0;

	// Main loop
  	while (ros::ok())
  	{
		// ---------------------------------------------------------------
		//	Continuously read the data and write to a file
		// ---------------------------------------------------------------
		
		TimeTag = ros::Time::now();

		if(post_processing_flag)
		{
			len = 3;
			fread(data, 1, len, fd_in);
			cnt = len;
			//for(int jj=0;jj<cnt;jj++) printf("%x ", data[jj]);
			//cout << endl;	

			nanosleep(&sleepDuration,NULL);
		} 
		else
		{
			// REad some bytes
			cnt =  read(fd, data, len);
		}
		

		if(cnt > 0)
		{
			// Write to the file
			fwrite(data, 1, cnt, fd_out);
			
			// Process the data
			ProcessSerialData(data, cnt);
			
			// Clear memory again
			memset(data, 0, len);
		}	
		
		// Perform standard ROS spin to check for CallBacks .... 
    	ros::spinOnce();
  	}
  	
  	fclose(fd_out);	
	return 0;
}

void ProcessSerialData(unsigned char *data, int len)
{
	// Go through all bytes in message
	for(int ii=0;ii<len;ii++)
	{
		// Message format: 0xAA 0x44 0x88  counter Laser1 Laser2 Laser3 Laser4 0xBB

				//printf("Lasers %d", data);
				//cout << endl;
				if (ii == 4)
				{
					printf("Laser1: %d ", data[ii]);
					cout << endl;
				}
				else if (ii == 5)
				{
					printf("Laser2: %d ", data[ii]);
					cout << endl;
				}
				else if (ii == 6)
				{
					printf("Laser3: %d ", data[ii]);
					cout << endl;
				}
				else if (ii == 7)
				{
					printf("Laser4: %d ", data[ii]);
					cout << endl;
				}

		if( (message_ptr == 8) && (data[ii]==0xBB) )
		{
			message_bytes[message_ptr] = data[ii];
			message_ptr++;

		
			// Assigne the sequence number
			custom_laser_msg.seq = 	(unsigned int)message_bytes[3];

			// Assign the laser range values
			custom_laser_msg.rho_forward 	= (double)message_bytes[4];
			custom_laser_msg.rho_left 	 	= (double)message_bytes[5];
			custom_laser_msg.rho_right 		= (double)message_bytes[6];
			custom_laser_msg.rho_down 		= (double)message_bytes[7];

			hedge_lasers.publish(custom_laser_msg);
			
			// Increment the new sequence number
			sequence_number++;

		} 
		else if( (message_ptr > 2) && (message_ptr < 8) )
		{
			message_bytes[message_ptr] = data[ii];
			message_ptr++;
		}
		else if( (message_ptr == 2) && (data[ii]==0x88) )
		{
			message_bytes[message_ptr] = data[ii];
			message_ptr++;
		}
		else if( (message_ptr == 1) && (data[ii]==0x44) )
		{
			message_bytes[message_ptr] = data[ii];
			message_ptr++;
		}
		else if( data[ii] == 0xAA )
		{
			message_bytes[0] = data[ii];
			message_ptr = 1;
		}
	}	
}

int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;
	
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
	
	ch = getchar();
	
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
	
	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}
	
	return 0;

}

void ParseArguments(int argc, char **argv)
{

	post_processing_flag = false;
	for(int ii=1;ii<argc; ii++)
	{
		//cout << argv[ii] << endl;

		// Verbose flag
		if(strcmp(argv[ii],"-f") == 0)
		{
			post_processing_flag = true;
			sprintf(input_filename, (char *)argv[ii+1]);

			cout << "Post-processing mode" << endl;
		} 
	}
}

/*
// Write to the file
data[0] = 0xAA;
data[1] = 0x44;
data[2] = 0x88;
data[3] = seq_counter++;
data[4] = 120;
data[5] = 30;
data[6] = 200;
data[7] = 40;
data[8] = 0xBB;

fwrite(data, 1, 9, fd_out);

*/
