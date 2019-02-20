#ifndef CONSOLE_MESSAGE
#define CONSOLE_MESSAGE

#include <string>
#include <console_message/console_msg.h>
#include <ros/node_handle.h>

class ConsoleMessage{

public:

	enum eLevels{
	  INFO = 0,
	  WARN,
	  ERROR
	};

	//User notes - there are five error levels (case matters):
	//Info - General useful information to log, or be aware about
	//Warning - Bringing attention to system issues that may produce behaviourial changes, but were corrected/compensated
	//Error - Something fatally wrong happened to the operation, or a component, but the system is still running overall
	//Fatal - The entire system (node, motor, arm, ect) is permanently offline
	//Debug - Info that team members other than the software team could use/would like to know
	
	//Console_Message();
	static void Initialize(ros::NodeHandle& nh);

	static void SendMessage(std::string msg, eLevels level=INFO);

private:
	static ros::Publisher* s_pPub;

};

#endif
