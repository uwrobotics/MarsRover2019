#include <string>

class Console_Message{

public:

	struct message_t{
	  string node_sender;
	  string msg;
	  string level;
	};

	//User notes - there are five error levels (case matters):
	//Info - General useful information to log, or be aware about
	//Warning - Bringing attention to system issues that may produce behaviourial changes, but were corrected/compensated
	//Error - Something fatally wrong happened to the operation, or a component, but the system is still running overall
	//Fatal - The entire system (node, motor, arm, ect) is permanently offline
	//Debug - Info that team members other than the software team could use/would like to know
	
	Console_Message();
	void sendMessage(message_t arg);

};
