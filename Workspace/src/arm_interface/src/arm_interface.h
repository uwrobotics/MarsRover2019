#include <arm_interface/ArmCmd.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <ik/Roboarm.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdbool.h>

class ArmControl
{
    public:

        ArmControl()
        : m_targetAnglesPublisher(NULL)
        , m_actualAnglesPublisher(NULL)
        , m_targetAnglesCanPublisher(NULL)
        {

        }

        ~ArmControl()
        {

        }

        void Initialize()
        {

        }

        void ArmCmdCallback(arm_interface::ArmCmdConstPtr msg)
        {

        }

        void CanCallback(can_msgs::FrameConstPtr msg)
        {

        }

        void Run()
        {

        }

        void SetPublishers(
            const ros::Publisher *targetAnglesPublisher,
            const ros::Publisher *actualAnglesPublisher,
            const ros::Publisher *targetAnglesCanPublisher
            )
        {

        }


    private:
        ros::Publisher *m_targetAnglesPublisher;
        ros::Publisher *m_actualAnglesPublisher;
        ros::Publisher *m_targetAnglesCanPublisher;


        std::vector<double> m_desiredPos(6); // Desired pos
        std::vector<double> m_desiredAngles(6); // Desired angles
        std::vector<double> m_actualAngles(6); // Actual angles
        std::vector<double> m_cmdVels(6); // IK vels

        int mode; // arm mode

        static const size_t s_velCtrlCanIds[] = {0x301, 0x302, 0x303, 0x401, 0x402, 0x403};
        static const size_t s_PosCtrlCanIds[] = {0x309, 0x30A, 0x30B, 0x409, 0x40A, 0x40B};

        void PublishAngles()
        {

        }

        void SendVelocities()
        {

        }

        void PublishCan(std::vector<double> cmds, std::vector<double> ids)
        {

        }
}