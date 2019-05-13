#include <arm_interface/ArmCmd.h>
#include <can_msgs/Frame.h>
#include <cmath>
#include <algorithm>
#include <ik/Roboarm.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <stdbool.h>

class ArmControlInterface
{
    public:

        ArmControlInterface(float dT)
        : m_dT(dT)
        , m_desiredAnglesPublisher(NULL)
        , m_actualAnglesPublisher(NULL)
        , m_canPublisher(NULL)
        , m_isReady(false)
        , m_isFirstRun(true)
        {
            ikControl = new Roboarm((double *)s_linkLengths, (double *)s_defaultAngles);

            m_jointsReady = std::vector<bool> (s_numJoints, false);
            m_desiredEndEffectorPos = std::vector<double> (s_numJoints, 0);
            m_desiredJointPos = std::vector<double> (s_numJoints, 0);
            m_actualJointPos = std::vector<double> (s_numJoints, 0);
            m_actualEndEffectorPos = std::vector<double> (s_numEndEffectorPos, 0);
            m_armCmdVels = std::vector<double> (s_numJoints, 0);
            m_canCmds = std::vector<double> (s_numJoints, 0);
        }

        ~ArmControlInterface()
        {
            delete ikControl;
        }

        void Initialize()
        {

        }

        void ArmCmdCallback(arm_interface::ArmCmdConstPtr msg)
        {
            ROS_INFO("Received Arm Cmd msg");
            for (int i = 0; i < 6; i++)
            {
                m_armCmdVels[i] = msg->data_points[i];
            }
        }

        void CanCallback(can_msgs::FrameConstPtr msg)
        {
            ROS_INFO("Received Can Cmd msg");
            size_t jointId = msg->id - s_jointIdBase;
            m_jointsReady[jointId] = true;
            m_actualJointPos[jointId] = *(double *)(msg->data.data());

            // Check if received initial position for all
            if(!m_isReady)
            {
                m_isReady = std::all_of(m_jointsReady.cbegin(), m_jointsReady.cend(), [](bool i){ return i == true; });
            }
        }

        bool SetMode(arm_interface::arm_mode::Request& req, arm_interface::arm_mode::Response& res)
        {
            m_currentMode = req.mode;
            res.mode = m_currentMode;
            ROS_INFO("request: req=%d, res=%d", req.mode, res.mode);
            ROS_INFO("mode set to %s", s_modes[m_currentMode]);
            return true;
        }

        void Run()
        {
            if(!m_isReady)
            {
                ROS_INFO("Not ready");
                return;
            }

            if (m_isFirstRun)
            {
                InitializePositions();
                m_isFirstRun = false;
            }

            // Process service requests and run

            switch(m_currentMode)
            {
                case s_modeOpenLoop:
                    RunOpenLoop(m_armCmdVels);
                    break;
                case s_modeIkVel:
                    RunIkVel(m_armCmdVels);
                    break;
                case s_modeIkPos:
                    RunIkPos(m_armCmdVels);
                    break;
                default:
                    break;
            }

            PublishCan();
        }


        void SetPublishers(
            ros::Publisher *desiredAnglesPublisher,
            ros::Publisher *actualAnglesPublisher,
            ros::Publisher *canPublisher
            )
        {
            m_desiredAnglesPublisher = desiredAnglesPublisher;
            m_actualAnglesPublisher = actualAnglesPublisher;
            m_canPublisher = canPublisher;
        }


    private:
        float m_dT; // time step

        bool m_isReady;
        bool m_isFirstRun;
        ros::Publisher *m_desiredAnglesPublisher;
        ros::Publisher *m_actualAnglesPublisher;
        ros::Publisher *m_canPublisher;

        Roboarm *ikControl; // ik controls
        
        std::vector<bool> m_jointsReady; // come back
        // To do: remove angles from positions to remove overlap
        std::vector<double> m_desiredEndEffectorPos; // Desired angles/pos  in degrees/cm
                                                        // 0: end effector x position
                                                        // 1: end effector y position
                                                        // 2: end effector theta angle / wrist pitch
        std::vector<double> m_desiredJointPos; // Desired angles in degrees
                                                        // 0: turn table angle
                                                        // 1: shoulder angle
                                                        // 2: elbow angle
                                                        // 3: wrist pitch angle
                                                        // 4: wrist Roll angle
                                                        // 5: claw position
        std::vector<double> m_actualJointPos; // Actual angles
        std::vector<double> m_actualEndEffectorPos; // Actual angles
        std::vector<double> m_armCmdVels; // commands
        std::vector<double> m_canCmds; // commands

        const double s_linkLengths[3] = {35.56, 40.64, 30.48};
        const double s_defaultAngles[3] = {45.0 * M_PI / 180, -30.0 * M_PI / 180,
                           -15.0 * M_PI / 180};

        static const size_t s_jointIdBase = 0x500;
        static const size_t s_numJoints = 6;
        static const size_t s_numEndEffectorPos = 3;

        // Arm joints indexes
        // General
        static const int s_turntableIdx = 0;
        static const int s_shoulderIdx = 1;
        static const int s_elbowIdx = 2;
        static const int s_wristPitchIdx = 3;
        static const int s_wristRollIdx = 4;
        static const int s_clawIdx = 5;

        // Ik mode
        static const int s_endEffectorXIdx = 0;
        static const int s_endEffectorYIdx = 1;
        static const int s_endEffectorThetaIdx = 2;

        static const int s_numModeIds = 4;
        const int s_modeCanIds[4] = {0x301, 0x302, 0x304, 0x400};
        const int s_ctrlCanIds[6] = {0x301, 0x303, 0x305, 0x401, 0x402, 0x403};

        // Arm control modes
        static const u_int8_t s_modeOpenLoop = 0x00;
        static const u_int8_t s_modeIkVel = 0x01;
        static const u_int8_t s_modeIkPos = 0x02;
        const std::string s_modes [3] = {"Open loop", "IK Velocity", "IK Position"};

        u_int8_t m_currentMode; // arm mode

        static const int s_dlc = 8; //not sure about this

        void InitializePositions()
        {
            double outPos[3];
            double linkAngles[3] = {
                                        m_actualJointPos[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPos[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPos[s_wristPitchIdx] * M_PI / 180
                                };
            CalculateFk(linkAngles, outPos);

            m_desiredJointPos[s_turntableIdx] = m_actualJointPos[s_turntableIdx];
            m_desiredJointPos[s_wristRollIdx] = m_actualJointPos[s_wristRollIdx];
            m_desiredJointPos[s_clawIdx] = m_actualJointPos[s_clawIdx];

            m_desiredEndEffectorPos[s_endEffectorXIdx] = outPos[s_endEffectorXIdx];
            m_desiredEndEffectorPos[s_endEffectorYIdx] = outPos[s_endEffectorXIdx];
            m_desiredEndEffectorPos[s_endEffectorThetaIdx] = outPos[s_endEffectorXIdx];
        }

        void CalculateFk(double linkAngles[3], double outPos[3]) {
            double c1 = cos(linkAngles[0]);
            double s1 = sin(linkAngles[0]);

            double c12 = cos(linkAngles[0] + linkAngles[1]);
            double s12 = sin(linkAngles[0] + linkAngles[1]);

            double c123 = cos(linkAngles[0] + linkAngles[1] + linkAngles[2]);
            double s123 = sin(linkAngles[0] + linkAngles[1] + linkAngles[2]);

            double l1 = s_linkLengths[0];
            double l2 = s_linkLengths[1];
            double l3 = s_linkLengths[2];

            outPos[0] = l1 * c1 + l2 * c12 + l3 * c123;
            outPos[1] = l1 * s1 + l2 * s12 + l3 * s123;
            outPos[2] = linkAngles[0] + linkAngles[1] + linkAngles[2];
        }

        void RunOpenLoop(std::vector<double> fkCmdVels)
        {
            m_canCmds[s_turntableIdx] = fkCmdVels[s_turntableIdx];
            m_canCmds[s_wristRollIdx] = fkCmdVels[s_wristRollIdx];
            m_canCmds[s_clawIdx] = fkCmdVels[s_clawIdx];
            m_canCmds[s_shoulderIdx] = fkCmdVels[s_shoulderIdx];
            m_canCmds[s_elbowIdx] = fkCmdVels[s_elbowIdx];
            m_canCmds[s_wristPitchIdx] = fkCmdVels[s_wristPitchIdx];
        }

        void RunIkVel(std::vector<double> ikCmdVels)
        {
            // TODO ik integration
            double endEffectorSpeed[] = {
                                            // To do: better naming for indexes
                                            ikCmdVels[s_shoulderIdx],
                                            ikCmdVels[s_elbowIdx],
                                            ikCmdVels[s_wristPitchIdx] * M_PI / 180
                                        };
            double currentAngles[] = {
                                        m_actualJointPos[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPos[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPos[s_wristPitchIdx] * M_PI / 180
                                     };
            if (ikControl->calculateVelocities(endEffectorSpeed, currentAngles))
            {
                ROS_INFO("ik succeeded");
                for (int i = 0; i < 3; i++)
                {
                    ROS_INFO("joint: %f", ikControl->linkVelocities[i]);
                    if (ikControl->linkVelocities[i] > 5.0 * M_PI / 180) {

                        double divisor = ikControl->linkVelocities[i] / (5.0 * M_PI / 180);
                        ikControl->linkVelocities[0] /= divisor;
                        ikControl->linkVelocities[1] /= divisor;
                        ikControl->linkVelocities[2] /= divisor;
                    }
                }
            } else {
                ROS_INFO("ik failed");
            }

            m_canCmds[s_turntableIdx] = ikCmdVels[s_turntableIdx];
            m_canCmds[s_wristRollIdx] = ikCmdVels[s_wristRollIdx];
            m_canCmds[s_clawIdx] = ikCmdVels[s_clawIdx];
            m_canCmds[s_shoulderIdx] = ikControl->linkVelocities[s_endEffectorXIdx] * 180 / M_PI;
            m_canCmds[s_elbowIdx] = ikControl->linkVelocities[s_endEffectorYIdx] * 180 / M_PI;
            m_canCmds[s_wristPitchIdx] = ikControl->linkVelocities[s_endEffectorThetaIdx] * 180 / M_PI;
        }

        void RunIkPos(std::vector<double> ikCmdVels)
        {
            m_desiredEndEffectorPos[s_endEffectorXIdx] += ikCmdVels[s_shoulderIdx] * m_dT;
            m_desiredEndEffectorPos[s_endEffectorYIdx] += ikCmdVels[s_elbowIdx] * m_dT;
            m_desiredEndEffectorPos[s_endEffectorThetaIdx] += ikCmdVels[s_wristPitchIdx] * m_dT;


            double endEffectorPos[s_numEndEffectorPos] = {
                                        m_desiredEndEffectorPos[s_endEffectorXIdx],
                                        m_desiredEndEffectorPos[s_endEffectorYIdx],
                                        m_desiredEndEffectorPos[s_endEffectorThetaIdx] * M_PI / 180
                                    };
            double currentAngles[s_numEndEffectorPos] = {
                                        m_actualJointPos[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPos[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPos[s_wristPitchIdx] * M_PI / 180
                                     };
            double anglesOut[s_numEndEffectorPos];
            if (ikControl->calculatePositionIK(endEffectorPos, currentAngles, anglesOut))
            {
                ROS_INFO("ik succeeded");
                for (int i = 0; i < s_numEndEffectorPos; i++)
                {
                    ROS_INFO("joint: %f", anglesOut[i]);
                }
            } else
            {
                ROS_INFO("ik failed");
            }

            m_desiredJointPos[s_turntableIdx] += ikCmdVels[s_turntableIdx] * m_dT;
            m_desiredJointPos[s_wristRollIdx] += ikCmdVels[s_wristRollIdx] * m_dT;
            m_desiredJointPos[s_clawIdx] += ikCmdVels[s_clawIdx] * m_dT;
            m_desiredJointPos[s_shoulderIdx] = anglesOut[0] * 180 / M_PI;
            m_desiredJointPos[s_elbowIdx] = anglesOut[1] * 180 / M_PI;
            m_desiredJointPos[s_wristPitchIdx] = anglesOut[2] * 180 / M_PI;

            m_canCmds[s_turntableIdx] = m_desiredJointPos[s_turntableIdx];
            m_canCmds[s_wristRollIdx] = m_desiredJointPos[s_wristRollIdx];
            m_canCmds[s_clawIdx] = m_desiredJointPos[s_clawIdx];
            m_canCmds[s_shoulderIdx] = m_desiredJointPos[s_shoulderIdx];
            m_canCmds[s_elbowIdx] = m_desiredJointPos[s_elbowIdx];
            m_canCmds[s_wristPitchIdx] = m_desiredJointPos[s_wristPitchIdx] ;
        }

        void PublishAngles()
        {
            //  Calculate actual positions
            double outPos[3];
            double linkAngles[3] = {
                                        m_actualJointPos[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPos[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPos[s_wristPitchIdx] * M_PI / 180
                                };
            CalculateFk(linkAngles, outPos);

            // Report angles and positions
        }

        void PublishCan()
        {
            can_msgs::Frame canMsg;
            canMsg.dlc = s_dlc;

            // Publish modes
            for (int i = 0; i < s_numModeIds; i++)
            {
                canMsg.id = s_modeCanIds[i];
                *(int *)(canMsg.data.data()) = m_currentMode;
            }

            for (int i = 0; i < s_numJoints; i++)
            {
                canMsg.id = s_ctrlCanIds[i];
                *(double *)(canMsg.data.data()) = m_canCmds[i];
                m_canPublisher->publish(canMsg);
            }
        }

};