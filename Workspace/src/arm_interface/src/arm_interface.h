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

class ArmControlService
{
    public:

        ArmControl()
        : m_targetAnglesPublisher(NULL)
        , m_actualJointPossPublisher(NULL)
        , m_targetAnglesCanPublisher(NULL)
        , m_canIds(NULL)
        , m_isReady(false)
        , m_isFirstRun(true)
        {
            ikControl = new Roboarm(s_linkLengths, s_defaultAngles);
        }

        ~ArmControl()
        {
            delete ikControl;
        }

        void Initialize()
        {

        }

        void ArmCmdCallback(arm_interface::ArmCmdConstPtr msg)
        {

        }

        void CanCallback(can_msgs::FrameConstPtr msg)
        {
            size_t jointId = msg->id - s_jointIdBase;
            m_jointsReady[jointId] = true;
            m_actualJointPoss[jointId] = *(double *)(msg->data.data());

            // Check if received initial position for all
            m_isReady = std::all_of(m_jointsReady.cbegin(), m_jointsReady.cend(), [](bool i){ return i == true; }
        }

        void Run()
        {
            if(!m_isReady)
            {
                return;
            }

            if (m_isFirstRun)
            {
                InitializePositions();
            }
            // Process service requests and run
            switch(m_currentMode)
            {
                case s_modeIkNone:
                    RunIkNone();
                    break;
                case s_modeIkVel:
                    RunIkVel();
                    break;
                case s_modeIkPos:
                    RunIkPos();
                    break;
                default:
                    break;
            }
            PublishCan();

        }


        void SetPublishers(
            const ros::Publisher *targetAnglesPublisher,
            const ros::Publisher *actualAnglesPublisher,
            const ros::Publisher *canPublisher
            )
        {
            m_targetAnglesPublisher = targetAnglesPublisher;
            m_actualJointPossPublisher = actualAnglesPublisher;
            m_canPublisher = canPublisher;
        }


    private:
        bool double dT;
        bool m_isReady;
        bool m_isFirstRun;
        ros::Publisher *m_targetAnglesPublisher;
        ros::Publisher *m_actualJointPossPublisher;
        ros::Publisher *m_canPublisher;

        static const size_t s_jointIdBase = 0x500;
        static const size_t s_numJoints = 6;
        static const size_t s_numEndEffectorPos = 3;
        std::vector<bool> m_jointsReady(s_numJoints, false);
        // To do: remove angles from positions to remove overlap
        std::vector<double> m_desiredEndEffectotPos(numEndEffectorPos); // Desired angles/pos  in degrees/cm
                                                        // 0: end effector x position
                                                        // 1: end effector y position
                                                        // 2: end effector theta angle / wrist pitch
        std::vector<double> m_desiredJointPos(s_numJoints); // Desired angles in degrees
                                                        // 0: turn table angle
                                                        // 1: shoulder angle
                                                        // 2: elbow angle
                                                        // 3: wrist pitch angle
                                                        // 4: wrist Roll angle
                                                        // 5: claw position
        std::vector<double> m_actualJointPos(s_numJoints); // Actual angles
        std::vector<double> m_actualEndEffectorPos(numEndEffectorPos); // Actual angles
        std::vector<double> m_canCmds(s_numJoints); // commands

        static const double s_linkLengths[] = {35.56, 40.64, 30.48};
        static const double s_defaultAngles[] = {45.0 * M_PI / 180, -30.0 * M_PI / 180,
                           -15.0 * M_PI / 180};

        Roboarm *ikControl; // ik controls

        int m_currentMode; // arm mode

        // Arm joints indexes
        // General
        static const int s_turntableIdx = 0
        static const int s_shoulderIdx = 1;
        static const int s_elbowIdx = 2;
        static const int s_wristPitchIdx = 3;
        static const int s_wristRollIdx = 4;
        static const int s_clawIdx = 5;

        // Ik mode
        static const int s_endEffectorXIdx = 0;
        static const int s_endEffectorYIdx = 1;
        static const int s_endEffectorThetaIdx = 2;

        static const size_t s_numModeIds = 4;
        static const size_t s_modeCanIds[] = {0x301, 0x302, 0x304, 0x400};
        static const size_t s_ctrlCanIds[] = {0x301, 0x303, 0x305, 0x401, 0x402, 0x403};
=
        size_t *m_canIds;

        // Arm control modes
        static const size_t s_modeIkNone = 0x00;
        static const size_t s_modeIkVel = 0x01;
        static const size_t s_modeIkPos = 0x02;


        static const int s_dlc = 8; //not sure about this

        void InitializePositons()
        {
            double outPos[3];
            double linkAngles[3] = {
                                        m_actualJointPos[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPos[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPos[s_wristPitchIdx] * M_PI / 180
                                };
            CalculateFk(linkAngles, outPos);

            m_desiredJointPos[s_turntableIdx] = m_actualJointPoss[s_turntableIdx];
            m_desiredJointPos[s_wristRollIdx] = m_actualJointPoss[s_wristRollIdx];
            m_desiredJointPos[s_clawIdx] = m_actualJointPoss[s_clawIdx];

            m_desiredEndEffectotPos[s_endEffectorXIdx] = outPos[s_endEffectorXIdx];
            m_desiredEndEffectotPos[s_endEffectorYIdx] = outPos[s_endEffectorXIdx];
            m_desiredEndEffectotPos[s_endEffectorThetaIdx] = outPos[s_endEffectorXIdx];
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

        void RunIkNone(std::vector<double> fkCmdVels)
        {
            m_canIds = s_velCtrlCanIds;

            m_canCmds[s_turntableIdx] = fkCmdVels[s_turntableIdx];
            m_canCmds[s_wristRollIdx] = fkCmdVels[s_wristRollIdx];
            m_canCmds[s_clawIdx] = fkCmdVels[s_clawIdx];
            m_canCmds[s_shoulderIdx] = fkCmdVels[s_shoulderIdx];
            m_canCmds[s_elbowIdx] = fkCmdVels[s_elbowIdx];
            m_canCmds[s_wristPitchIdx] = fkCmdVels[s_wristPitchIdx];
        }

        void RunIkVel(std::vector<double> ikCmdVels)
        {
            m_canIds = s_velCtrlCanIds;

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
            if (ikControl.calculateVelocities(endEffectorSpeed, currentAngles))
            {
                ROS_INFO("ik succeeded");
                for (int i = 0; i < 3; i++)
                {
                    ROS_INFO("joint: %f", ikControl.linkVelocities[i]);
                    if (ikControl.linkVelocities[i] > 5.0 * M_PI / 180) {

                        double divisor = ikControl.linkVelocities[i] / (5.0 * M_PI / 180);
                        ikControl.linkVelocities[0] /= divisor;
                        ikControl.linkVelocities[1] /= divisor;
                        ikControl.linkVelocities[2] /= divisor;
                    }
                }
            } else {
                ROS_INFO("ik failed");
            }

            m_canCmds[s_turntableIdx] = ikCmdVels[s_turntableIdx];
            m_canCmds[s_wristRollIdx] = ikCmdVels[s_wristRollIdx];
            m_canCmds[s_clawIdx] = ikCmdVels[s_clawIdx];
            m_canCmds[s_shoulderIdx] = ikControl.linkVelocities[s_endEffectorXIdx] * 180 / M_PI;
            m_canCmds[s_elbowIdx] = ikControl.linkVelocities[s_endEffectorYIdx] * 180 / M_PI;
            m_canCmds[s_wristPitchIdx] = ikControl.linkVelocities[s_endEffectorThetaIdx] * 180 / M_PI;
        }

        void RunIkPos(std::vector<double> ikCmdVels)
        {
            m_canIds = s_posCtrlCanIds;

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
            if (ikControl.calculatePositionIK(endEffectorPos, currentAngles, anglesOut)
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
                                        m_actualJointPoss[s_shoulderIdx] * M_PI / 180,
                                        m_actualJointPoss[s_elbowIdx] * M_PI / 180,
                                        m_actualJointPoss[s_wristPitchIdx] * M_PI / 180
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
                m_canPublisher.publish(canMsg);
            }
        }

}