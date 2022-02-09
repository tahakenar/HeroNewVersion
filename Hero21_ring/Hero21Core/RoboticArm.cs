using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;


using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.MotorControl;

namespace Hero21Core
{
    class RoboticArm
    {
        // Robotic arm motor drivers
        public static TalonSRX armAxis1 = new TalonSRX(1);
        public static TalonSRX armAxis2 = new TalonSRX(2);
        public static TalonSRX armAxis3 = new TalonSRX(3);
        public static TalonSRX armAxis4 = new TalonSRX(4);
        public static TalonSRX armAxis5 = new TalonSRX(5);
        public static TalonSRX armAxis6 = new TalonSRX(6);
        //private static TalonSRX armGripper = new TalonSRX(7);

        public const int timeOutMs = 30;

        public static int armMotorNum = 6; // Including gripper

        public static int armMode = 0;
        // 0 -> position command, 1 -> velocity command ...

        public static double[] armPositionCommands = new double[armMotorNum];
        public static double[] armVelocityCommands = new double[armMotorNum];

        /* 
         * These mapping coefficients are used to convert encoder ticks into meaningful integers
         * {999 / 4032, 999 / 700 ...}
         * CAUTION: If you are getting the encoder values --> multiply sensor values with this array
         *          If you are setting position commands as encoder ticks --> divide the commands using this array
         */
        private static double[] armMappingCoefs = { 0.24776786, 1.42714286, 1.665, 0.04065941, 0.04065941, 0.20324707, 0.43945312 };

        /*
         * This function factory defaults all talons to prevent unexpected behaviour. It is used to initialize the robotic arm talons
         */
        public static void SetFactoryDefault()
        {
            armAxis1.ConfigFactoryDefault();
            armAxis2.ConfigFactoryDefault();
            armAxis3.ConfigFactoryDefault();           
            armAxis4.ConfigFactoryDefault();         
            armAxis5.ConfigFactoryDefault();
            armAxis6.ConfigFactoryDefault();
            /*
           armGripper.ConfigFactoryDefault();
          */
            Watchdog.Feed();
        }

        /*
         * This function configures the CTRE_MagEncoders that is used on the robotic arm. It is used to initialize the robotic arm talons and sensors.
         */
        public static void ConfigureEncoders()
        {
            armAxis1.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, timeOutMs);
            armAxis2.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, timeOutMs);
            armAxis3.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, timeOutMs);
            armAxis4.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);        
            armAxis5.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);            
            armAxis6.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            /*
           armGripper.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
           */
            Watchdog.Feed();
        }

        /*
         * This function sets specific parameters to specific talons if necessary. It is used to initialize the robotic arm system
         */
        public static void SetAxisSpecificParams()
        {
            // AXIS 1
            armAxis1.Config_kP(0, 40f, timeOutMs);
            armAxis1.ConfigClosedloopRamp(0.35f, timeOutMs);
            armAxis1.ConfigAllowableClosedloopError(0, 5, timeOutMs);
     
            // AXIS 2
            armAxis2.Config_kP(0, 80f, timeOutMs);
            armAxis2.ConfigClosedloopRamp(0.35f, timeOutMs);
            armAxis2.ConfigAllowableClosedloopError(0, 8, timeOutMs);

            // AXIS 3
            armAxis3.Config_kP(0, 20f, timeOutMs);
            armAxis3.ConfigAllowableClosedloopError(0, 15, timeOutMs);

            // AXIS 4
            armAxis4.Config_kP(0, 10f, timeOutMs);
            armAxis4.ConfigAllowableClosedloopError(0, 20, timeOutMs);

            // AXIS 5
            armAxis5.Config_kP(0, 10f, timeOutMs);
            armAxis5.ConfigAllowableClosedloopError(0, 20, timeOutMs);

            // AXIS 6
            armAxis6.Config_kP(0, 10f, timeOutMs);
            armAxis6.ConfigAllowableClosedloopError(0, 20, timeOutMs);
            Watchdog.Feed();
            
        }

        /*
         * This function sets the sensor phases. It is used to initialize the robotic arm talons and sensors.
         */
        public static void SetEncoderPhases()
        {
            armAxis1.SetSensorPhase(false);            
            armAxis2.SetSensorPhase(true);            
            armAxis3.SetSensorPhase(true);           
            armAxis4.SetSensorPhase(true);            
            armAxis5.SetSensorPhase(true);
            armAxis6.SetSensorPhase(true);
            //armGripper.SetSensorPhase(true);            
            Watchdog.Feed();

            armAxis2.SetInverted(true);
            armAxis3.SetInverted(true);
            armAxis4.SetInverted(true);
            armAxis5.SetInverted(true);
            armAxis6.SetInverted(true);
            Watchdog.Feed();
        }

        /*
         * This function selects profile slots of the Talon SRX motor drivers
         */
        public static void SelectTalonsProfileSlots()
        {
            armAxis1.SelectProfileSlot(0, 0);            
            armAxis2.SelectProfileSlot(0, 0);            
            armAxis3.SelectProfileSlot(0, 0);          
            armAxis4.SelectProfileSlot(0, 0);            
            armAxis5.SelectProfileSlot(0, 0);            
            armAxis6.SelectProfileSlot(0, 0);
            //armGripper.SelectProfileSlot(0, 0);           
            Watchdog.Feed();
        }

        /*
         * This function resets the position sensors data. It can be used to initialize the sensors and 
         * it can also be used to configure the robotic arm physically
         */
        public static void ResetArmSensors()
        {
            armAxis1.SetSelectedSensorPosition(2048);
            armAxis2.SetSelectedSensorPosition(1024);           
            armAxis3.SetSelectedSensorPosition(1024);            
            armAxis4.SetSelectedSensorPosition(0);            
            armAxis5.SetSelectedSensorPosition(0);
            armAxis6.SetSelectedSensorPosition(0);
            //armGripper.SetSelectedSensorPosition(0);
            Watchdog.Feed();
        }

        /*
         * This function calls all the necessary methods respectively to initialize the robotic arm system.
         * It it enough to call this method in the main program
         */
        public static void InitializeRoboticArmSys()
        {
            SetFactoryDefault();
            ConfigureEncoders();
            SetAxisSpecificParams();
            SetEncoderPhases();
            SelectTalonsProfileSlots();
            //ResetArmSensors();
        }

        /*
         * This function sets position commands to the motor drivers
         * Motor drivers have their own feedback control mechanism to perform necessary action.
         * Position commands are passed as encoder ticks and reduction rate is considered.
         */
        public static void SetPositionCommand()
        {
            armAxis1.Set(ControlMode.Position, ((int)((armPositionCommands[0] / 999) * 2048 + 2048)));
            armAxis2.Set(ControlMode.Position, ((int)((armPositionCommands[1] / 999) * 768 + 768)));
            armAxis3.Set(ControlMode.Position, ((int)((armPositionCommands[2] / 999) * 768 + 1024 + 768)));
            armAxis4.Set(ControlMode.Position, ((int)(armPositionCommands[3] * 81920 / 999)));            
            armAxis5.Set(ControlMode.Position, ((int)(armPositionCommands[4] * 20480 / 999)));
            armAxis6.Set(ControlMode.Position, ((int)(armPositionCommands[5] * 81920 / 999)));
            Watchdog.Feed();
            //armGripper.Set(ControlMode.Position, (int)(armPositionCommands[6] / armMappingCoefs[6]));
        }

        /*
         * This function stops all the actuators by sending 0 voltage commands. 
         * This function should be called when a termination of the movement is desired
         */
        public static void StopArmActuators()
        {
            armAxis1.Set(ControlMode.PercentOutput, 0);
            armAxis2.Set(ControlMode.PercentOutput, 0);
            armAxis3.Set(ControlMode.PercentOutput, 0);
            armAxis4.Set(ControlMode.PercentOutput, 0);
            armAxis5.Set(ControlMode.PercentOutput, 0);
            armAxis6.Set(ControlMode.PercentOutput, 0);
            Watchdog.Feed();
        }

        /*
         * This function returns robotic arm encoder feedbacks with mapped values as a concatenated string
         * String feedback has the following pattern
         * 1 digit direction + 3 digits magnitude for each axis
         * 4 digits 0000 -> axis 1
         * 4 digits 0000 -> axis 2
         * 4 digits 0000 -> axis 3
         * 4 digits 0000 -> axis 4
         * 4 digits 0000 -> axis 5
         * 4 digits 0000 -> axis 6
         */
        public static string GetArmFeedback()
        {
            string armFeedback;
            int[] encoderData = new int[6];
            string[] encoderStr = new string[6];

            encoderData[0] = armAxis1.GetSelectedSensorPosition();
            encoderData[0] = GetMappedSensorPosition(encoderData[0],0);
            encoderStr[0] = SerialCom.ConvertIntToSerialPiece(encoderData[0], 1);
            Watchdog.Feed();

            encoderData[1] = armAxis2.GetSelectedSensorPosition();
            encoderData[1] = GetMappedSensorPosition(encoderData[1], 1);
            encoderStr[1] = SerialCom.ConvertIntToSerialPiece(encoderData[1], 1);
            Watchdog.Feed();

            encoderData[2] = armAxis3.GetSelectedSensorPosition();
            encoderData[2] = GetMappedSensorPosition(encoderData[2], 2);
            encoderStr[2] = SerialCom.ConvertIntToSerialPiece(encoderData[2], 1);
            Watchdog.Feed();

            encoderData[3] = armAxis4.GetSelectedSensorPosition();
            encoderData[3] = GetMappedSensorPosition(encoderData[3], 3);
            encoderStr[3] = SerialCom.ConvertIntToSerialPiece(encoderData[3], 1);
            Watchdog.Feed();

            encoderData[4] = armAxis5.GetSelectedSensorPosition();
            encoderData[4] = GetMappedSensorPosition(encoderData[4], 4);
            encoderStr[4] = SerialCom.ConvertIntToSerialPiece(encoderData[4], 1);
            Watchdog.Feed();

            encoderData[5] = armAxis6.GetSelectedSensorPosition();
            encoderData[5] = GetMappedSensorPosition(encoderData[5], 5);
            encoderStr[5] = SerialCom.ConvertIntToSerialPiece(encoderData[5], 1);
            Watchdog.Feed();


            armFeedback = encoderStr[0] + encoderStr[1] + encoderStr[2] + encoderStr[3] + encoderStr[4] + encoderStr[5];

            return armFeedback;
        }

        /*
         * This function maps encoder ticks of a sensor into meaningful integer by considering reduction rate of the axes (sensorIndex = axis index)
         * Meaningful integer: [-999,999] to express radians between [-3.14,3.14]
         */
        public static int GetMappedSensorPosition(int encoderTicks, int sensorIndex)
        {
            Watchdog.Feed();
            int mappedPosition = (int)(encoderTicks * armMappingCoefs[sensorIndex]);
            return mappedPosition;
        }

        /*
         * This function updates current position commmands with the new ones (passed as an argument)
         */
        public static void UpdatePositionCommands(int[] newCommands)
        {
            for (int i = 0; i < armMotorNum; i++)
            {
                armPositionCommands[i] = newCommands[i];
            }
        }
    }
}
