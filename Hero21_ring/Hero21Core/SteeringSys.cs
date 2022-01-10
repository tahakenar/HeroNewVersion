
using System;
using System.Threading;
using Microsoft.SPOT;
using System.Text;
using Microsoft.SPOT.Hardware;
using CTRE.HERO;



using CTRE.Phoenix;
using CTRE.Phoenix.Controller;
using CTRE.Phoenix.MotorControl.CAN;
using CTRE.Phoenix.Motion;
using CTRE.Phoenix.MotorControl;

namespace Hero21Core
{
    class Sterring
    {
        public static TalonSRX steerAxis_frontRight = new TalonSRX(13);
        public static TalonSRX steerAxis2_frontLeft = new TalonSRX(6);
        public static TalonSRX steerAxis3_rearRight = new TalonSRX(7);
        public static TalonSRX steerAxis4_rearLeft = new TalonSRX(8);

        public const int timeOutMs = 30;

        public static int steerMotorNum = 4;

        public static double[] steerPositionCommands = new double[steerMotorNum];
        public static double[] steerVelocityCommands = new double[steerMotorNum];

        /* 
        * These mapping coefficients are used to convert encoder ticks into meaningful integers
        * 
        * CAUTION: If you are getting the encoder values --> multiply sensor values with this array
        *          If you are setting position commands as encoder ticks --> divide the commands using this array
        */
        
        private static double[] steerMappingCoefs = { 999 / 4032, 999 / 700, 999 / 600, 999 / 24576, 999 / 24576, 999 / 49152, 360 * 5 / 4096 };

        /*
         * This function factory defaults all talons to prevent unexpected behaviour. It is used to initialize the steering talons
         */
        
        public static void SetFactoryDefault()
        {
            steerAxis_frontRight.ConfigFactoryDefault();
            steerAxis2_frontLeft.ConfigFactoryDefault();
            steerAxis3_rearRight.ConfigFactoryDefault();
            steerAxis4_rearLeft.ConfigFactoryDefault();
            
            Watchdog.Feed();
        }

        /*
        * This function configures the CTRE_MagEncoders that is used on the steering. It is used to initialize the steering talons and sensors.
        */
        public static void ConfigureEncoders()
        {
            steerAxis_frontRight.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            steerAxis2_frontLeft.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            steerAxis3_rearRight.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            steerAxis4_rearLeft.ConfigSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, timeOutMs);
            
            Watchdog.Feed();
        }

        /*
        * This function sets specific parameters to specific talons if necessary. It is used to initialize the steering system
        */

        public static void SetAxisSpecificParams()
        {
            // frontRight
            steerAxis_frontRight.ConfigClosedloopRamp(3.41f, timeOutMs);
            steerAxis_frontRight.ConfigPeakOutputForward(0.3f, timeOutMs);
            steerAxis_frontRight.ConfigPeakOutputReverse(-0.3f, timeOutMs);
            steerAxis_frontRight.Config_kP(0, 1.2f, timeOutMs);
            steerAxis_frontRight.Config_kD(0, 100, timeOutMs);
            steerAxis_frontRight.ConfigAllowableClosedloopError(0, 40, timeOutMs);

            // frontLeft
            steerAxis2_frontLeft.ConfigClosedloopRamp(3.41f, timeOutMs);
            steerAxis2_frontLeft.ConfigPeakOutputForward(0.3f, timeOutMs);
            steerAxis2_frontLeft.ConfigPeakOutputReverse(-0.3f, timeOutMs);
            steerAxis2_frontLeft.Config_kP(0, 1.2f, timeOutMs);
            steerAxis2_frontLeft.Config_kD(0, 100, timeOutMs);
            steerAxis2_frontLeft.ConfigAllowableClosedloopError(0, 40, timeOutMs);

            // rearRight
            steerAxis3_rearRight.ConfigClosedloopRamp(3.41f, timeOutMs);
            steerAxis3_rearRight.ConfigPeakOutputForward(0.3f, timeOutMs);
            steerAxis3_rearRight.ConfigPeakOutputReverse(-0.3f, timeOutMs);
            steerAxis3_rearRight.Config_kP(0, 1.2f, timeOutMs);
            steerAxis3_rearRight.Config_kD(0, 100, timeOutMs);
            steerAxis3_rearRight.ConfigAllowableClosedloopError(0, 40, timeOutMs);

            // rearLeft
            steerAxis4_rearLeft.ConfigClosedloopRamp(3.41f, timeOutMs);
            steerAxis4_rearLeft.ConfigPeakOutputForward(0.3f, timeOutMs);
            steerAxis4_rearLeft.ConfigPeakOutputReverse(-0.3f, timeOutMs);
            steerAxis4_rearLeft.Config_kP(0, 1.2f, timeOutMs);
            steerAxis4_rearLeft.Config_kD(0, 100, timeOutMs);
            steerAxis4_rearLeft.ConfigAllowableClosedloopError(0, 40, timeOutMs);
        }

        /*
        * This function sets the sensor phases. It is used to initialize the steering talons and sensors.
        */

        public static void SetEncoderPhases()
        {
            steerAxis_frontRight.SetSensorPhase(true);
            steerAxis2_frontLeft.SetSensorPhase(false);
            steerAxis3_rearRight.SetSensorPhase(true);
            steerAxis4_rearLeft.SetSensorPhase(false);
                      
            Watchdog.Feed();
        }

        /*
         * This function selects profile slots of the Talon SRX motor drivers
         */
        public static void SelectTalonsProfileSlots()
        {
            steerAxis_frontRight.SelectProfileSlot(0, 0);
            steerAxis2_frontLeft.SelectProfileSlot(0, 0);
            steerAxis3_rearRight.SelectProfileSlot(0, 0);
            steerAxis4_rearLeft.SelectProfileSlot(0, 0);
                      
            Watchdog.Feed();
        }

        /*
         * This function resets the position sensors data. It can be used to initialize the sensors and 
         * it can also be used to configure the steering physically
         */
        public static void ResetSteerSensors()
        {
            steerAxis_frontRight.SetSelectedSensorPosition(0);
            steerAxis2_frontLeft.SetSelectedSensorPosition(0);
            steerAxis3_rearRight.SetSelectedSensorPosition(0);
            steerAxis4_rearLeft.SetSelectedSensorPosition(0);
            
            Watchdog.Feed();
        }

        /*
         * This function calls all the necessary methods respectively to initialize the steering system.
         * It it enough to call this method in the main program
         */
        public static void InitializeSteerSys()
        {
            SetFactoryDefault();
            ConfigureEncoders();
            SetAxisSpecificParams();
            SetEncoderPhases();
            SelectTalonsProfileSlots();
            ResetSteerSensors();
        }

        /*
        * This function sets position commands to the motor drivers
        * Motor drivers have their own feedback control mechanism to perform necessary action.
        * Position commands are passed as encoder ticks and reduction rate is considered.
        */
        public static void SetPositionCommand()
        {
            steerAxis_frontRight.Set(ControlMode.Position, ((int)(steerPositionCommands[0] / steerMappingCoefs[0])));
            steerAxis2_frontLeft.Set(ControlMode.Position, ((int)(steerPositionCommands[1] / steerMappingCoefs[1])));
            steerAxis3_rearRight.Set(ControlMode.Position, ((int)(steerPositionCommands[2] / steerMappingCoefs[2])));
            steerAxis4_rearLeft.Set(ControlMode.Position, ((int)(steerPositionCommands[3] / steerMappingCoefs[3])));
            
        }

        /*
         * This function stops all the actuators by sending 0 voltage commands. 
         * This function should be called when a termination of the movement is desired
         */
        public static void StopSteerActuators()
        {
            steerAxis_frontRight.Set(ControlMode.PercentOutput, 0);
            steerAxis2_frontLeft.Set(ControlMode.PercentOutput, 0);
            steerAxis3_rearRight.Set(ControlMode.PercentOutput, 0);
            steerAxis4_rearLeft.Set(ControlMode.PercentOutput, 0);
            
            Watchdog.Feed();
        }

        /*
         * This function returns steering encoder feedbacks with mapped values as a concatenated string
         * String feedback has the following pattern
         * 1 digit direction + 3 digits magnitude for each axis
         * 4 digits 0000 -> axis 1
         * 4 digits 0000 -> axis 2
         * 4 digits 0000 -> axis 3
         * 4 digits 0000 -> axis 4
         * 4 digits 0000 -> axis 5
         * 4 digits 0000 -> axis 6
         */
        public static string GetSteerFeedback()
        {
            string steerFeedback;
            int[] encoderData = new int[6];
            string[] encoderStr = new string[6];

            encoderData[0] = steerAxis_frontRight.GetSelectedSensorPosition();
            encoderStr[0] = SerialCom.ConvertIntToSerialPiece(encoderData[0], 1);
            Watchdog.Feed();

            encoderData[1] = steerAxis2_frontLeft.GetSelectedSensorPosition();
            encoderStr[1] = SerialCom.ConvertIntToSerialPiece(encoderData[1], 1);
            Watchdog.Feed();

            encoderData[2] = steerAxis3_rearRight.GetSelectedSensorPosition();
            encoderStr[2] = SerialCom.ConvertIntToSerialPiece(encoderData[2], 1);
            Watchdog.Feed();

            encoderData[3] = steerAxis4_rearLeft.GetSelectedSensorPosition();
            encoderStr[3] = SerialCom.ConvertIntToSerialPiece(encoderData[3], 1);
            Watchdog.Feed();

  

            steerFeedback = encoderStr[0] + encoderStr[1] + encoderStr[2] + encoderStr[3]

            return steerFeedback;
        }

        /*
        * This function maps encoder ticks of a sensor into meaningful integer by considering reduction rate of the axes (sensorIndex = axis index)
        * Meaningful integer: [-999,999] to express radians between [-3.14,3.14]
        */
        public static int GetMappedSensorPosition(int encoderTicks, int sensorIndex)
        {
            Watchdog.Feed();
            int mappedPosition = (int)(encoderTicks * steerMappingCoefs[sensorIndex]);
            return mappedPosition;
        }

        /*
        * This function updates current position commmands with the new ones (passed as an argument)
        */
        public static void UpdatePositionCommands(int[] newCommands)
        {
            for (int i = 0; i < steerMotorNum; i++)
            {
                steerPositionCommands[i] = newCommands[i];
            }

        }


}
}
