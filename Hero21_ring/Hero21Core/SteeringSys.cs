using Microsoft.SPOT;
using System;

namespace Hero21Core
{
    class SteeringSys
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
    }
}
