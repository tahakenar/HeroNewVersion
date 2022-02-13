//#define DEBUG

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
    public class Program
    {

        public static void Main()
        {
            // DO ALL THE INITIALIZATION STUFF HERE!
            SerialCom.InitializeSerialCom();
            RoboticArm.InitializeRoboticArmSys();
            
            // For ring buffer implementation
            byte[] scratch = new byte[1];

            while (true)
            {
                Watchdog.Feed();
                SerialCom.IncreaseNoMsgCounter();
                // GET THE SENSOR DATA AND TRANSMIT IT TO THE HIGH LEVEL CONTROLLER HERE!
                if (SerialCom._uart.CanWrite)
                {
                    SerialCom.SendFeedbackMsg();   
                }

                // GET SERIAL DATA AND PROCESS IT TO SET COMMANDS FOR MOTOR DRIVERS HERE!
                if (SerialCom._uart.BytesToRead > 0)
                {
                    int readCnt = SerialCom._uart.Read(SerialCom._rx, 0, SerialCom.CalcRemainingCap());
      
                    for (int i = 0; i < readCnt; ++i)
                    {

                        SerialCom.ReadCommand(SerialCom._rx[i]);                   
                        if (SerialCom.assignCommands == true)
                        {
                            string[] testDebug = new string[RoboticArm.armMotorNum];

                            //SerialCom.CheckMsgContinuity();
                            SerialCom.AssignArmCommands();
                            RoboticArm.UpdatePositionCommands(SerialCom.armCommandsArray);
                            RoboticArm.SetPositionCommand();
                            SerialCom.assignCommands = false;

                            testDebug[0] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[0], 1);
                            testDebug[1] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[1], 1);
                            testDebug[2] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[2], 1);
                            testDebug[3] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[3], 1);
                            testDebug[4] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[4], 1);
                            testDebug[5] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[5], 1);

                            Debug.Print("Commands: " + testDebug[0] + testDebug[1] + testDebug[2] + testDebug[3] + testDebug[4] + testDebug[5]);

                        }
                    }
 
                       /*
                    if (SerialCom.CheckSerialErrCnt() == false || SerialCom.CheckNoMsgCnt() == false)
                    {
                        // EMERGENCY STOP CONDITION
                        RoboticArm.StopArmActuators();
                        Debug.Print("EMERGENCY STOP");
                    }
                       */
                }

                if (SerialCom._txCnt > 0)
                {
                    scratch[0] = SerialCom.PopByte();
                }

            }
        }
    }
}