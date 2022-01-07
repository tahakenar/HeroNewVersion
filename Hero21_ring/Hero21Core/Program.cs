#define DEBUG

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
                        SerialCom.PushByte(SerialCom._rx[i]);
                    }
                }
                /* if there are bufferd bytes echo them back out */
                if (SerialCom._txCnt > 0)
                {
                    scratch[0] = SerialCom.PopByte();
                    //TODO: Read byte by byt
                    SerialCom.ReadCommand(scratch[0]);

                    if (SerialCom.assignCommands == true)
                    {
                        SerialCom.AssignArmCommands();
                        //TODO: Set talon commands

                        string[] testDebug = new string[RoboticArm.armMotorNum];
                        for (int i = 0; i < 7; i++)
                        {
                            RoboticArm.armPositionCommands[i] = (int) SerialCom.armCommandsArray[i];
                        }
                        RoboticArm.SetPositionCommand();
                        


                        // DEBUGGING
                        
                        testDebug[0] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[0], 1);
                        testDebug[1] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[1], 1);
                        testDebug[2] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[2], 1);
                        testDebug[3] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[3], 1);
                        testDebug[4] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[4], 1);
                        testDebug[5] = SerialCom.ConvertIntToSerialPiece(SerialCom.armCommandsArray[5], 1);

                        Debug.Print("Commands: " + testDebug[0] + testDebug[1] + testDebug[2] + testDebug[3] + testDebug[4] + testDebug[5]);

                        SerialCom.assignCommands = false;
                    }
                }

                System.Threading.Thread.Sleep(10);

#if DEBUG
                // CALL ALL THE DEBUG METHODS IF YOU WANT TO DEBUG FROM A CONSOLE
#endif

            }
        }
    }
}