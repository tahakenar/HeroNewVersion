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
    class SerialCom
    {
        public static System.IO.Ports.SerialPort _uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port1.UART, 115200);
        public static byte[] _rx = new byte[1024];
        static byte[] _tx = new byte[1024];
        public static int _txIn = 0;
        public static int _txOut = 0;
        public static int _txCnt = 0;
        public static int readCnt;

        private static int receiveCounter = 0;
        private static bool receiveFlag = false;
        private static int[] incomingData = new int[24];    // 24 is arbitrarily given
        public static bool assignCommands = false;

        private static int armMsgLen = 24;                  // string length of arm msgs
        private static int strPieceLen = 4;                 // string length of each motor info

        public static int[] armCommandsArray = new int[RoboticArm.armMotorNum];
        //public static int[] steeringCommandsArray = new int[SteeringSys.armMotorNum];

        private static string armFeedbackMsg = "";
        private static string steeringFeedbackMsg = "";
        private static string finalFeedbackMsg = "";

        /*
         * This function initializes the serial communication -> it opens the serial port
         */
        public static void InitializeSerialCom()
        {
            _uart.Open();
            Watchdog.Feed();
        }

        /*
         * This function converts string argument to bytes and then transmits it via uart
         */
        private static void Write(string msg)
        {
            byte[] msg_byte = String2Byte(msg);
            Watchdog.Feed();
            _uart.Write(msg_byte, 0, msg.Length);
        }

        /*
         * This function converts string into bytes
         */
        private static byte[] String2Byte(String msg) // return string type to byte type for better processing
        {
            byte[] retval = new byte[msg.Length];
            for (int i = 0; i < msg.Length; ++i)
                retval[i] = (byte)msg[i];
            Watchdog.Feed();
            return retval;
        }

        /*
         * This function converts integer value into meaningful serial piece
         * Example: -150 -> "0150" or +150 -> "1150"
         *
         */
        public static string ConvertIntToSerialPiece(int i, int sign)
        {
            if (sign == 0)
                return (i < 0 ? "1" : "0") + (i < 0 ? -i : i).ToString("D3");
            else
                return (i > 0 ? "1" : "0") + (i < 0 ? -i : i).ToString("D3");
        }

        /*
         * This function gets systems feedback msgs and sends it via uart
         */
        public static void SendFeedbackMsg()
        {
            armFeedbackMsg = RoboticArm.GetArmFeedback();
            //TODO: Get steering system Feedback string

            finalFeedbackMsg = "A" + armFeedbackMsg + steeringFeedbackMsg + "B";

            //Debug.Print(finalFeedbackMsg);

            Write(finalFeedbackMsg);

        }

        // Rest are the methods used for cicular buffer
        public static int CalcRemainingCap()
        {
            /* first calc the remaining capacity in the ring buffer */
            int rem = _tx.Length - _txCnt;
            /* cap the return to the maximum capacity of the rx array */
            if (rem > _rx.Length)
                rem = _rx.Length;
            return rem;
        }

        /** @param received byte to push into ring buffer */
        public static void PushByte(byte datum)
        {
            _tx[_txIn] = datum;
            if (++_txIn >= _tx.Length)
                _txIn = 0;
            ++_txCnt;
        }

        /** 
         * Pop the oldest byte out of the ring buffer.
         * Caller must ensure there is at least one byte to pop out by checking _txCnt.
         * @return the oldest byte in buffer.
         */
        public static byte PopByte()
        {
            byte retval = _tx[_txOut];
            if (++_txOut >= _tx.Length)
                _txOut = 0;
            --_txCnt;
            return retval;
        }

        /*
         * This function is called whenever a byte in circular buffer is poppin out.
         * It processes the incoming byte (as ascii chars) and raises a flag after a whole msg is received.
         * TODO: Double check communication safety
         */
        public static void ReadCommand(byte incomingASCII)
        {
            if (incomingASCII == 83)    // Capture 'S' character
            {
                receiveFlag = true;
                receiveCounter = 0;
            }
            if (receiveFlag == true && incomingASCII != 83 && incomingASCII != 70)      // If not 'S' and 'F'
            {
                incomingData[receiveCounter] = (incomingASCII) - 48;                    // ASCII to integer conversion
                receiveCounter++;
            }
            if (receiveFlag == true && incomingASCII == 70)                             // 'F' check -> finish condition
            {
                receiveCounter = 0;
                receiveFlag = false;
                assignCommands = true;
                //TODO: add a new flag to parse incoming data
            }

        }

        public static void AssignArmCommands()
        {
            for (int i = 0, j = 0; i < armMsgLen; i += strPieceLen, j++)
            {
                armCommandsArray[j] = (incomingData[i] == 1 ? 1 : -1) * (incomingData[i + 1] * 100 + incomingData[i + 2] * 10 + incomingData[i + 3]); 
            }
        }

    }
}

