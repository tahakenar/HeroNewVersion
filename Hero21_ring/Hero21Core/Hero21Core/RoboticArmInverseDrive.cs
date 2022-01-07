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
    class RoboticArmInverseDrive
    {
        // written for only first 3 axis of robotic arm

        public static int[] uniqieCoefficients = { 797, 1020 / 90, 1020 / 90 };  //encoder turning value for 1 degree turn for each axis
        public static double[] axisAngle = new double[5];
        public static int prevEncoderData1 = 0;
        public static int prevEncoderData2 = 0;
        public static int prevEncoderData3 = 0;
        public static int encoderDataAxis1 = 0;
        public static int encoderDataAxis2 = 0;
        public static int encoderDataAxis3 = 0;


        public static void InverseKinematicDrive()
        {
            axisAngle = SerialCommunicaton.SerialExtractAll();

            // calculated encoder turning value for given degree in serial
            encoderDataAxis1 = (int)axisAngle[2] * uniqieCoefficients[0];
            encoderDataAxis2 = (int)axisAngle[3] * uniqieCoefficients[1];
            encoderDataAxis3 = (int)axisAngle[4] * uniqieCoefficients[2];

            //IO.axis1.Set(ControlMode.Position, (int)axisAngel[2]-prevEncoderData);// konum temelli sürüş için test edilmeli


            // while (!(VelocityDetermination(3) == 0 && VelocityDetermination(2) == 0 && VelocityDetermination(1) == 0)) 
            // {
            IO.axis1.Set(ControlMode.PercentOutput, VelocityDetermination(1));
            IO.axis2.Set(ControlMode.PercentOutput, VelocityDetermination(2));
            IO.axis3.Set(ControlMode.PercentOutput, VelocityDetermination(3));
            // Debug.Print("çıkmadııı");
            Watchdog.Feed();

            // } 
            // IO.axis1.Set(ControlMode.PercentOutput, 0);
            // IO.axis2.Set(ControlMode.PercentOutput, 0);
            // IO.axis3.Set(ControlMode.PercentOutput, 0);
            //  IO.axis1.Set(ControlMode.PercentOutput, -0.5);
            //  IO.axis2.Set(ControlMode.PercentOutput, 0.5);
            //  IO.axis3.Set(ControlMode.PercentOutput, -0.5);

            //  IO.axis1.Set(ControlMode.PercentOutput, -0.5);
            //prevEncoderData = (int)axisAngel[2];
            //  Watchdog.Feed();


        }

        public static double VelocityDetermination(int axisNumber)// PID yerine kullanılabilcek alternetif hız fonksiyonu
                                                                  //denenmeli ve katsayılar belirlenmeli
                                                                  //hız değeri döndürür
        {

            if (axisNumber == 1)
            {
                double velocity = 0;

                int encoder1Now = IO.axis1.GetSelectedSensorPosition();

                if (encoderDataAxis1 - encoder1Now >= 26000)
                {
                    velocity = 0.5;
                }
                else if (26000 > encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now >= 13000)
                {
                    velocity = 0.5;
                }
                else if (13000 > encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now >= 6500)
                {
                    velocity = 0.25;
                    Debug.Print("111111111111");
                }
                else if (100000000 > encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now >= 3250)
                {
                    velocity = 0.25;
                    Debug.Print("222222222222222");
                }
                else if (3250 > encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now >= 1625)
                {
                    velocity = 0.25;
                    Debug.Print("333333333333333333");

                }

                else if (1625 > encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now > 300)
                {
                    velocity = 0.25;
                    Debug.Print("444444444444444444444444");
                }

                // **************************************************************************************
                else if (encoderDataAxis1 - encoder1Now <= -26000)
                {
                    velocity = -0.5;
                }
                else if (-26000 < encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now <= -13000)
                {
                    velocity = -0.5;
                }
                else if (-13000 < encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now <= -6500)
                {
                    velocity = -0.25;
                    Debug.Print("555555555555555555555");
                }
                else if (-6500 < -encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now <= -3250)
                {
                    velocity = -0.25;
                    Debug.Print("66666666666666666666666666");
                }
                else if (-3250 < -encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now <= -1625)
                {
                    velocity = -0.25;
                    Debug.Print("777777777777777777777777");
                }
                else if (-1625 < encoderDataAxis1 - encoder1Now && encoderDataAxis1 - encoder1Now < -300)
                {
                    velocity = -0.25;
                    Debug.Print("8888888888888888888888888888");
                }
                else
                {
                    velocity = 0;
                    Debug.Print("9999999999999999999999999999999");
                }
                return velocity;
            }

            else if (axisNumber == 2)
            {

                double velocity = 0;
                int encoder2Now = IO.axis2.GetSelectedSensorPosition();

                if (encoderDataAxis2 - encoder2Now >= 370)
                {
                    velocity = -0.5;
                }
                else if (370 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 185)
                {
                    velocity = -0.5;
                }
                else if (185 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 128)
                {
                    velocity = -0.25;
                    Debug.Print("aaaaaaaaaaaaaaa");
                }
                else if (128 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 64)
                {
                    velocity = -0.25;
                    Debug.Print("bbbbbbbbbbbbb");
                }
                else if (64 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 32)
                {
                    velocity = -0.25;
                    Debug.Print("xxxxxxxxxxxxxxxxxx");
                }
                else if (32 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now > 7)
                {
                    velocity = -0.25;
                    Debug.Print("dddddddddddddd");
                }

                // **************************************************************************************
                else if (encoderDataAxis2 - encoder2Now <= -370)
                {
                    velocity = 0.5;
                }
                else if (-370 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -185)
                {
                    velocity = 0.5;
                }
                else if (-185 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -128)
                {
                    velocity = 0.25;
                    Debug.Print("aaaaaaaaaaaaaaa");
                }
                else if (-128 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -64)
                {
                    velocity = 0.25;
                    Debug.Print("bbbbbbbbbbbbb");
                }
                else if (-64 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -32)
                {
                    velocity = 0.25;
                    Debug.Print("yyyyyyyyyyyyyyyyy");
                }
                else if (-32 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now < -7)
                {
                    velocity = 0.25;
                    Debug.Print("dddddddddddddd");
                }
                else
                {
                    velocity = 0;
                    Debug.Print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
                }
                return velocity;
            }

            else if (axisNumber == 3)
            {
                int encoder3Now = IO.axis3.GetSelectedSensorPosition();
                double velocity = 0;
                if (encoderDataAxis3 - encoder3Now >= 370)
                {
                    velocity = -0.5;
                }
                else if (370 > encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now >= 185)
                {
                    velocity = -0.5;
                }
                else if (185 > encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now >= 128)
                {
                    velocity = -0.25;
                    Debug.Print("aaaaaaaaaaaaaaa");
                }
                else if (128 > encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now >= 64)
                {
                    velocity = -0.25;
                    Debug.Print("bbbbbbbbbbbbb");
                }
                else if (64 > encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now >= 32)
                {
                    velocity = -0.25;
                    Debug.Print("zzzzzzzzzzzzzzzzz");
                }
                else if (32 > encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now > 7)
                {
                    velocity = -0.25;
                    Debug.Print("dddddddddddddd");
                }


                // **************************************************************************************
                else if (encoderDataAxis3 - encoder3Now <= -370)
                {
                    velocity = 0.5;
                }
                else if (-370 < encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now <= -185)
                {
                    velocity = 0.5;
                }
                else if (-185 < encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now <= -128)
                {
                    velocity = 0.25;
                    Debug.Print("aaaaaaaaaaaaaaa");
                }
                else if (-128 < encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now <= -64)
                {
                    velocity = 0.25;
                    Debug.Print("bbbbbbbbbbbbb");
                }
                else if (-64 < encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now <= -32)
                {
                    velocity = 0.25;
                    Debug.Print("wwwwwwwwwwwwwwwwwwwwwwwwwwww");
                }
                else if (-32 < encoderDataAxis3 - encoder3Now && encoderDataAxis3 - encoder3Now < -7)
                {
                    velocity = 0.25;
                    Debug.Print("dddddddddddddd");
                }
                else
                {
                    velocity = 0;
                    Debug.Print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
                }

                return velocity;

            }

            return 0;

            // wrote the code with for loop
            // not complated
            // not tested
            // for commiting

            /*  
            for (int i=1; i<3; i++ )
            {
                if (i == axisNumber)
                {
                    string axisString = "axis" + i;
                    int encoderNow = IO.axisString.GetSelectedSensorPosition();
                    double velocity = 0;
                    int encoder2Now = IO.axis2.GetSelectedSensorPosition();

                    if (encoderDataAxis2 - encoder2Now >= 370)
                    {
                        velocity = -0.5;
                    }
                    else if (370 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 185)
                    {
                        velocity = -0.5;
                    }
                    else if (185 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 128)
                    {
                        velocity = -0.25;
                        Debug.Print("aaaaaaaaaaaaaaa");
                    }
                    else if (128 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 64)
                    {
                        velocity = -0.25;
                        Debug.Print("bbbbbbbbbbbbb");
                    }
                    else if (64 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now >= 32)
                    {
                        velocity = -0.25;
                        Debug.Print("xxxxxxxxxxxxxxxxxx");
                    }
                    else if (32 > encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now > 7)
                    {
                        velocity = -0.25;
                        Debug.Print("dddddddddddddd");
                    }

                    // **************************************************************************************
                    else if (encoderDataAxis2 - encoder2Now <= -370)
                    {
                        velocity = 0.5;
                    }
                    else if (-370 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -185)
                    {
                        velocity = 0.5;
                    }
                    else if (-185 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -128)
                    {
                        velocity = 0.25;
                        Debug.Print("aaaaaaaaaaaaaaa");
                    }
                    else if (-128 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -64)
                    {
                        velocity = 0.25;
                        Debug.Print("bbbbbbbbbbbbb");
                    }
                    else if (-64 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now <= -32)
                    {
                        velocity = 0.25;
                        Debug.Print("yyyyyyyyyyyyyyyyy");
                    }
                    else if (-32 < encoderDataAxis2 - encoder2Now && encoderDataAxis2 - encoder2Now < -7)
                    {
                        velocity = 0.25;
                        Debug.Print("dddddddddddddd");
                    }
                    else
                    {
                        velocity = 0;
                        Debug.Print("eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee");
                    }
                    return velocity;
                }
            }
            */

        }

    }
}
