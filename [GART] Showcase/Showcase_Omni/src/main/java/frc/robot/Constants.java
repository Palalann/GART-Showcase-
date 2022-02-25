// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class OMNIH{
        public static final int VERTMASTER_LEFT_ID = 1;
        public static final int VERTMASTER_RIGHT_ID = 2;
        public static final int VERTFOLLOW_LEFT = 3;
        public static final int VERTFOLLOW_RIGHT = 4;
        public static final int HORIMASTER_MOTOR1_ID = 5;
        public static final int HORIMASTER_MOTOR2_ID = 6;
        
    }
    public static final class SPEED{
        public static final double maxSpeed = 0.8;
        public static final double minSpeed = 0.4;
    }
    public static final class Joystick{
        public static final int Sq = 1;
        public static final int X = 2;
        public static final int O = 3;
        public static final int Tri = 4;
       
        public static final int L1= 5;
        public static final int R1 = 6;
        public static final int L2 = 7;
        public static final int R2 = 8;

        public static final int L3 = 11;
        public static final int R3 = 12;

        public static final int Share = 9;
        public static final int Option = 10;
        public static final int TouchPad = 13;
         
        public static final int Up = 14;
        public static final int Down = 15;
        public static final int Left = 16;
        public static final int Right = 17;
               
    }
    public static final class RAMSETE{
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public static final class SYSID{
        public static final double ks = 1; //Volt
        public static final double kv= 1; //Volt x second / meter
        public static final double ka = 1; //Volt x second^2 / meter
        
        public static final double kP = 1; //Volt x second / meter, drive velocity
    }
    public static class MEASURES{
        public static final int TRACKWIDTH = 2; //meters
        public static final int WHEEL_RADIUS = 2; //meters
        public static final int ENCODER_RESOLUTION = 2; //meters

        public static final int kMaxAngularSpeed = 2;
        public static final int kMaxSpeed = 2;
    }
    public static final class PiAiDi{
        public static final int kP = 1;
        public static final int kI = 1;
        public static final int kD = 1;
        
        public static final int KPL = 1;
        public static final int KIL = 1;
        public static final int KDL = 1;

        public static final int KPR = 1;
        public static final int KIR = 1;
        public static final int KDR = 1;

        public static final int kToleranceDegree = 1;
        public static final int kAngularVelocity = 1;
    }
}
