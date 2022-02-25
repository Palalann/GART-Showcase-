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
    public static class DRIVEBASE{ //motor number
        public static final int LEFTMAS_MOTOR = 1;
        public static final int LEFTFOL_MOTOR = 2;

        public static final int RIGHTMAS_MOTOR = 3;
        public static final int RIGHTFOL_MOTOR = 4;
    }
    public static class OTH_SPEED{
        public static final double shootV = 0.8; //this number should be between -1 and 1
        public static final double inV = 0.8; //this number should be between -1 and 1
    }
    public static class OTH_PART{ //motor number
        public static final int INTAKE = 5;
        public static final int SHOOTER = 6;

        public static final int RIGHTMAS_MOTOR = 3;
        public static final int RIGHTFOL_MOTOR = 4;
    }
    public static class PiAiDi{ 
        public static final int KDL = 1;
        public static final int KPL = 0;
        public static final int KIL = 0;

        public static final int KDR = 1;
        public static final int KPR = 0;
        public static final int KIR = 0;
    }
    public static class MEASURES{
        public static final double TRACKWIDTH = 1; //meters
        public static final double WHEEL_RADIUS = 1;//meters
        public static final double ENCODER_RESOLUTION = 1;

        public static final double maxSpeed = 0.6;//this number should be between -1 and 1
        public static final double maxVoltage = 0.6;//this number should be between -1 and 1
    
        
    }
    public static class SYSID{ //Take this const in sysid
        public static final double kS = 1;
        public static final double kV = 1;
        public static final double kA = 1;
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

    /////AUTO
    public static final class PATH1{
        ///Traj1 white mouse :
        //Go straight. 
        //Start at (1, 4.572)[In the corner]. Heading = 90 ( perpendicular with x)
        //End at (2, 2.572), heading 45
        public static final double start1_x = 0;
        public static final double start1_y = 0;
        public static final double start1_theta = 90;

        public static final double end1_x = 2;
        public static final double end1_y = -2;
        public static final double end1_theta = 90;

        ///Traj2 white mouse : 
        //Start from point (0, 3) [start corner]
        // G0 through point (2, 2)
        //End at point (5, 3) [end corner], heading still the same

        /*public static final double start1_x = 0;
        public static final double start1_y = 0;
        public static final double start1_theta = 90;

        public static final double end1_x = 4;
        public static final double end1_y = -3.5;
        public static final double end1_theta = 90;*/
    }
    public static final class POINT{
        public static final double point1x = 1;
        public static final double point1y = 3.5;

        //[Traj2 white mouse]
        /*public static final double point1x = 2;
        public static final double point1y = -2;*/

    }
    public static final class TRAJ{
        public static final double maxV = 1; // ms^-1
        public static final double maxA = 1;// ms^-2

        public static final double startV = 1; // ms^-1
        public static final double endV = 1; //ms^-1
    }
    public static final class RAMSETE{ //This const should be B = 2 and zeta = 0.7
        public static final double B = 2;
        public static final double Zeta = 0.7;
    }

}
