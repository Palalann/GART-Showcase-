// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MECA{
        public static final int RFMOTOR = 1;
        public static final int LFMOTOR = 2;
        public static final int RBMOTOR = 3;
        public static final int LBMOTOR = 4;
    }
    public static class controller{
        public static final int Sq = 1;
        public static final int X = 2;
        public static final int O = 3;
        public static final int Tri = 4;
    
        public static final int L1 = 5;
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
    public static class Speed{
        public static final double maxSpeed = 0.8;
        public static final double maxRotation = 0.5;
        public static final double minSpeed = 0.4;
    }

    public static class FIELD_DATA {
        public static final double g = 9.81;                // gravity acceleration in m/s^2
        public static final double mass = 1;                // mass of the ball, in kilograms
        public static final double radius = 2;              // radius of the ball in meters
        public static final double target_height = 12;      // in meters
        public static final double air_density = 1206.17;   // kg/m^3 using the p-V-T equations
        public static final double drag_coefficient = 0.15; // by experimental
        public static final double cross_section_area = Math.pow(radius,2)*Math.PI;
        public static final double vt = Math.sqrt(2*mass*g/ 
        (drag_coefficient*cross_section_area*air_density)); // terminal velocity
        public static final double tolerance = 0.03;        // tolerance of the velocity of the shooter
    }

    public static class ROBOT_DATA {
        // the 4 wheels position in the center of robot mass frame
        public static final Translation2d m_frontLeft = new Translation2d(2,3); 
        public static final Translation2d m_frontRight = new Translation2d(2,3);
        public static final Translation2d m_backRight = new Translation2d(2,3);
        public static final Translation2d m_backLeft = new Translation2d(2,3);
    }

    public static class CAMERA_DATA {
        public static final double camera_height = 1;        // compare to the floor
        public static final double target_height = 1;        // compare to the floor
        public static final double camera_pitch = 
        Units.degreesToRadians(23);                          // in field's frame
    }

    public static class SHOOTER_CONSTANTS {
        public static final double angular_kp = 0.5;
        public static final double angular_kd = 0.6;
    }
}
