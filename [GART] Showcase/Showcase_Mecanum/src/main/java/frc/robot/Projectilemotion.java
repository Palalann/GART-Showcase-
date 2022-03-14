package frc.robot;

import static frc.robot.Constants.FIELD_DATA.*;
public class Projectilemotion { 
    /* ___Program for calculate the initial velocity of the motor to reach the target___ */
    
    public double calculateInitY_axis() {
        double v_0 = sqrt*(Math.pow(Math.E, 2*g*target_height/sqrt)-1);
        return v_0;
    }

    public double calculateTimetravel() {
        return vt/g*Math.atan(calculateInitY_axis()/vt);
    }

    public double horizontalFunction(double t_camera_frame) {
        return alpha*(Math.pow(Math.E, t_camera_frame/alpha)-1)/calculateTimetravel();
    }

    public double calculateInitX_axis(double tx_camera_frame, double velocity, double theta) {
        return horizontalFunction(tx_camera_frame) - velocity*Math.sin(theta);
    }

    public double calculateInitZ_axis(double tz_camera_frame, double velocity, double theta) {
        return horizontalFunction(tz_camera_frame) - velocity*Math.cos(theta);
    }

    public double calculateInitVelocity(double tx_camera_frame, double tz_camera_frame, double velocity, double theta) {
        double sqrt_vx = Math.pow(calculateInitX_axis(tx_camera_frame, velocity, theta),2);
        double sqrt_vy = Math.pow(calculateInitY_axis(),2);
        double sqrt_vz = Math.pow(calculateInitZ_axis(tz_camera_frame, velocity, theta),2);
        return Math.sqrt(sqrt_vx+sqrt_vy+sqrt_vz) + tolerance;
    }

    
 
    // Link https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/
    // https://towardsdatascience.com/design-of-a-double-flywheel-variable-angle-ball-shooter-33221fa64866
}
