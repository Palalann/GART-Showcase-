package frc.robot;

import static frc.robot.Constants.FIELD_DATA.*;
public class Projectilemotion { 
    /* ___Program for calculate the initial velocity of the motor to reach the target___ */
    
    public double calculateInitVelocity(double distance, double r, double m_w_ratio) {
        // r is the radius of the shooter's wheel
        double sqrt = Math.pow(vt, 2);
        double v_0 = sqrt*(Math.pow(Math.E, 2*g*target_height/sqrt)-1);
        double u_0 = vt/Math.atan(v_0/vt)*(Math.pow(Math.E, g*distance/sqrt)-1);
        double init_v = Math.sqrt(v_0+Math.pow(u_0,2)) + tolerance;
        return init_v*2/(r*m_w_ratio) + tolerance;
    }

    // Link https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/
    // https://towardsdatascience.com/design-of-a-double-flywheel-variable-angle-ball-shooter-33221fa64866
}
