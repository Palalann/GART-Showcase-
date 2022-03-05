package frc.robot;

public class Projectilemotion { 
    // Program for calculate the initial velocity of the motor to reach the target
    double g = 9.81; // gravity acceleration
    double drag_coefficient = 0.15;
    double radius = 2;
    double mass = 1;
    double air_density = 1206.17; //kg/m^3
    double cross_section_area = Math.pow(radius,2)*Math.PI;
    double vt = Math.sqrt(2*mass*g/ 
    (drag_coefficient*cross_section_area*air_density)); // terminal velocity
    double target_height = 2;
    double tolerance = 0.03;

    public double calculateInitVelocity(double distance, double r, double m_w_ratio) {
        double sqrt = Math.pow(vt, 2);
        double v_0 = sqrt*(Math.pow(Math.E, 2*g*target_height/sqrt)-1);
        double u_0 = vt/Math.atan(v_0/vt)*(Math.pow(Math.E, g*distance/sqrt)-1);
        double init_v = Math.sqrt(v_0+Math.pow(u_0,2)) + tolerance;
        return init_v*2/(r*m_w_ratio) + tolerance;
    }

    // Link https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/
    // https://towardsdatascience.com/design-of-a-double-flywheel-variable-angle-ball-shooter-33221fa64866
}
