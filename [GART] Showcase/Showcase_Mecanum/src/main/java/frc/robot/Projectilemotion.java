package frc.robot;

import static frc.robot.Constants.FIELD_DATA.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.Nat;
public class Projectilemotion { 
    /* ___Program for calculate the initial velocity of the motor to reach the target___ */
    
    public double calculateInitVertical() {
        double v_0 = sqrt*(Math.pow(Math.E, 2*g*target_height/sqrt)-1);
        return v_0;
    }

    public double calculateTimetravel() {
        return vt/g*Math.atan(calculateInitVertical()/vt);
    }

    public double ZerothFuntion(double variable) {
        return 1+variable*calculateTimetravel()/alpha;
    }

    public double FirstFunction(double init_z, double init_x) {
        return Math.sqrt(2*Math.log(ZerothFuntion(init_z)*ZerothFuntion(init_x)));
    }

    public Matrix<N2,N2> JacobinMatrix(double init_z, double init_x, double angle) {
        Matrix<N2,N2> jacobin = new Matrix<>(Nat.N2(), Nat.N2());
        jacobin.set(0, 0, calculateTimetravel()/(FirstFunction(init_z, init_x)*ZerothFuntion(init_x)));
        jacobin.set(0, 1, calculateTimetravel()/(FirstFunction(init_z, init_x)*ZerothFuntion(init_z)));
        jacobin.set(1, 0, 1);
        jacobin.set(1, 1, -Math.tan(angle));
        return jacobin;
    }

    public Matrix<N2,N1> FunctionMatrix(double init_x, double init_z, double angle) {
        Matrix<N2,N1> function = new Matrix<>(Nat.N2(), Nat.N1());
        function.set(0, 0, alpha*FirstFunction(init_z, init_x));
        function.set(1, 0, init_x - Math.tan(angle)*init_z);
        return function;
    }

    public Matrix<N2,N1> VariableMatrix(double init_x, double init_z) {
        Matrix<N2,N1> variable = new Matrix<>(Nat.N2(), Nat.N1());
        variable.set(0, 0, init_x);
        variable.set(1, 0, init_z);
        return variable;
    }

    public Matrix<N2,N1> SolvingEquation(double init_x, double init_z, double angle, double velocity) {
        Matrix<N2,N1> variable = VariableMatrix(init_x, init_z).minus(JacobinMatrix(init_z, init_x, angle).inv().times(FunctionMatrix(init_x, init_z, angle)));
        Matrix<N2,N1> delta = variable.minus(VariableMatrix(init_x, init_z));
        if (delta.get(0, 0) < 0.001 && delta.get(1, 0) < 0.001) {
            variable.set(1, 0, variable.get(1,0) - velocity);
            return variable;
        }
        else {
            variable = SolvingEquation(variable.get(0, 0), variable.get(1, 0), angle, velocity);
            return variable;
        }
    }

    public double VelocityCalculation(double init_x, double init_y, double angle, double velocity) {
        double y_axis = calculateInitVertical();
        double x_axis = SolvingEquation(init_x, init_y, angle, velocity).get(0,0);
        double z_axis = SolvingEquation(init_x, init_y, angle, velocity).get(1,0);
        return Math.sqrt(Math.pow(y_axis,2)+Math.pow(x_axis,2)+Math.pow(z_axis,2)) + tolerance;
    }

    // Link https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/flight-equations-with-drag/
    // https://towardsdatascience.com/design-of-a-double-flywheel-variable-angle-ball-shooter-33221fa64866
}
