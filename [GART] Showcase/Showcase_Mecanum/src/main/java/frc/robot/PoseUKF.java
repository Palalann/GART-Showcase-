package frc.robot;

import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.estimator.MerweScaledSigmaPoints;
import static frc.robot.Constants.ROBOT_DATA.*;

public class PoseUKF {
    ChassisSpeeds chassisSpeeds;
    
    Matrix<N3,N1> state = new Matrix<>(Nat.N3(), Nat.N1()); // for x, y
    
    Matrix<N3,N3> p_0 = new Matrix<>(Nat.N3(), Nat.N3());
    Matrix<N3,N3> p_z = new Matrix<>(Nat.N3(), Nat.N3());
    Matrix<N3,N3> p_xz = new Matrix<>(Nat.N3(), Nat.N3());
    
    Matrix<N3,N1> measurement_matrix = new Matrix<>(Nat.N3(), Nat.N1());
    Matrix<N3,N3> h_function = new Matrix<>(Nat.N3(), Nat.N3());
    
    Matrix<N3,N1> state_standard_deviation = new Matrix<>(Nat.N3(), Nat.N1());
    Matrix<N3,N1> output_standard_deviation = new Matrix<>(Nat.N3(), Nat.N1());

    Matrix<N3,N3> m_Q;
    Matrix<N3,N3> m_r;
    
    MerweScaledSigmaPoints<N3> m_set = new MerweScaledSigmaPoints<>(Nat.N3(),0.5,2,0);

    MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
        m_frontLeft, m_frontRight, m_backLeft, m_backRight);

    double dt;
    
    public PoseUKF(double t, double x_t, double y_t, double th_t, double x_s_t, double y_s_t, double th_s_t) {
        dt = t;
        state_standard_deviation.set(0,0,x_t);
        state_standard_deviation.set(1,0,y_t);
        state_standard_deviation.set(2,0,th_t);

        output_standard_deviation.set(0,0,x_s_t);
        output_standard_deviation.set(1,0,y_s_t);
        output_standard_deviation.set(2,0,th_s_t);

        h_function.set(0,0,1);
        h_function.set(1,1,1);
        h_function.set(2,2,1);

        m_Q = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), state_standard_deviation);
        m_r = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), output_standard_deviation);
    }

    // Pre-calculation
    public void convertMotortoRobotframe(double v_1, double v_2, double v_3, double v_4) {
        chassisSpeeds = kinematics.toChassisSpeeds(new MecanumDriveWheelSpeeds(v_1, v_2, v_3, v_4));
    }

    public double robotFrameVx() {
        return chassisSpeeds.vxMetersPerSecond;
    }

    public double robotFrameVy() {
        return chassisSpeeds.vyMetersPerSecond;
    }

    public double robotFrameThetarate() {
        return chassisSpeeds.omegaRadiansPerSecond;
    }

    public void collectMeasurement(double x, double y, double theta) {
        measurement_matrix.set(0,0,x);
        measurement_matrix.set(1,0,y);
        measurement_matrix.set(2,0,theta);
    }

    // limit the range of angle in 0 to 2*pi
    public double angle_normalize(double angle) { 
        angle = angle %(2*Math.PI);
        if (angle > Math.PI){
            angle -= Math.PI;
        }
        return angle;
    }

    // Calculate Sigma point, Wm, Wc
    public Matrix<N3, ?> calculateSigmaPoint() {
        return m_set.sigmaPoints(state, p_0);
    }

    public Matrix<?,N1> calculateWm() {
        return m_set.getWm();
    }

    public Matrix<?,N1> calculateWc() {
        return m_set.getWc();
    }

    // Our system matrix
    public Matrix<N3,N3> CosMatrix(double theta) {
        Matrix<N3,N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        matrix.set(0,0,Math.cos(theta));
        matrix.set(0,1,-Math.sin(theta));
        matrix.set(1,0,Math.sin(theta));
        matrix.set(1,1,Math.cos(theta));
        matrix.set(2,2,1);
        return matrix;
    }

    public Matrix<N3,N3> CosTMatrix() {
        Matrix<N3,N3> matrix = new Matrix<>(Nat.N3(), Nat.N3());
        if (robotFrameThetarate() == 0) {
            matrix.set(0,0,dt);
            matrix.set(1,1,dt);
            matrix.set(2,2,dt);
        }
        else{
            matrix.set(0,0,Math.sin(dt*robotFrameThetarate())/robotFrameThetarate());
            matrix.set(0,1,(Math.cos(dt*robotFrameThetarate())-1)/robotFrameThetarate());
            matrix.set(0,1,(1-Math.cos(dt*robotFrameThetarate()))/robotFrameThetarate());
            matrix.set(1,1,Math.sin(dt*robotFrameThetarate())/robotFrameThetarate());
        }
        return matrix;
    }

    public Matrix<N3,N1> ControlInput() {
        Matrix<N3,N1> control_input = new Matrix<>(Nat.N3(), Nat.N1());
        control_input.set(0,0,robotFrameVx());
        control_input.set(1,0,robotFrameVy());
        control_input.set(2,0,robotFrameThetarate());
        return control_input;
    }

    //The system matrix x = x + deltax 
    public Matrix<N3,N1> PredictY(int y) {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        Matrix<N3,N1> sigma_point = calculateSigmaPoint().extractColumnVector(y);
        
        matrix = (CosMatrix(sigma_point.get(2, 0)).times(
        CosTMatrix()).times(ControlInput()));
        matrix = sigma_point.plus(matrix);
        return matrix;
    }

    public Matrix<N3,N1> PredictX() {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        for(int i=0; i<=7; i++) {
            matrix = matrix.plus(PredictY(i).times(calculateWm().get(i, 0)));
        }
        return matrix;
    }

    public void PredictP() {
        Matrix<N3,N3> zeroth = Matrix.eye(Nat.N3());
        for (int i=0; i<=7; i++) {
            var delta_0 = PredictY(i).minus(PredictX());
            var delta_1 = (PredictY(i).minus(PredictX())).transpose();
            zeroth = zeroth.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
        p_0 = zeroth.plus(m_Q);
    }

    // Update
    public Matrix<N3,N1> ZMatrix(int y) {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        matrix = h_function.times(PredictY(y));
        return matrix;
    }
    
    public Matrix<N3,N1> Meanz() {
        Matrix<N3,N1> matrix = new Matrix<>(Nat.N3(), Nat.N1());
        for (int i = 0; i<=7; i++) {
            matrix = matrix.plus(ZMatrix(i).times(calculateWm().get(i,0)));
        }
        return matrix;
    }

    public void Pz() {
        for (int i=0; i<=7; i++) {
            var delta_0 = ZMatrix(i).minus(Meanz());
            var delta_1 = (ZMatrix(i).minus(Meanz())).transpose();
            p_z = p_z.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
        p_z = p_z.plus(m_r);
    }

    public void Pxz() {
        for (int i=0; i<=7; i++) {
            var delta_0 = PredictY(i).minus(PredictX());
            var delta_1 = 
            (ZMatrix(i).minus(Meanz())).transpose();
            p_xz = p_xz.plus(delta_0.times(delta_1).times(calculateWc().get(i,0)));
        }
        p_xz = p_xz.plus(m_r);
    }

    public Pair<Matrix<N3,N1>, Matrix<N3,N3>> calculate() {
        measurement_matrix.set(2,0,angle_normalize(measurement_matrix.get(2,0)));

        PredictP();
        Pz();
        Pxz();

        var k_gain = p_xz.times(p_z.inv());
        var y = measurement_matrix.minus(Meanz());
        
        state = state.plus(k_gain.times(y));
        p_0 = p_0.minus(k_gain.times(p_z).times(k_gain.transpose()));
        return new Pair<>(state, p_0);
    }
}

