package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;
import static frc.robot.Constants.CAMERA_DATA.*;

public class VisionPhoton {
    PhotonCamera camera;    
    PIDController turn_Controller = new PIDController(angular_kp, 0, angular_kd);

    public VisionPhoton(PhotonCamera photonCamera) {
        turn_Controller.setTolerance(0.08);
        turn_Controller.setIntegratorRange(0, 2*Math.PI);
        camera = photonCamera;
    }

    public PhotonTrackedTarget getBestTarget(PhotonPipelineResult result) {
        return result.getBestTarget();
    }

    public Pose2d getPose2d(
        PhotonPipelineResult result, double gyro_angle, Pose2d pose_1, Transform2d pose_2) {
        
        Pose2d pose = PhotonUtils.estimateFieldToRobot(
        camera_height, 
        target_height, 
        camera_pitch, 
        -result.getBestTarget().getPitch(), 
        new Rotation2d(-result.getBestTarget().getYaw()), 
        new Rotation2d(gyro_angle), 
        pose_1, 
        pose_2);
        return pose;
    }

    /*
    _______Demo program for measuring the position of robot in robot frame______
    Timer time = new Timer();
    double dt = 2;
    public void demoProgram() {
        time.start();
        if (time.hasElapsed(dt)){
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()){
                Pose2d pose = getPose2d(result, gyro_angle, pose_1, pose_2)
                Using the UnscentedKalmanFilter
            }
        }
    }
    */

    public double rotateTurret(double alpha) {
        double velocity = turn_Controller.calculate(alpha, 0);
        return velocity;
    }
}
