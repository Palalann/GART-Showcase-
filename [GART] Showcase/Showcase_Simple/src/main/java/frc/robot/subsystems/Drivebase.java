// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.lang.Math;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.DRIVEBASE.*;
import static frc.robot.Constants.PiAiDi.*;
import static frc.robot.Constants.SYSID.*;
import static frc.robot.Constants.MEASURES.*;
import static frc.robot.Constants.Joystick.*;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */
  //Left drive
  public static WPI_TalonSRX leftMaster = new WPI_TalonSRX(LEFTMAS_MOTOR);
  public static WPI_TalonSRX leftFollow = new WPI_TalonSRX(LEFTFOL_MOTOR);
  public final MotorControllerGroup left = new MotorControllerGroup(leftMaster,leftFollow);
  //Right drive
  public static WPI_TalonSRX rightMaster = new WPI_TalonSRX(RIGHTMAS_MOTOR);
  public static WPI_TalonSRX rightFollow = new WPI_TalonSRX(RIGHTFOL_MOTOR);
  public final MotorControllerGroup right = new MotorControllerGroup(rightMaster, rightFollow);

  //Drive
  public DifferentialDrive drive = new DifferentialDrive(left, right);
  //Encoders
  public static final Encoder leftEncoder = new Encoder(0, 1);
  public static final Encoder rightEncoder = new Encoder(2, 3);

  //Gyros
  AHRS gyro = new AHRS();

  //PID
  public static final PIDController leftPID = new PIDController(KPL, KIL, KDL);
  public static final PIDController rightPID = new PIDController(KPR, KIR, KDR);
 
  //Kinematics 
  public DifferentialDriveKinematics kine = new DifferentialDriveKinematics(TRACKWIDTH);
  //Odometry
  public DifferentialDriveOdometry odometry;

  //Slew Rate Limiter
  public SlewRateLimiter speedLimiter = new SlewRateLimiter(3);
  public SlewRateLimiter rotateLimiter = new SlewRateLimiter(3);
  
  //Compute feedforward
  public final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  //Joystick
  public static Joystick joystick = new Joystick(1);

  //Field2d
  private final Field2d field = new Field2d();
 
  public Drivebase() {
    //[GYRO] reset
    gyro.reset();
    //[WHEEL]
    left.setInverted(true);
    right.setInverted(false);
    //[ENCODER] take distance per pulse
    leftEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS/ ENCODER_RESOLUTION );
    rightEncoder.setDistancePerPulse(2 * Math.PI * WHEEL_RADIUS/ ENCODER_RESOLUTION );
    //[ENCODER] reset
    leftEncoder.reset();
    rightEncoder.reset();
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    SmartDashboard.putData("Field", field);
  
  }
  public void setSpeeds(DifferentialDriveWheelSpeeds speed){
    final double leftFeedforward = feedforward.calculate(speed.leftMetersPerSecond);
    final double rightFeedforward = feedforward.calculate(speed.rightMetersPerSecond);
    
    final double leftFeedbackward = leftPID.calculate(leftEncoder.getRate(), speed.leftMetersPerSecond);
    final double rightFeedbackward = leftPID.calculate(rightEncoder.getRate(), speed.leftMetersPerSecond);
    
    right.setVoltage(rightFeedforward + rightFeedbackward);
    left.setVoltage(leftFeedforward + leftFeedbackward);
  }

  public void db(double xspeed, double rotation){
    var wheelSpeed = kine.toWheelSpeeds(new ChassisSpeeds(xspeed, 0.0, rotation));
    setSpeeds(wheelSpeed);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
    
    double l1 = RobotContainer.joystick.getRawButton(L1)? 0.8 : 0.4;
    final var xspeed = -speedLimiter.calculate(joystick.getRawAxis(1) * l1 * maxSpeed);
    final var rotation = -rotateLimiter.calculate((Math.atan(joystick.getRawAxis(1)/ joystick.getRawAxis(0))));
    db(xspeed, rotation);

    field.setRobotPose(odometry.getPoseMeters());
  }

  //Return current pose
  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  //Current wheel speed
  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  //Reset odometry to specified pose
  public void resetOdometry(Pose2d pose){
    rightEncoder.reset();
    leftEncoder.reset();

    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  //Use arcade to drive bot
  public void arcadeDrive(double forward, double rotation){
    drive.arcadeDrive(forward, rotation);
  }

  //Control motor with voltage
  public void driveVolt(double leftV, double rightV){
    left.setVoltage(leftV);
    right.setVoltage(rightV);
    drive.feed();
  }

  //Reset encoder
  public void resetEncoder(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  //In : dist from 2 encoder, out : avr reading
  public double avrReading(){
    return ((leftEncoder.getDistance() + rightEncoder.getDistance())/2);
  }
  
  //Get left encoder
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  //Get right encoder
  public Encoder getRightEncoder(){
    return rightEncoder;
  }

  //MaxOutput
  public void maxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }
  //Zeros the heading of the bot
  public void zeroHead() {
    gyro.reset();
  }

  //Heading of bot
  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }

  //Turn Rate
  public double turnRate(){
    return -gyro.getRate();
  }
}
