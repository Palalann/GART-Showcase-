// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

//import com.kauailabs.navx.frc.AHRS;

//import com.kauailabs.navx.frc.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import static frc.robot.Constants.MECA.*;
import static frc.robot.Constants.controller.*;
import static frc.robot.Constants.Speed.*;


/** Add your docs here. */
public class Drivebase extends SubsystemBase
{
    public static WPI_TalonSRX rightFront = new WPI_TalonSRX(RFMOTOR);
    public static WPI_TalonSRX leftFront = new WPI_TalonSRX(LFMOTOR);
    public static WPI_TalonSRX rightBack = new WPI_TalonSRX(RBMOTOR);
    public static WPI_TalonSRX leftBack = new WPI_TalonSRX(LBMOTOR);

    public static MecanumDrive meca;
    //public static AHRS ahrs;

    public Drivebase(){      
        //ahrs = new AHRS(SPI.Port.kMXP);

        final MotorControllerGroup frontLeft = new MotorControllerGroup(leftFront);
        frontLeft.setInverted(false);
        final MotorControllerGroup frontRight = new MotorControllerGroup(rightFront, rightBack);
        frontRight.setInverted(true);
        final MotorControllerGroup backLeft = new MotorControllerGroup(leftBack);
        frontLeft.setInverted(false);
        final MotorControllerGroup backRight = new MotorControllerGroup(rightBack);
        frontRight.setInverted(true);
        meca = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
    }
    public static void mdrive(double lfS, double rfS, double lbS, double rbS){
        rightFront.set(rfS);
        leftFront.set(lfS);
        rightBack.set(rfS);
        leftBack.set(lbS);
    }
   
    /*public static void drive(double ySpeed, double xSpeed, double zRotation){
        meca.driveCartesian(ySpeed, xSpeed, zRotation);
     //mecDrive.driveCartesian(RobotContainer.controller.getRawAxis(1),RobotContainer.controller.getRawAxis(0), RobotContainer.controller.getRawAxis(0));
    }*/


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*double l2 = RobotContainer.controller.getRawButton(L2)? 0.8: 0.4;
        drive(l2 * maxSpeed * RobotContainer.controller.getRawAxis(1),
            l2 * maxSpeed * RobotContainer.controller.getRawAxis(1),
            l2 * maxRotation * RobotContainer.controller.getRawAxis(1)/RobotContainer.controller.getRawAxis(0) );*/
    
    double l2 = RobotContainer.controller.getRawButton(L2)? 0.8 : 0.4;
    double lfV =  Math.sin( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double rfV =  Math.cos( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double lbV =  Math.cos( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double rbV =  Math.sin( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double vrot =  Math.sqrt(Math.pow(RobotContainer.controller.getRawAxis(1), 2) + Math.pow(RobotContainer.controller.getRawAxis(0), 2));
    //lf = v sin(theta + pi/4) + vrot
    //rf = v cos(theta + pi/4) - vrot
    //lb = v cos(theta + pi/4) + vrot
    //rb = v sin(theta + pi/4) - vrot

    
    //Drive Straight  
    if (RobotContainer.controller.getRawAxis(1) < -0.3 || RobotContainer.controller.getRawAxis(1) > 0.3){
        mdrive(l2 * -RobotContainer.controller.getRawAxis(1), 
        l2 * -RobotContainer.controller.getRawAxis(1),
        l2* -RobotContainer.controller.getRawAxis(1), 
        l2* -RobotContainer.controller.getRawAxis(1));
    }

    //Hori Straight
    else if (RobotContainer.controller.getRawAxis(0) < -0.3 || RobotContainer.controller.getRawAxis(0) > 0.3){
        mdrive(l2 * RobotContainer.controller.getRawAxis(0) ,
        l2 * -RobotContainer.controller.getRawAxis(0), 
        l2 * - RobotContainer.controller.getRawAxis(0), 
        l2 * RobotContainer.controller.getRawAxis(0));
    }
    
    //Cross  
    else if ((RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) < -0.3) || 
            (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) > 0.3) ||
            (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) > 0.3) ||
            (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) < -0.3)) {
        mdrive(maxSpeed * lfV + vrot, maxSpeed * rfV - vrot, maxSpeed * lbV + vrot, maxSpeed * rbV - vrot);
    }
/*45o
    //Cross Forward Right 
    else if (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) > 0.3){
        mdrive(0, l2 * maxSpeed , l2 * maxSpeed, 0);
    }
    //Cross Forward Left 
    else if (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) < -0.3){
        mdrive(l2 * maxSpeed, 0 ,0 ,l2 * maxSpeed);
    }
    //Cross Backward Right
    else if (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) > 0.3){
        mdrive(l2 * -maxSpeed , 0 ,l2 * -maxSpeed ,0);
    }
    //Cross Forward Left 
    else if (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) < -0.3){
        mdrive(0, l2 * -maxSpeed,0 ,l2 * -maxSpeed);
    }*/

    //Turn right
    if (RobotContainer.controller.getRawAxis(2) > 0.3){
        mdrive(l2 * maxSpeed, l2 * -maxSpeed, l2 * maxSpeed, l2 * -maxSpeed);
    }
    //Turn left
    else if (RobotContainer.controller.getRawAxis(2) > 0.3){
        mdrive(- l2 * maxSpeed, l2 * maxSpeed, l2 * -maxSpeed, l2 * maxSpeed);
    }
    //Curve traj right
    else if (RobotContainer.controller.getRawAxis(2) > 0.5 && RobotContainer.controller.getRawAxis(5) < -0.5){
        mdrive(l2 * maxSpeed, l2 * minSpeed, l2 * maxSpeed, l2 * minSpeed);
    }
    //Curve traj left
    else if (RobotContainer.controller.getRawAxis(2) > 0.5 && RobotContainer.controller.getRawAxis(5) < -0.5){
        mdrive(l2 * minSpeed, l2 * maxSpeed, l2 * minSpeed, l2 * maxSpeed);
    }
}
}
