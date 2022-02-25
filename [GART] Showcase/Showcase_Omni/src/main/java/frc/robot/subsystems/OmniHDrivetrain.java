// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.OMNIH.*;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.Joystick.*;
import static frc.robot.Constants.SPEED.*;

public class OmniHDrivetrain extends SubsystemBase {
  /** Creates a new OmniHdb. */
  public static WPI_TalonSRX vertmaster_left = new WPI_TalonSRX(VERTMASTER_LEFT_ID);
  public static WPI_TalonSRX vertmaster_right = new WPI_TalonSRX(VERTMASTER_RIGHT_ID);
  public static WPI_TalonSRX vertfollow_left = new WPI_TalonSRX(VERTFOLLOW_LEFT);
  public static WPI_TalonSRX vertfollow_right = new WPI_TalonSRX(VERTFOLLOW_RIGHT);
  public static WPI_TalonSRX hori1 = new WPI_TalonSRX(HORIMASTER_MOTOR1_ID);
  public static WPI_TalonSRX hori2 = new WPI_TalonSRX(HORIMASTER_MOTOR2_ID);
 

  public OmniHDrivetrain() {
    vertfollow_left.follow(vertmaster_left);
    vertfollow_right.follow(vertmaster_right);
    hori2.follow(hori1);

    vertmaster_left.setInverted(true);
    vertmaster_right.setInverted(false);

    vertmaster_left.setNeutralMode(NeutralMode.Brake);
    vertmaster_right.setNeutralMode(NeutralMode.Brake);
    hori1.setNeutralMode(NeutralMode.Brake);
  }
  public static void drive(double left, double right, double hori){
    vertmaster_left.set(left);
    vertmaster_right.set(right);
    hori1.set(hori);
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double l2 = RobotContainer.controller.getRawButton(L2)? 0.8 : 0.4;
    ///////////////////////////////FOR A JS FOR A SIMPLE DB, JUST PRESS L1
    //Go straight forward
    if(RobotContainer.controller.getRawAxis(1) < -0.5  && RobotContainer.controller.getRawButton(L1) ){
      drive(l2 * maxSpeed, l2 * maxSpeed , l2 * 0);
    }
    //Go straight backward
    else if(RobotContainer.controller.getRawAxis(1) > 0.5  && RobotContainer.controller.getRawButton(L1)){
      drive(l2 * -maxSpeed, l2 * -maxSpeed , l2 * -0);
    }
    //////////////////////////////

    //Turn left forward
     else if(RobotContainer.controller.getRawAxis(0) < -0.5 && RobotContainer.controller.getRawButton(L1)) {
      drive(l2 * maxSpeed, l2 * minSpeed , l2 * 0);
    }
    //Turn right forward
    else if(RobotContainer.controller.getRawAxis(0) > 0.5 && RobotContainer.controller.getRawButton(L1)) {
      drive(l2 * minSpeed, l2 * maxSpeed , l2 * 0);
    }
    //Turn left backward
    else if(RobotContainer.controller.getRawAxis(0) < -0.5 && RobotContainer.controller.getRawButton(L1)) {
      drive(l2 * -maxSpeed, l2 *- minSpeed , l2 * 0);
    }
    //Turn right
    else if(RobotContainer.controller.getRawAxis(0) > 0.5 && RobotContainer.controller.getRawButton(L1)) {
      drive(l2 * -minSpeed, l2 * -maxSpeed , l2 * 0);
    }


    ////////////////////////////////////FOR A JS FOR OMNI DB, DO NOT PRESS L1
    
    //Go straight forward
    else if(RobotContainer.controller.getRawAxis(1) < -0.35){
      drive(l2 * -RobotContainer.controller.getRawAxis(1), l2 * -RobotContainer.controller.getRawAxis(1) , l2 * 0);
    }
    //Go straight backward
    else if(RobotContainer.controller.getRawAxis(1) > 0.35){
      drive(l2 * RobotContainer.controller.getRawAxis(1), l2 * RobotContainer.controller.getRawAxis(1) , l2 * 0);
    }

    //Straight right side
    else if(RobotContainer.controller.getRawAxis(0) > 0.5){
      drive(l2 * 0, l2 * 0, l2 * maxSpeed);
    }
    //Straight left side
    else if(RobotContainer.controller.getRawAxis(0) < -0.5){
      drive(l2 * 0, l2 * 0, l2 * -maxSpeed);
    }

    ////////////////////////////////

    //Turn round right
    else if(RobotContainer.controller.getRawAxis(2) > 0.5){
      drive(l2 * maxSpeed, l2 * - maxSpeed, 0);
    }
    //Turn round left
    else if(RobotContainer.controller.getRawAxis(2) < -0.5){
      drive(l2 * -maxSpeed, l2 * minSpeed, 0);
    }

    ///////////////////////////////Angle 
    double horiV = RobotContainer.controller.getRawAxis(2) * l2;
    double leftV = RobotContainer.controller.getRawAxis(0) * l2;
    double rightV = RobotContainer.controller.getRawAxis(0) * l2;
    //Cross right 
    if (RobotContainer.controller.getRawAxis(1) > 0.35 && RobotContainer.controller.getRawAxis(3) > 0.35){
      drive(leftV, rightV, horiV);
    }
    //Cross left 
    else if(RobotContainer.controller.getRawAxis(1) > 0.35 && RobotContainer.controller.getRawAxis(3) < -0.35){
      drive(leftV, rightV, - horiV);
    }
    

    

    
    }

    


    

  }
