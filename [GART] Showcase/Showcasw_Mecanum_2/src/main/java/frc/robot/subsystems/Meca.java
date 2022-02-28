// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.RobotContainer;
import static frc.robot.Constants.MOTOR.*;
import static frc.robot.Constants.controller.*;


public class Meca extends SubsystemBase {
  /** Creates a new Meca. */
  public static WPI_TalonSRX leftFront = new WPI_TalonSRX(LFMOTOR);
  public static WPI_TalonSRX rightFront = new WPI_TalonSRX(RFMOTOR);
  public static WPI_TalonSRX leftBack = new WPI_TalonSRX(LBMOTOR);
  public static WPI_TalonSRX rightBack = new WPI_TalonSRX(RBMOTOR);

  public static MecanumDrive meca = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);

  public Meca() {
    leftBack.setInverted(false);
    leftFront.setInverted(false);
    rightBack.setInverted(true);
    rightFront.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double max = RobotContainer.controller.getRawButton(L1)? 0.8 : 0.4;
    meca.driveCartesian(RobotContainer.controller.getRawAxis(1) * max,
                       RobotContainer.controller.getRawAxis(0) * max, 
                       RobotContainer.controller.getRawAxis(2) * max);
  }
}
