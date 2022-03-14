// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.RobotContainer.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Turret extends SubsystemBase {
  private WPI_TalonSRX rotate = turret;
  /** Creates a new Turret. */
  public Turret() {}

  public void rotate(double velocity) {
    rotate.set(velocity);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
