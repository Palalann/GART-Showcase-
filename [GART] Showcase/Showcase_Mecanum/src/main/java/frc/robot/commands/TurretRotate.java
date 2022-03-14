// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.VisionPhoton;
import frc.robot.subsystems.Turret;
import static frc.robot.Constants.SHOOTER_CONSTANTS.*;
import static frc.robot.RobotContainer.*;

public class TurretRotate extends CommandBase {
  private double init_aim;
  private VisionPhoton vision = new VisionPhoton(photon_camera);
  private Turret spinner = new Turret();
  
  /** Creates a new TurretRotate. */
  public TurretRotate(double init) {
    init_aim = init;
    addRequirements(spinner);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current_yaw = photon_camera.getLatestResult().getBestTarget().getYaw();
    double delta = Math.abs(current_yaw-init_aim);
    if (delta > const_delta) {
      init_aim = -current_yaw;
    }
    spinner.rotate(vision.rotateTurret(init_aim));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
