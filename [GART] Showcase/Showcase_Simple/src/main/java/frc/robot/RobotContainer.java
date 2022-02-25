// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Shoot;
import frc.robot.commands.Eatball;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.MEASURES.*;
import static frc.robot.Constants.SYSID.*;
import static frc.robot.Constants.PATH1.*;
import static frc.robot.Constants.POINT.*;
import static frc.robot.Constants.TRAJ.*;
import static frc.robot.Constants.PiAiDi.*;
import static frc.robot.Constants.RAMSETE.*;
import static frc.robot.Constants.Joystick.*;
import static frc.robot.Constants.OTH_SPEED.*;
import java.util.ArrayList;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 
  public static final Joystick joystick = new Joystick(1);

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TRACKWIDTH);
  public final Drivebase db = new Drivebase();
  public final Shooter m_shooter = new Shooter();
  public final Intake m_intake = new Intake();

  public Command shoot = new Shoot(m_shooter, shootV);
  public  Command intake = new Eatball(m_intake, inV);
  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Configure default commands
    db.setDefaultCommand(new RunCommand(() -> db.arcadeDrive(joystick.getRawAxis(1), 
                          joystick.getRawAxis(2)), db));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick, Tri).whenPressed(shoot);
    new JoystickButton(joystick, O).whenPressed(intake);
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    var VoltConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(kS, kV, kA), kinematics, maxVoltage);
    ///////////////
    var start1 = new Pose2d(start1_x, start1_y, Rotation2d.fromDegrees(start1_theta));
    var end1 = new Pose2d(end1_x, end1_y, Rotation2d.fromDegrees(end1_theta));

   var point = new ArrayList<Translation2d>();
    point.add(new Translation2d(point1x, point1y));

    TrajectoryConfig cf = new TrajectoryConfig(maxV, maxA);
    cf.setStartVelocity(startV);
    cf.setEndVelocity(endV); 

    var traj1 = TrajectoryGenerator.generateTrajectory(start1,point, end1, cf);
    double duration = traj1.getTotalTimeSeconds();
    Trajectory.State wpoint = traj1.sample(1.2);
 
    Field2d m_field = new Field2d();
    SmartDashboard.putData(m_field);

    // Push the trajectory to Field2d.
    m_field.getObject("trajectory 1").setTrajectory(traj1);

    RamseteController m_disabledRamsete = new RamseteController(); //Constructing a new Ramsete controller
    m_disabledRamsete.setEnabled(false);
    RamseteCommand ramsete =
        new RamseteCommand(
            traj1,
            db::getPose, 
            new RamseteController(B, Zeta),
            new SimpleMotorFeedforward(
                kS,kV, kA),
            kinematics,
            db::getWheelSpeeds,
            new PIDController(KPL, 0, 0),
            new PIDController(KPR, 0, 0),
            // RamseteCommand passes volts to the callback
            db::driveVolt,
            db);
    // Reset odometry to the starting pose of the trajectory.
    db.resetOdometry(traj1.getInitialPose());

    // Run path following command, then stop at the end.
    return ramsete.andThen(() -> db.driveVolt(0, 0));
    //return new SequentialCommandGroup(ramsete.andThen(() -> db.driveVolt(0, 0)), new Shoot(m_shooter, 0.8));
   
  }

}

