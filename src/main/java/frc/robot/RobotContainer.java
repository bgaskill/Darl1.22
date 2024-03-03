// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.AngleAdjust;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;


import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final AngleAdjust m_angleAdjust = new AngleAdjust();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();

  // The driver's controller               replace joystick with xbox controller to use
  Joystick m_driverController = new Joystick (OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick (OIConstants.kOperatorControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                //Xbox controller VVV
                //-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                //-MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                //-MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getZ(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));


    //rightstick for Arm
    m_angleAdjust.setDefaultCommand(new RunCommand(() -> m_angleAdjust.angleAdjustJoystickControl(m_operatorController.getRawAxis(5)), m_angleAdjust));

    //leftstick for <intake
    m_intake.setDefaultCommand(new RunCommand(() -> m_intake.intakeJoystickControl(m_operatorController.getRawAxis(1)*.6), m_intake));
 
    m_climber.setDefaultCommand(new RunCommand(() -> m_climber.climberControl(0), m_climber));

    m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.shooterStop(), m_shooter));
}



  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */

   //Reset Gyro
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, 7)
    .whileTrue(new RunCommand(
        () -> m_robotDrive.zeroHeading(),
        m_robotDrive));     
    
    //wheels in x position--brake--driver top middle button    
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

   //climber--back button and right trigger  forward only
 new JoystickButton(m_operatorController, 7)
            .whileTrue(new RunCommand(
                    () -> m_climber.climberControl(m_operatorController.getRawAxis(3)*-1), 
                    m_climber));                    
      
  //Fire--Driver trigger                  
new JoystickButton(m_driverController, 1)
.whileTrue(new RunCommand(
    () -> m_intake.intakeRun(), 
    m_intake)); 
        
//Speaker shot  y button
new JoystickButton(m_operatorController, 4)
            .whileTrue(new RunCommand(
                    () -> m_shooter.shooterSpeaker(), 
                    m_shooter));   

                    //manual shooter speed start and left trigger
new JoystickButton(m_operatorController, 8)
            .whileTrue(new RunCommand(
                    () -> m_shooter.shooterControl(m_operatorController.getRawAxis(2)*-1), 
                    m_shooter));   

                    //amp shot
new JoystickButton(m_operatorController, 1)
            .whileTrue(new RunCommand(
                    () -> m_shooter.shooterAmp(), 
                    m_shooter));   

                   //trap shot
new JoystickButton(m_operatorController, 2)
            .whileTrue(new RunCommand(
                    () -> m_shooter.shooterTrap(), 
                    m_shooter));   

                    //shooter intake
new JoystickButton(m_operatorController, 3)
            .whileTrue(new RunCommand(
                    () -> m_shooter.shooterSuck(), 
                    m_shooter));   



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1.2, 0), new Translation2d(0, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(1.2, 1, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
  }
}
