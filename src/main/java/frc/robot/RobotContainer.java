// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//check speed change buttons  drive subsystem

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

//Chooser Set up
  SendableChooser<Command> chooser = new SendableChooser<>();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
      // chooser stuff
    chooser.addOption("Exit Only", getAutonomousCommand());
    chooser.addOption("Shoot and exit", getAutonomousCommand2());
    chooser.addOption("Straight to Centerline 45 3m/s", getAutonomousCommand3());
    chooser.addOption("Shoot 2", getAutonomousCommand4());
    chooser.addOption("4 shot no aim", getAutonomousCommand5());
    chooser.addOption("4 shot dev", getAutonomousCommand6());
    chooser.addOption("Red HomeWrekker", getAutonomousCommand7());
    chooser.addOption("Blue HomeWrekker", getAutonomousCommand8());
    //chooser.addOption("Red Four", getAutonomousCommand9());
    //chooser.addOption("Blue Four", getAutonomousCommand10());
    
    SmartDashboard.putData(chooser);
   
    SmartDashboard.putNumber("high speed", 4.8);
    SmartDashboard.putNumber("low speed", 2.0);


    
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
    
    //changes max speed to low
    new JoystickButton(m_driverController, 3)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.changeSpeedLow(),
                m_robotDrive));
    
    //changes max speed to high
    new JoystickButton(m_driverController, 4)
            .whileTrue(new RunCommand(
                () -> m_robotDrive.changeSpeedHigh(),
                m_robotDrive));

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
                    () -> m_shooter.shooterAmpRPM(), 
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


//                  Auto1: Exit Only


//Exit Only
  public Command getAutonomousCommand() {
 // Create config for trajectory
 TrajectoryConfig config = new TrajectoryConfig(
    AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straight = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(2.2, 0, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        straight,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straight.getInitialPose());

// Run path following command, then stop at the end.
return swerveControllerCommand2
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
}


//                                            Auto 2: Shoot and exit


public Command getAutonomousCommand2() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
     AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigth2Note = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(2, 0, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        straigth2Note,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

Command highSpeaker = new RunCommand(
() -> m_shooter.shooterTrap(), 
m_shooter).withTimeout(1.1);

Command shooterStop = new RunCommand(
() -> m_shooter.shooterStop(), 
m_shooter).withTimeout(1.1);

Command launch = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(1.1);

Command suck = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(3);

Command stopSuck = new RunCommand(
() -> m_intake.intakeStop(), 
m_intake);
    
Command lowerArm = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(-.4), 
m_angleAdjust).withTimeout(1.3);


Command armStop = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(0), 
m_angleAdjust).withTimeout(1.1);

//ParallelCommandGroup

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigth2Note.getInitialPose());

return highSpeaker
.andThen(launch)
.andThen(swerveControllerCommand2)
.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//.andThen(launch)
;
}
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));;
// Run path following command, then stop at the end.
//return highSpeaker.
//andThen(launch).andThen(lowerArm).andThen(armStop).andThen(suck).
//andThen(swerveControllerCommand2).
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));



//                              Auto 3:to centerline 3 m/s and 45 to right


public Command getAutonomousCommand3() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
     3,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigthGamePiece2 = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(2, 0), new Translation2d(5, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(5.8, 0, new Rotation2d(45)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        straigthGamePiece2,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);



    
// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigthGamePiece2.getInitialPose());


 

     
// Run path following command, then stop at the end.
return swerveControllerCommand3
    //.andThen(m_robotDrive.run(() -> m_robotDrive.drive(0.2, 0, 0, false, false))).withTimeout(5.0)
    .andThen(m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, false)));
}

   
//                              Auto 4: Shoot 2


public Command getAutonomousCommand4() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
     AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigth2Note = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.5, 0), new Translation2d(1, 0)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(1.5, 0, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
        straigth2Note,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

Command highSpeaker = new RunCommand(
() -> m_shooter.shooterTrap(), 
m_shooter).withTimeout(1.1);

Command shooterStop = new RunCommand(
() -> m_shooter.shooterStop(), 
m_shooter).withTimeout(1.1);

Command launch = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(1.1);

Command suck = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(3);

Command stopSuck = new RunCommand(
() -> m_intake.intakeStop(), 
m_intake).withTimeout(1);
    
Command lowerArm = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(-.5), 
m_angleAdjust).withTimeout(1.5);


Command armStop = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(0), 
m_angleAdjust).withTimeout(1.1);

//ParallelCommandGroup

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigth2Note.getInitialPose());

return highSpeaker
.andThen(launch)
.andThen(lowerArm)
.andThen(armStop)
.andThen(swerveControllerCommand4)
.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//.andThen(launch)
;
}
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));;
// Run path following command, then stop at the end.
//return highSpeaker.
//andThen(launch).andThen(lowerArm).andThen(armStop).andThen(suck).
//andThen(swerveControllerCommand2).
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));




//                      Auto 5: 4 note no airm


public Command getAutonomousCommand5() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
     1.5,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigth2Note = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.5, 1.2), new Translation2d(1.5, 1.2)
, new Translation2d(.5, 0),  new Translation2d(1.5, 0)
, new Translation2d(.5, -1.20),  new Translation2d(1.5, -1.2)
    //new Translation2d(1, 2),new Translation2d(1.5, 2)
),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(5.8, -1.2, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
        straigth2Note,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

Command highSpeaker = new RunCommand(
() -> m_shooter.shooterTrap(), 
m_shooter).withTimeout(.5);

Command shooterStop = new RunCommand(
() -> m_shooter.shooterStop(), 
m_shooter).withTimeout(1.1);

Command launch = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(.5);

Command suck = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(3);

Command stopSuck = new RunCommand(
() -> m_intake.intakeStop(), 
m_intake).withTimeout(1);
    
Command lowerArm = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(-.4), 
m_angleAdjust).withTimeout(1.3);


Command armStop = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(0), 
m_angleAdjust).withTimeout(1.1);

//ParallelCommandGroup

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigth2Note.getInitialPose());

return highSpeaker
.andThen(launch)
.andThen(lowerArm)
.andThen(armStop)
.andThen(swerveControllerCommand4)
.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//.andThen(launch)
;
}
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));;
// Run path following command, then stop at the end.
//return highSpeaker.
//andThen(launch).andThen(lowerArm).andThen(armStop).andThen(suck).
//andThen(swerveControllerCommand2).
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));


//                      Auto 6:4 shot dev


public Command getAutonomousCommand6() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
     1.5,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigth2Note = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.5, 1.0), new Translation2d(1.3, 1.1)
 
    //new Translation2d(1, 2),new Translation2d(1.5, 2)
),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(.5, 0, new Rotation2d(.6)),
    config);

Trajectory middleNote = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.5, 0), new Translation2d(1.3, 0)
 
    //new Translation2d(1, 2),new Translation2d(1.5, 2)
),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(1.6, 0, new Rotation2d(-.1)),
    config);

Trajectory sideNote = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(.2, -.5), new Translation2d(1.1, -1.2)
 
    //new Translation2d(1, 2),new Translation2d(1.5, 2)
),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(1.7, -1.4, new Rotation2d(-.4)),
    config);
var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand4 = new SwerveControllerCommand(
        straigth2Note,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        SwerveControllerCommand swerveControllerCommand14 = new SwerveControllerCommand(
        middleNote,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

        SwerveControllerCommand swerveControllerCommand24 = new SwerveControllerCommand(
        sideNote,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


Command highSpeaker = new RunCommand(
() -> m_shooter.shooterTrap(), 
m_shooter).withTimeout(.5);

Command shooterStop = new RunCommand(
() -> m_shooter.shooterStop(), 
m_shooter).withTimeout(1.1);

Command launch = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(.5);

Command suck = new RunCommand(
() -> m_intake.intakeRun(), 
m_intake).withTimeout(3);

Command stopSuck = new RunCommand(
() -> m_intake.intakeStop(), 
m_intake).withTimeout(1);
    
Command lowerArm = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(-.4), 
m_angleAdjust).withTimeout(1.4);


Command armStop = new RunCommand(
() -> m_angleAdjust.angleAdjustAuto(0), 
m_angleAdjust).withTimeout(1.1);

//ParallelCommandGroup

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigth2Note.getInitialPose());

return highSpeaker
.andThen(launch)
.andThen(lowerArm)
.andThen(armStop)
.andThen(swerveControllerCommand4)
.andThen(swerveControllerCommand14)
.andThen(swerveControllerCommand24)
.andThen(() -> m_robotDrive.drive(0, 0, 0, false, false))
//.andThen(launch)
;
}
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));;
// Run path following command, then stop at the end.
//return highSpeaker.
//andThen(launch).andThen(lowerArm).andThen(armStop).andThen(suck).
//andThen(swerveControllerCommand2).
//andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));


//                      Auto 7:

public Command getAutonomousCommand7() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
    1,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigthGamePiece = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, -.4), new Translation2d(3, -.4)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(4.75, -.1, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
        straigthGamePiece,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);


    
// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigthGamePiece.getInitialPose());


 

     
// Run path following command, then stop at the end.
return swerveControllerCommand2
    //.andThen(m_robotDrive.run(() -> m_robotDrive.drive(0.2, 0, 0, false, false))).withTimeout(5.0)
    .andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));
}



//                              Auto 8:


//red Outside
public Command getAutonomousCommand8() {
// Create config for trajectory
TrajectoryConfig config = new TrajectoryConfig(
    1,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics);
   
// An example trajectory to follow. All units in meters.
Trajectory straigthGamePiece2 = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, .3), new Translation2d(3.3, .3)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(4.75, 0, new Rotation2d(0)),
    config);


var thetaController = new ProfiledPIDController(
    AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);


    SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
        straigthGamePiece2,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);



    
// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(straigthGamePiece2.getInitialPose());


 

     
// Run path following command, then stop at the end.
return swerveControllerCommand3
    //.andThen(m_robotDrive.run(() -> m_robotDrive.drive(0.2, 0, 0, false, false))).withTimeout(5.0)
    .andThen(m_robotDrive.run(() -> m_robotDrive.drive(0, 0, 0, false, false)));
}
  }

