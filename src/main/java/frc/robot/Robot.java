// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
     

  }
/* 
  public void updatePoseEstimatorWithVisionBotPose() {
    PoseLatency visionBotPose = m_visionSystem.getPoseLatency();
    // invalid LL data
    if (visionBotPose.pose2d.getX() == 0.0) {
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = m_poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(visionBotPose.pose2d.getTranslation());

    if (m_visionSystem.areAnyTargetsValid()) {
      double xyStds;
      double degStds;
      // multiple targets detected
      if (m_visionSystem.getNumberOfTargetsVisible() >= 2) {
        xyStds = 0.5;
        degStds = 6;
      }
      // 1 target with large area and close to estimated pose
      else if (m_visionSystem.getBestTargetArea() > 0.8 && poseDifference < 0.5) {
        xyStds = 1.0;
        degStds = 12;
      }
      // 1 target farther away and estimated pose is close
      else if (m_visionSystem.getBestTargetArea() > 0.1 && poseDifference < 0.3) {
        xyStds = 2.0;
        degStds = 30;
      }
      // conditions don't match to add a vision measurement
      else {
        return;
      }

      m_poseEstimator.setVisionMeasurementStdDevs(
          VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      m_poseEstimator.addVisionMeasurement(visionBotPose.pose2d,
          Timer.getFPGATimestamp() - visionBotPose.latencySeconds);
    }
  }
  
  public void smartGyroReset() {
    // Get rotation offset from Limelight
  double gryoFixy = 0; //the 0 might be wrong, try 180 if it doesn't work right. More tweaking needs to be done because the gryo my physically be a lil off
  if (botpose.length == 6) {
              
              double limelightRotationOffset = gryoFixy - botpose[5];
  
              // Print the yaw value
              System.out.println("limelightRotationOffset: " + limelightRotationOffset);
          } else {
               double limelightRotationOffset = gryoFixy - limelightTable.getEntry("ts").getDouble(0.0); 
          }        
  
  
      
      // Calculate offset angle between Limelight and robot front
      offsetAngle = limelightRotationOffset;
      
      // Adjust gyro angle with offset angle
      gyro.reset(); //cheeky lil reset just to make it all 0 to be safe
      gyro.adjustAngle(offsetAngle); //SHOULD make the gryo say hey I'm at this angle
}
*/ 
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}



//141