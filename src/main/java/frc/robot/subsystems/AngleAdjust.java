// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleAdjustConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

public class AngleAdjust extends SubsystemBase {

  private final CANSparkMax angler = new CANSparkMax(AngleAdjustConstants.kAngleAdjustID,MotorType.kBrushless );

  private SparkPIDController anglerpidController;
  private RelativeEncoder anglerencoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new AngleAdjust. */
  public AngleAdjust() {

    anglerpidController = angler.getPIDController();
     anglerencoder = angler.getEncoder();

    // PID coefficients
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    anglerpidController.setP(kP);
    anglerpidController.setI(kI);
    anglerpidController.setD(kD);
    anglerpidController.setIZone(kIz);
    anglerpidController.setFF(kFF);
    anglerpidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
// read PID coefficients from SmartDashboard
double p = SmartDashboard.getNumber("P Gain", 0);
double i = SmartDashboard.getNumber("I Gain", 0);
double d = SmartDashboard.getNumber("D Gain", 0);
double iz = SmartDashboard.getNumber("I Zone", 0);
double ff = SmartDashboard.getNumber("Feed Forward", 0);
double max = SmartDashboard.getNumber("Max Output", 0);
double min = SmartDashboard.getNumber("Min Output", 0);
double rotations = SmartDashboard.getNumber("Set Rotations", 0);

// if PID coefficients on SmartDashboard have changed, write new values to controller
if((p != kP)) { anglerpidController.setP(p); kP = p; }
if((i != kI)) { anglerpidController.setI(i); kI = i; }
if((d != kD)) { anglerpidController.setD(d); kD = d; }
if((iz != kIz)) { anglerpidController.setIZone(iz); kIz = iz; }
if((ff != kFF)) { anglerpidController.setFF(ff); kFF = ff; }
if((max != kMaxOutput) || (min != kMinOutput)) { 
  anglerpidController.setOutputRange(min, max); 
  kMinOutput = min; kMaxOutput = max;

  anglerpidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", anglerencoder.getPosition());


  }


}
    
  public void angleAdjustJoystickControl(double speed)
  {  
  angler.set(speed);
  }

public void angleAdjustStop(double speed)
  {  
  angler.set(0);
  }  

  public void angleAdjustPresets()
  {  
  
  }



}
