// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.controls.*;



import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

private final TalonFX talonShooter = new TalonFX (ShooterConstants.kShooterID);
private final TalonFX talonShooter2 = new TalonFX (ShooterConstants.kShooter2ID);


  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  
  
  
  }

  public void shooterControl(double speed){

    talonShooter.set(speed*.6);
    talonShooter2.set(speed*.6);
    var rotorRPM = talonShooter.getVelocity();
    var rotorRPMLatency =rotorRPM.getTimestamp().getLatency();
    rotorRPM.waitForUpdate(.020);
    SmartDashboard.putNumber("RPM",rotorRPM.getValue());
  }
public void shooterAmp(){

    talonShooter.set(-.22);
    talonShooter2.set(-.05);
    var rotorRPM = talonShooter.getVelocity();
    var rotorRPMLatency =rotorRPM.getTimestamp().getLatency();
    rotorRPM.waitForUpdate(.020);
    SmartDashboard.putNumber("RPM",rotorRPM.getValue());
    
  }

public void shooterSpeaker(){

    talonShooter.set(-.6);
    talonShooter2.set(-.6);
    var rotorRPM = talonShooter.getVelocity();
    var rotorRPMLatency =rotorRPM.getTimestamp().getLatency();
    rotorRPM.waitForUpdate(.020);
    SmartDashboard.putNumber("RPM",rotorRPM.getValue());
    
  }

public void shooterStop(){

    talonShooter.set(.0);
    talonShooter2.set(0);
   
   
    
  }
public void shooterSuck(){

    talonShooter.set(.3);
    talonShooter2.set(.30);


}
public void shooterTrap(){

    talonShooter.set(-.5);
    talonShooter2.set(-.50);


}

}

//47