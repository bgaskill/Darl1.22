// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;
import com.ctre.phoenix6.hardware.TalonFX;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private final TalonFX talonClimber = new TalonFX (ClimberConstants.kClimberID);



  public Climber() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberControl(double speed){

    talonClimber.set(speed);
  }




}
