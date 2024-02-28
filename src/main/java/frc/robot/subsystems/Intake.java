// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleAdjustConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;




public class Intake extends SubsystemBase {

private final CANSparkMax intakeBottom = new CANSparkMax(IntakeConstants.kBottomIntakeID,MotorType.kBrushless );
private final TalonFX talonIntake = new TalonFX (IntakeConstants.kIntakeID);



  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeJoystickControl(double speed){
    
    talonIntake.set(speed*-1);
    intakeBottom.set(speed*-1);
  }

  public void intakeRun(){
    talonIntake.set(IntakeConstants.intakeSpeed);
  }

  

  
  public void intakeStop(){
    talonIntake.set(0);


}
}

