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

import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.ColorSensorV3;



public class Intake extends SubsystemBase {

private final CANSparkMax intakeBottom = new CANSparkMax(IntakeConstants.kBottomIntakeID,MotorType.kBrushless );
private final TalonFX talonIntake = new TalonFX (IntakeConstants.kIntakeID);

private final I2C.Port i2cPort = I2C.Port.kOnboard;
private final ColorSensorV3 m_colorSensorV3 = new ColorSensorV3(i2cPort);

  /** Creates a new Intake. */
  public Intake() {
    intakeBottom.setSecondaryCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeJoystickControl(double speed){
    
    //SmartDashboard.putNumber("proximity sensor", m_colorSensorV3.getProximity());

    talonIntake.set(((speed/m_colorSensorV3.getProximity())*-100));
    intakeBottom.set(((speed/m_colorSensorV3.getProximity())*-100));
  }

  public void intakeRun(){
    talonIntake.set(1);
     intakeBottom.set(1);
  }

  public void intakeInterupt(){
    
    talonIntake.set(0);
  }

  
  public void intakeStop(){
    talonIntake.set(0);


}
}

