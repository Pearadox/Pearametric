// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
private static final Intake INTAKE = new Intake();
private PearadoxSparkMax intakeRoller;
  /** Creates a new Intake. */
  public static Intake getInstance() {
    return INTAKE;
  }
  
  public Intake () {
    intakeRoller = new PearadoxSparkMax(IntakeConstants.INTAKE_ROLLER_ID, MotorType.kBrushless, IdleMode.kCoast, 80, false);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void IntakeIn(){
    intakeRoller.set(0.5);
  }
  public void IntakeOut(){
    intakeRoller.set(-0.5);
  }
  public void IntakeStop(){
    intakeRoller.set(0);
  }
}

