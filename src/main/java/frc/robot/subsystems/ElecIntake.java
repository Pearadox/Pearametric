// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.lib.util.SmarterDashboard;
import frc.robot.Constants.IntakeConstants;

public class ElecIntake extends SubsystemBase {
private PearadoxTalonFX elecIntakeMotor;

  private static final ElecIntake ELEC_INTAKE = new ElecIntake();

  public static ElecIntake getInstance() {
    return ELEC_INTAKE;
  }
  
  public ElecIntake () {
    elecIntakeMotor = new PearadoxTalonFX(IntakeConstants.ELEC_INTAKE_MOTOR_ID, NeutralModeValue.Coast, 70, false);
  }
  @Override
  public void periodic() {
    SmarterDashboard.putNumber("ElecIntake Pos", elecIntakeMotor.getPosition().getValueAsDouble(), "ElecIntake");
  }
  public void intakeIn(){
    elecIntakeMotor.set(0.5);
  }
  public void intakeOut(){
    elecIntakeMotor.set(-0.5);
  }
  public void stop(){
    elecIntakeMotor.set(0);
  }
}
