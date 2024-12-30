// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.PearadoxTalonFX;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  private PearadoxTalonFX HopTransportMotor;
  private PearadoxTalonFX HopOuttakeMotor;

  private static final Hopper HOPPER = new Hopper();

  public static Hopper getInstance() {
    return HOPPER;
  }
  public Hopper() {
    HopTransportMotor = new PearadoxTalonFX(HopperConstants.HOP_OUTTAKE_MOTOR_ID, NeutralModeValue.Coast, 70, false);
    HopOuttakeMotor = new PearadoxTalonFX(HopperConstants.HOP_OUTTAKE_MOTOR_ID, NeutralModeValue.Coast, 70, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stash(){
    //initial phase of stashing all the balls before dump
    HopTransportMotor.setInverted(true);
    HopOuttakeMotor.setInverted(true);
    HopTransportMotor.set(-0.5);
    HopTransportMotor.set(-0.5);

  }
  
  public void dump(){
    //dumps the electrolytes
    HopTransportMotor.set(0.5);
    HopTransportMotor.set(-0.5);
  }

  public void outtake(){
    //outtake electrolytes if overflow/issue
    HopTransportMotor.set(0.5);
    HopTransportMotor.set(0.5);
  }

  public void stop(){
    HopTransportMotor.set(0);
    HopOuttakeMotor.set(0);
  }
}
