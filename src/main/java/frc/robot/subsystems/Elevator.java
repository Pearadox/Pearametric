// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.PearadoxSparkMax;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final PearadoxSparkMax elevMotor;
  private final RelativeEncoder elevEncoder;
  private final ProfiledPIDController elevPID;
  private final ElevatorFeedforward elevFeedForward;

  public enum ElevMode {
    HighHoop, MidHoop, LowHoop, Stowed
  }

  public ElevMode elevMode = ElevMode.Stowed;

  public static final Elevator ELEVATOR = new Elevator();

  public static Elevator getInstance() {
    return ELEVATOR;
  }

  /** Creates a new Elevator. */
  public Elevator() {
    elevMotor = new PearadoxSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID,
        MotorType.kBrushless, IdleMode.kBrake, 45, false);
    elevEncoder = elevMotor.getEncoder();

    elevPID = new ProfiledPIDController(ElevatorConstants.ELEVATOR_kP,
        ElevatorConstants.ELEVATOR_kI, ElevatorConstants.ELEVATOR_kD,
        new TrapezoidProfile.Constraints(2.45, 2.45));
    elevFeedForward = new ElevatorFeedforward(
        ElevatorConstants.ELEVATOR_kS, ElevatorConstants.ELEVATOR_kG,
        ElevatorConstants.ELEVATOR_kV, ElevatorConstants.ELEVATOR_kA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elev Pos", elevEncoder.getPosition());
    SmartDashboard.putNumber("Elev Current", elevMotor.getOutputCurrent());

  }

  public void reachGoal(double goal) {
    elevPID.setGoal(goal);

    double pidOut = elevPID.calculate(elevEncoder.getPosition());
    double feedForwardOut = elevFeedForward.calculate(elevPID.getSetpoint().velocity);
    elevMotor.setVoltage(pidOut + feedForwardOut);
  }

  public void stop() {
    elevPID.setGoal(0);
    elevMotor.set(0);
  }

  public void elevHold() {
    if (elevMode == ElevMode.HighHoop) {
      reachGoal(ElevatorConstants.HIGH_HOOP_POS);

    } else if (elevMode == ElevMode.MidHoop) {
      reachGoal(ElevatorConstants.MID_HOOP_POS);

    } else if (elevMode == ElevMode.LowHoop) {
      reachGoal(ElevatorConstants.LOW_HOOP_POS);

    } else if (elevMode == ElevMode.Stowed) {
      reachGoal(ElevatorConstants.STOWED_POS);
    }
  }

  public void setHighHoop() {
    elevMode = ElevMode.HighHoop;
  }

  public void setMidHoop() {
    elevMode = ElevMode.MidHoop;
  }

  public void setLowHoop() {
    elevMode = ElevMode.LowHoop;
  }

  public void setStowed() {
    elevMode = ElevMode.Stowed;
  }

  public ElevMode getElevMode() {
    return elevMode;
  }
}
