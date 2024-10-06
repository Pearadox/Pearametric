// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

public class Elevate extends Command {
  Elevator elev = Elevator.getInstance();

  /** Creates a new Elevate. */
  public Elevate() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elev.elevHold();

    if (RobotContainer.driverController.getRawButton(XboxController.Button.kY.value)) {
      elev.setHighHoop();
    } else if (RobotContainer.driverController.getRawButton(XboxController.Button.kX.value)) {
      elev.setMidHoop();
    } else if (RobotContainer.driverController.getRawButton(XboxController.Button.kA.value)) {
      elev.setLowHoop();
    } else if (RobotContainer.driverController.getRawButton(XboxController.Button.kB.value)) {
      elev.setStowed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elev.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
