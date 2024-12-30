// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ElecIntake;
import frc.robot.subsystems.Hopper;

public class ElectInOut extends Command {
  ElecIntake elecIntake = ElecIntake.getInstance();
  Hopper hop = Hopper.getInstance();
  /** Creates a new ElecIntakeIn. */
  public ElectInOut() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elecIntake);
    addRequirements(hop);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
   if (RobotContainer.driverController.getRawButton(XboxController.Button.kX.value)){
      elecIntake.intakeOut();
      hop.outtake();
    } else {
      elecIntake.intakeIn();
      hop.stash();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elecIntake.stop();
    hop.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
