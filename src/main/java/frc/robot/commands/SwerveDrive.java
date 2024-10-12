// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;

public class SwerveDrive extends Command {
  private Drivetrain drivetrain = Drivetrain.getInstance();

  // private final SwerveRequest.FieldCentric drive =
  //   new SwerveRequest.FieldCentric()
  //       .withDeadband(SwerveConstants.TELE_DRIVE_MAX_SPEED * 0.1)
  //       .withRotationalDeadband(
  //           SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED * 0.1)
  //       .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.swerveDrive(
        -RobotContainer.driverController.getLeftY(), 
        -RobotContainer.driverController.getLeftX(), 
        -RobotContainer.driverController.getRightX(),
        !RobotContainer.driverController.getRawButton(XboxController.Button.kB.value),
        new Translation2d(),
        true);
    // drivetrain
    //     .applyRequest(
    //         () ->
    //             drive
    //                 .withVelocityX(-RobotContainer.driverController.getLeftY() * SwerveConstants.TELE_DRIVE_MAX_SPEED)
    //                 .withVelocityY(-RobotContainer.driverController.getLeftX() * SwerveConstants.TELE_DRIVE_MAX_SPEED)
    //                 .withRotationalRate(
    //                     -RobotContainer.driverController.getRightX() * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED))
    //     .ignoringDisable(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
