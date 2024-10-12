// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SmarterDashboard;
import frc.robot.TunerConstants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private SwerveModule leftFront;
  private SwerveModule rightFront;
  private SwerveModule leftBack;
  private SwerveModule rightBack;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  private Pigeon2 gyro;

  private SwerveDrivePoseEstimator poseEstimator;

  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  private final SwerveRequest.ApplyChassisSpeeds AutoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  public static Drivetrain getInstance(){
    return TunerConstants.DriveTrain;
  }

  /** Creates a new SwerveDrivetrain. */
  public Drivetrain(SwerveDrivetrainConstants driveConstants, SwerveModuleConstants... modules) {
    super(driveConstants, 0.005, modules);
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    leftFront = new SwerveModule(
      SwerveConstants.LEFT_FRONT_DRIVE_ID, 
      SwerveConstants.LEFT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_FRONT_CANCODER_ID, 
      SwerveConstants.LEFT_FRONT_OFFSET, 
      false);

    rightFront = new SwerveModule(
      SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
      SwerveConstants.RIGHT_FRONT_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
      SwerveConstants.RIGHT_FRONT_OFFSET, 
      false);

    leftBack = new SwerveModule(
      SwerveConstants.LEFT_BACK_DRIVE_ID, 
      SwerveConstants.LEFT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.LEFT_BACK_CANCODER_ID, 
      SwerveConstants.LEFT_BACK_OFFSET, 
      false);
    
    rightBack = new SwerveModule(
      SwerveConstants.RIGHT_BACK_DRIVE_ID, 
      SwerveConstants.RIGHT_BACK_TURN_ID, 
      false, 
      true, 
      SwerveConstants.RIGHT_BACK_CANCODER_ID, 
      SwerveConstants.RIGHT_BACK_OFFSET, 
      false);

    frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    gyro = new Pigeon2(SwerveConstants.PIGEON_ID);

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.DRIVE_KINEMATICS,
      getHeadingRotation2d(),
      getModulePositions(),
      new Pose2d());

    AutoBuilder.configureHolonomic(
      this::getPose,
      this::resetPose,
      this::getRobotRelativeSpeeds,
      this::driveRobotRelative,
      SwerveConstants.AUTO_CONFIG,
      () -> isRedAlliance(),
      this);

      if (Utils.isSimulation()) { startSimThread(); }
  }

  @Override
  public void periodic() {
    poseEstimator.update(getHeadingRotation2d(), getModulePositions());

    SmarterDashboard.putNumber("Robot Angle", getHeading(), "Drivetrain");
    SmarterDashboard.putString("Angular Speed", 
        new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "pi rad/s",
        "Drivetrain");

    SmarterDashboard.putData("Left Front Module State", leftFront.getState(), "Drivetrain");
    SmarterDashboard.putData("Right Front Module State", rightFront.getState(), "Drivetrain");
    SmarterDashboard.putData("Left Back Module State", leftBack.getState(), "Drivetrain");
    SmarterDashboard.putData("Right Back Module State", rightBack.getState(), "Drivetrain");
    SmarterDashboard.putData("Odometry", getPose(), "Drivetrain");
    SmarterDashboard.putData("Module States", getModuleStates(), "Drivetrain");
  }

  public void swerveDrive(double frontSpeed, double sideSpeed, double turnSpeed, 
    boolean fieldOriented, Translation2d centerOfRotation, boolean deadband){ //Drive with rotational speed control w/ joystick
    if(deadband){
      frontSpeed = Math.abs(frontSpeed) > 0.1 ? frontSpeed : 0;
      sideSpeed = Math.abs(sideSpeed) > 0.1 ? sideSpeed : 0;
      turnSpeed = Math.abs(turnSpeed) > 0.1 ? turnSpeed : 0;
    }

    frontSpeed = frontLimiter.calculate(frontSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    sideSpeed = sideLimiter.calculate(sideSpeed) * SwerveConstants.TELE_DRIVE_MAX_SPEED;
    turnSpeed = turnLimiter.calculate(turnSpeed) * SwerveConstants.TELE_DRIVE_MAX_ANGULAR_SPEED;

    ChassisSpeeds chassisSpeeds;
    if(fieldOriented){
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontSpeed, sideSpeed, turnSpeed, getHeadingRotation2d());
    }
    else{
      chassisSpeeds = new ChassisSpeeds(frontSpeed, sideSpeed, turnSpeed);
    }

    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, centerOfRotation);

    this.setControl(AutoRequest.withSpeeds(chassisSpeeds));

    setModuleStates(moduleStates);
  }

  public void setAllIdleMode(boolean brake){
    if(brake){
      leftFront.setBrake(true);
      rightFront.setBrake(true);
      leftBack.setBrake(true);
      rightBack.setBrake(true);
    }
    else{
      leftFront.setBrake(false);
      rightFront.setBrake(false);
      leftBack.setBrake(false);
      rightBack.setBrake(false);
    }
  }

  public void resetAllEncoders(){
    leftFront.resetEncoders();
    rightFront.resetEncoders();
    leftBack.resetEncoders();
    rightBack.resetEncoders();
  }

  public Pose2d getPose(){
    return (this.getState().Pose != null) ? this.getState().Pose : poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
  }

  public void zeroHeading(){
    gyro.setYaw(0);
  }

  public void setHeading(double heading){
    gyro.setYaw(heading);
  }

  public double getHeading(){
    return Math.IEEEremainder(-gyro.getAngle(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    leftFront.stop();
    leftBack.stop();
    rightFront.stop();
    rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    leftFront.setDesiredState(moduleStates[0]);
    rightFront.setDesiredState(moduleStates[1]);
    leftBack.setDesiredState(moduleStates[2]);
    rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = leftFront.getState();
    states[1] = rightFront.getState();
    states[2] = leftBack.getState();
    states[3] = rightBack.getState();
    return states;
  } 

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = leftFront.getPosition();
    positions[1] = rightFront.getPosition();
    positions[2] = leftBack.getPosition();
    positions[3] = rightBack.getPosition();
    return positions;
  }

  public boolean isRedAlliance(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }
}