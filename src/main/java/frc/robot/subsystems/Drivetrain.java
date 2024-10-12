// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SmarterDashboard;
import frc.robot.TunerConstants;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  // private SwerveModule leftFront;
  // private SwerveModule rightFront;
  // private SwerveModule leftBack;
  // private SwerveModule rightBack;

  private SlewRateLimiter frontLimiter;
  private SlewRateLimiter sideLimiter;
  private SlewRateLimiter turnLimiter;

  // private Pigeon2 gyro;

  // this.poseEstimator = new SwerveDrivePoseEstimator(
  //     m_kinematics,
  //     getHeadingRotation2d(),
  //     getModulePositions(),
  //     new Pose2d());

  private static final double kSimLoopPeriod = 0.02;
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
    super(driveConstants, kSimLoopPeriod, modules);
    new Thread(() -> {
      try{
        Thread.sleep(1000);
        zeroHeading();
      }
      catch(Exception e){}
    }).start();

    // this.leftFront = TunerConstants.DriveTrain.leftFront;
    // this.leftBack = TunerConstants.DriveTrain.leftBack;
    // this.rightFront = TunerConstants.DriveTrain.rightFront;
    // this.rightBack = TunerConstants.DriveTrain.rightBack;
    // this.leftFront = swervemod
    // leftFront = new SwerveModule(
    //   SwerveConstants.LEFT_FRONT_DRIVE_ID, 
    //   SwerveConstants.LEFT_FRONT_TURN_ID, 
    //   false, 
    //   true, 
    //   SwerveConstants.LEFT_FRONT_CANCODER_ID, 
    //   SwerveConstants.LEFT_FRONT_OFFSET, 
    //   false);

    // rightFront = new SwerveModule(
    //   SwerveConstants.RIGHT_FRONT_DRIVE_ID, 
    //   SwerveConstants.RIGHT_FRONT_TURN_ID, 
    //   false, 
    //   true, 
    //   SwerveConstants.RIGHT_FRONT_CANCODER_ID, 
    //   SwerveConstants.RIGHT_FRONT_OFFSET, 
    //   false);

    // leftBack = new SwerveModule(
    //   SwerveConstants.LEFT_BACK_DRIVE_ID, 
    //   SwerveConstants.LEFT_BACK_TURN_ID, 
    //   false, 
    //   true, 
    //   SwerveConstants.LEFT_BACK_CANCODER_ID, 
    //   SwerveConstants.LEFT_BACK_OFFSET, 
    //   false);
    
    // rightBack = new SwerveModule(
    //   SwerveConstants.RIGHT_BACK_DRIVE_ID, 
    //   SwerveConstants.RIGHT_BACK_TURN_ID, 
    //   false, 
    //   true, 
    //   SwerveConstants.RIGHT_BACK_CANCODER_ID, 
    //   SwerveConstants.RIGHT_BACK_OFFSET, 
    //   false);

    frontLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    sideLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ACCELERATION);
    turnLimiter = new SlewRateLimiter(SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION);

    // gyro = this.m_pigeon2; //new Pigeon2(SwerveConstants.PIGEON_ID);
    
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
    // this.poseEstimator.update(getHeadingRotation2d(), getModulePositions());

    try {
    
        SmarterDashboard.putNumber("Robot Angle", getHeading(), "Drivetrain");
        // SmarterDashboard.putString("Angular Speed", 
        //     new DecimalFormat("#.00").format((-gyro.getRate() / 180)) + "pi rad/s",
        //     "Drivetrain");
    
        // SmarterDashboard.putData("Left Front Module State", this.leftFront.getState(), "Drivetrain");
        // SmarterDashboard.putData("Right Front Module State", this.rightFront.getState(), "Drivetrain");
        // SmarterDashboard.putData("Left Back Module State", this.leftBack.getState(), "Drivetrain");
        // SmarterDashboard.putData("Right Back Module State", this.rightBack.getState(), "Drivetrain");
        SmarterDashboard.putData("Odometry", getPose(), "Drivetrain");
        SmarterDashboard.putData("Module States", getModuleStates(), "Drivetrain");
    } catch (Exception e) {
      e.printStackTrace();
    }
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

    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
    
    setModuleStates(moduleStates);
    
    this.setControl(AutoRequest.withSpeeds(chassisSpeeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public void setAllIdleMode(boolean brake){
    // if(brake){
    //   this.leftFront.setBrake(true);
    //   this.rightFront.setBrake(true);
    //   this.leftBack.setBrake(true);
    //   this.rightBack.setBrake(true);
    // }
    // else{
    //   this.leftFront.setBrake(false);
    //   this.rightFront.setBrake(false);
    //   this.leftBack.setBrake(false);
    //   this.rightBack.setBrake(false);
    // }
  }

  public void resetAllEncoders(){
    // this.leftFront.resetEncoders();
    // this.rightFront.resetEncoders();
    // this.leftBack.resetEncoders();
    // this.rightBack.resetEncoders();
  }

  public Pose2d getPose(){
    return /* (this.getState().Pose != null) ? */ this.getState().Pose /* : new Pose2d()*/;
  }

  public void resetPose(Pose2d pose) {
    // this.poseEstimator.resetPosition(getHeadingRotation2d(), getModulePositions(), pose);
    this.seedFieldRelative(pose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    return m_kinematics.toChassisSpeeds(getModuleStates());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(moduleStates);
    this.setControl(AutoRequest.withSpeeds(chassisSpeeds));
  }

  public void zeroHeading(){
    this.m_pigeon2.setYaw(0);
  }

  public void setHeading(double heading){
    this.m_pigeon2.setYaw(heading);
  }

  public double getHeading(){
    return this.getPose().getRotation().getDegrees(); // Math.IEEEremainder(-gyro.getAngle(), 360); //clamp heading between -180 and 180
  }

  public Rotation2d getHeadingRotation2d(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public void stopModules(){
    // this.leftFront.stop();
    // this.leftBack.stop();
    // this.rightFront.stop();
    // this.rightBack.stop();
  }

  public void setModuleStates(SwerveModuleState[] moduleStates){
    // SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.DRIVETRAIN_MAX_SPEED);
    // this.leftFront.setDesiredState(moduleStates[0]);
    // this.rightFront.setDesiredState(moduleStates[1]);
    // this.leftBack.setDesiredState(moduleStates[2]);
    // this.rightBack.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates(){
    // SwerveModuleState[] states = new SwerveModuleState[4];
    // states[0] = this.leftFront.getState();
    // states[1] = this.rightFront.getState();
    // states[2] = this.leftBack.getState();
    // states[3] = this.rightBack.getState();
    // return states;
    return getState().ModuleStates;
  } 

  public SwerveModulePosition[] getModulePositions(){
    // SwerveModulePosition[] positions = new SwerveModulePosition[4];
    // positions[0] = this.leftFront.getPosition();
    // positions[1] = this.rightFront.getPosition();
    // positions[2] = this.leftBack.getPosition();
    // positions[3] = this.rightBack.getPosition();
    return this.m_modulePositions;
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