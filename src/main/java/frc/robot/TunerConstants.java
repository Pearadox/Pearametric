package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Drivetrain;

public class TunerConstants {

  // Motor ID constants
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 5;
  private static final int kFrontLeftEncoderId = 11;
  private static final double kFrontLeftEncoderOffset = -0.415;

  private static final int kFrontRightDriveMotorId = 2;
  private static final int kFrontRightSteerMotorId = 6;
  private static final int kFrontRightEncoderId = 12;
  private static final double kFrontRightEncoderOffset = -0.317;

  private static final int kBackLeftDriveMotorId = 3;
  private static final int kBackLeftSteerMotorId = 7;
  private static final int kBackLeftEncoderId = 13;
  private static final double kBackLeftEncoderOffset = -0.389;

  private static final int kBackRightDriveMotorId = 4;
  private static final int kBackRightSteerMotorId = 8;
  private static final int kBackRightEncoderId = 14;
  private static final double kBackRightEncoderOffset = 0.235;

  // Pigeon ID
  private static final int kPigeonId = 15;

  // Drive and steering configuration values
  private static final double kDriveGearRatio = 6.75;
  private static final double kSteerGearRatio = 150.0 / 7;
  private static final double kWheelRadiusInches = 2.0;
  private static final boolean kSteerMotorReversed = true;
  private static final boolean kInvertLeftSide = false;
  private static final boolean kInvertRightSide = true;

  // Control Gains
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // Drive and steering motor configurations
  private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(60)
                  .withStatorCurrentLimitEnable(true));
  private static final CANcoderConfiguration cancoderInitialConfigs = new CANcoderConfiguration();
  private static final Pigeon2Configuration pigeonConfigs = null;

  // Speed and kinematics constants
  private static final double kMaxSpeedMps = 4.0;
  private static final double kMaxAngularSpeedRadPerS = 3.5 * Math.PI;
  private static final double kTrackWidthMeters = Units.inchesToMeters(20.75);
  private static final double kWheelBaseMeters = Units.inchesToMeters(20.75);

  // Module constants factory
  private static final SwerveModuleConstantsFactory ConstantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorGearRatio(kDriveGearRatio)
          .withSteerMotorGearRatio(kSteerGearRatio)
          .withWheelRadius(kWheelRadiusInches)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSteerMotorInverted(kSteerMotorReversed)
          .withDriveMotorInitialConfigs(driveInitialConfigs)
          .withSteerMotorInitialConfigs(steerInitialConfigs)
          .withCANcoderInitialConfigs(cancoderInitialConfigs);

  // Module configurations
  private static final SwerveModuleConstants FrontLeft =
      ConstantCreator.createModuleConstants(
          kFrontLeftSteerMotorId,
          kFrontLeftDriveMotorId,
          kFrontLeftEncoderId,
          kFrontLeftEncoderOffset,
          Units.inchesToMeters(kTrackWidthMeters / 2),
          Units.inchesToMeters(kWheelBaseMeters / 2),
          kInvertLeftSide);

  private static final SwerveModuleConstants FrontRight =
      ConstantCreator.createModuleConstants(
          kFrontRightSteerMotorId,
          kFrontRightDriveMotorId,
          kFrontRightEncoderId,
          kFrontRightEncoderOffset,
          Units.inchesToMeters(kTrackWidthMeters / 2),
          Units.inchesToMeters(-kWheelBaseMeters / 2),
          kInvertRightSide);

  private static final SwerveModuleConstants BackLeft =
      ConstantCreator.createModuleConstants(
          kBackLeftSteerMotorId,
          kBackLeftDriveMotorId,
          kBackLeftEncoderId,
          kBackLeftEncoderOffset,
          Units.inchesToMeters(-kTrackWidthMeters / 2),
          Units.inchesToMeters(kWheelBaseMeters / 2),
          kInvertLeftSide);

  private static final SwerveModuleConstants BackRight =
      ConstantCreator.createModuleConstants(
          kBackRightSteerMotorId,
          kBackRightDriveMotorId,
          kBackRightEncoderId,
          kBackRightEncoderOffset,
          Units.inchesToMeters(-kTrackWidthMeters / 2),
          Units.inchesToMeters(-kWheelBaseMeters / 2),
          kInvertRightSide);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANbusName("rio")
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  public static final Drivetrain DriveTrain =
      new Drivetrain(DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight);
}
