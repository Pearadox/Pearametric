// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class IOConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OP_CONTROLLER_PORT = 1;
  }

  public static final class SwerveConstants{
    //Drivetrain motor/encoder IDs
    public static final int LEFT_FRONT_DRIVE_ID = 1;
    public static final int RIGHT_FRONT_DRIVE_ID = 2;
    public static final int LEFT_BACK_DRIVE_ID = 3;
    public static final int RIGHT_BACK_DRIVE_ID = 4;
    
    public static final int LEFT_FRONT_TURN_ID = 5;
    public static final int RIGHT_FRONT_TURN_ID = 6;
    public static final int LEFT_BACK_TURN_ID = 7;
    public static final int RIGHT_BACK_TURN_ID = 8;
    
    public static final int LEFT_FRONT_CANCODER_ID = 11;
    public static final int RIGHT_FRONT_CANCODER_ID = 12;
    public static final int LEFT_BACK_CANCODER_ID = 13;
    public static final int RIGHT_BACK_CANCODER_ID = 14;

    public static final int PIGEON_ID = 15;

    //Drivetrain characteristics
    public static final double LEFT_FRONT_OFFSET = -0.415; //change
    public static final double RIGHT_FRONT_OFFSET = -0.317; //change
    public static final double LEFT_BACK_OFFSET = -0.389; //change
    public static final double RIGHT_BACK_OFFSET = 0.235; //change

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_MOTOR_GEAR_RATIO = 6.75;
    public static final double TURN_MOTOR_GEAR_RATIO = 150.0/7;
    public static final double DRIVE_MOTOR_PCONVERSION = WHEEL_DIAMETER * Math.PI / DRIVE_MOTOR_GEAR_RATIO;
    public static final double TURN_MOTOR_PCONVERSION = 2 * Math.PI / TURN_MOTOR_GEAR_RATIO;
    public static final double DRIVE_MOTOR_VCONVERSION = DRIVE_MOTOR_PCONVERSION / 60.0;
    public static final double TURN_MOTOR_VCONVERSION = TURN_MOTOR_PCONVERSION / 60.0;
    public static final double KP_TURNING = 0.5;

    public static final double DRIVETRAIN_MAX_SPEED = 4.0;
    public static final double DRIVETRAIN_MAX_ANGULAR_SPEED = 3.5 * Math.PI;

    //Swerve Kinematics
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.75);
    public static final double WHEEL_BASE = Units.inchesToMeters(20.75);
    public static final double DRIVE_BASE_RADIUS = Math.sqrt(Math.pow(TRACK_WIDTH, 2) + Math.pow(WHEEL_BASE, 2)) / 2.0;

    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)
    );

    //Teleop constraints
    public static final double TELE_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1;
    public static final double TELE_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 1.75;
    public static final double TELE_DRIVE_MAX_ACCELERATION = 3;
    public static final double TELE_DRIVE_MAX_ANGULAR_ACCELERATION = 3;

    //Auton constraints
    public static final double AUTO_kP_TRANSLATION = 0.4;
    public static final double AUTO_kP_ROTATION = 2.4;

    public static final HolonomicPathFollowerConfig AUTO_CONFIG = new HolonomicPathFollowerConfig(
      new PIDConstants(AUTO_kP_TRANSLATION, 0.0, 0.0),
      new PIDConstants(AUTO_kP_ROTATION, 0.0, 0.0),
      DRIVETRAIN_MAX_SPEED, // Max module speed, in m/s
      DRIVE_BASE_RADIUS,
      new ReplanningConfig());

    public static final double AUTO_DRIVE_MAX_SPEED = DRIVETRAIN_MAX_SPEED / 1.5;
    public static final double AUTO_DRIVE_MAX_ANGULAR_SPEED = DRIVETRAIN_MAX_ANGULAR_SPEED / 2.0;
    public static final double AUTO_DRIVE_MAX_ACCELERATION = 3;
    public static final double AUTO_DRIVE_MAX_ANGULAR_ACCELERATION = Math.PI;
  }

  public static final class ElevatorConstants {
    public static final int ELEVATOR_MOTOR_ID = 41;

    public static final double ELEVATOR_kP = 0.25; //TODO: elev PID Constants
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0;

    public static final double ELEVATOR_kS = 0;
    public static final double ELEVATOR_kG = 0;
    public static final double ELEVATOR_kV = 0;
    public static final double ELEVATOR_kA = 0;

    public static final double LOW_HOOP_POS = 3; // TODO: find hoop pos
    public static final double MID_HOOP_POS = 5;
    public static final double HIGH_HOOP_POS = 7;
    public static final double STOWED_POS = 1;

    // how far can each component of the elevator extend vertically
    public static final double MANIPULATOR_MAX_EXTEND = Units.inchesToMeters(86);
    public static final double TOP_STAGE_MAX_EXTEND = Units.inchesToMeters(58);
    public static final double MID_STAGE_MAX_EXTEND = Units.inchesToMeters(30);

    public static final Transform3d HELD_BASKETBALL_POS = new Transform3d(0.25, 0, 0.05, new Rotation3d());
  }
  
  public static final class IntakeConstants{
    public static final int ELEC_INTAKE_MOTOR_ID = 21; //change it later

    public static final Transform3d HELD_ELECTROLYTE_POS = new Transform3d(
      -0.26, FieldConstants.ELECTROLYTE_RADIUS * -2.5, 0.72, new Rotation3d());
    //TODO: find stowed and deployed
  }

  public static final class HopperConstants {
    public static final int HOP_TRANSPORT_MOTOR_ID = 22;
    public static final int HOP_OUTTAKE_MOTOR_ID = 23;  
  }
  
  public static final class IntakeConstants{
    public static final int INTAKE_ROLLER_ID = 21; //chang it later
  }

  public class FieldConstants {
    public static final double FIELD_LENGTH = Units.feetToMeters(54); // Units.inchesToMeters(651.223);
    public static final double FIELD_WIDTH = Units.feetToMeters(27); // Units.inchesToMeters(323.277);
    
    public static final double FIRST_BLUE_BASKETBALL_X = Units.inchesToMeters(24);
    public static final double BASKETBALL_SEPARATION = Units.inchesToMeters(30);
    public static final double BASKETBALL_RADIUS = Units.inchesToMeters(5);
    public static final double BASKETBALL_CAD_OFFSET = Units.inchesToMeters(1.7);

    public static final double ELECTROLYTE_SEPARATION = Units.inchesToMeters(6);
    public static final double ELECTROLYTE_RADIUS = Units.inchesToMeters(2.5);

    // game piece locations
    public static final Pose3d[] BASKETBALLS = new Pose3d[20];
    static {
      for (int i = 0; i < BASKETBALLS.length / 2; i++) {
        BASKETBALLS[i] = new Pose3d(new Translation3d(
            FIRST_BLUE_BASKETBALL_X + BASKETBALL_SEPARATION * i + BASKETBALL_CAD_OFFSET, 
            FIELD_WIDTH / 2, BASKETBALL_RADIUS), new Rotation3d());
      }
      for (int i = 0; i < BASKETBALLS.length / 2; i++) {
        BASKETBALLS[i + BASKETBALLS.length / 2] = new Pose3d(new Translation3d(
            FIELD_LENGTH - FIRST_BLUE_BASKETBALL_X - BASKETBALL_SEPARATION * i + BASKETBALL_CAD_OFFSET, 
            FIELD_WIDTH / 2, BASKETBALL_RADIUS), new Rotation3d());
        // BASKETBALLS[i + BASKETBALLS.length / 2] = flipAlliance(BASKETBALLS[i]);
      }
    }

    public static final Pose3d[] ELECTROLYTES = new Pose3d[8];
    static{
      for (int i = 0; i < ELECTROLYTES.length/2; i++){
        ELECTROLYTES[i] = new Pose3d(new Translation3d(
          ELECTROLYTE_SEPARATION * (i + 1),
          FIELD_WIDTH / 3, ELECTROLYTE_RADIUS), new Rotation3d());
      }
      for (int i = 0; i <ELECTROLYTES.length/2; i++){
        ELECTROLYTES[i + ELECTROLYTES.length / 2] = new Pose3d(new Translation3d(
          FIELD_LENGTH - ELECTROLYTE_SEPARATION * (i + 1),
          FIELD_WIDTH / 3, ELECTROLYTE_RADIUS), new Rotation3d());
      }
    }

    public static final double HOOP_RADIUS = Units.inchesToMeters(12);

    public static final Pose3d HIGH_HOOP = new Pose3d(
        new Translation3d(1.688, FIELD_WIDTH - 6.991, Units.inchesToMeters(90)), new Rotation3d());
    public static final Pose3d MID_HOOP = new Pose3d(
        new Translation3d(2.493, FIELD_WIDTH - 7.452, Units.inchesToMeters(54)), new Rotation3d());
    public static final Pose3d LOW_HOOP = new Pose3d(
        new Translation3d(0.883, FIELD_WIDTH - 6.531, Units.inchesToMeters(36)), new Rotation3d());

    public static final Pose3d[] HOOPS = { 
      HIGH_HOOP, 
      MID_HOOP, 
      LOW_HOOP, 
      flipAlliance(HIGH_HOOP),
      flipAlliance(MID_HOOP),
      flipAlliance(LOW_HOOP),
    };

    public static final double DUNK_TANK_RADIUS = Units.inchesToMeters(24);

    public static final Pose3d BLUE_TANK = new Pose3d(new Translation3d(
        FIELD_LENGTH - Units.inchesToMeters(120),
        Units.inchesToMeters(60), 
        Units.inchesToMeters(24)),
        new Rotation3d());

    public static final Pose3d BONUS_MODE_TANK = new Pose3d(new Translation3d(
        FIELD_LENGTH / 2, FIELD_WIDTH / 2, Units.inchesToMeters(24)), new Rotation3d());

    public static final Pose3d[] DUNK_TANKS = { BLUE_TANK, BONUS_MODE_TANK, flipAlliance(BLUE_TANK) };
    
    public static Pose3d flipAlliance(Pose3d blue) {
      return new Pose3d(new Translation3d(
          FieldConstants.FIELD_LENGTH - blue.getX(),
          FieldConstants.FIELD_WIDTH - blue.getY(),
          blue.getZ()),
          blue.getRotation().plus(new Rotation3d(0, 0, Math.PI)));
    }
  }
}
