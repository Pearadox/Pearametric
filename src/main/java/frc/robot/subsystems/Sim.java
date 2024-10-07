// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;

public class Sim extends SubsystemBase {  
  private static final Sim sim = new Sim();
  public static Sim getInstance() { return sim; }

  private Drivetrain drivetrain = Drivetrain.getInstance();
  
  public final Pose3d origin = new Pose3d(
    new Translation3d(0,0,0), new Rotation3d(0,0,0));

  private static final double shooterZeroX = -0.238;
  private static final double shooterZeroZ = 0.298;
    
  private static ShuffleboardTab componentConfig = Shuffleboard.getTab("shooter");
    
  private static GenericEntry shooterX = componentConfig.add("shooter x", shooterZeroX)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 5))
      .withPosition(2, 0).getEntry();
  private static GenericEntry shooterY = componentConfig.add("shooter y", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 5))
      .withPosition(2, 1).getEntry();
  private static GenericEntry shooterZ = componentConfig.add("shooter z", shooterZeroZ)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 5))
      .withPosition(2, 2).getEntry();
  private static GenericEntry shooterPitch = componentConfig.add("shooter pitch", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 3).getEntry();
  private static GenericEntry shooterRoll = componentConfig.add("shooter roll", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 4).getEntry();
  private static GenericEntry shooterYaw = componentConfig.add("shooter yaw", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 5).getEntry();

  private static ShuffleboardTab robotConfig = Shuffleboard.getTab("robot");

  private static GenericEntry robotX = robotConfig.add("robot x", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 5))
      .withPosition(6, 0).getEntry();
  private static GenericEntry robotY = robotConfig.add("robot y", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 5))
      .withPosition(6, 1).getEntry();
  private static GenericEntry robotYaw = robotConfig.add("robot yaw", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(6, 2).getEntry();

  private static Pose3d[] basketballs = Arrays.copyOf(FieldConstants.BASKETBALLS, FieldConstants.BASKETBALLS.length);
  private static boolean[] ballIsPresent = new boolean[FieldConstants.BASKETBALLS.length];
  static { Arrays.fill(ballIsPresent, true); }

  private boolean testing = true;
  private Pose3d robotPose;

  public Sim() {}
  
  @Override
  public void periodic() {
    // testing with brownout in advantagescope, shooter = model_0, amp = model_1
    visualizeOrigin();
    visualizeRobot();
    visualizeComponents();
    visualizeBasketballs();
  }

  public void visualizeOrigin() {            
    Logger.recordOutput("Sim/Zeroed Pose3d", origin);
  }

  public void visualizeRobot() {
    if (testing) {
      robotPose = new Pose3d(new Pose2d(
          robotX.getDouble(0), robotY.getDouble(0), 
          new Rotation2d(Units.degreesToRadians(robotYaw.getDouble(0)))));
    } else {
      robotPose = new Pose3d(drivetrain.getPose());
    }

    Logger.recordOutput("Sim/Robot Pose", robotPose);
  }

  public void visualizeComponents() {
    // found that shooter should move 20cm +x and 10cm +z when roll is 45 to mimic a pivot point
    // TODO: create more realistic pivot point
    Transform3d shooter = new Transform3d(
        shooterX.getDouble(shooterZeroX) + (0.2 * shooterRoll.getDouble(0) / 45.0), 
        shooterY.getDouble(0), 
        shooterZ.getDouble(shooterZeroZ) + (0.1 * shooterRoll.getDouble(0) / 45.0), 
        new Rotation3d(
            Units.degreesToRadians(shooterPitch.getDouble(0)),
            Units.degreesToRadians(shooterRoll.getDouble(0)), 
            Units.degreesToRadians(shooterYaw.getDouble(0))));

    // TODO: amp bar
    Transform3d amp = new Transform3d(shooterZeroX, 0, shooterZeroZ, new Rotation3d());
    
    Logger.recordOutput("Sim/Robot Pose");
    Logger.recordOutput("Sim/Components Tranform3d[]", new Transform3d[] { shooter, amp });
    // Logger.recordOutput("Sim/Components Pose3d[]", 
    //     new Pose3d[] { robotPose.transformBy(shooter), robotPose.transformBy(amp) });
  }

  public void visualizeBasketballs() {
    for (int i = 0; i < basketballs.length; i++) {
      if (!ballIsPresent[i]) {
        basketballs[i] = null;
      } else if (ballIsPresent[i] && basketballs[i] == null) {
        basketballs[i] = FieldConstants.BASKETBALLS[i];
      }

      // used this to find FieldConstants.BASKETBALL_CAD_OFFSET
      // basketballs[i] = FieldConstants.BASKETBALLS[i].transformBy(
      //     new Transform3d(new Translation3d(
      //         ballOffset.getDouble(0), 0, 0), 
      //     new Rotation3d(0, 0, 0)));

      // makes basketball spin lol
      // basketballs[i] = basketballs[i].transformBy(
      //     new Transform3d(new Translation3d(0, 0, 0), 
      //     new Rotation3d(0, 0.01, 0))); 
    }

    Logger.recordOutput("Sim/Basketballs", basketballs);
  }
}
