// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
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

  // private static final double compZeroX = 0.0;// for brownout shooter: -0.05;
  // private static final double compZeroY = 0;
  // private static final double compZeroZ = 0.0;// for brownout shooter: -0.06;

  private static final int numComponents = 5;
  private static final int X = 0, Y = 1, Z = 2;
  private static double[][] compZeros = new double[3][numComponents];
  static {
    compZeros[X][0] = -0.35; // bball intake zeroX
    compZeros[Z][0] = -0.2; // bball intake zeroZ
    compZeros[X][4] = 0.34; // eintake intake zeroX
    compZeros[Z][4] = -0.165; // eintake intake zeroZ
  }
    
  private static ShuffleboardTab componentConfig = Shuffleboard.getTab("component");
  
  private static GenericEntry componentNum = componentConfig.add("component num", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", numComponents - 1))
      .withPosition(2, 6).getEntry();
  private static int currNum = (int) (componentNum.getDouble(0) + 0.5);

  private static GenericEntry compX = componentConfig.add("comp x", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(2, 0).getEntry();
  private static GenericEntry compY = componentConfig.add("comp y", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(2, 1).getEntry();
  private static GenericEntry compZ = componentConfig.add("comp z", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(2, 2).getEntry();
  private static GenericEntry compRoll = componentConfig.add("comp roll", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 3).getEntry();
  private static GenericEntry compPitch = componentConfig.add("comp pitch", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 4).getEntry();
  private static GenericEntry compYaw = componentConfig.add("comp yaw", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(2, 5).getEntry();

  private static ShuffleboardTab robotConfig = Shuffleboard.getTab("robot");

  private static GenericEntry robotX = robotConfig.add("robot x", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 4))
      .withPosition(6, 0).getEntry();
  private static GenericEntry robotY = robotConfig.add("robot y", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 4))
      .withPosition(6, 1).getEntry();
  private static GenericEntry robotZ = robotConfig.add("robot z", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 4))
      .withPosition(6, 2).getEntry();
  private static GenericEntry robotRoll = robotConfig.add("robot roll", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(6, 3).getEntry();
  private static GenericEntry robotPitch = robotConfig.add("robot pitch", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(6, 4).getEntry();
  private static GenericEntry robotYaw = robotConfig.add("robot yaw", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(6, 5).getEntry();

  private static Pose3d[] basketballs = Arrays.copyOf(FieldConstants.BASKETBALLS, FieldConstants.BASKETBALLS.length);
  private static boolean[] ballIsPresent = new boolean[FieldConstants.BASKETBALLS.length];
  static { Arrays.fill(ballIsPresent, true); }

  private static Transform3d[] components = new Transform3d[5];
  static { 
    for (int i = 0; i < components.length; i++) { 
      components[i] = new Transform3d(
          -compZeros[X][i],
          -compZeros[Y][i], 
          -compZeros[Z][i],
          new Rotation3d());
    }
  }

  private boolean testing = true;
  private Pose3d robotPose;

  public Sim() {}
  
  @Override
  public void periodic() {
    // testing with brownout in advantagescope, shooter = model_0, amp = model_1
    visualizeOrigin();
    visualizeRobot();
    visualizeComponents();
    // visualizeBasketballs();
  }

  public void visualizeOrigin() {            
    Logger.recordOutput("Sim/Zeroed Pose3d", origin);
  }

  public void visualizeRobot() {
    if (testing) {
      robotPose = new Pose3d(new Translation3d(
          robotX.getDouble(0), 
          robotY.getDouble(0), 
          robotZ.getDouble(0)),
          new Rotation3d(
              Units.degreesToRadians(robotRoll.getDouble(0)),
              Units.degreesToRadians(robotPitch.getDouble(0)),
              Units.degreesToRadians(robotYaw.getDouble(0))));
    } else {
      robotPose = new Pose3d(drivetrain.getPose());
    }

    Logger.recordOutput("Sim/Robot Pose", robotPose);
  }

  public void visualizeComponents() {
    int currNum = (int) (componentNum.getDouble(0) + 0.5);
    Transform3d currComponent = new Transform3d(
        compX.getDouble(0) - compZeros[X][currNum],
        compY.getDouble(0) - compZeros[Y][currNum], 
        compZ.getDouble(0) - compZeros[Z][currNum],
        new Rotation3d(
            Units.degreesToRadians(compRoll.getDouble(0)), 
            Units.degreesToRadians(compPitch.getDouble(0)),
            Units.degreesToRadians(compYaw.getDouble(0))));

    components[currNum] = currComponent;
            
    Logger.recordOutput("Sim/Robot Pose");
    Logger.recordOutput("Sim/Components Tranform3d[]", components );
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
