// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;

public class Sim extends SubsystemBase {  
  private static final Sim sim = new Sim();
  public static Sim getInstance() { return sim; }

  private Drivetrain drivetrain = Drivetrain.getInstance();
  
  public final Pose3d origin = new Pose3d();

  // private static final double compZeroX = 0.0;// for brownout shooter: -0.05;
  // private static final double compZeroY = 0;
  // private static final double compZeroZ = 0.0;// for brownout shooter: -0.06;

  private static final int numComponents = 6; //5
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

  private static GenericEntry manipZ = componentConfig.add("manip z", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 3))
      .withPosition(7, 1).getEntry();

  private static GenericEntry elecIntakeX = componentConfig.add("elecIntake x", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(3, 0).getEntry();
  private static GenericEntry elecIntakeY = componentConfig.add("elecIntake y", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(3, 1).getEntry();
  private static GenericEntry elecIntakeZ = componentConfig.add("elecIntake z", currNum)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", numComponents - 1))
      .withPosition(3, 2).getEntry();
  private static GenericEntry elecIntakeRoll = componentConfig.add("elecIntake roll", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(3, 3).getEntry();
  private static GenericEntry elecIntakePitch = componentConfig.add("elecIntake pitch", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(3, 4).getEntry();
  private static GenericEntry elecIntakeYaw = componentConfig.add("elecIntake yaw", 0)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -180, "max", 180))
      .withPosition(3, 5).getEntry();

  private static ShuffleboardTab robotConfig = Shuffleboard.getTab("robot");

  private static GenericEntry robotX = robotConfig.add("robot x", 2.06)
      .withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -5, "max", 4))
      .withPosition(6, 0).getEntry();
  private static GenericEntry robotY = robotConfig.add("robot y", 1.98)
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

  private static Pose3d[] stagedBasketballs = Arrays.copyOf(FieldConstants.BASKETBALLS, FieldConstants.BASKETBALLS.length);
  private static boolean[] ballIsPresent = new boolean[FieldConstants.BASKETBALLS.length];
  static { Arrays.fill(ballIsPresent, true); }
  private static Pose3d[] scoredBasketballs = new Pose3d[1];

  private static Transform3d[] components = new Transform3d[6]; //5
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
  private Pose3d robotPose, intake, startPose, endPose;
  private double x = 0, y = 0, yaw = 0;

  private boolean hasBasketball = false, dunkInit = false, dunking = false;

  private double dunkSpeed = 2.0;
  private double duration;

  private Timer timer = new Timer();

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
      robotPose = new Pose3d(new Translation3d(
          robotX.getDouble(0), 
          robotY.getDouble(0), 
          robotZ.getDouble(0)),
          new Rotation3d(
              Units.degreesToRadians(robotRoll.getDouble(0)),
              Units.degreesToRadians(robotPitch.getDouble(0)),
              Units.degreesToRadians(robotYaw.getDouble(0))));

      x += Math.abs(RobotContainer.driverController.getLeftY()) > 0.1 ? -RobotContainer.driverController.getLeftY() * 0.1 : 0;
      y += Math.abs(RobotContainer.driverController.getLeftX()) > 0.1 ? -RobotContainer.driverController.getLeftX() * 0.1 : 0;
      yaw += Math.abs(RobotContainer.driverController.getRightX()) > 0.1 ? -RobotContainer.driverController.getRightX() * 0.1 : 0;
  
      robotPose = new Pose3d(robotPose.toPose2d().plus(new Transform2d(new Translation2d(x, y), new Rotation2d(yaw))));
    } else {
      robotPose = new Pose3d(drivetrain.getPose());
    }

    Logger.recordOutput("Sim/Robot Pose", robotPose);
  }

  public void visualizeComponents() {
    double manipulatorHeight = RobotContainer.driverController.getRightTriggerAxis()
        * ElevatorConstants.MANIPULATOR_MAX_EXTEND 
        + manipZ.getDouble(0);
    double manipPitch = -Math.PI / 2.0 + 
        (RobotContainer.driverController.getLeftTriggerAxis() * Math.PI * 2.0 / 3.0);

    components[1] = new Transform3d(new Translation3d(
        0, 
        0, 
        Math.max(0, Math.min(manipulatorHeight, ElevatorConstants.MANIPULATOR_MAX_EXTEND))), 
        new Rotation3d());
    components[2] = new Transform3d(new Translation3d(
        0, 
        0, 
        Math.max(0, Math.min(manipulatorHeight, ElevatorConstants.TOP_STAGE_MAX_EXTEND))), 
        new Rotation3d());
    components[3] = new Transform3d(new Translation3d(
        0, 
        0, 
        Math.max(0, Math.min(manipulatorHeight, ElevatorConstants.MID_STAGE_MAX_EXTEND))), 
        new Rotation3d());

    int currNum = (int) (componentNum.getDouble(0) + 0.5);
    Transform3d currComponent = new Transform3d(
        compX.getDouble(0) - compZeros[X][currNum],
        compY.getDouble(0) - compZeros[Y][currNum], 
        compZ.getDouble(0) - compZeros[Z][currNum],
        new Rotation3d(
            Units.degreesToRadians(compRoll.getDouble(0)), 
            Units.degreesToRadians(compPitch.getDouble(0)),
            Units.degreesToRadians(compYaw.getDouble(0))));

    components[4] = new Transform3d(
        elecIntakeX.getDouble(-0.34),
        elecIntakeY.getDouble(0),
        elecIntakeZ.getDouble(0.165),
        new Rotation3d(
            Units.degreesToRadians(elecIntakeRoll.getDouble(0)),
            Units.degreesToRadians(elecIntakePitch.getDouble(0)),
            Units.degreesToRadians(elecIntakeYaw.getDouble(0))));
            
    components[5] = new Transform3d(
        elecIntakeX.getDouble(0),
        elecIntakeY.getDouble(0),
        elecIntakeZ.getDouble(0),
        new Rotation3d(
            Units.degreesToRadians(elecIntakeRoll.getDouble(0)),
            Units.degreesToRadians(elecIntakePitch.getDouble(0)),
            Units.degreesToRadians(elecIntakeYaw.getDouble(0))));

    components[currNum] = currComponent;
    components[0] = components[1].plus(components[0]).plus(new Transform3d(
        new Translation3d(), 
        new Rotation3d(0, manipPitch, 0)));

    Logger.recordOutput("Sim/Components Tranform3d[]", components );
    Logger.recordOutput("Sim/Robot Pose");
    // Logger.recordOutput("Sim/Components Pose3d[]", 
    //     new Pose3d[] { robotPose.transformBy(shooter), robotPose.transformBy(amp) });
  }

  public void visualizeBasketballs() {
    intake = robotPose.transformBy(components[0].plus(ElevatorConstants.HELD_BASKETBALL_POS));

    for (int i = 0; i < stagedBasketballs.length; i++) {
      if (RobotContainer.driverController.getLeftTriggerAxis() > 0.5 && 
          get3dDistance(stagedBasketballs[i], intake) < 
              FieldConstants.BASKETBALL_RADIUS) {
        ballIsPresent[i] = false;
        hasBasketball = true;
      }

      if (!ballIsPresent[i]) {
        stagedBasketballs[i] = origin;
      } else if (ballIsPresent[i] && stagedBasketballs[i].equals(origin)) {
        stagedBasketballs[i] = FieldConstants.BASKETBALLS[i];
      }      
    }

    if (hasBasketball) {
      // for (Pose3d hoop : FieldConstants.HOOPS) {
        // if (intake.getZ() > hoop.getZ() 
        //     && get2dDistance(intake, hoop) < FieldConstants.HOOP_RADIUS * 5
        //     && RobotContainer.driverController.getBButtonPressed()) {
        //   scoringBasketball = true;
        //   break;
        // }
      // }
      if (RobotContainer.driverController.getBButton()) {
        dunkInit = true;
      }
      Logger.recordOutput("Sim/Held Basketballs", new Pose3d[] { intake });
    } else {
      Logger.recordOutput("Sim/Held Basketballs", new Pose3d[] { } );
    }

    if (dunkInit && !dunking) {
      hasBasketball = false;
      dunkInit = false;
      dunking = true;

      startPose = intake;
      endPose = FieldConstants.LOW_HOOP;
      duration = get3dDistance(startPose, endPose) / dunkSpeed;
      
      timer.restart();
    }

    if (dunking) {
      if (timer.hasElapsed(duration)) { dunking = false; }
      Logger.recordOutput("Sim/dur", duration);
      Logger.recordOutput("Sim/timer", timer.get());

      Logger.recordOutput("Sim/Scored Basketballs", new Pose3d[] {
          startPose.interpolate(endPose, timer.get() / duration) });
    } else {
      Logger.recordOutput("Sim/Scored Basketballs", new Pose3d[] { } );
    }

    if (RobotContainer.driverController.getAButtonPressed()) {
      hasBasketball = false;
    }

    Logger.recordOutput("Sim/dist", get2dDistance(intake, FieldConstants.HIGH_HOOP));
    Logger.recordOutput("Sim/Staged Basketballs", stagedBasketballs);
  }

  public double get2dDistance(Pose3d a, Pose3d b) {
    return Math.sqrt(
      Math.pow(b.getX() - a.getX(), 2) + 
      Math.pow(b.getY() - a.getY(), 2));
  }

  public double get3dDistance(Pose3d a, Pose3d b) {
    return Math.sqrt(
      Math.pow(b.getX() - a.getX(), 2) + 
      Math.pow(b.getY() - a.getY(), 2) + 
      Math.pow(b.getZ() - a.getZ(), 2));
  }
}


//5 values — [(X: 0.35m, Y: 0.00m, Z: 2.38m, Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°),
//(X: 0.00m, Y: 0.00m, Z: 2.18m, Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°), 
//(X: 0.00m, Y: 0.00m, Z: 1.47m, Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°), 
//(X: 0.00m, Y: 0.00m, Z: 0.76m, Roll: 0.00°, Pitch: 0.00°, Yaw: 0.00°),
//(X: 0.35m, Y: 0.00m, Z: 0.00m, Roll: -2.25°, Pitch: 0.00°, Yaw: 0.00°)]
