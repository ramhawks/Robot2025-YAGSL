// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotMath.Arm;
import swervelib.math.Matter;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Simulator constants
  public static final Mechanism2d sideView = new Mechanism2d(
                                              ArmConstants.kArmLength * 2,
                                              ArmConstants.kArmLength + ElevatorConstants.kElevatorLength);
  public static final MechanismRoot2d elevatorCarriage;
  public static final MechanismLigament2d armMech;
  public static final MechanismLigament2d elevatorMech;

  // static method to initalize the values of the static vars
  static {
    elevatorCarriage = Constants.sideView.getRoot("Elevator Carriage",
                                                        ArmConstants.kArmLength,
                                                        ElevatorConstants.kElevatorStartingHeightSim.in(Meters));
    armMech = elevatorCarriage.append(new MechanismLigament2d("Arm",
                                                        ArmConstants.kArmLength,
                                                        ArmConstants.kArmStartingAngle.in(Degrees),
                                                        6,
                                                        new Color8Bit(Color.kOrange)));
    elevatorMech = elevatorCarriage.append(new MechanismLigament2d("Elevator",
                                                        ElevatorConstants.kElevatorLength,
                                                        ElevatorConstants.kElevatorStartingAngle.in(Degrees),
                                                        6,
                                                        new Color8Bit(Color.kRed)));
  }

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  //  public static final class AutonConstants
  //  {
  //
  //    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
  //    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
  //  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class ElevatorConstants {
    public static final double kElevatorKp = 5; //5
    public static final double kElevatorKi = 0;
    public static final double kElevatorKd = 0; //
    public static final double kMaxVelocity = Meters.of(4).per(Second).in(MetersPerSecond);
    public static final double kMaxAcceleration = Meters.of(8).per(Second).per(Second).in(MetersPerSecondPerSecond);
    public static final double kElevatorkS = 0.02;
    public static final double kElevatorkG = 0.9;
    public static final double kElevatorkV = 3.8;
    public static final double kElevatorkA = 0.17;
    public static final double kElevatorRampRate = 0.1;
    public static final double kElevatorGearing = 12.0;
    public static final double kElevatorCarriageMass = 4.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double kElevatorMinHeightMeters = 0.0;
    public static final double kElevatorMaxHeightMeters = 10.25;
    public static final double kElevatorLength = Inches.of(33).in(Meters);
    public static final Distance kElevatorStartingHeightSim = Meters.of(0.0);
    public static final Angle kElevatorStartingAngle = Degrees.of(-90);
    public static final Distance kLaserCANOffset          = Inches.of(3);
    public static final double kElevatorDefaultTolerance = Inches.of(1).in(Meters);
    public static double kLowerToScoreHeight =  Units.inchesToMeters(6);
  }

  public static class ArmConstants {
    public static final int armMotorID = 15;
    public static final double kArmkS = 0;    // Volts (V)
    public static final double kArmkG = 1.53; // Volts (V)
    public static final double kArmkV = 1.58; // Volts Per Velocity (V/(rad/s))
    public static final double kArmkA = 0.08; // Volts Per Acceleration (V/(rad/s^2))
    public static final double kArmkP = 0.5;  //?
    public static final double kArmkI = 0.0;
    public static final double kArmkD = 0.0;
    public static final double kArmReduction = 81;
    public static final double kArmMaxVelocity = Arm.convertArmAngleToSensorUnits(Degrees.of(90))//?
                                                .per(Second).in(RPM) ;
    public static final double kArmMaxAcceleration = Arm.convertArmAngleToSensorUnits(Degrees.of(180))
                                                    .per(Second).per(Second).in(RPM.per(Second)) ;
    public static final int ArmStallCurrentLimit = 40;
    public static final double ArmRampRate = 0.5;
    public static final boolean ArmInverted = false;
    public static final double kArmAllowedClosedLoopError = Arm.convertArmAngleToSensorUnits(Degrees.of(0.01)).in(Rotations);
    public static final double kArmLength = Inches.of(31).in(Meters);
    public static final double kArmMass = 8.0; // kg//?
    public static final Angle kArmMinAngle = Degrees.of(-90);
    public static final Angle kArmMaxAngle = Degrees.of(255);//?
    public static final Angle kArmStartingAngle = Degrees.of(0);
    public static final Angle kArmOffsetToHorizontalZero = Rotations.of(0);
    public static final double kArmDefaultTolerance = 1;
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}