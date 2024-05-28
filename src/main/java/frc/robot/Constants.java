// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

  public static class DriverConstants {
    public static final int weaponsControllerPort = 0;
  }

  /* All constants related to ElevatorSubsystem.java and TeleopElevator.java */
  public static class ElevatorConstants {
    /* The constants below are used in ElevatorSubsystem.java */
    public static final int leftElevatorMotorID = 1;
    public static final int rightElevatorMotorID = 2;

    public static final boolean leftElevatorMotorInvert = true;
    public static final boolean rightElevatorMotorInvert = true;

    public static final double leftElevatorMotorOutput = 0.25;
    public static final double rightElevatorMotorOutput = 0.25;

    public static final double leftElevatorMotorMaxPosition = 36000;
    public static final double leftElevatorMotorMinPosition = -36000;
    public static final double rightElevatorMotorMaxPosition = 36000;
    public static final double rightElevatorMotorMinPosition = -36000;

    public static final double leftElevatorMotorStartPosition = 0;
    public static final double rightElevatorMotorStartPosition = 0;

    public static final double maxElevatorMotorVoltage = 6;

    /* The constants below are used in ElevatorCommand.java */
    public static final double elevatorStickDeadband = 0.1;

    /* The constants below are used in PIDElevator.java */
    public static final double elevatorP = 0.1;
    public static final double elevatorI = 0;
    public static final double elevatorD = 0;
  }
}
