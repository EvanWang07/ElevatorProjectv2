// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  /* Controllers */
  private final Joystick weapons = new Joystick(DriverConstants.weaponsControllerPort);

  /* Weapons Controls */
  private final int w_elevatorMovementAxis = XboxController.Axis.kLeftY.value;

  /* Weapons Buttons */
  private final JoystickButton w_runElevatorPID = new JoystickButton(weapons, XboxController.Button.kA.value);
  private final JoystickButton w_rotateElevatorForward = new JoystickButton(weapons, XboxController.Button.kB.value);
  private final JoystickButton w_rotateElevatorBackward = new JoystickButton(weapons, XboxController.Button.kX.value);

  /* Subsystems */
  private final ElevatorSubsystem e_ElevatorSubsystem = new ElevatorSubsystem();

  public RobotContainer() {

    e_ElevatorSubsystem.setDefaultCommand(
      new TeleopElevator(
        e_ElevatorSubsystem, 
        () -> -weapons.getRawAxis(w_elevatorMovementAxis)
      )
    );
    
    configureBindings();
  }

  private void configureBindings() {
    w_runElevatorPID.onTrue(new PIDElevator(e_ElevatorSubsystem, 1800).withTimeout(5));
    w_rotateElevatorForward.onTrue(new PIDElevator(e_ElevatorSubsystem, e_ElevatorSubsystem.getAverageElevatorMotorPosition() + 1800).withTimeout(5));
    w_rotateElevatorBackward.onTrue(new PIDElevator(e_ElevatorSubsystem, e_ElevatorSubsystem.getAverageElevatorMotorPosition() - 1800).withTimeout(5));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
