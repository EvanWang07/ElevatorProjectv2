package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

import edu.wpi.first.math.MathUtil;

import java.util.function.DoubleSupplier;

public class TeleopElevator extends Command {
    private ElevatorSubsystem e_ElevatorSubsystem;
    private DoubleSupplier speedSup;

    public TeleopElevator(ElevatorSubsystem e_ElevatorSubsystem, DoubleSupplier speedSup) {
        this.e_ElevatorSubsystem = e_ElevatorSubsystem;
        addRequirements(e_ElevatorSubsystem);

        this.speedSup = speedSup;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double currentSpeed = MathUtil.applyDeadband(speedSup.getAsDouble(), ElevatorConstants.elevatorStickDeadband);

        e_ElevatorSubsystem.setElevatorMotorSpeeds(currentSpeed, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        e_ElevatorSubsystem.brakeElevatorMotors();
    }
}
