package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class PIDElevator extends Command {
    private ElevatorSubsystem e_ElevatorSubsystem;
    private double targetPosition;
    private PIDController e_PIDController = new PIDController(ElevatorConstants.elevatorP, ElevatorConstants.elevatorI, ElevatorConstants.elevatorD);

    public PIDElevator(ElevatorSubsystem e_ElevatorSubsystem, double targetPosition) {
        this.e_ElevatorSubsystem = e_ElevatorSubsystem;
        addRequirements(e_ElevatorSubsystem);

        this.targetPosition = targetPosition;
    }

    @Override
    public void initialize() {
        e_PIDController.setSetpoint(targetPosition);
        e_PIDController.reset();
    }

    @Override
    public void execute() {
        double newElevatorSpeed = e_PIDController.calculate(e_ElevatorSubsystem.getAverageElevatorMotorPosition());
        e_ElevatorSubsystem.setElevatorMotorSpeeds(newElevatorSpeed, false);
    }

    @Override
    public boolean isFinished() {
        if (e_PIDController.getPositionError() < 10) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
