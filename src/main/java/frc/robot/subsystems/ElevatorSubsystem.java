package frc.robot.subsystems;

import frc.robot.ScorpsUtility;
import frc.robot.Constants.ElevatorConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;

public class ElevatorSubsystem extends SubsystemBase {
    private TalonFX leftElevatorMotor;
    private TalonFX rightElevatorMotor;
    private final VoltageOut elevatorRequest = new VoltageOut(0);
    private double outputMultiplier;

    public ElevatorSubsystem() {
        leftElevatorMotor = new TalonFX(ElevatorConstants.leftElevatorMotorID);
        rightElevatorMotor = new TalonFX(ElevatorConstants.rightElevatorMotorID);
        outputMultiplier = 1.0;

        new Thread(() -> {
        try {
            Thread.sleep(250);
            leftElevatorMotor.setInverted(ElevatorConstants.leftElevatorMotorInvert);
            rightElevatorMotor.setInverted(ElevatorConstants.rightElevatorMotorInvert);
            leftElevatorMotor.setPosition(ElevatorConstants.leftElevatorMotorStartPosition);
            rightElevatorMotor.setPosition(ElevatorConstants.rightElevatorMotorStartPosition);
            leftElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
            rightElevatorMotor.setNeutralMode(NeutralModeValue.Coast);
        } catch (Exception e) {
            // Intended to be blank
        }
        }).start();
    }

    public void setElevatorMotorSpeeds(double speed, boolean ignoreBoundsRequirements) {
        if (checkElevatorMotorInvertsAreCorrect()) {
            if (checkElevatorMotorMovementsAreValid(speed) || ignoreBoundsRequirements) {
                leftElevatorMotor.setControl(elevatorRequest.withOutput(speed * ElevatorConstants.maxElevatorMotorVoltage * ElevatorConstants.leftElevatorMotorOutput * outputMultiplier));
                rightElevatorMotor.setControl(elevatorRequest.withOutput(speed * ElevatorConstants.maxElevatorMotorVoltage * ElevatorConstants.rightElevatorMotorOutput * outputMultiplier));
            } else {
                brakeElevatorMotors();
            }
        } else {
            brakeElevatorMotors();
            System.out.println("[setElevatorMotorSpeeds] WARNING: Elevator motor inverts are INCORRECT!");
        }
    }

    public void brakeElevatorMotors() {
        leftElevatorMotor.setControl(elevatorRequest.withOutput(0));
        rightElevatorMotor.setControl(elevatorRequest.withOutput(0));
    }

    public double getLeftElevatorMotorPosition() {
        var leftElevatorMotorPositionSignal = leftElevatorMotor.getPosition();
        double leftElevatorMotorAnglePosition = leftElevatorMotorPositionSignal.getValueAsDouble();
        return Units.rotationsToDegrees(leftElevatorMotorAnglePosition);
    }

    public double getRightElevatorMotorPosition() {
        var rightElevatorMotorPositionSignal = rightElevatorMotor.getPosition();
        double rightElevatorMotorAnglePosition = rightElevatorMotorPositionSignal.getValueAsDouble();
        return Units.rotationsToDegrees(rightElevatorMotorAnglePosition);
    }

    public double getLeftElevatorMotorVelocity() {
        var leftElevatorMotorVelocitySignal = leftElevatorMotor.getVelocity();
        double leftElevatorMotorAnglePerTimeVelocity = leftElevatorMotorVelocitySignal.getValueAsDouble();
        return Units.rotationsToDegrees(leftElevatorMotorAnglePerTimeVelocity);
    }

    public double getRightElevatorMotorVelocity() {
        var rightElevatorMotorVelocitySignal = rightElevatorMotor.getVelocity();
        double rightElevatorMotorAnglePerTimeVelocity = rightElevatorMotorVelocitySignal.getValueAsDouble();
        return Units.rotationsToDegrees(rightElevatorMotorAnglePerTimeVelocity);
    }

    public void toggleSlowMode() {
        if (outputMultiplier == 1.0) {
            outputMultiplier = 0.2; // Speed is 20% when slow mode is on
        } else {
            outputMultiplier = 1.0;
        }
    }

    public boolean checkElevatorMotorMovementsAreValid(double speed) {
        if (getLeftElevatorMotorPosition() < ElevatorConstants.leftElevatorMotorMinPosition) {
            if (speed > 0) {
                return true;
            } else {
                return false;
            }
        } else if (getLeftElevatorMotorPosition() > ElevatorConstants.leftElevatorMotorMaxPosition) {
            if (speed < 0) {
                return true;
            } else {
                return false;
            }
        } else if (getRightElevatorMotorPosition() < ElevatorConstants.rightElevatorMotorMinPosition) {
            if (speed > 0) {
                return true;
            } else {
                return false;
            }
        } else if (getRightElevatorMotorPosition() > ElevatorConstants.rightElevatorMotorMaxPosition) {
            if (speed < 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return true;
        }
    }

    public boolean checkElevatorMotorInvertsAreCorrect() {
        if (leftElevatorMotor.getInverted() != ElevatorConstants.leftElevatorMotorInvert) {
            return false;
        } else if (rightElevatorMotor.getInverted() != ElevatorConstants.rightElevatorMotorInvert) {
            return false;
        } else {
            return true;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Motor Position (Degrees): ", getLeftElevatorMotorPosition());
        SmartDashboard.putNumber("Right Motor Position (Degrees): ", getRightElevatorMotorPosition());
        SmartDashboard.putNumber("Motor Position Difference (Degrees): ", ScorpsUtility.getAbsoluteDifference(getLeftElevatorMotorPosition(), getRightElevatorMotorPosition()));

        SmartDashboard.putNumber("Left Motor Velociy (Degrees Per Second): ", getLeftElevatorMotorVelocity());
        SmartDashboard.putNumber("Right Motor Velocity (Degrees Per Second): ", getRightElevatorMotorVelocity());
        SmartDashboard.putNumber("Motor Velocity Difference (Degrees Per Second): ", ScorpsUtility.getAbsoluteDifference(getLeftElevatorMotorVelocity(), getRightElevatorMotorVelocity()));
    }
    
}
