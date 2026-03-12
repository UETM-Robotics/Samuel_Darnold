package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private final TalonSRX intakeDriver = new TalonSRX(IntakeConstants.INTAKE_MOTOR_CAN);


    public IntakeSubsystem () {

    }
    /**
     * Starts the Indexer Motor to run at INTAKE_MOTOR_SPEED determined in {@link MotorConstants}
     */
    public Command startMotor() {
    return runOnce(
        () -> {
            intakeDriver.set(TalonSRXControlMode.PercentOutput, IntakeConstants.INTAKE_MOTOR_SPEED);
        });
    }

    public Command stopMotor() {
    return runOnce(
        () -> {
            intakeDriver.set(TalonSRXControlMode.PercentOutput, 0.0);
        });
    }

    public void start () {
        intakeDriver.set(TalonSRXControlMode.PercentOutput, IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void stop () {
        intakeDriver.set(TalonSRXControlMode.PercentOutput, 0.0);
    }
}
