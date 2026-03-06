package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;

public class IndexerSubsystem extends SubsystemBase {
    private final VictorSPX indexerDriver;
    
    public IndexerSubsystem (int intakeDriverCANID) {
        indexerDriver = new VictorSPX(intakeDriverCANID);
        indexerDriver.setNeutralMode(NeutralMode.Coast);
    }

    public Command startMotor() {
    return runOnce(
        () -> {
            indexerDriver.set(ControlMode.PercentOutput, MotorConstants.INDEXER_MOTOR_SPEED);
        });
    }

    public Command stopMotor() {
    return runOnce(
        () -> {
            indexerDriver.set(ControlMode.PercentOutput, 0);
        });
    }
}
