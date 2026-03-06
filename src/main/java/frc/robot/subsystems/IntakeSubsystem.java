package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeDriver;

    public IntakeSubsystem (int intakeDriverCANID) {
        intakeDriver = new SparkMax(intakeDriverCANID, MotorType.kBrushless);
        SparkMaxConfig cSparkMax = new SparkMaxConfig();
        cSparkMax.idleMode(IdleMode.kCoast);

        intakeDriver.configure(cSparkMax, null,  com.revrobotics.PersistMode.kPersistParameters);
    }

    public Command startMotor() {
    return runOnce(
        () -> {
            intakeDriver.set(MotorConstants.INDEXER_MOTOR_SPEED);
        });
    }

    public Command stopMotor() {
    return runOnce(
        () -> {
            intakeDriver.set(0);
        });
    }

    public void stop () {
        intakeDriver.stopMotor();
    }
}
