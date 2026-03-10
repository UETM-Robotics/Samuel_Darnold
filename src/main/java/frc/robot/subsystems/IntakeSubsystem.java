package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeDriver = new SparkMax(IntakeConstants.INTAKE_MOTOR_CAN, MotorType.kBrushless);


    public IntakeSubsystem (int intakeDriverCANID) {
        SparkMaxConfig cSparkMax = new SparkMaxConfig();
        cSparkMax.idleMode(IdleMode.kCoast);

        intakeDriver.configure(cSparkMax, null,  com.revrobotics.PersistMode.kPersistParameters);
    }

    /**
     * Starts the Indexer Motor to run at INTAKE_MOTOR_SPEED determined in {@link MotorConstants}
     */
    public Command startMotor() {
    return runOnce(
        () -> {
            intakeDriver.set(IntakeConstants.INTAKE_MOTOR_SPEED);
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
