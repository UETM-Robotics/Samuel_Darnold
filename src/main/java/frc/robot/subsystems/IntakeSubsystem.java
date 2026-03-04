package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final SparkMax intakeDriver;

    public IntakeSubsystem (int intakeDriverCANID) {
        intakeDriver = new SparkMax(intakeDriverCANID, MotorType.kBrushless);
        SparkMaxConfig cSparkMax = new SparkMaxConfig();
        //cSparkMax.idleMode(IdleMode.kCoast);
        
        //intakeDriver.configure()
        //new CANSparkMax()
    }

    public void start () {
        intakeDriver.set(IntakeConstants.INTAKE_MOTOR_SPEED);
    }

    public void stop () {
        intakeDriver.stopMotor();
    }
}
