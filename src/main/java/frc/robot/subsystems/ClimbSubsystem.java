package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    

    private final SparkMax ClimbDriver = new SparkMax(ClimbConstants.CLIMB_MOTOR_CAN, MotorType.kBrushless);


    public ClimbSubsystem (int ClimbDriverCANID) {
        SparkMaxConfig cSparkMax = new SparkMaxConfig();
        cSparkMax.idleMode(IdleMode.kBrake);

        ClimbDriver.configure(cSparkMax, null,  com.revrobotics.PersistMode.kPersistParameters);
    }

    /**
     * Starts the climb Motor to run at CLIMB_MOTOR_SPEED determined in {@link MotorConstants}
     */
    public Command startMotor() {
    return runOnce(
        () -> {
            ClimbDriver.set(ClimbConstants.CLIMB_MOTOR_SPEED);
        });
    }

    public Command stopMotor() {
    return runOnce(
        () -> {
            ClimbDriver.set(0);
        });
    }

    public void stop () {
        ClimbDriver.stopMotor();
    }


    public Command startMotorReverse() {
    return runOnce(
        () -> {
            ClimbDriver.set(-ClimbConstants.CLIMB_MOTOR_SPEED);
        });
    }

}
