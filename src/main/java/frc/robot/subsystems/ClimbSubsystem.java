package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
    private final PIDController pidController;

    private final SparkMax climbDriver = new SparkMax(ClimbConstants.LEFT_CLIMB_MOTOR_CAN, MotorType.kBrushless);
    private final SparkMax ClimbFollower = new SparkMax(ClimbConstants.RIGHT_CLIMB_MOTOR_CAN, MotorType.kBrushless);

    public ClimbSubsystem () {
        this.pidController = new PIDController(ClimbConstants.kP, ClimbConstants.kI, ClimbConstants.kD);
        SparkMaxConfig cSparkMax = new SparkMaxConfig();
        cSparkMax.idleMode(IdleMode.kBrake);

        climbDriver.configure(cSparkMax, null,  com.revrobotics.PersistMode.kPersistParameters);
        cSparkMax.follow(ClimbConstants.LEFT_CLIMB_MOTOR_CAN);
        ClimbFollower.configure(cSparkMax, null, com.revrobotics.PersistMode.kPersistParameters);
    }

    public void setPIDPoint (double point) {
        pidController.setSetpoint(point);
    }

    public double getEncoder () {
        return climbDriver.getEncoder().getPosition();
    }

    public void updateClimbMotors () {
        setClimbSpeed(pidController.calculate(getEncoder()));
    }

    private void setClimbSpeed(double speed) {
        climbDriver.set(speed);
    }

    /**
     * Starts the climb Motor to run at CLIMB_MOTOR_SPEED determined in {@link MotorConstants}
     */
    public Command startMotor() {
    return runOnce(
        () -> {
            climbDriver.set(ClimbConstants.CLIMB_MOTOR_SPEED);
        });
    }

    public Command stopMotor() {
    return runOnce(
        () -> {
            climbDriver.set(0);
        });
    }

    // public void startMotor () {
    //     climbDriver.stopMotor();
    // }

    public void stop () {
        climbDriver.stopMotor();
    }


    public Command startMotorReverse() {
    return runOnce(
        () -> {
            climbDriver.set(-ClimbConstants.CLIMB_MOTOR_SPEED);
        });
    }

}
