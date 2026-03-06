// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    //All three krakens
    private final TalonFX shooterDriver;
    private final TalonFX krakenShooterMiddle;
    private final TalonFX krakenShooterRight;

    //The motor for the covers of the shooter
    private final SparkMax roofThing;
    


    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem(int leftShooterCANID, int middleShooterCANID, int rightShooterCANID, int sparkMaxCANID) {
        //set up the krakens
        shooterDriver = new TalonFX(leftShooterCANID);
        krakenShooterMiddle = new TalonFX(leftShooterCANID);
        krakenShooterRight = new TalonFX(leftShooterCANID);

        //set the other two to follow the left motor (driver)
        krakenShooterMiddle.setControl(new com.ctre.phoenix6.controls.Follower(leftShooterCANID, MotorAlignmentValue.Aligned));
        krakenShooterRight.setControl(new com.ctre.phoenix6.controls.Follower(leftShooterCANID, MotorAlignmentValue.Aligned));

        roofThing = new SparkMax(sparkMaxCANID, MotorType.kBrushless);
    }

    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.
        return runOnce(
            () -> {
            /* one-time action goes here */
            });
    }

    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
        // Query some boolean state, such as a digital sensor.
        return false;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
