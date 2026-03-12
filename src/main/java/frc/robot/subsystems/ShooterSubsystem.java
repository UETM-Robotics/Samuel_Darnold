// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    //All three krakens
    private final TalonFX flywheelDriver = new TalonFX(ShooterConstants.FLYWHEEL_DRIVER_CANID);
    private final TalonFX krakenShooterMiddle = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_MID_CANID);
    private final TalonFX krakenShooterRight = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_RIGHT_CANID);

    private final StatusSignal<AngularVelocity> flywheelVelocity = flywheelDriver.getVelocity();
    private final StatusSignal<AngularAcceleration> flywheelAcceleration = flywheelDriver.getAcceleration();

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(ShooterConstants.ks, ShooterConstants.kv, ShooterConstants.ka);


    //The motor for the covers of the shooter
    private final SparkMax shooterHoodMotor = new SparkMax(ShooterConstants.HOOD_CANID, MotorType.kBrushless);

    //Motor for the indexer
    private final TalonSRX indexerDriver = new TalonSRX(ShooterConstants.INDEXER_MOTOR_CAN);

        // NOTE: the output type is amps, NOT volts (even though it says volts)
    // https://www.chiefdelphi.com/t/sysid-with-ctre-swerve-characterization/452631/8
    private final SysIdRoutine flywheelSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(5).per(Second),
            Volts.of(10),
            Seconds.of(10),
            state -> SignalLogger.writeString("Flywheel SysId", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> flywheelDriver.setControl(new VoltageOut(volts)),
            null,
            this));


    /** Creates a new ExampleSubsystem. */
    public ShooterSubsystem(int leftShooterCANID, int middleShooterCANID, int rightShooterCANID, int shooterHoodMotorCANID, int intakeDriverCANID) {
        //set the other two to follow the left motor (driver)
        //krakenShooterMiddle.setControl(new com.ctre.phoenix6.controls.Follower(leftShooterCANID, MotorAlignmentValue.Aligned));
        //krakenShooterRight.setControl(new com.ctre.phoenix6.controls.Follower(leftShooterCANID, MotorAlignmentValue.Aligned));
        configureFlywheels();
        //shooterHoodMotor = new SparkMax(shooterHoodMotorCANID, MotorType.kBrushless);

        indexerDriver.setNeutralMode(NeutralMode.Coast);
    }

    private void configureFlywheels() {
        TalonFXConfiguration flywheelConfig = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)/*.withInverted(InvertedValue.CounterClockwise_Positive)*/)

            .withTorqueCurrent(
                new TorqueCurrentConfigs().withPeakForwardTorqueCurrent(Amps.of(160))
                    .withPeakReverseTorqueCurrent(Amps.of(15)))

            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(170))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(40))
                    .withSupplyCurrentLimitEnable(true))

            .withSlot0(Slot0Configs.from(new SlotConfigs().withKP(23.0).withKS(20.0).withKV(2.5)));

        flywheelDriver.getConfigurator().apply(flywheelConfig);
        krakenShooterMiddle.getConfigurator().apply(flywheelConfig);
        krakenShooterRight.getConfigurator().apply(flywheelConfig);
        // Max the leader update frequency so follower can respond quickly
        flywheelDriver.getTorqueCurrent().setUpdateFrequency(1000);

        krakenShooterMiddle.setControl(new Follower(flywheelDriver.getDeviceID(), MotorAlignmentValue.Aligned));
        krakenShooterRight.setControl(new Follower(flywheelDriver.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    /**
     * Builds a command that runs flywheel SysId in quasistatic mode.
     *
     * @param direction direction for the SysId sweep
     * @return command that runs flywheel quasistatic SysId and stops flywheel on exit
     */
    public Command sysIdFlywheelQuasistaticCommand(Direction direction) {
        return flywheelSysIdRoutine.quasistatic(direction)
            .withName("SysId flywheel quasi " + direction)
            .finallyDo(this::stopFlywheels);
    }

    /**
     * Builds a command that runs flywheel SysId in dynamic mode.
     *
     * @param direction direction for the SysId sweep
     * @return command that runs flywheel dynamic SysId and stops flywheel on exit
     */
    public Command sysIdFlywheelDynamicCommand(Direction direction) {
        return flywheelSysIdRoutine.dynamic(direction)
            .withName("SysId flywheel dynamic " + direction)
            .finallyDo(this::stopFlywheels);
    }

    public void startFlywheels(double vel) {
        double voltage = ff.calculate(vel);
        flywheelDriver.setVoltage(voltage);
    }

    public void stopFlywheels() {
        flywheelDriver.stopMotor();
    }

    public double measureFlywheelRPM() {
        return flywheelDriver.getRotorVelocity().getValueAsDouble();
    }

    public double measureFlywheelAccel() {
        return flywheelDriver.getAcceleration().getValueAsDouble();
    }

    /**
     * Starts the Indexer Motor to run at INDEXER_MOTOR_SPEED determined in {@link MotorConstants}
     */
    public void startIndexerMotor() {
        indexerDriver.set(ControlMode.PercentOutput, ShooterConstants.INDEXER_MOTOR_SPEED);
    }
 
    /**
     * Stops the Indexer Motor
     */
    public void stopIndexerMotor() {
        indexerDriver.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * NOT DONEEEEEEEEEEEEEEEEEEEEEEEEEEEE
     * @return 
     * angle
     */
    public double getHoodAngle() {
        double angle =  70 + shooterHoodMotor.getAbsoluteEncoder().getPosition() * 0.5;
        return angle;
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

    public void setHoodAngle(double hoodAngle) {

        
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setHoodAngle'");
    }

    @Logged(name = "Flywheel Velocity")
        public AngularVelocity getFlywheelVelocity() {
        BaseStatusSignal.refreshAll(flywheelVelocity, flywheelAcceleration);
        return BaseStatusSignal.getLatencyCompensatedValue(flywheelVelocity, flywheelAcceleration);
    }


    // public Command measureRPMDrop() {
    //     return run(() -> {
    //         shooterDriver.setVoltage(6.0);
    //         SmartDashboard.putNumber("Current RPM", shooterDriver.getRotorVelocity().getValueAsDouble());
    //         SmartDashboard.putNumber("Low RPM", shooterDriver.getRotorVelocity().getValueAsDouble());
    //         SmartDashboard.putNumber("Max RPM", shooterDriver.getRotorVelocity().getValueAsDouble());
    //     }).withTimeout(15.0);
    // }
}
