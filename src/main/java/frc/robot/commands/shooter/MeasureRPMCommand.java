package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class MeasureRPMCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private double min;
    private double max;
    private double current;
    private boolean startMeasurement;


    public MeasureRPMCommand (ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    } 

    @Override 
    public void initialize() {
        shooterSubsystem.startKrakens();
        startMeasurement = false;
    }

    @Override 
    public void execute() {
        current = shooterSubsystem.measureRPM()*60;
        if (!startMeasurement && shooterSubsystem.measureRPM()*60>3000) {
            startMeasurement = true;
            min = current;
            max = current;
        } else if (startMeasurement) {
            if (min > current) {
                min = current;
            } if (max < current) {
                max = current;
            }

            SmartDashboard.putNumber("Min RPM", min);
            SmartDashboard.putNumber("Max RPM", max);
            SmartDashboard.putNumber("Current RPM", current);
        }
    }

    @Override 
    public void end(boolean interrupted) {
        /*
        Turn Off Shooter Motors
        */
       shooterSubsystem.stopIndexerMotor();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
