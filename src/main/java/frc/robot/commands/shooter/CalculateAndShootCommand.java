package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class CalculateAndShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;



    public CalculateAndShootCommand (ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        
        addRequirements(shooterSubsystem);
    } 

    @Override 
    public void initialize() {
        /*
        Begin spin up of krakens
        */
    }

    @Override 
    public void execute() {
        /*
        Get distance from vision
        Get angle from odometry 
        Get angle from shooterHoodMotor
        Calculate Optimal Angle
        Calculate Optimal RPM
        Set Angle to shootHoodMotor
        Set Voltage to TalonFXDriver
        Wait until TalonFXDriver && shooterHoodMotor.encoder is within a certain range {
            Run Indexer
        }
        */
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
