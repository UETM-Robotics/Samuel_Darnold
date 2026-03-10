package frc.robot.commands.shooter;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class CalculateAndShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    private Pose2d pose;

    private Translation2d aimPoint;



    public CalculateAndShootCommand (ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        //this.visionSubsystem = vistionSusystem;
        this.pose = swerveSubsystem.getPose();
        
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
        pose = swerveSubsystem.getPose();
        
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
