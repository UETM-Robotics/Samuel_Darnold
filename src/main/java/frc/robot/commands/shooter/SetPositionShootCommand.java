package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetPositionShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    private Pose2d targetPose;

    public SetPositionShootCommand (ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;

        if(swerveSubsystem.isRedAlliance()) {
            this.targetPose = new Pose2d(); //red target
        } else {
            this.targetPose = new Pose2d(); //blue target
        }
        
        addRequirements(shooterSubsystem);
        addRequirements(swerveSubsystem);
    } 

    @Override 
    public void initialize() {
        shooterSubsystem.startFlywheels(10);
    }

    @Override 
    public void execute() {
        swerveSubsystem.driveToPose(targetPose);
        shooterSubsystem.setHoodAngle(ShooterConstants.HOOD_ANGLE);
        
        if (/*within target rpm and angle*/ true) {
            shooterSubsystem.startIndexerMotor();
        }
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
       shooterSubsystem.stopFlywheels();
       shooterSubsystem.stopIndexerMotor();
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
