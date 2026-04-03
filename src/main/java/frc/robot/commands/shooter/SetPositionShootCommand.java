package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class SetPositionShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final SwerveSubsystem swerveSubsystem;
    //private final double h = -1.13665;

    private Pose2d targetPose;

    public SetPositionShootCommand (ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;

         if(swerveSubsystem.isRedAlliance()) {
             this.targetPose = new Pose2d(13.2, 4.0, new Rotation2d(270)); //red target
         } else {
             this.targetPose = new Pose2d(3.2, 4.0, new Rotation2d(90)); //blue target
         }
        
        addRequirements(shooterSubsystem);
        addRequirements(swerveSubsystem);
    } 

    @Override 
    public void initialize() {
        swerveSubsystem.toggleRotationPoint();
        shooterSubsystem.setFlywheels(5.1);
        shooterSubsystem.setHoodAngle(76.0);
        swerveSubsystem.driveToPose(targetPose);
    }

    @Override 
    public void execute() {
        Pose2d dist = swerveSubsystem.getPose().relativeTo(targetPose);
        if (shooterSubsystem.getFlywheelVelocity().baseUnitMagnitude()> 5.05 && Math.sqrt(Math.pow(dist.getX(), 2) + Math.pow(dist.getY(), 2)) < 0.1) {
            shooterSubsystem.startIndexerMotors();
        }
        /*
        Get distance from Odometery
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
       shooterSubsystem.stopFlywheels();
       shooterSubsystem.stopIndexerMotors();
       shooterSubsystem.setHoodAngle(0.0);
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
