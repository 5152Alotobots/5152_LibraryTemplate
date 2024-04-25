package frc.robot.crescendo.commands.expirimental;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.expirimental.AimModule;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;

import java.util.Optional;

import static frc.robot.Constants.Robot.Calibrations.DriveTrain.IdleSpeeds.IDLE_ROTATION_RADS_PER_SEC;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOTER_WAIT_AFTER_SHOOT;
import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.USE_VISION_POSE_ESTIMATION;

public class Cmd_TransferAutoAngleAlignShootImmediately extends Command {
    SubSys_Shooter subSysShooter;
    CommandSwerveDrivetrain drivetrain;
    Timer timer = new Timer();

    private SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public Cmd_TransferAutoAngleAlignShootImmediately(
            SubSys_Shooter subSysShooter,
            CommandSwerveDrivetrain drivetrain) {
        this.subSysShooter = subSysShooter;
        this.drivetrain = drivetrain;

        addRequirements(subSysShooter, drivetrain);
        drive.HeadingController.setPID(7,0,0);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        // Update drive pose
        drivetrain.updateVisionPoseEstimate();
        // Immediately start spinning shooter wheels
        subSysShooter.setShooterOutput(ShooterDirection.OUT);
        // Align to speaker
        alignToSpeaker();
        // Aim the shooter
        subSysShooter.setShooterArmDegree(AimModule.calculateLaunchAngle(drivetrain.getState().Pose));
        SmartDashboard.putNumber("Launch Angle", AimModule.calculateLaunchAngle(drivetrain.getState().Pose));

        // Shoot when we are angled and ready
        if (drivetrain.getState().speeds.omegaRadiansPerSecond <= IDLE_ROTATION_RADS_PER_SEC &&
                subSysShooter.shooterReady() && subSysShooter.shooterArmMtrAtSetpoint()) {
            subSysShooter.setIntakeOutput(ShooterIntakeDirection.SHOOT);
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("interupted: "+interrupted);
        subSysShooter.stopAll();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !subSysShooter.getIntakeOccupied() && timer.hasElapsed(SHOOTER_WAIT_AFTER_SHOOT);
    }

    private void alignToSpeaker() {
        Rotation2d targetHeading = AimModule.calculateRobotHeadingAlignShooterToSpeaker(drivetrain.getState().Pose);
        // Align
        SmartDashboard.putNumber("Auto Aim Target Heading", targetHeading.getDegrees());
        drivetrain.applyRequest(() -> drive
                        .withTargetDirection(targetHeading))
                .execute();
    }
}
