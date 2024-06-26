package frc.robot.library.drivetrains.swervectre;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.library.drivetrains.swervectre.mk4il32024.TunerConstants_MK4iL3_2024;
import frc.robot.library.vision.photonvision.SubSys_Photonvision;

import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.ONLY_USE_POSE_ESTIMATION_IN_TELEOP;
import static frc.robot.library.vision.photonvision.SubSys_Photonvision_Constants.USE_VISION_POSE_ESTIMATION;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private boolean m_flipPath = false;

    private Field2d field = new Field2d();

    private SubSys_Photonvision subSysPhotonvision;
   
    @Override
    public void periodic() {
        
        SmartDashboard.putNumber("Drive Yaw", this.m_yawGetter.getValueAsDouble());
        
        SmartDashboard.putNumber("Drive Angular Velocity",this.m_angularVelocity.getValueAsDouble());
        SmartDashboard.putNumber("Drive Opr Forward Direction",this.m_operatorForwardDirection.getDegrees());

        SmartDashboard.putNumber("PoseX", this.m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("PoseY", this.m_odometry.getEstimatedPosition().getY());
        field.setRobotPose(this.m_odometry.getEstimatedPosition());
        SmartDashboard.putData("Field", field);

        boolean lclFlipPath = false;
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
                lclFlipPath = true;              
            }
        }
        SmartDashboard.putBoolean("Periodic_FlipPath", lclFlipPath);

        // Vision estimate
            if (ONLY_USE_POSE_ESTIMATION_IN_TELEOP) {
                if (DriverStation.isTeleopEnabled()) {
                    updateVisionPoseEstimate();
                }
            } else {
                updateVisionPoseEstimate();
            }
    }

    /**
     * Updates the robot pose with PhotonVision ONCE if tags can be seen
     * */
    public void updateVisionPoseEstimate() {
        if (subSysPhotonvision != null && USE_VISION_POSE_ESTIMATION) {
            Optional<Pair<Pose2d, Double>> estimatedVisionPose2d = subSysPhotonvision.getEstimatedVisionPose2d(this.m_odometry.getEstimatedPosition());
            estimatedVisionPose2d.ifPresent(pose2dDoublePair -> this.addVisionMeasurement(pose2dDoublePair.getFirst(), pose2dDoublePair.getSecond()));
        }
    }

    /**
     * Sets the SubSys_PhotonVision object to use
     *
     * @param subSysPhotonvision The object
     */
    public void setPhotonVisionSubSys(SubSys_Photonvision subSysPhotonvision) {
        this.subSysPhotonvision = subSysPhotonvision;
    }


    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        
        
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public void updatePoseWithAprilTag(Pose2d aprilTagPose){
        this.addVisionMeasurement(aprilTagPose, 0.0);

    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }


    // ***** PathPlanner *****

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public SendableChooser<Command> getAutoChooser(){
        //configurePathPlanner();
        return AutoBuilder.buildAutoChooser("Auto_JustShoot");
        // Default Path
        //return AutoBuilder.buildAutoChooser(null);
    }


    public Command getPath(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return AutoBuilder.followPath(path);
    }

    public Command getPathFinderCommand(Pose2d targetPose){
        //Pose2d targetPose = new Pose2d(10, 5, Rotation2d.fromDegrees(180));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        return pathfindingCommand;
    }

    /**
     * configurePathPlanner
     * 
     * Sets up PathPlanner for use
     */
    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }
             
        m_flipPath = false;
        if(DriverStation.getAlliance().isPresent()){
            if(DriverStation.getAlliance().get()==DriverStation.Alliance.Red){
                m_flipPath = true;              
            }
        SmartDashboard.putBoolean("ConfigPP_flipPath", m_flipPath);
        }

        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative,  // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds)->this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                                            new PIDConstants(10, 0, 0),
                                            TunerConstants_MK4iL3_2024.kSpeedAt12VoltsMps,
                                            driveBaseRadius,
                                            new ReplanningConfig()),
            ()-> m_flipPath, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }   
}
