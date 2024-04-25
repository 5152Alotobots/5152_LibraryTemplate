package frc.robot.swervebot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Robot.Calibrations.DriveTrain;

public class HMIstation {
    SlewRateLimiter driveXSpdFilter = new SlewRateLimiter(DriveTrain.DriveTrainMaxAccel, DriveTrain.DriveTrainMaxDeccel,
            0);
    SlewRateLimiter driveYSpdFilter = new SlewRateLimiter(DriveTrain.DriveTrainMaxAccel, DriveTrain.DriveTrainMaxDeccel,
            0);

    SlewRateLimiter driveSpdPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveXYSpdPerfModeSwFilterRate);
    SlewRateLimiter driveRotPerfModeSwFilter = new SlewRateLimiter(DriveTrain.driveRotSpdPerfModeSwFilterRate);

    public class DriverController {
        private final XboxController driverController = new XboxController(0);

        public double driveStrAxisRaw() {
            return driverController.getRawAxis(0);
        }
        
        public double driveFwdAxisRaw() {
            return driverController.getRawAxis(1);
        }

        public double driveRotAxisRaw() {
            return driverController.getRawAxis(4);
        }
    }

    public class CoDriverController {
    }

    public class AuxDriverController {

    }
}
