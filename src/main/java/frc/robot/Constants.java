/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Robot {

    public static final class Calibrations {

      public static final class DriveTrain {
        public static final double driveXYSpdPerfModeSwFilterRate = 8.0; // m/s/s
        public static final double driveRotSpdPerfModeSwFilterRate = 4.0; // rad/s/s
        public static final double DriveTrainMaxAccel = 3.0; // m/s^2
        public static final double DriveTrainMaxDeccel = -2.0; // m/s^2

        public static final class PerformanceMode_Default {
          // Default Performance Mode Speeds
          //public static double DriveTrainMaxPctOutput = 0.50; // 0-1
          public static double DriveTrainMaxSpd = 2.0; // m/s
          public static double DriveTrainMaxAccel = 2.0; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.4; // 0-1
          public static double DriveTrainMaxRotSpd = 0.75 * 2*Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.75 * 2*Math.PI; // rad/s^2
        }

        public static final class PerformanceMode_Turtle {
          // Performance Mode A Speeds (Slow)
          //public static double DriveTrainMaxPctOutput = 0.25; // 0-1
          public static double DriveTrainMaxSpd = 1.0; // m/s
          public static double DriveTrainMaxAccel = 0.35; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.6; // 0-1
          public static double DriveTrainMaxRotSpd = 0.5 * 2*Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.15 * 2*Math.PI; // rad/s^2
        }

        public static final class PerformanceMode_Turbo {
          // Performance Mode B Speeds (Fast)
          //public static double DriveTrainMaxPctOutput = 0.75; // 0-1
          public static double DriveTrainMaxSpd = 10.0; // m/s
          public static double DriveTrainMaxAccel = 1.00; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.2; // 0-1
          public static double DriveTrainMaxRotSpd = 1.0 * 2*Math.PI; // rad/s
          public static double DriveTrainMaxRotAccel = 0.75 * 2*Math.PI; // rad/s^2
        }
        public static final class DriveTrainTrajSettings {
          // PathPlanner Speeds
          //public static double DriveTrainMaxPctOutput = 0.50; // 0-1
          public static double DriveTrainMaxSpd = 4.0; // m/s
          public static double DriveTrainMaxAccel = 0.35; // m/s^2
          //public static double DriveTrainMaxRotPctOutput = 0.4; // 0-1
          public static double DriveTrainMaxRotSpd = 140 * Math.PI / 180; // rad/s
          public static double DriveTrainMaxRotAccel = 200 * Math.PI / 180; // rad/s^2
        }

        public static final class IdleSpeeds {
          public static double IDLE_ROTATION_RADS_PER_SEC = 0.001; // +-X rads/s counts as not moving
        }
      }

      public static final class Arm {
        public static double ArmMaxRotSpd = 100 * Math.PI / 150; // rad/s    //150
        public static double ArmMaxRotAccel = 150 * Math.PI / 180; // rad/s/s

        public static double ArmExtensionMaxSpd = 0.5; // m/s
        public static double ArmExtensionMaxAccel = 0.5; // m/s/s

        public static double ArmExtendPosCtrlFastSpd = 0.8; // %
        public static double ArmExtendPosCtrlSlowRange = 0.1; // m
        public static double ArmExtendPosCtrlSlowSpd = 0.15; // %
        public static double ArmExtendPosCtrlAtPositionRange = 0.02; // m

        public static double HighConeArmPosInit = -158; // degree
        public static double HighConeArmExtensionInit = -158; // degree
        public static double HighConeArmPosFinal = -158; // degree
        public static double HighConeArmExtensionFinal = -158; // degree
      }
    }

    public static final class MaxSpeeds {
      // Maximum Achieveable Speeds
      public static final class DriveTrain {
        // public static double DriveTrainMaxPctOutput = 1.00;               // 0-1
        public static double DriveTrainMaxSpd = 5.4; // m/s
        public static double DriveTrainMaxAccel = 5.0; // m/s^2
        // public static double DriveTrainMaxRotPctOutput = 1.0;             // 0-1
        public static double DriveTrainMaxRotSpeed = 360 * Math.PI / 180; // rad/s
        public static double DriveTrainMaxRotAccel = 360 * Math.PI / 180; // rad/s^2
      }

      public static final class Arm {
        public static double ArmShoulderMaxRotSpd = 360 * Math.PI / 180; // rad/s
        public static double ArmShoulderMaxRotAccel = 360 * Math.PI / 180; // rad/s

        public static double ArmExtensionMaxSpd = 1.0; // m/s
        public static double ArmExtensionMaxAccel = 1.0; // m/s/s
      }
    }

    public static final class Dimensions {

      public static final class Frame {
        // Robot Origin (0,0,0 at center and bottom of wheels)
        public static final Translation3d FrameOrigin = new Translation3d(0, 0, 0);

        public static final double Length = Units.inchesToMeters(23.75);
        public static final double Width = Units.inchesToMeters(23.85);
        public static final double BumperThickness = Units.inchesToMeters(3.25);
      }

      public static final class DriveTrain {
        public static final double WheelBase = Units.inchesToMeters(18.5);
        public static final double TrackWidth = Units.inchesToMeters(18.5);
      }

      public static final class Arm {
        public static final Translation3d ArmShoulder =
            new Translation3d(
                Units.inchesToMeters(-2.0),
                Units.inchesToMeters(.0),
                Units.inchesToMeters(23.5)); // Relative to Frame Origin
        public static final double ArmMinLength = Units.inchesToMeters(22);
        public static final double ArmMaxExtensionLength = Units.inchesToMeters(54.5);
      }

      public static final class Hand {
        public static final double HandForwardLength = Units.inchesToMeters(9.75);
        public static final double HandRetractLength = Units.inchesToMeters(7.75);
      }

      public static final class RobotBoundaries {
        public static final double MaxExtensionOverFrame = 1.20; // .120m or 120cm (width)
        public static final double MaxHeight = 1.98; // .198m or 198cm (Height)
        public static final double MinHeight = 0.01;
      }

      public static final class Limelight {
        public static final double kCameraHeight = Units.inchesToMeters(35); // m
        public static final double kCameraAngle = 29.0; // Degrees
      }
    }
  }

  public static final class Field {
    public static final class Notes{
      public static final Translation2d NoteLeft =
        new Translation2d(2.90,4.10);
      public static final Translation2d NoteCenter =
        new Translation2d(2.90,5.50);
      public static final Translation2d NoteRight =
        new Translation2d(2.90,7.00);
      public static final Translation2d Note1 =
        new Translation2d(8.30,0.75);
      public static final Translation2d Note2 =
        new Translation2d(8.30,2.45);
      public static final Translation2d Note3 =
        new Translation2d(8.30,4.10);
      public static final Translation2d Note4 =
        new Translation2d(8.30,5.80);
      public static final Translation2d Note5 =
        new Translation2d(8.30,7.45);
    }
    
    public static final class Hub {
      public static final Translation2d kHubCenter =
          new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(162));

      public static final Translation2d kH1 =
          new Translation2d(Units.inchesToMeters(308), Units.inchesToMeters(130));

      public static final Translation2d kH2 =
          new Translation2d(Units.inchesToMeters(294), Units.inchesToMeters(174));

      public static final double kTargetRingHeight = Units.inchesToMeters(105);

      public static final double kTargetRingDist2Ctr = Units.inchesToMeters(26);
    }

    public static final class Tarmac {
      public static final Translation2d kT11 =
          new Translation2d(Units.inchesToMeters(364), Units.inchesToMeters(53));

      public static final Translation2d kT12 =
          new Translation2d(Units.inchesToMeters(281), Units.inchesToMeters(51));

      public static final Translation2d kT13 =
          new Translation2d(Units.inchesToMeters(221), Units.inchesToMeters(108));
    }
  }

  public static final class CAN_IDs {

    /** **** Start Library Components CAN ID's **** */ 
    public static final int PDP_CAN_ID = 1;   // Power Distribution Panel
    public static final int PCM_CAN_ID = 2;   // Pneumatic Control Module

    public static final int Pigeon2_ID = 3;

    public static final int FrontLeftDriveMtr_CAN_ID = 10;
    public static final int FrontLeftSteerMtr_CAN_ID = 11;
    public static final int FrontLeftSteerCANCoder_CAN_ID = 12;
    public static final int FrontRightDriveMtr_CAN_ID = 13;
    public static final int FrontRightSteerMtr_CAN_ID = 14;
    public static final int FrontRightSteerCANCoder_CAN_ID = 15;
    public static final int BackLeftDriveMtr_CAN_ID = 16;
    public static final int BackLeftSteerMtr_CAN_ID = 17;
    public static final int BackLeftSteerCANCoder_CAN_ID = 18;
    public static final int BackRightDriveMtr_CAN_ID = 19;
    public static final int BackRightSteerMtr_CAN_ID = 20;
    public static final int BackRightSteerCANCoder_CAN_ID = 21; 
    /** **** End Library Components CAN ID's **** */ 
    
    /** **** Start Crescendo Components CAN ID's **** */
    // ---- Intake ----
    public static final int IntakeArmMtr_CAN_ID = 30;       // Talon FX
    public static final int IntakeArmCANCoder_CAN_ID = 31;  
    public static final int IntakeRollerMtr_CAN_ID = 32;    // Spark Max 550
    
    // ---- Slider ----

    // ---- Shooter ----
    public static final int ShooterArmMtr_CAN_ID = 33;      // Talon FX
    public static final int ShooterArmCANCoder_CAN_ID = 34;
    public static final int ShooterWheelsMtrRight_CAN_ID = 35;  // Talon SRX?
    public static final int ShooterWheelsMtrLeft_CAN_ID = 36;  // Talon SRX?
    public static final int ShooterRollerMtr_CAN_ID = 37;   // Spark Max 550

    // ---- Climber ----
    public static final int ClimberLeftMtr_CAN_ID = 38;        // Talon FX
    public static final int ClimberRightMtr_CAN_ID = 39;        // Talon FX

      // ---- Other ----
      public static final int CANDLE_CAN_ID = 40;

    /** **** End Crescendo Components CAN ID's **** */
  }

  public static final class AnalogInput_IDs {}

  public static final class DigitalIO_IDs {
    /** **** Start Crescendo Components CAN ID's **** */
    // ---- Intake ----  
    public static final int IntakeRollerIRDetector_ID = 1;
    public static final int IntakeArmFwdLimitSw_ID = 1;
     public static final int IntakeArmRevLimitSw_ID = 2;    
    public static final int climberLeftHighSwitchID = 3;
    public static final int climberLeftLowSwitchID = 4;
    public static final int climberRightHighSwitchID = 5;
    public static final int climberRightLowSwitchID = 6;

    // ---- Slider ----

    // ---- Shooter ----
    public static final int ShooterRollerIREmitter_ID = 30;    
    public static final int ShooterRollerIRDetector_ID = 5;
    
    // ---- Climber ----
    /** **** Start Crescendo Components CAN ID's **** */
  }

  public static final class PWM_IDs {}
}
