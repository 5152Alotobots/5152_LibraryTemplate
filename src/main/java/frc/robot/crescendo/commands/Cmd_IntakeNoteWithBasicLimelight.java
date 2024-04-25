// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.INTAKE_PRESET_PICKUP;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_IntakeNote;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_RotateToDegree;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_RotateToDegreeWithLimitSwitch;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.commands.Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget;
import frc.robot.library.vision.limelight.SubSys_Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Cmd_IntakeNoteWithBasicLimelight extends SequentialCommandGroup {
  /** Creates a new Cmd_IntakeNoteWithBasicLimelight. */
  private final SubSys_Intake intakeSubSys;
  private final CommandSwerveDrivetrain drivetrain;
  private final SubSys_Limelight limelightSubSys;
  private final Pose2d targetNotePose2d;
  
  public Cmd_IntakeNoteWithBasicLimelight(
    SubSys_Intake intakeSubSys,
    CommandSwerveDrivetrain drivetrain,
    SubSys_Limelight limelightSubSys,
    Pose2d targetNotePose2d) {
    this.intakeSubSys = intakeSubSys;
    this.drivetrain = drivetrain;
    this.limelightSubSys = limelightSubSys;
    this.targetNotePose2d = targetNotePose2d;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new Cmd_SubSys_Intake_RotateToDegreeWithLimitSwitch(intakeSubSys, () -> INTAKE_PRESET_PICKUP),
          new Cmd_SubSys_Intake_IntakeNote(intakeSubSys)),
        new SequentialCommandGroup(
          //drivetrain.getPathFinderToNoteCommand(targetNotePose2d).until(() -> limelightSubSys.getNoteDetected()),
          new Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget(drivetrain, ()->limelightSubSys.getNoteDetected(), ()->limelightSubSys.getNoteTx()))
      ).until(intakeSubSys::getIntakeOccupied)
    );
  }
}
