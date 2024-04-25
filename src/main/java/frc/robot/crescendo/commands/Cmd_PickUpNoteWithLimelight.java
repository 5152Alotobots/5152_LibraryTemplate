// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.crescendo.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.crescendo.subsystems.intake.SubSys_Intake;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_IntakeNote;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_RotateToDegree;
import frc.robot.crescendo.subsystems.intake.commands.Cmd_SubSys_Intake_RotateToDegreeWithLimitSwitch;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.commands.Cmd_SubSys_Shooter_RotateToDegree;
import frc.robot.library.drivetrains.swerve_ctre.CommandSwerveDrivetrain;
import frc.robot.library.drivetrains.swerve_ctre.commands.Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget;
import frc.robot.library.vision.limelight.SubSys_Limelight;

import static frc.robot.crescendo.subsystems.intake.SubSys_Intake_Constants.PresetIntakePositions.*;
import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.PresentArmPositions.ARM_PRESET_TRANSFER;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class Cmd_PickUpNoteWithLimelight extends SequentialCommandGroup {
  private final SubSys_Intake intakeSubSys;
  private final SubSys_Shooter shooterSubSys;
  private final CommandSwerveDrivetrain drivetrain;
  private final SubSys_Limelight limelightSubSys;

  public Cmd_PickUpNoteWithLimelight(
    SubSys_Intake intakeSubSys, 
    SubSys_Shooter shooterSubSys,
    CommandSwerveDrivetrain drivetrain,
    SubSys_Limelight limelightSubSys) {
      
    this.intakeSubSys = intakeSubSys;
    this.shooterSubSys = shooterSubSys;
    this.drivetrain = drivetrain;
    this.limelightSubSys = limelightSubSys;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new Cmd_SubSys_Shooter_RotateToDegree(shooterSubSys, () -> ARM_PRESET_TRANSFER),
        new Cmd_SubSys_Intake_RotateToDegreeWithLimitSwitch(intakeSubSys, () -> INTAKE_PRESET_PICKUP)
      ),
      new ParallelCommandGroup(  
        new Cmd_SubSys_Intake_IntakeNote(intakeSubSys),
        new Cmd_SubSys_Drivetrain_DriveToLimelightNoteTarget(drivetrain, 
          ()->limelightSubSys.getNoteDetected(), 
          ()->limelightSubSys.getNoteTx())).until(() -> intakeSubSys.getIntakeOccupied()).withTimeout(4)
    );
  }
}
//We still need to go forward to intake and backward so pathplanner is accurate