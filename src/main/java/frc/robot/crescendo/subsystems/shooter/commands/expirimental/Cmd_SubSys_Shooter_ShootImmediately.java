package frc.robot.crescendo.subsystems.shooter.commands.expirimental;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.crescendo.subsystems.shooter.SubSys_Shooter;
import frc.robot.crescendo.subsystems.shooter.util.ShooterDirection;
import frc.robot.crescendo.subsystems.shooter.util.ShooterIntakeDirection;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.crescendo.subsystems.shooter.SubSys_Shooter_Constants.AutoAim.SHOOTER_WAIT_AFTER_SHOOT;



public class Cmd_SubSys_Shooter_ShootImmediately extends Command {
    SubSys_Shooter subSysShooter;
    Timer timer = new Timer();

    public Cmd_SubSys_Shooter_ShootImmediately(
            SubSys_Shooter subSysShooter) {
        this.subSysShooter = subSysShooter;

        addRequirements(subSysShooter);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        subSysShooter.setShooterOutput(ShooterDirection.OUT);
        subSysShooter.setIntakeOutput(ShooterIntakeDirection.SHOOT);
        timer.start();
    }

    @Override
    public void end(boolean interrupted) {
        subSysShooter.setIntakeOutput(ShooterIntakeDirection.OFF);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return !subSysShooter.getIntakeOccupied() && timer.get() > SHOOTER_WAIT_AFTER_SHOOT;
    }
}