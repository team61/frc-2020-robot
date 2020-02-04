package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    private ShooterSubsystem m_shooterSubsystem;

    private boolean pastState;

    public Shoot(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.set(ShooterConstants.kSpeedPer);

    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }
}
