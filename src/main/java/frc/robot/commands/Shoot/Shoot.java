package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    private ShooterSubsystem m_shooterSubsystem;

    public Shoot(ShooterSubsystem shooterSubsystem) {
        m_shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        m_shooterSubsystem.set(ShooterConstants.kMaxVoltage);

    }

    @Override
    public void end(boolean interrupted) {
        m_shooterSubsystem.stop();
    }
}
