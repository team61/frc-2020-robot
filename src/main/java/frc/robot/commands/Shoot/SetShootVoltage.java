package frc.robot.commands.Shoot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class SetShootVoltage extends InstantCommand {
    private ShooterSubsystem m_shooterSubsystem;
    private double m_voltage;

    public SetShootVoltage(ShooterSubsystem shooterSubsystem, double voltage) {
        m_shooterSubsystem = shooterSubsystem;
        m_voltage = voltage;
    }

    @Override
    public void execute() {
        m_shooterSubsystem.setConfigVoltage(m_voltage);
    }
}