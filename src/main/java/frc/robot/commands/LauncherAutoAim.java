package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class LauncherAutoAim extends CommandBase {
    private Launcher m_launcher;

    public LauncherAutoAim(Launcher launcher) {
        m_launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        m_launcher.set(m_launcher.getSpeed());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_launcher.isSwitchSet();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_launcher.stop();
    }
}
