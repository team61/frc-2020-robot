package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher;

public class SwitchLaunchSpeed extends InstantCommand {

    private Launcher m_launcher;

    public SwitchLaunchSpeed(Launcher launcher) {
        m_launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        m_launcher.setSpeed((m_launcher.getSpeed() == LauncherConstants.kFastLauncherSpeedPer) ? LauncherConstants.kSlowLauncherSpeedPer : LauncherConstants.kFastLauncherSpeedPer);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_launcher.stop();
    }
}
