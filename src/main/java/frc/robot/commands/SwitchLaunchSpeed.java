package frc.robot.commands;

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
        m_launcher.setTargetSpeedPer((m_launcher.getTargetSpeedPer() == LauncherConstants.kFastSpeedPer) ? LauncherConstants.kSlowSpeedPer : LauncherConstants.kFastSpeedPer);
        m_launcher.setTargetSpeedPer((m_launcher.getTargetSpeedPer() == LauncherConstants.kFastSpeedRPM) ? LauncherConstants.kSlowSpeedRPM : LauncherConstants.kFastSpeedRPM);

    }
}
