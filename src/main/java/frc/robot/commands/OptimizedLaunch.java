package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher;

public class OptimizedLaunch extends CommandBase {

    private Launcher m_launcher;

    public OptimizedLaunch(Launcher launcher) {
        m_launcher = launcher;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        m_launcher.setSpeed(m_launcher.getTargetSpeedRPM());
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
        m_launcher.resetController();
    }
}
