package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

import java.util.function.DoubleSupplier;

public class LauncherWithJoysticks extends CommandBase {

    private Launcher m_launcher;

    private DoubleSupplier m_x;

    public LauncherWithJoysticks(Launcher launcher, DoubleSupplier x) {
        m_launcher = launcher;
        m_x = x;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        m_launcher.setAngle(m_x.getAsDouble());
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
