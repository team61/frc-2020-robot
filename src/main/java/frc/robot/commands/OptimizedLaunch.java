package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LauncherConstants;
import frc.robot.subsystems.Launcher;

public class OptimizedLaunch extends CommandBase {

    private Launcher m_launcher;

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(LauncherConstants.kS, LauncherConstants.kV, LauncherConstants.kA);
    private PIDController m_controller = new PIDController(LauncherConstants.kP, LauncherConstants.kI, LauncherConstants.kD);

    boolean isContinuous;

    public OptimizedLaunch(Launcher launcher, boolean isContinuous) {
        m_launcher = launcher;
        this.isContinuous = isContinuous;

        addRequirements(launcher);
    }

    public OptimizedLaunch(Launcher launcher) {
        m_launcher = launcher;
        this.isContinuous = false;

        addRequirements(launcher);
    }

    @Override
    public void execute() {
        double targetSpeed = m_launcher.getTargetSpeedPer();
        m_launcher.setVoltage(m_feedforward.calculate(targetSpeed, LauncherConstants.kMaxAcc) + m_controller.calculate(m_launcher.getEncoderRate(), targetSpeed));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_launcher.isSwitchSet() && !isContinuous;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_launcher.stop();
        m_controller.reset();

    }
}
