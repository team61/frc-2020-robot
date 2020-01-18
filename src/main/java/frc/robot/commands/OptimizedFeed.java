package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class OptimizedFeed extends CommandBase {

    private Feeder m_feeder;

    private SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(FeederConstants.kS, FeederConstants.kV, FeederConstants.kA);
    private PIDController m_controller = new PIDController(FeederConstants.kP, Constants.LauncherConstants.kI, FeederConstants.kD);

    public OptimizedFeed(Feeder feeder) {
        m_feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void execute() {
        m_feeder.setVoltage(m_feedforward.calculate(FeederConstants.kFeederSpeedRPM, FeederConstants.kMaxAcc) + m_controller.calculate(m_feeder.getEncoderRate(), FeederConstants.kFeederSpeedRPM));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_feeder.isSwitchSet();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        m_controller.reset();

    }
}
