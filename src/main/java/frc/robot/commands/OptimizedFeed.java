package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;

public class OptimizedFeed extends CommandBase {

    private Feeder m_feeder;

    public OptimizedFeed(Feeder feeder) {
        m_feeder = feeder;

        addRequirements(feeder);
    }

    @Override
    public void initialize() {
        for (int i = 0; i < FeederConstants.kLimitSwitchPorts.length; i++) {
            m_feeder.setSolenoidState(i, true);
        }
    }

    @Override
    public void execute() {

        int limitSwitchPort = (FeederConstants.kLimitSwitchPorts.length - 1) - m_feeder.getNumPowerCells();
        if (m_feeder.isSwitchSet(limitSwitchPort) && m_feeder.getSolenoidState(limitSwitchPort)) {
            m_feeder.setSolenoidState(limitSwitchPort, false);

            m_feeder.setNumPowerCells(m_feeder.getNumPowerCells() + 1);
        }

        m_feeder.set(FeederConstants.kFeederSpeedPer);
    }

    @Override
    public boolean isFinished() {
        return m_feeder.getNumPowerCells() == 3;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feeder.stop();
        if (interrupted) {
            m_feeder.setNumPowerCells(0);
        }
    }
}
