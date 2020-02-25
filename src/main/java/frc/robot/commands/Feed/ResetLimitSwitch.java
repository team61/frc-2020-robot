package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.Constants.FeederConstants;

public class ResetLimitSwitch extends InstantCommand {

    private FeederSubsystem m_feederSubsystem;
    private int m_limitSwitch;

    public ResetLimitSwitch(FeederSubsystem feederSubsystem, int limitSwitch) {
        m_feederSubsystem = feederSubsystem;
        m_limitSwitch = limitSwitch;
    }

    @Override
    public void execute() {
        for (int i = m_limitSwitch; i >= 0; i--) {
            m_feederSubsystem.setSolenoidState(i, true);
        }

        m_feederSubsystem.setNumPowerCells(FeederConstants.kLimitSwitchPorts.length - 1 - m_limitSwitch);
    }
}
