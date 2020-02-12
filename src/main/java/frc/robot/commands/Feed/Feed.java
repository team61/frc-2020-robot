package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Feed extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private boolean pastState;
    private int topLimitSwitch = FeederConstants.kLimitSwitchPorts.length - 1;


    public Feed(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        pastState = m_feederSubsystem.isSwitchSet(topLimitSwitch);

        for (int i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            m_feederSubsystem.setSolenoidState(i, true);
        }
    }

    @Override
    public void execute() {
        m_feederSubsystem.setVoltage(FeederConstants.kMaxVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        m_feederSubsystem.setNumPowerCells(0);
    }

}
