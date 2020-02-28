package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Feed extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private Timer m_timer = new Timer();

    private int solenoid;

    public Feed(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        solenoid = FeederConstants.kSolenoidPorts.length - 1;
        for (int i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            m_feederSubsystem.setSolenoidState(i, false);
        }
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() >= FeederConstants.kBallDelay[solenoid]) {
            m_feederSubsystem.setSolenoidState(solenoid, true);
            if (solenoid > 0) {
                solenoid--;
            }
            m_timer.reset();
            m_timer.start();
        }
        m_feederSubsystem.setVoltage(FeederConstants.kMaxVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        m_feederSubsystem.setNumPowerCells(0);
    }

}
