package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Dump extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private Timer m_timer = new Timer();

    private int solenoid;

    public Dump(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        solenoid = 0;
        for (int i = 1; i < FeederConstants.kSolenoidPorts.length; i++) {
            m_feederSubsystem.setSolenoidState(i, false);
        }
        m_feederSubsystem.setSolenoidState(solenoid, true);
        solenoid++;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() >= 0.3) {
            m_feederSubsystem.setSolenoidState(solenoid, true);
            if (solenoid > 0) {
                solenoid++;
            }
            m_timer.reset();
            m_timer.start();
        }
        m_feederSubsystem.setVoltage(-FeederConstants.kMaxVoltage);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        m_feederSubsystem.setNumPowerCells(0);
    }

}