package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class VictoryLift extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private boolean state = false;

    public VictoryLift(LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        state = false;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        for(int port : Constants.LEDContants.kLiftPorts) {
            if (m_timer.get() >= Constants.LEDContants.kLiftFlashDelay) {
                state = !state;
                m_timer.reset();
                m_timer.start();
            }
            if (state) {
                for (int led = 0; led < Constants.LEDContants.kLengths[port]; led++) {
                    m_ledSubsystem.setLEDRGB(port, led, 100, 100, 0);
                }
            } else {
                m_ledSubsystem.turnOffLEDs(port);
            }
            m_ledSubsystem.setData(port);
        }
    }

    @Override
    public void end(boolean interrupted) {
        for(int port : Constants.LEDContants.kLiftPorts) {
            m_ledSubsystem.turnOffLEDs(port);
            m_ledSubsystem.setData(port);
        }
    }
}
