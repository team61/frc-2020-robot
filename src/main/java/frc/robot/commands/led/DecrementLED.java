package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class DecrementLED extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int head = Constants.LEDContants.kLiftLength - Constants.LEDContants.kLiftSnakeSize + 1;
    private int tail = Constants.LEDContants.kLiftLength;

    public DecrementLED(LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        head = Constants.LEDContants.kLiftLength - Constants.LEDContants.kLiftSnakeSize + 1;
        tail = Constants.LEDContants.kLiftLength;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        for (int port : Constants.LEDContants.kLiftPorts) {
            if (m_timer.get() >= Constants.LEDContants.kLiftIncrementDelay) {
                m_ledSubsystem.turnOffLED(port, tail);
                head--;
                tail--;

                // Checks to see if led is within bounds of led strip
                head = (head < 0) ? Constants.LEDContants.kLiftLength : head;
                tail = (tail < 0) ? Constants.LEDContants.kLiftLength : tail;

                m_timer.reset();
                m_timer.start();
            }
            if (tail > head) {
                for (int i = head; i <= tail; i++) {
                    m_ledSubsystem.setLEDRGB(port, Math.abs(i - Constants.LEDContants.kLiftLength), 100, 0, 100);
                }
            } else if (tail < head) {
                for (int i = head; i <= Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.setLEDRGB(port, i, 100, 0, 100);
                }
                for (int i = 0; i <= tail; i++) {
                    m_ledSubsystem.setLEDRGB(port, i, 100, 0, 100);
                }
                m_ledSubsystem.setData(port);
            }
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
