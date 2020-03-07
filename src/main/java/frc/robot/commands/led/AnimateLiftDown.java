package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class AnimateLiftDown extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int head = Constants.LEDContants.kLiftLength - 1 - Constants.LEDContants.kLiftSnakeSize;
    private int tail = Constants.LEDContants.kLiftLength - 1;

    public AnimateLiftDown(LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        head = Constants.LEDContants.kLiftLength - 1 - Constants.LEDContants.kLiftSnakeSize;
        tail = Constants.LEDContants.kLiftLength - 1;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        for(int port : Constants.LEDContants.kLiftPorts) {
            if (m_timer.get() >= Constants.LEDContants.kLiftIncrementDelay) {
                m_ledSubsystem.turnOffLED(port, tail);
                head--;
                tail--;

                if (head <= 0) {
                    head = Constants.LEDContants.kLiftLength - 1;
                }
                if (tail <= 0) {
                    tail = Constants.LEDContants.kLiftLength - 1;
                }

                m_timer.reset();
                m_timer.start();
            }
            if (tail > head) {
                for (int i = head; i <= tail; i++) {
                    m_ledSubsystem.setLED(port, i, Color.kYellow);
                }
            } else if (tail < head) {
                for (int i = head; i < Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.setLED(port, i, Color.kYellow);
                }
                for (int i = 0; i <= tail; i++) {
                    m_ledSubsystem.setLED(port, i, Color.kYellow);
                }
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
