package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class IncrementLED extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int head = Constants.LEDContants.kLiftSnakeSize - 1;
    private int tail = 0;

    public IncrementLED(LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        head = Constants.LEDContants.kLiftSnakeSize - 1;
        tail = 0;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
            if (m_timer.get() >= Constants.LEDContants.kLiftIncrementDelay) {
                m_ledSubsystem.turnOffLED(tail);
                head++;
                tail++;

                if (head >= Constants.LEDContants.kLength) {
                    head = 0;
                }
                if (tail >= Constants.LEDContants.kLength) {
                    tail = 0;
                }

                m_timer.reset();
                m_timer.start();
            }
            if (tail < head) {
                for (int i = tail; i <= head; i++) {
                   m_ledSubsystem.setLEDRGB(i, 100, 0, 100);
                }
            } else if (tail > head) {
                for (int i = tail; i < Constants.LEDContants.kLength; i++) {
                    m_ledSubsystem.setLEDRGB(i, 100, 0, 100);
                }
                for (int i = 0; i <= head; i++) {
                    m_ledSubsystem.setLEDRGB(i, 100, 0, 100);
                }
            }

            m_ledSubsystem.setData();
        }


    @Override
    public void end(boolean interrupted) {
            m_ledSubsystem.turnOffLEDs();
            m_ledSubsystem.setData();
    }
}
