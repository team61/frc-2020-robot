package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class IncrementLED extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int m_start;
    private int m_snakeSize;
    private int m_length;
    private int m_intensityStart;
    private double m_intensityIncrement;
    private double r;
    private double g;
    private double b;
    private int head;
    private int tail;
    private double m_time;

    public IncrementLED(LEDSubsystem ledSubsystem, int start, int length, int snakeSize, double time, Color color, int intensityStart, double intensityIncrement) {
        m_ledSubsystem = ledSubsystem;
        m_time = time;
        m_start = start;
        m_snakeSize = snakeSize;
        m_length = length;
        m_intensityStart = intensityStart;
        m_intensityIncrement = intensityIncrement;
        r = color.red;
        g = color.green;
        b = color.blue;

        addRequirements(ledSubsystem);
    }

    public IncrementLED(LEDSubsystem ledSubsystem, int start, int length, int snakeSize, double time, Color color) {
        m_ledSubsystem = ledSubsystem;
        m_time = time;
        m_start = start;
        m_snakeSize = snakeSize;
        m_length = length;
        m_intensityStart = 0;
        m_intensityIncrement = 0;
        r = color.red;
        g = color.green;
        b = color.blue;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        head = m_snakeSize - 1 + m_start;
        tail = m_start;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
            if (m_timer.get() >= m_time) {
                m_ledSubsystem.turnOffLED(tail);
                head++;
                tail++;

                if (head >= m_length + m_start) {
                    head = m_start;
                }
                if (tail >= m_length + m_start) {
                    tail = m_start;
                }

                m_timer.reset();
                m_timer.start();
            }
            if (tail < head) {
                int intensity = m_intensityStart;
                for (int i = tail; i <= head + m_start; i++) {
                    m_ledSubsystem.setLEDRGB(i, (int) (r * intensity), (int) (g * intensity), (int) (b * intensity));
                    intensity += m_intensityIncrement;
                }
            } else if (tail > head) {
                int intensity = m_intensityStart;
                for (int i = tail; i < m_length + m_start; i++) {
                    m_ledSubsystem.setLEDRGB(i, (int) (r * intensity), (int) (g * intensity), (int) (b * intensity));
                    intensity += m_intensityIncrement;

                }
                intensity = m_intensityStart;
                for (int i = m_start; i <= head; i++) {
                    m_ledSubsystem.setLEDRGB(i, (int) (r * intensity), (int) (g * intensity), (int) (b * intensity));
                    intensity += m_intensityIncrement;
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
