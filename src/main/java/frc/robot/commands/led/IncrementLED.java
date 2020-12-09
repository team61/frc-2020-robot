package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class IncrementLED extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int m_snakeSize;
    private int m_intensityStart;
    private double m_intensityIncrement;
    private Color m_color;
    private int[] snakes;
    private double m_time;
    private int[][] m_strips;
    private boolean[] m_reverse;
    private boolean m_isUp;

    public IncrementLED(LEDSubsystem ledSubsystem, int[][] strips, boolean[] reverse, int snakeSize, double time, Color color, boolean isUp) {
        m_ledSubsystem = ledSubsystem;
        m_time = time;
        m_strips = strips;
        m_reverse = reverse;
        m_snakeSize = snakeSize;
        m_color = color;
        m_isUp = isUp;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        for(int i = 0; i < m_strips.length; i++) {
            snakes[i] = strips[i][0];
        }
    
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        if (m_timer.get() >= m_time) {
            for(int i = 0; i < snakes.length; i++) {
                snakes[i]++;
            }

            m_timer.reset();
            m_timer.start();
        }

        for(int strip = 0; strip < m_strips.length; strip++) {
            m_ledSubsystem.turnOffLEDs();
            if(m_isUp) {
                for(int i = snakes[strip]; i < snakes[strip] + length && i < m_strips[strip][1]; i++) {
                    m_ledSubsystem.setLED(i, m_color);
                }

                for(int i = m_strips[strip][0]; i < length - m_strips[strip][0] + snakes[strip]; i++) {
                    m_ledSubsystem.setLED(i, m_color);
                }
            } else {
                for(int i = snakes[strip]; i > snakes[strip] + length && i >= m_strips[strip][0]; i--) {
                    m_ledSubsystem.setLED(i, m_color);
                }

                for(int i = m_strips[strip][1] - 1; i < length - m_strips[strip][0] + snakes[strip]; i--) {
                    m_ledSubsystem.setLED(i, m_color);
                }
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
