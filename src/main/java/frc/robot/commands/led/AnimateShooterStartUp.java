package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class AnimateShooterStartUp extends CommandBase {

    private LEDSubsystem m_ledSubsystem;

    private Timer m_timer = new Timer();
    private int led = 0;
    private int intensity = 100;

    public AnimateShooterStartUp(LEDSubsystem ledSubsystem) {
        m_ledSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        led = 0;
        intensity = 100;
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        for(int port : Constants.LEDContants.kLiftPorts) {
            if (m_timer.get() >= Constants.LEDContants.kShootChargeDelay) {
                led++;
                intensity += 5;

                if (intensity >= 255) {
                    intensity = 255;
                }

                m_timer.reset();
                m_timer.start();
            }
            m_ledSubsystem.setLEDRGB(port, led, intensity, 0, intensity);

            m_ledSubsystem.setData(port);
        }
    }

    @Override
    public boolean isFinished() {
        return led >= Constants.LEDContants.kLiftLength - 1;
    }

    @Override
    public void end(boolean interrupted) {
        for(int port : Constants.LEDContants.kLiftPorts) {
            m_ledSubsystem.turnOffLEDs(port);
            m_ledSubsystem.setData(port);
        }
    }
}
