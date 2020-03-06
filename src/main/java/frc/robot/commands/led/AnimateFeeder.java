package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LEDSubsystem;

import java.util.function.BooleanSupplier;

public class AnimateFeeder extends CommandBase {

    private LEDSubsystem m_ledSubsystem;
    private BooleanSupplier[] m_states;

    public AnimateFeeder(LEDSubsystem ledSubsystem, BooleanSupplier[] states) {
        m_ledSubsystem = ledSubsystem;
        m_states = states;

        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        for (int port : Constants.LEDContants.kLiftPorts) {
            if (!m_states[0].getAsBoolean()) {
                for (int i = 0; i < 7; i++) {
                    m_ledSubsystem.setLEDRGB(port, i, 0, 255, 0);
                }
            } else {
                for (int i = 16; i < Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.turnOffLED(port, i);
                }
            }
            m_ledSubsystem.setLEDRGB(port, 7, 100, 100, 100);
            if (!m_states[1].getAsBoolean()) {
                for (int i = 8; i < 15; i++) {
                    m_ledSubsystem.setLEDRGB(port, i, 0, 255, 0);
                }
            } else {
                for (int i = 16; i < Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.turnOffLED(port, i);
                }
            }
            m_ledSubsystem.setLEDRGB(port, 15, 100, 100, 100);
            if (!m_states[2].getAsBoolean()) {
                for (int i = 16; i < Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.setLEDRGB(port, i, 0, 255, 0);
                }
            } else {
                for (int i = 16; i < Constants.LEDContants.kLiftLength; i++) {
                    m_ledSubsystem.turnOffLED(port, i);
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
