package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

import java.util.function.BooleanSupplier;

public class AnimateFeeder extends CommandBase {

    private final int r = 100;
    private final int g = 0;
    private final int b = 100;

    private LEDSubsystem m_ledSubsystem;
    private BooleanSupplier[] m_solenoidStates;

    public AnimateFeeder(LEDSubsystem ledSubsystem, BooleanSupplier[] solenoidStates) {
        m_ledSubsystem = ledSubsystem;
        m_solenoidStates = solenoidStates;

        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
            if (!m_solenoidStates[0].getAsBoolean()) {
                for (int i = 0; i < 7; i++) {
                    m_ledSubsystem.setLEDRGB(i, 0, 255, 0);
                }
            } else {
                for (int i = 0; i < 7; i++) {
                    m_ledSubsystem.turnOffLED(i);
                }
            }
            m_ledSubsystem.setLEDRGB(7, 100, 100, 100);
            if (!m_solenoidStates[1].getAsBoolean()) {
                for (int i = 8; i < 15; i++) {
                    m_ledSubsystem.setLEDRGB(i, 0, 255, 0);
                }
            } else {
                for (int i = 8; i < 15; i++) {
                    m_ledSubsystem.turnOffLED(i);
                }
            }
            m_ledSubsystem.setLEDRGB(15, 100, 100, 100);
            if (!m_solenoidStates[2].getAsBoolean()) {
                for (int i = 16; i < Constants.LEDContants.kLength; i++) {
                    m_ledSubsystem.setLEDRGB(i, 0, 255, 0);
                }
            } else {
                for (int i = 16; i < Constants.LEDContants.kLength; i++) {
                    m_ledSubsystem.turnOffLED(i);
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
