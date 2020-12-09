package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

import java.util.function.BooleanSupplier;

public class AnimateFeeder extends CommandBase {

    private LEDSubsystem m_ledSubsystem;
    private BooleanSupplier[] m_solenoidStates;
    private int[][] m_strips;
    private boolean[] m_reverse;

    public AnimateFeeder(LEDSubsystem ledSubsystem, BooleanSupplier[] solenoidStates, int[][] strips, boolean[] reverse) {
        m_ledSubsystem = ledSubsystem;
        m_solenoidStates = solenoidStates;
        m_strips = strips;
        m_reverse = reverse;

        addRequirements(ledSubsystem);
    }

    public void setBranchColor(int i, int strip, int solenoid) {
        int section = 21 / m_solenoidStates.length;
            
        int start = i * section + m_strips[strip][0];
        int end = (i + 1) * section + m_strips[strip][0] - 1;
     //System.out.println("Start:" + start + " End: " + end);
        // Set LEDs on branch green if false
       if (!m_solenoidStates[solenoid].getAsBoolean()) {
            for (int j = start; j < end && j < m_strips[strip][1]; j++) {
                //System.out.println("LED: " + j + " ");
                m_ledSubsystem.setLED(j, Color.kGreen);
            }
        } else {
            for (int j =  start; j < end && j < m_strips[strip][1]; j++) {
                m_ledSubsystem.turnOffLED(j);
            }
        }
    //     System.out.print("LED: " + end + " ");
        //m_ledSubsystem.setLED(end, Color.kWhite); // divider LED
    }

    @Override
    public void execute() {
        
       m_ledSubsystem.turnOffLEDs();
        for(int strip = 0; strip < m_strips.length; strip++) {
             // Loop through each strip
           
           
                for (int i = 0; i < m_solenoidStates.length; i++) {
                     // Loop through each branch
                    setBranchColor(i, strip, !m_reverse[strip] ? i : m_solenoidStates.length - i - 1);
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
