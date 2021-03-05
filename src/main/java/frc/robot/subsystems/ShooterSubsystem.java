package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.beans.Encoder;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_instance;

    private WPI_TalonFX m_master = new WPI_TalonFX(ShooterConstants.kMasterPort);
    private WPI_TalonFX m_slave = new WPI_TalonFX(ShooterConstants.kSlavePort);

    private double speed = 0;
    private double voltage = ShooterConstants.kMaxVoltage;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private NetworkTableEntry targetEntry = tab.add("Tagret RPM", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
    private NetworkTableEntry actualEntry = tab.add("Actual RPM", 0).withWidget(BuiltInWidgets.kTextView).getEntry();
  
    NetworkTableEntry sEntry = tab.add("kS", ShooterConstants.kS).withWidget(BuiltInWidgets.kTextView).getEntry();
    NetworkTableEntry vEntry = tab.add("kV", ShooterConstants.kV).withWidget(BuiltInWidgets.kTextView).getEntry();
    NetworkTableEntry aEntry = tab.add("kA", ShooterConstants.kA).withWidget(BuiltInWidgets.kTextView).getEntry();
    public ShooterSubsystem() {
        m_master.configFactoryDefault();
        m_slave.setInverted(true);
        m_slave.follow(m_master);
        m_master.setSelectedSensorPosition(0);
        m_slave.setSelectedSensorPosition(0);
        
    }

    @Override
    public void periodic() {
        actualEntry.setNumber(getVelocityRPM());
        //System.out.println(rpmEntry.getDouble(0));
        speed = targetEntry.getDouble(0);
    }

    public double getS() {
        return sEntry.getDouble(ShooterConstants.kS);
    }
 
    public double getV() {
     return vEntry.getDouble(ShooterConstants.kV);
 }
 
 public double getA() {
     return aEntry.getDouble(ShooterConstants.kA);
 }

    public double getFowardOutput(double targetVelocity) {
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(getS(), getV(), getA());
        return feedforward.calculate(targetVelocity);
    }

    public double getPIDOutput(double targetVelocity) {
        PIDController pid = new PIDController(ShooterConstants.kP, 0, 0);
        return pid.calculate(targetVelocity);
    }

    public double getOutput(double targetVelocity) {
return getFowardOutput(targetVelocity);
// + getPIDOutput(targetVelocity);
    }

    public double getVelocityRaw() {
        return (m_master.getSelectedSensorVelocity() + m_slave.getSelectedSensorVelocity()) / 2.0;
    }

    public double getVelocityRPM() {
        return getVelocityRaw() / ShooterConstants.kEncoderCPR;
    }

    public static ShooterSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ShooterSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_master.set(speed);
    }

    public double getSpeed() {
        return speed;
    }

public double getVoltage() {
    return voltage;
}

public void setConfigVoltage(double voltage) {
    this.voltage = voltage;
}

    public void setVoltage(double voltage) {
        m_master.setVoltage(voltage);
        this.voltage = voltage;
    }

    public void stop() {
        set(0);
    }
}
