package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import lib.components.LimitSwitch;

public class TurretSubsystem extends SubsystemBase {

    private static TurretSubsystem m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(TurretConstants.kMotorPort);

   private Encoder m_encoder = new Encoder(TurretConstants.kEncoderPorts[0], TurretConstants.kEncoderPorts[1], TurretConstants.kEncoderReversed);

   private LimitSwitch m_limitSwitch = new LimitSwitch(TurretConstants.kLimitSwitchPort);

   private double position = 0;

   ShuffleboardTab tab = Shuffleboard.getTab("Turret");
   NetworkTableEntry sEntry = tab.add("kS", TurretConstants.kS).withWidget(BuiltInWidgets.kTextView).getEntry();
   NetworkTableEntry vEntry = tab.add("kV", TurretConstants.kV).withWidget(BuiltInWidgets.kTextView).getEntry();
   NetworkTableEntry aEntry = tab.add("kA", TurretConstants.kA).withWidget(BuiltInWidgets.kTextView).getEntry();
   NetworkTableEntry maxSpeedEntry = tab.add("Max Speed", TurretConstants.kMaxSpeed).withWidget(BuiltInWidgets.kTextView).getEntry();
   NetworkTableEntry angleEntry = tab.add("Angle", position * TurretConstants.kDistanceToDegrees).withWidget(BuiltInWidgets.kDial).getEntry();

public double getAngle() {
return angleEntry.getDouble(position * TurretConstants.kDistanceToDegrees);
}

public double getOutput(double targetVelocity) {
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(getS(), getV(), getA());
    return feedforward.calculate(targetVelocity);
}

   public double getS() {
       return sEntry.getDouble(TurretConstants.kS);
   }

   public double getV() {
    return vEntry.getDouble(TurretConstants.kV);
}

public double getA() {
    return aEntry.getDouble(TurretConstants.kA);
}

public double getMaxSpeed() {
    return maxSpeedEntry.getDouble(TurretConstants.kS);
}

   public TurretSubsystem() {
   
       m_encoder.setDistancePerPulse(TurretConstants.kEncoderDistancePerPulse);
   }

    @Override
    public void periodic() {
       

        if (isSwitchSet()) {
            position = 0;
        }
        position += getEncoderDistance();
        SmartDashboard.putNumber("Turret Angle", position * TurretConstants.kDistanceToDegrees);
        resetEncoder();
    }

    public static TurretSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new TurretSubsystem();
        }

        return m_instance;
    }

    public double getPosition() {
       return position;
    }

    public void set(double speed) {
       if (position >= TurretConstants.kMaxDistance) {
           if (speed <= 0) {
               m_motor.set(speed);
           } else {
               stop();
           }
       } else if (isSwitchSet()) {
           if (speed >= 0) {
               m_motor.set(speed);
           } else {
               stop();
           }
       } else {
           m_motor.set(speed);
       }
    }

    public void setVoltage(double voltage) {
        if (position >= TurretConstants.kMaxDistance) {
            if (voltage <= 0) {
                m_motor.setVoltage(voltage);
            } else {
                stop();
            }
        } else if (isSwitchSet()) {
            if (voltage >= 0) {
                m_motor.setVoltage(voltage);
            } else {
                stop();
            }
        } else {
            m_motor.setVoltage(voltage);
        }
    }

    public void stop() {
        m_motor.setVoltage(0);
    }

    public int getEncoderValue() {
        return m_encoder.get();
    }

    public double getEncoderDistance() {
       return m_encoder.getDistance();
    }

    public double getEncoderRate() {
        return m_encoder.getRate();
    }

    public void resetEncoder() {
        m_encoder.reset();
    }

    public boolean isSwitchSet() {
        return m_limitSwitch.isSwitchSet();
    }

}
