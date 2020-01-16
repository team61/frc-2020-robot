package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretAutoAim extends CommandBase {

    private Turret m_turret;

    private ProfiledPIDController m_controller;

    private DoubleSupplier m_turretHeading;
    private DoubleSupplier m_driveTrainX;
    private DoubleSupplier m_driveTrainY;
    private DoubleSupplier m_driveTrainHeading;

    public TurretAutoAim(Turret turret, DoubleSupplier turretHeading, DoubleSupplier driveTrainX, DoubleSupplier driveTrainY, DoubleSupplier driveTrainHeading) {
        m_turret = turret;

        m_turretHeading = turretHeading;
        m_driveTrainX = driveTrainX;
        m_driveTrainY = driveTrainY;
        m_driveTrainHeading = driveTrainHeading;

        m_controller =
        new ProfiledPIDController(
                TurretConstants.kP, TurretConstants.kI, TurretConstants.kD,
                m_turret.getConstrants(), MiscellaneousConstants.kDt);

        addRequirements(turret); // Only turret is required as only the coordinate data is being retrieved from DriveTrain and Shooter; no motors are being controlled

    }

    @Override
    public void execute() {

        // Note: The turret is located backwards of the drive train and has its own coordinates, this is important for calculations

        // Finds displacements for turret coordinates to goal
        double dx = ShooterConstants.goalPosition.getX() - (m_driveTrainX.getAsDouble() + Math.cos(ShooterConstants.kAngleFromRobot) * ShooterConstants.kDistanceFromRobot);
        double dy = ShooterConstants.goalPosition.getY() - (m_driveTrainY.getAsDouble() + Math.sin(ShooterConstants.kAngleFromRobot) * ShooterConstants.kDistanceFromRobot);

        // Finds the angle between the turret point and the goal point on the field
        double TurretToGoalAngle = Math.toDegrees(Math.atan(dy / dx));

        // Convert the angle between the turret and angle to make an angle that the gyro will read when facing the goal which is a -180 to 180 representation
        double turretAngleToGoal =
                ((dy >= 0 && dx >= 0 || dy < 0 && dx >= 0) ? TurretToGoalAngle :
                        (dy < 0 && dx < 0) ? -90 - TurretToGoalAngle : 90 - TurretToGoalAngle);

        // If turret is with view of goal then set turretAngle to turretAngleToGoal otherwise set the angle moving based on how close it is to the goal
        double turretAngle = 0;
        if (turretAngleToGoal >= TurretConstants.kUpperLimit + m_driveTrainHeading.getAsDouble() ||
                turretAngleToGoal <= TurretConstants.kLowerLimit + m_driveTrainHeading.getAsDouble()) {
            turretAngle = turretAngleToGoal;
        } else {
            double angleAwayFromGoal = 2 * m_driveTrainHeading.getAsDouble() - turretAngleToGoal + 180;
            // Convert angleAwayFromGoal from a 0-360 or -360-0 representation to a -180 to 180 one
            turretAngle =
                    (angleAwayFromGoal > 180) ? angleAwayFromGoal - 360 :
                            (angleAwayFromGoal < -180) ? angleAwayFromGoal + 360 : angleAwayFromGoal;
        }

        m_turret.set(m_controller.calculate(m_turretHeading.getAsDouble(), turretAngle));
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stop();
    }
}

