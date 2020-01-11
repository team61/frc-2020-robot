package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class TurretAutoAim extends CommandBase {

    private Turret m_turret;
    private DriveTrain m_driveTrain;

    private final ProfiledPIDController m_controller =
    new ProfiledPIDController(
        TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 
        new TrapezoidProfile.Constraints(TurretConstants.kMaxVel, TurretConstants.kMaxAcc), MiscellaneousConstants.kDt);

    public TurretAutoAim(Turret turret, DriveTrain driveTrain) {
        m_turret = turret;
        m_driveTrain = driveTrain;
        addRequirements(turret);

    }

    @Override
    public void execute() {

        // Note: The turret is located backwards of the drivetrain and has its own coords, this is important for calculations

        // Finds displacements for turret coords to goal
        double dx = TurretConstants.goalPosition.getX() - m_turret.getX();
        double dy = TurretConstants.goalPosition.getY() - m_turret.getY();

        // Find the angle for the displacements
        double angledisplacement = Math.toDegrees(Math.atan(dy/dx));

        // Convert the angle displacement from that of a triangle to a -180 to 180 representation
        double turretAngleToGoal = 
            ((dy >= 0 && dx >= 0 || dy < 0 && dx >= 0) ? angledisplacement :
            (dy < 0 && dx < 0) ? -90 - angledisplacement : 90 - angledisplacement);

        // If turret is with view of goal then set turretAngle to turretAngleToGoal other set the angle moving based on how close it is to the goal
        double turretAngle = 0;
        
        if(turretAngleToGoal >= TurretConstants.kUpperLimit + m_driveTrain.getAngle() || 
        turretAngleToGoal <= TurretConstants.kLowerLimit + m_driveTrain.getAngle()) {
            turretAngle = turretAngleToGoal;
        } else {
            double angleAwayFromGoal =  m_driveTrain.getAngle() + (m_driveTrain.getAngle() - turretAngleToGoal) + 180;
            // Convert angleAwayFromGoal from a 0-360 or -360-0 representation to a -180 to 180 one
            turretAngle = 
                (angleAwayFromGoal > 180) ? angleAwayFromGoal - 360 :
                (angleAwayFromGoal < -180) ? angleAwayFromGoal + 360 : angleAwayFromGoal;
        }

        m_turret.setSpeed(m_controller.calculate(m_turret.getAngle(), turretAngle));
    }
    @Override
    public void end(boolean interrupted) {
        
    }
}

