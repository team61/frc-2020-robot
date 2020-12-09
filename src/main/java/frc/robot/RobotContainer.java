/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.Drive.SimpleDrive;
import frc.robot.commands.Drive.TankDrive;
import frc.robot.commands.Feed.*;
import frc.robot.commands.Lift.Climb;
import frc.robot.commands.Shoot.Fire;
import frc.robot.commands.Shoot.Shoot;
import frc.robot.commands.Turret.TurretAutoAimVision;
import frc.robot.commands.Turret.TurretWithJoysticks;
//import frc.robot.commands.led.IncrementLED;
import frc.robot.subsystems.*;
import lib.components.QuickXboxController;

import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final TurretSubsystem m_turretSubsystem = TurretSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
    private final VisionSubsystem m_visionSubsystem = VisionSubsystem.getInstance();
  //  private final WheelSpinner m_wheelSpinner = WheelSpinner.getInstance();
    private final LEDSubsystem m_LEDSubsystem = LEDSubsystem.getInstance();

    private final QuickXboxController driveController = new QuickXboxController(OIConstants.driveControllerPin);
    private final QuickXboxController controlController = new QuickXboxController(OIConstants.controlControllerPin);

    private Trigger BeltTrigger = new Trigger(()-> controlController.getAButton() || controlController.getBButton() || controlController.getYButton());
    private ParallelDeadlineGroup m_fire = new ParallelDeadlineGroup(
            new WaitCommand(FeederConstants.kAutoDelay),
            new Fire(m_shooterSubsystem, m_feederSubsystem)
    );

    private Command m_autoCommand = m_fire.andThen(new SimpleDrive(m_driveSubsystem, 6, 2));

    private final TurretAutoAimVision m_aim = new TurretAutoAimVision(m_turretSubsystem,m_visionSubsystem::getYaw);

    SendableChooser<Command> m_chooser = new SendableChooser<>();


    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(new TankDrive(m_driveSubsystem, () -> driveController.getY(GenericHID.Hand.kLeft), () -> driveController.getY(GenericHID.Hand.kRight)));
        m_turretSubsystem.setDefaultCommand(new TurretWithJoysticks(m_turretSubsystem, () -> controlController.getX(GenericHID.Hand.kRight)));
        m_feederSubsystem.setDefaultCommand(new Feed(m_feederSubsystem, () -> controlController.getY(GenericHID.Hand.kLeft)));
        
       m_LEDSubsystem.setDefaultCommand(new AnimateFeeder(m_LEDSubsystem, new BooleanSupplier[]{
           () -> m_feederSubsystem.getSolenoidState(0),
               () -> m_feederSubsystem.getSolenoidState(1),
               () -> m_feederSubsystem.getSolenoidState(2)}
               , new int[][]{{0, 20}, {21, 40}, {41, 60}}, new boolean[]{false, true, false}));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driveController.bumperRight.whileHeld(new Intake(m_feederSubsystem));

        controlController.buttonStart.whenPressed(new Climb(m_liftSubsystem));

        controlController.bumperRight.whileHeld(new Fire(m_shooterSubsystem, m_feederSubsystem));

        controlController.buttonA.whileHeld(new ResetLimitSwitch(m_feederSubsystem, 0));
        controlController.buttonB.whileHeld(new ResetLimitSwitch(m_feederSubsystem, 1));
        controlController.buttonY.whileHeld(new ResetLimitSwitch(m_feederSubsystem, 2));


        controlController.bumperLeft.whileHeld(m_aim);
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return m_autoCommand;
    }
}
