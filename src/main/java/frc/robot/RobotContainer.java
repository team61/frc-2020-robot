/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.NormalTurretWithJoysticks;
import frc.robot.commands.NormalDriveWithJoysticks;
import frc.robot.commands.TurretAutoAim;
import lib.components.LogitechJoystick;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Turret m_turret = new Turret(m_driveTrain);

  private final LogitechJoystick jLeft = new LogitechJoystick(OIConstants.jLeft);
  private final LogitechJoystick jRight = new LogitechJoystick(OIConstants.jRight);

  private final LogitechJoystick jTurretHeading = new LogitechJoystick(OIConstants.jTurretHeading);
  private final LogitechJoystick jTurretAngle = new LogitechJoystick(OIConstants.jTurretAngle);

  private final NormalDriveWithJoysticks m_normalDriveWithJoysticks = new NormalDriveWithJoysticks(m_driveTrain, jLeft::getYAxis, jRight::getYAxis);
  private final TurretAutoAim m_turretAutoAim = new TurretAutoAim(m_turret, m_driveTrain);
  private final NormalTurretWithJoysticks m_normalTurretWithJoysticks = new NormalTurretWithJoysticks(m_turret, jTurretHeading::getYAxis);

  private final Command m_autoCommand = null;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private NetworkTableEntry m_maxSpeed;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(m_normalDriveWithJoysticks);
    m_turret.setDefaultCommand(m_turretAutoAim);

    m_maxSpeed = Shuffleboard.getTab("Configuration")
                           .add("Max Speed", 1)
                           .withWidget("Number Slider")
                           .withPosition(1, 1)
                           .withSize(2, 1)
                           .getEntry();

    // Add the tank drive and encoders to a 'Drivebase' tab
    ShuffleboardTab driveBaseTab = Shuffleboard.getTab("Drivebase");
    driveBaseTab.add("Tank Drive", m_driveTrain);
    // Put both encoders in a list layout
    ShuffleboardLayout encoders = driveBaseTab.getLayout("List Layout", "Encoders")
                                              .withPosition(0, 0)
                                              .withSize(2, 2);
    encoders.add("Left Encoder", m_driveTrain.getLeftEncoderDistance());
    encoders.add("Right Encoder", m_driveTrain.getRightEncoderDistance());

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
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
