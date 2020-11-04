package lib.components;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.DoubleSupplier;

public class QuickXboxController extends XboxController {

    public QuickXboxController(int port) {
        super(port);
    }

    public final JoystickButton buttonA = new JoystickButton(this, XboxController.Button.kA.value);
    public final JoystickButton buttonB = new JoystickButton(this, XboxController.Button.kB.value);
    public final JoystickButton buttonY = new JoystickButton(this, XboxController.Button.kY.value);
    public final JoystickButton buttonX = new JoystickButton(this, XboxController.Button.kX.value);
    public final JoystickButton bumperLeft = new JoystickButton(this, Button.kBumperLeft.value);
    public final JoystickButton bumperRight = new JoystickButton(this, Button.kBumperRight.value);
    public final JoystickButton buttonBack = new JoystickButton(this, Button.kBack.value);
    public final JoystickButton buttonStart = new JoystickButton(this, Button.kStart.value);


}