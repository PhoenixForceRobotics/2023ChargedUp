package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;

public class PFRController extends XboxController {

    public PFRController(int port) {
        super(port);
    }

    /** Adds ability to add deadzone (because our controllers drift... :p) */
    @Override
    public double getLeftX() {
        return super.getLeftX() < ControllerConstants.AXIS_DEADZONE ? 0 : super.getLeftX();
    }
    /** Adds ability to add deadzone (because our controllers drift... :p) */
    @Override
    public double getLeftY() {
        return super.getLeftY() < ControllerConstants.AXIS_DEADZONE ? 0 : super.getLeftY();
    }
    /** Adds ability to add deadzone (because our controllers drift... :p) */
    @Override
    public double getRightX() {
        return super.getRightX() < ControllerConstants.AXIS_DEADZONE ? 0 : super.getRightX();
    }
    /** Adds ability to add deadzone (because our controllers drift... :p) */
    @Override
    public double getRightY() {
        return super.getRightY() < ControllerConstants.AXIS_DEADZONE ? 0 : super.getRightY();
    }

    public double getLeftXSquared() {
        return getValueSquaredAndSign(getLeftX());
    }

    public double getLeftYSquared() {

        return getValueSquaredAndSign(getLeftY());
    }

    public double getRightXSquared() {

        return getValueSquaredAndSign(getRightX());
    }

    public double getRightYSquared() {
        return getValueSquaredAndSign(getRightY());
    }

    public static double getValueSquaredAndSign(double value) {
        return Math.pow(value, 2) * Math.signum(value);
    }

    public JoystickButton aButton() {
        return new JoystickButton(this, Button.kA.value);
    }

    public JoystickButton bButton() {
        return new JoystickButton(this, Button.kB.value);
    }

    public JoystickButton yButton() {
        return new JoystickButton(this, Button.kY.value);
    }

    public JoystickButton xButton() {
        return new JoystickButton(this, Button.kX.value);
    }

    public JoystickButton lBumper() {
        return new JoystickButton(this, Button.kLeftBumper.value);
    }

    public JoystickButton rBumper() {
        return new JoystickButton(this, Button.kRightBumper.value);
    }

    public JoystickButton lJoystickButton() {
        return new JoystickButton(this, Button.kLeftStick.value);
    }

    public JoystickButton rJoystickButton() {
        return new JoystickButton(this, Button.kLeftStick.value);
    }

    public POVButton dPadDownButton() {
        return new POVButton(this, ControllerConstants.DPAD_DOWN);
    }

    public POVButton dPadRightButton() {
        return new POVButton(this, ControllerConstants.DPAD_RIGHT);
    }

    public POVButton dPadUpButton() {
        return new POVButton(this, ControllerConstants.DPAD_UP);
    }

    public POVButton dPadLeftButton() {
        return new POVButton(this, ControllerConstants.DPAD_UP);
    }

    public Trigger lTriggerButton() {
        return new Trigger(this::lTriggerPressed);
    }

    public Trigger rTriggerButton() {
        return new Trigger(this::rTriggerPressed);
    }

    public boolean lTriggerPressed() {
        return getLeftTriggerAxis() > ControllerConstants.AXIS_DEADZONE;
    }

    public boolean rTriggerPressed() {
        return getRightTriggerAxis() > ControllerConstants.AXIS_DEADZONE;
    }
}
