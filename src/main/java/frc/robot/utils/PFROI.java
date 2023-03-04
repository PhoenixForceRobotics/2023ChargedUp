package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class PFROI extends GenericHID{
    public PFROI(int port)
    {
        super(port);
    }

    /**
     * Creates a new JoystickButton
     * @param buttonNumber - <b>Index starts from 1!</b>
     * @return the new JoystickButton that can be attached to commands
     */
    public JoystickButton getButton(int buttonNumber)
    {
        return new JoystickButton(this, buttonNumber);
    }

    @Override
    public boolean getRawButtonPressed(int button) {
        return super.getRawButtonPressed(button);
    }
}
