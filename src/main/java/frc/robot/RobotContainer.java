// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.SetAngle;
import frc.robot.commands.arm.SetLength;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.utils.PFRController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here...
    // private final Drivebase drivebase = new Drivebase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // The robot's controllers are defined here...
    private final PFRController operatorController = new PFRController(0);
    private final PFRController driverController = new PFRController(1);

    // The robot's commands are defined here...
    // private final ClawIntakeSequence pickUpCube = new ClawIntakeSequence(claw, true);
    // private final ClawIntakeSequence pickUpCone = new ClawIntakeSequence(claw, false);
    private final SetLength startingPosition = new SetLength(arm, ArmConstants.FIRST_STAGE_MIN_EXTENSION + ArmConstants.SECOND_STAGE_MIN_EXTENSION);
    private final SetLength testing1 = new SetLength(arm,  (ArmConstants.FIRST_STAGE_MIN_EXTENSION + ArmConstants.SECOND_STAGE_MIN_EXTENSION) * 0.2);
    private final SetLength testing2 = new SetLength(arm, (ArmConstants.FIRST_STAGE_MIN_EXTENSION + ArmConstants.SECOND_STAGE_MIN_EXTENSION) * 0.2);
    
    private final SetAngle startingAngle = new SetAngle(arm, ArmConstants.ARM_ROTATION_STARTING_ANGLE);
    private final SetAngle testingAngle1 = new SetAngle(arm, 0);
    private final SetAngle testingAngle2 = new SetAngle(arm, Math.toRadians(90));


    // It's useful to set the autonomous commands seperately


    // And the NetworkTable/NetworkTable/CommandChooser variables :)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        initializeListenersAndSendables();
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverController.aButton().onTrue(startingPosition);
        driverController.xButton().onTrue(testing1);
        driverController.yButton().onTrue(testing2);

        operatorController.aButton().onTrue(startingAngle);
        operatorController.xButton().onTrue(testingAngle1);
        operatorController.bButton().whileTrue(testingAngle2);
    }

    public void initializeListenersAndSendables() {
        // Main Tab

        // Add options for chooser

        // Places chooser on mainTab (where all configs are)
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public void initializeTeleopCommands() {
    }

    public PFRController getDriverController() {
        return driverController;
    }

    public PFRController getOperatorController() {
        return operatorController;
    }
}
