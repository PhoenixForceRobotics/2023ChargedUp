package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class SetAngle extends CommandBase {
    private Arm arm;
    private double desiredAngle;

    private PIDController pidController;

    private ShuffleboardTab rotationTab;
    private GenericEntry angleError;
    private GenericEntry currentAngle;

    public SetAngle(Arm arm, double desiredAngle)
    {
        this.arm = arm;
        this.desiredAngle = desiredAngle;

        pidController = new PIDController(ArmConstants.ROTATION_PID_P, ArmConstants.ROTATION_PID_I, ArmConstants.ROTATION_PID_D);

        rotationTab = Shuffleboard.getTab("Rotation");
        angleError = rotationTab.add("Angle Error", 0).getEntry();
        currentAngle = rotationTab.add("Current Angle", 0).getEntry();
        rotationTab.add("PID Controller", pidController);
    }

    @Override
    public void initialize()
    {
        pidController.setSetpoint(desiredAngle); // Where desired angle is in degrees
    }

    @Override
    public void execute()
    {
        currentAngle.setDouble(arm.getRotationAngle());

        double error = desiredAngle - arm.getRotationAngle();
        angleError.setDouble(error);

        double output = MathUtil.clamp(pidController.calculate(arm.getRotationAngle()), -0.9, 0.9);
        arm.setRotationMetersPerSecond(output);
    }

    @Override
    public boolean isFinished()
    {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted)
    {

    }
}
