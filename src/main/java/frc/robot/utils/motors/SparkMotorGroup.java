package frc.robot.utils.motors;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.utils.PIDValues;

public class SparkMotorGroup extends MotorControllerGroup {

    private CANSparkMax leader;
    private CANSparkMax[] followers;
    private RelativeEncoder encoder;

    public SparkMotorGroup(
            boolean isLeaderInverted,
            boolean areFollowersInverted,
            CANSparkMax leader,
            CANSparkMax... followers) {
        super(leader, followers);
        this.leader = leader;
        this.followers = followers;

        leader.setInverted(isLeaderInverted);

        // Set settings for followers
        for (CANSparkMax motor : this.followers) {
            motor.follow(leader, areFollowersInverted);
        }

        encoder = this.leader.getEncoder();
    }

    @Override
    public void set(double percentage) {
        leader.set(percentage);
    }

    public void setVoltage(double voltage) {
        leader.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        leader.stopMotor();
    }

    public void setPID(PIDValues pidValues, double minOutput, double maxOutput) {
        SparkMaxPIDController pidController = leader.getPIDController();
        pidController.setP(pidValues.P);
        pidController.setI(pidValues.I);
        pidController.setD(pidValues.D);
        pidController.setOutputRange(minOutput, maxOutput);
    }

    public void setCoast() {
        leader.setIdleMode(IdleMode.kCoast);
        for (CANSparkMax motor : followers) {
            motor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setBrake() {
        leader.setIdleMode(IdleMode.kBrake);
        for (CANSparkMax motor : followers) {
            motor.setIdleMode(IdleMode.kBrake);
        }
    }

    public CANSparkMax getLeader() {
        return leader;
    }

    public RelativeEncoder getEncoder() {
        return encoder;
    }
}
