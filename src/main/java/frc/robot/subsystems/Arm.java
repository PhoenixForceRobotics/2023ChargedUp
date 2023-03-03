package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.utils.Motor;
import frc.robot.utils.PFRArmPIDController;
import frc.robot.utils.PFRExtensionPIDController;

public class Arm extends SubsystemBase {
    // Rotation motors
    private Motor armRotationLeader;
    private Motor armRotationFollower;
    private PFRArmPIDController armRotationController;

    // Extension motors
    private Motor firstStageExtensionMotor; // Fullsized Neo (bottom side)
    private Motor secondStageExtensionMotor; // Neo 550 (top side)
    private PFRExtensionPIDController firstStageExtensionController; // PID + feedforward for full neo
    private PFRExtensionPIDController secondStageExtensionContoller; // PID + feedforward for neo 550 

    // Motors for rotating claw independently to keep it level when arm has rotated
    private Motor clawRotationLeader;
    private Motor clawRotationFollower;
    private PFRArmPIDController clawRotationController;

    private boolean isClawIndependentlyControlled = true;
    private double desiredArmRadiansPerSecond = 0;
    private double desiredFirstStageMetersPerSecond = 0;
    private double desiredSecondStageMetersPerSecond = 0;
    private double desiredClawRadiansPerSecond = 0;


    /**
     * The arm that picks up game pieces from the floor through the use of the intake. It can rotate
     * -180 to 180 degrees and can extend a certain distance.
     */
    public Arm() {

        // Define rotational motors
        armRotationLeader =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_1_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);
        armRotationFollower =
                new Motor(
                        ArmConstants.ARM_ROTATION_MOTOR_2_PORT,
                        ArmConstants.ARM_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.ARM_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.ROTATION_MOTOR_WHEEL_DIAMETER);

        // set follower motor to follow leader
        armRotationFollower.follow(armRotationLeader);

        // create controller for arm rotation
        armRotationController = new PFRArmPIDController(ArmConstants.ARM_ROTATION_PID_VALUES);

        // Define extension motors
        firstStageExtensionMotor =
                new Motor(
                        ArmConstants.FIRST_STAGE_PORT,
                        ArmConstants.FIRST_STAGE_REVERSED,
                        ArmConstants.FIRST_STAGE_GEAR_RATIO,
                        ArmConstants.FIRST_STAGE_WHEEL_DIAMETER);
        secondStageExtensionMotor =
                new Motor(
                        ArmConstants.SECOND_STAGE_PORT,
                        ArmConstants.SECOND_STAGE_REVERSED,
                        ArmConstants.SECOND_STAGE_GEAR_RATIO,
                        ArmConstants.SECOND_STAGE_WHEEL_DIAMETER);
        firstStageExtensionController = new PFRExtensionPIDController(ArmConstants.FIRST_STAGE_PID_VALUES);
        secondStageExtensionContoller = new PFRExtensionPIDController(ArmConstants.SECOND_STAGE_PID_VALUES);
        
        // Define claw rotational motors
        clawRotationLeader =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_1_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_1_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        clawRotationFollower =
                new Motor(
                        ArmConstants.CLAW_ROTATION_MOTOR_2_PORT,
                        ArmConstants.CLAW_ROTATION_MOTOR_2_REVERSED,
                        ArmConstants.CLAW_ROTATION_MOTOR_GEAR_RATIO,
                        ArmConstants.CLAW_ROTATION_MOTOR_WHEEL_DIAMETER);
        
        // make follwer follw the leader, BUT INVERTED (due to how it works)
        clawRotationFollower.follow(clawRotationLeader, true);

        // Claw rotation controller
        clawRotationController = new PFRArmPIDController(ArmConstants.CLAW_ROTATION_PID_VALUES);
    }

    @Override
    public void periodic() {
        if(ArmConstants.MIN_ARM_ANGLE >= getArmRadians())
        {
            desiredArmRadiansPerSecond = Math.min(0, desiredArmRadiansPerSecond);
        }
        else if(ArmConstants.MAX_ARM_ANGLE <= getArmRadians())
        {
            desiredArmRadiansPerSecond = Math.max(desiredArmRadiansPerSecond, 0);
        }

        if(ArmConstants.FIRST_STAGE_MIN_EXTENSION >= getArmRadians())
        {
            desiredFirstStageMetersPerSecond = Math.min(0, desiredFirstStageMetersPerSecond);
        }
        else if(ArmConstants.FIRST_STAGE_MAX_EXTENSION <= getArmRadians())
        {
            desiredFirstStageMetersPerSecond = Math.max(desiredFirstStageMetersPerSecond, 0);
        }
        if(ArmConstants.SECOND_STAGE_MIN_EXTENSION >= getArmRadians())
        {
            desiredSecondStageMetersPerSecond = Math.min(0, desiredSecondStageMetersPerSecond);
        }
        else if(ArmConstants.SECOND_STAGE_MAX_EXTENSION <= getSecondStageMeters())
        {
            desiredSecondStageMetersPerSecond = Math.min(desiredSecondStageMetersPerSecond, 0);
        }
         
        if(ArmConstants.MIN_CLAW_ANGLE >= getArmRadians())
        {
            desiredClawRadiansPerSecond = Math.min(0, desiredArmRadiansPerSecond);
        }
        else if(ArmConstants.MAX_CLAW_ANGLE <= getArmRadians())
        {
            desiredClawRadiansPerSecond = Math.max(desiredArmRadiansPerSecond, 0);
        }
    }

    /**
     * Sets the angular velocity of the rotation motors in radians per second
     *
     * @param angularVelocity - velocity of the rotation motors in radians per second
     */
    public void setArmRotationRadiansPerSecond(double angularVelocity) {
        desiredArmRadiansPerSecond = angularVelocity;
    }

    /**
     * Sets the velocity for the extension motors
     *
     * @param velocity - velocity in meters per second you want to set it to
     */
    public void setFirstStageMetersPerSecond(double velocity) {
        desiredFirstStageMetersPerSecond = velocity;
    }

    public void setSecondStageMetersPerSecond(double velocity)
    {
        desiredSecondStageMetersPerSecond = velocity;
    }

    public void setExtensionMetersPerSecond(double firstStageVelocity, double secondStageVelocity)
    {
        setFirstStageMetersPerSecond(firstStageVelocity);
        setSecondStageMetersPerSecond(secondStageVelocity);
    }

    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @param angularVelocity - angular velocity of the claw rotation motors in radians per second
     */
    public void setClawRotationRadiansPerSecond(double angularVelocity) {
        desiredClawRadiansPerSecond = angularVelocity;
    }    
    /**
     * Sets the angular velocity of the claw in radians per second RELATIVE TO GROUND 
     * @param angularVelocity - angular velocity of the claw relative to gorund in radians per second 
     */
    public void setClawAbsoluteRadiansPerSecond(double angularVelocity)
    {
        desiredClawRadiansPerSecond = angularVelocity - getArmRadiansPerSecond();
    }
    
    /**
    * Gets the arm angle relative to level to ground
    * @return radians (CCW+, 🔄 positive, when )
    */
   public double getArmRadians()
   {
       return armRotationLeader.getRotations() * 2 * Math.PI;
   }

   /**
    * Gets the arm angular velocity
    * @return radians per second (CCW+, 🔄 positive, when FACING PORT SIDE)
    */
   public double getArmRadiansPerSecond()
   {
    return armRotationLeader.getRPM() / 60 * 2 * Math.PI;
   }

    public double getFirstStageMeters() {
        return firstStageExtensionMotor.getMeters();
    }

    public double getSecondStageMeters() {
        return secondStageExtensionMotor.getMeters();
    }

    /**
     * Gets the length of the extension arm from the center of rotation to tip
     *
     * @return the length in meters of the arm from the center of rotation to tip
     */
    public double getFullExtensionMeters() {
        return getFirstStageMeters() + getSecondStageMeters() + ArmConstants.EXTENSION_STARTING_LENGTH;
    }
   
    public double getFirstStageMetersPerSecond()
    {
        return firstStageExtensionMotor.getMetersPerSecond();
    }

    public double getSecondStageMetersPerSecond()
    {
        return secondStageExtensionMotor.getMetersPerSecond();
    }

    public double getFullExtensionMetersPerSecond()
    {
        return getFirstStageMetersPerSecond() + getSecondStageMeters();
    }

        
    /**
    * Gets the claw offset angle relative to parralel with arm
    * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
    */
    public double getClawRelativeAngleRadians()
    {
        return clawRotationLeader.getRPM() / 60 * Math.PI * 2;
    }

    
    /**
    * Gets the claw angle relative to level to ground
    * @return radians (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
    */
    public double getClawAbsoluteAngleRadians()
    {
        return 2 * Math.PI -  getClawAbsoluteAngleRadians() - getArmRadians();
    }

    /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO ARM
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawRelativeRadiansPerSecond() {
        return clawRotationLeader.getRPM() / 60 * 2 * Math.PI;
    }

     /**
     * Gets the rotational velocity of the claw in radians per second RELATIVE TO GROUND
     *
     * @return radians per second (CCW+, 🔄 positive, when viewed from STARBOARD SIDE)
     */
    public double getClawAbsoluteRadiansPerSecond() {
        return getArmRadiansPerSecond() + getClawRelativeRadiansPerSecond();
    }
}
