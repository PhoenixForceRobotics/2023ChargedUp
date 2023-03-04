// package frc.robot.commands.arm;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
// import frc.robot.utils.CoordinateMath;
// import frc.robot.utils.PFRController;

// public class SetIntakeRelativeVelocities extends CommandBase {
//     private Arm arm;
//     private PFRController operatorController;

//     public SetIntakeRelativeVelocities(Arm arm, PFRController operatorController) {
//         this.arm = arm;
//         this.operatorController = operatorController;
//     }

//     @Override
//     public void initialize() {
//         // Love you amanuel :33333 <33333 - Vi
//         // Indeed - amanuel
//         // Do your work ! :)
//         // Pokemon supremacy
//     }

//     @Override
//     public void execute() {
//         double horizontalVelocity = operatorController.getLeftYSquared();
//         double verticalVelocity = operatorController.getLeftXSquared();

//         CoordinateMath.PolarCoordinates currentArmPolarCoordinates =
//                 new CoordinateMath.PolarCoordinates(
//                         arm.getArmRotationRadians(), arm.getFullExtensionMeters());
//         CoordinateMath.CartesianVelocities desiredCartesianVelocities =
//                 new CoordinateMath.CartesianVelocities(horizontalVelocity, verticalVelocity);

//         CoordinateMath.PolarVelocities desiredPolarVelocities =
//                 CoordinateMath.cartesianVelocitiesToPolarVelocities(
//                         currentArmPolarCoordinates, desiredCartesianVelocities);

//         arm.setArmRotationRadiansPerSecond(desiredPolarVelocities.getAngularVelocity());
//         arm.setExtensionMetersPerSecond(
//                 desiredPolarVelocities.getRadialVelocity() / 2,
//                 desiredPolarVelocities.getRadialVelocity() / 2);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         arm.setArmRotationRadiansPerSecond(0);
//         arm.setExtensionMetersPerSecond(0, 0);
//     }
// }
