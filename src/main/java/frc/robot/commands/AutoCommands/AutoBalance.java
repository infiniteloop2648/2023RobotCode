package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.DrivetrainCommands.PIDDrive;
import frc.robot.commands.DrivetrainCommands.TankDrive;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class AutoBalance extends SequentialCommandGroup {
    

    public AutoBalance(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        addCommands(
            Commands.parallel(
                Commands.runOnce(
                    () -> arm.setGoal(Constants.kArmLevel3Angle), arm
                ), 
                Commands.runOnce(
                    () -> wrist.setGoal(Constants.kWristLevel3Angle)
                )
            ),
            new WaitCommand(1),
            new TankDrive(drivetrain, 0.4, 0.4).withTimeout(0.5),
            Commands.runOnce(
                () -> drivetrain.resetEncoders(), drivetrain
            ),
            new OpenManipulator(manipulator),
            Commands.parallel(
                Commands.runOnce(
                    () -> arm.setGoal(Constants.kArmInitialPosition), arm
                ), 
                Commands.runOnce(
                    () -> wrist.setGoal(Constants.kWristStowedAngle)
                )
            ),
            new PIDDrive(-Constants.kAutoStart2StationEdge, -Constants.kAutoStart2StationEdge, drivetrain).withTimeout(2.5),
            Commands.run(
                () -> drivetrain.chargeStationBalance(), drivetrain
            )
        );
    }
}
