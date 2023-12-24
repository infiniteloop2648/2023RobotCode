package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DrivetrainCommands.TankDrive;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class OnePieceDrive extends SequentialCommandGroup{

    public OnePieceDrive(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        addCommands(
            Commands.either(
                Commands.runOnce(() -> arm.toggleFeedForward(), arm), 
                Commands.runOnce(() -> {}, arm), 
                arm::isFeedForwarding),
            Commands.parallel(
                Commands.runEnd(() -> arm.setPower(.5), () -> arm.toggleFeedForward(), arm).withTimeout(.25),
                new TankDrive(drivetrain, .5, .5).withTimeout(1),
                Commands.runEnd(() -> wrist.powerWrist(-.5), () -> {}, wrist)
            ),
            new OpenManipulator(manipulator),
            Commands.parallel(
                Commands.runEnd(() -> {
                    if(arm.isFeedForwarding()) {
                        arm.toggleFeedForward();
                    }
                    arm.setPower(-.5);
                }, () -> arm.toggleFeedForward(), arm).withTimeout(.25),
                Commands.runEnd(() -> wrist.powerWrist(.5), () -> {}, wrist),
                new TankDrive(drivetrain, -.5, -.5).withTimeout(3)
            )
        );

        /*
        addCommands(
            Commands.runEnd(()-> arm.setPower(0.5), () -> arm.setPower(0), arm).withTimeout(0.25),
            new TankDrive(drivetrain, 0.5, 0.5).withTimeout(1),
            Commands.run(() -> wrist.powerWrist(-0.5), wrist).withTimeout(1), 
            new OpenManipulator(manipulator),
            Commands.parallel(
                Commands.runOnce(() -> arm.setPower(-0.75), arm).withTimeout(1), 
                Commands.runOnce(() -> wrist.powerWrist(0.75), wrist).withTimeout(1), 
                new TankDrive(drivetrain, -0.75, -0.75).withTimeout(3)
            )
        ); */
    }
}
