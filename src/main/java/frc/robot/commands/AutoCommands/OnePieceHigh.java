package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.DrivetrainCommands.PIDDrive;
import frc.robot.commands.DrivetrainCommands.TankDrive;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.commands.WristCommands.SetVoltsWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class OnePieceHigh extends SequentialCommandGroup {
    public OnePieceHigh(Arm arm, Wrist wrist, Drivetrain drivetrain, Manipulator manipulator) {
        addCommands(
            new PIDArm(Constants.kArmLevel3Angle, arm), new TankDrive(drivetrain, 0.3, 0.3).withTimeout(0.5),
            new SetVoltsWrist(wrist, 8).withTimeout(3), new OpenManipulator(manipulator),
            new SetVoltsWrist(wrist, -10).withTimeout(3),
            Commands.parallel(new PIDDrive(Units.inchesToMeters(-13*12), Units.inchesToMeters(-13*12), drivetrain), new PIDArm(Constants.kArmFloorAngle, arm))
        );
    }
}
