package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.DrivetrainCommands.PIDDrive;
import frc.robot.commands.DrivetrainCommands.PIDTurnAngle;
import frc.robot.commands.ManipulatorCommands.CloseManipulator;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.commands.WristCommands.SetVoltsWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;


public class PositionalPIDRed extends SequentialCommandGroup{

    public PositionalPIDRed(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){

        addCommands(
            new CloseManipulator(manipulator),
            new PIDArm(Constants.kArmLevel3Angle, arm),
            new PIDDrive(Units.inchesToMeters(5), Units.inchesToMeters(5), drivetrain),
            new SetVoltsWrist(wrist, 8).withTimeout(3),
            new OpenManipulator(manipulator),
            new SetVoltsWrist(wrist, -11).withTimeout(3),
            Commands.parallel(new PIDDrive(-2.68, -2.68, drivetrain), new PIDArm(Constants.kArmFloorAngle, arm)),
            new PIDTurnAngle(10, drivetrain),
            new PIDDrive(-1.8, -1.8, drivetrain),
            new PIDTurnAngle(0, drivetrain),
            new SetVoltsWrist(wrist, 8).withTimeout(3),
            new CloseManipulator(manipulator),
            new SetVoltsWrist(wrist, -11).withTimeout(3),
            new PIDTurnAngle(-180, drivetrain),
            new PIDDrive(4.41, 4.41, drivetrain)

            /* 
            new PIDDrive(Units.inchesToMeters(5), Units.inchesToMeters(5), drivetrain),
            new OpenManipulator(manipulator),
            new SetVoltsWrist(wrist, -9).withTimeout(1),
            Commands.parallel( new PIDDrive(-4.25, -4.25, drivetrain).withTimeout(4), new PIDArm(Constants.kArmFloorAngle, arm)),
            new PIDTurnAngle(-175, drivetrain).withTimeout(3),
            new SetVoltsWrist(wrist, 8).withTimeout(3),
            new CloseManipulator(manipulator),
            new SetVoltsWrist(wrist, -9).withTimeout(3),
            new PIDTurnAngle(-15, drivetrain),
            new PIDDrive(1.46, 1.46, drivetrain),
            new PIDTurnAngle(0, drivetrain),
            Commands.parallel(new PIDDrive(2.82, 2.82, drivetrain), new PIDArm(Constants.kArmLevel3Angle, arm)),
            new SetVoltsWrist(wrist, 8).withTimeout(3),
            new OpenManipulator(manipulator)
            */
        );
    }
    
}
