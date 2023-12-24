package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.WristCommands.PIDWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class StowCommand extends SequentialCommandGroup{

    public StowCommand(Arm arm, Wrist wrist){
        addCommands(new PIDWrist(Constants.kWristStowedAngle, wrist), new PIDArm(Constants.kArmInitialPosition, arm));
        addRequirements(arm, wrist);
    }

}
