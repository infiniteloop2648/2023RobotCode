package frc.robot.commands.AutoCommands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.ManipulatorCommands.CloseManipulator;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.commands.WristCommands.PIDWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;

public class CubeAuto extends SequentialCommandGroup{
    
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Arm arm;
    private Wrist wrist;
    public CubeAuto(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        this.addRequirements(drivetrain, arm, wrist, manipulator);

    List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("2 Cube Auto", new PathConstraints(4.34, 2));

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("level2ArmMove", Commands.parallel(new PIDArm(Constants.kArmLevel2Angle, arm), new PIDWrist(Constants.kWristLevel2Angle, wrist)));
    eventMap.put("manipulatorOpen", new OpenManipulator(manipulator));
    eventMap.put("groundPickupArm", Commands.parallel(new PIDArm(Constants.kArmFloorAngle, arm), new PIDWrist(Constants.kWristFloorAngle, wrist)));
    eventMap.put("manipulatorClose", new CloseManipulator(manipulator));
    eventMap.put("level3ArmMove", Commands.parallel( new PIDArm(Constants.kArmLevel3Angle, arm), new PIDWrist(Constants.kWristFloorAngle, wrist)));
    eventMap.put("manipulatorOpen", new OpenManipulator(manipulator));

    RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, 
    drivetrain::resetOdometry, 
    new RamseteController(), 
    drivetrain.driveKinematics, 
    drivetrain::PIDVeloDrive, 
    eventMap,
    drivetrain);

    addCommands(autoBuilder.fullAuto(path));
    }
}

