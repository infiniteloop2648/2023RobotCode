package frc.robot.commands.AutoCommands;

import java.nio.file.Path;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.RamseteAutoBuilder;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PIDArmWrist;
import frc.robot.commands.StowCommand;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.ManipulatorCommands.CloseManipulator;
import frc.robot.commands.ManipulatorCommands.OpenManipulator;
import frc.robot.commands.ManipulatorCommands.ToggleManipulator;
import frc.robot.commands.WristCommands.PIDWrist;
import frc.robot.commands.WristCommands.SetVoltsWrist;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;
import frc.robot.Constants;

public class ConeCube extends SequentialCommandGroup{
    
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Arm arm;
    private Wrist wrist;
    public ConeCube(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        this.drivetrain = drivetrain;
        this.manipulator = manipulator;
        this.arm = arm;
        this.wrist = wrist;
        this.addRequirements(drivetrain, arm, wrist, manipulator);

/* 
        PathPlannerTrajectory please = PathPlanner.loadPath("Cone_Cube_Hump", new PathConstraints(2, 1));

        addCommands( drivetrain.followTrajectory(please, true));
        */
         
        
        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("One Cone_Cube", new PathConstraints(4.00, 2));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("level3Position",
            new PIDArmWrist(Constants.kArmLevel3Angle, Units.degreesToRadians(40), arm, wrist));

        eventMap.put("moveWrist", 
            new PIDArmWrist(Constants.kArmLevel3Angle, Units.degreesToRadians(0.0), arm, wrist));

        eventMap.put("placeCone", 
            new ToggleManipulator(manipulator));

        eventMap.put("stow", 
            Commands.parallel(new PIDArm(Constants.kArmInitialPosition, arm), new PIDWrist(Constants.kWristStowedAngle, wrist)));

        eventMap.put("groundPosition", 
            Commands.parallel(new PIDArm(Constants.kArmFloorAngle, arm), new PIDWrist(Units.degreesToRadians(-15), wrist)));

        eventMap.put("pickup", Commands.parallel(
            new CloseManipulator(manipulator)));

        eventMap.put("stow", 
            Commands.parallel(new PIDArm(Constants.kArmInitialPosition, arm), new PIDWrist(Constants.kWristStowedAngle, wrist)));

        eventMap.put("placeCube", 
            //new ToggleManipulator(manipulator));
            new WaitCommand(5));

        eventMap.put("level3PositionCube",
                new PIDArmWrist(Constants.kArmLevel3Angle, Units.degreesToRadians(0.0), arm, wrist));
        

        RamseteAutoBuilder autoBuilder = new RamseteAutoBuilder(drivetrain::getPose, 
        drivetrain::resetOdometry, 
        new RamseteController(), 
        drivetrain.driveKinematics, 
        drivetrain::PIDVeloDrive, 
        eventMap,
        drivetrain);
        

        addCommands(new CloseManipulator(manipulator), autoBuilder.fullAuto(path));
    
        
    }

}

