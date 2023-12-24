package frc.robot.commands.AutoCommands;

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

public class ConeHalfCube extends SequentialCommandGroup{
    
    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Arm arm;
    private Wrist wrist;
    
    public ConeHalfCube(Drivetrain drivetrain, Arm arm, Wrist wrist, Manipulator manipulator){
        this.drivetrain = drivetrain;
        this.manipulator = manipulator;
        this.arm = arm;
        this.wrist = wrist;
        this.addRequirements(drivetrain, arm, wrist, manipulator);

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup("Cone_0.5_Cube", new PathConstraints(4, 2));

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("level3Position",
            Commands.parallel(
                new PIDArmWrist(Constants.kArmLevel3Angle, Units.degreesToRadians(40), arm, wrist)));
        eventMap.put("moveWrist", 
            new PIDArmWrist(Constants.kArmLevel3Angle, Units.degreesToRadians(0.0), arm, wrist));
        eventMap.put("placeCone",
             new OpenManipulator(manipulator));
             
        eventMap.put("stow",
            Commands.parallel(new PIDArm(Constants.kArmInitialPosition, arm),
            new PIDWrist(Constants.kWristStowedAngle, wrist)));
            
        eventMap.put("groundPosition", 
            new PIDArmWrist(Constants.kArmFloorAngle, Units.degreesToRadians(0.0), arm, wrist));
            
        eventMap.put("pickup", 
            new CloseManipulator(manipulator));
            

        
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

