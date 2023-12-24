// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.RamseteAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PIDArmWrist;
import frc.robot.commands.StowCommand;
import frc.robot.commands.ArmCommands.PIDArm;
import frc.robot.commands.AutoCommands.AutoBalance;
import frc.robot.commands.AutoCommands.ConeCube;
import frc.robot.commands.AutoCommands.ConeCubeHump;
import frc.robot.commands.AutoCommands.ConeHalfCube;
import frc.robot.commands.AutoCommands.CubeAuto;
import frc.robot.commands.AutoCommands.DriveBack;
import frc.robot.commands.AutoCommands.LowPieceDrive;
import frc.robot.commands.AutoCommands.OnePieceDrive;
import frc.robot.commands.AutoCommands.OnePieceHigh;
import frc.robot.commands.AutoCommands.PIDDriveTest;
import frc.robot.commands.AutoCommands.PositionalPIDBlue;
import frc.robot.commands.DrivetrainCommands.ArcadeDrive;
import frc.robot.commands.DrivetrainCommands.PIDTurnAngle;
import frc.robot.commands.DrivetrainCommands.PIDVeloTest;
import frc.robot.commands.WristCommands.ManualWrist;
import frc.robot.commands.WristCommands.PIDWrist;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;


public class RobotContainer {

    private Drivetrain drivetrain;
    private Manipulator manipulator;
    private Arm arm;
    private Wrist wrist;

    private XboxController primary;
    private XboxController secondary;

    private Compressor compressor;

    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        drivetrain = new Drivetrain();
        drivetrain.resetEncoders();

        manipulator = new Manipulator();
        arm = new Arm();
        wrist = new Wrist(arm::armPosition, () -> {return arm.getController().getGoal().position;});
        
        compressor = new Compressor(PneumaticsModuleType.REVPH);
        compressor.enableDigital();

        primary = new XboxController(Constants.kPrimaryControllerUSB);
        secondary = new XboxController(Constants.kSecondaryControllerUSB);

        UsbCamera camera0 = CameraServer.startAutomaticCapture();
        UsbCamera camera1 = CameraServer.startAutomaticCapture();

        autoChooser = new SendableChooser<>();

        PathPlannerServer.startServer(5811);

        bradsBindings();

        //configureBindings();

        shuffleboardSetup();
    }

    private void bradsBindings() {
        drivetrain.setDefaultCommand(
            new ArcadeDrive(
                () -> -primary.getLeftY() * .5, 
                () -> -primary.getLeftX() * .5, 
                drivetrain)
        );

        arm.setDefaultCommand(Commands.runEnd(
            () -> {
                arm.setPower(-primary.getRightY() * .75);
            }, 
            () -> {}, 
            arm));


        wrist.setDefaultCommand(
            new ManualWrist(wrist, () -> {
                return (primary.getLeftTriggerAxis() - primary.getRightTriggerAxis()) * .75;
            })
        );

        new JoystickButton(primary, Button.kRightBumper.value).onTrue(Commands.runOnce(manipulator::ToggleManipulator, manipulator));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new ArcadeDrive(() -> -primary.getLeftY()-primary.getRightY()*0.5, () -> -primary.getLeftX()-primary.getRightX()*0.5, 
            drivetrain));

        /*arm.setDefaultCommand(Commands.runEnd(
            () -> arm.setPower(secondary.getRightTriggerAxis()-secondary.getLeftTriggerAxis()), 
            () -> {}, 
            arm));*/

        new Trigger(() -> {
            if(arm.isEnabled() && Math.abs(secondary.getLeftY()) > .05) {
                return true;
            } else {
                return false;
            }
        }).onTrue(Commands.runOnce(() -> {
            //I do nothing
        }, arm));

        new Trigger(() -> {
            if(wrist.isEnabled() && Math.abs(secondary.getRightY()) > .05) {
                return true;
            } else {
                return false;
            }
        }).onTrue(Commands.parallel(
            new PrintCommand("Cancel Active Wrist Command"),
            Commands.runOnce(() -> {}, wrist)
        ));
        
        arm.setDefaultCommand(Commands.runEnd(
            () -> {
                if(!arm.isEnabled()){
                    if(Math.abs(secondary.getLeftY()) > .05) {
                        if(arm.isFeedForwarding()) {
                            arm.toggleFeedForward();
                        }

                        arm.setPower(-secondary.getLeftY());
                    } else {
                        if(!arm.isFeedForwarding()) {
                            arm.toggleFeedForward();
                        }
                    }
                }
            }, 
            () -> {}, 
            arm));


        wrist.setDefaultCommand(
            Commands.parallel(
                new PrintCommand("Wrist Default Command Triggered"),
                new ManualWrist(wrist, secondary::getRightY)
            ));

        /* 
        if(secondary.getYButtonPressed()){
            if(wrist.isEnabled()){
                wrist.disable();
            }else{
                wrist.enable();
            }
        }*/

        
      //  new JoystickButton(secondary, Button.kA.value).onTrue(new PIDWrist(Constants.kWristFloorAngle, wrist));

        new JoystickButton(primary, Button.kLeftBumper.value).onTrue(Commands.runOnce(
            manipulator::ToggleManipulator, manipulator));


        //new JoystickButton(secondary, Button.kLeftBumper.value).whileTrue(new ManualWrist(wrist, () -> 1.0));
        
        //new JoystickButton(secondary, Button.kRightBumper.value).whileTrue(new ManualWrist(wrist, () -> -1.0));
        
        /*
         * Yes, I am aware that the buttons I've specified for positions really
         * don't make sense. I am not here to pick what the driver controls are.
         * I don't care, this is just a setup example.
         */

        //Command based style toggle to enable/disable PID Control of the arm
        //using secondary drivers back button
         
        /*new JoystickButton(secondary, Button.kBack.value).onTrue(
            Commands.runOnce(() -> {
                    if(arm.getCurrentCommand() instanceof PIDArm) {
                        arm.getCurrentCommand().cancel();
                        arm.disable();
                    }  
                }
            )
        );*/

        /* 
        new JoystickButton(secondary, Button.kA.value).onTrue(
            Commands.either(
                Commands.runOnce(wrist::disable, wrist),
                Commands.runOnce(wrist::enable, wrist),
                wrist::isEnabled)
        );*/

           /*    
        new JoystickButton(secondary, Button.kB.value).onTrue(
            Commands.parallel(
                Commands.runOnce(
                    () -> arm.setGoal(Constants.kArmInitialPosition), 
                    arm
                ), 
                Commands.runOnce(
                    () -> wrist.setGoal(Constants.kWristStowedAngle), 
                    wrist
                )
            )
        );*/

        new POVButton(secondary, 180).onTrue(
            new PIDArmWrist(Constants.kArmFloorAngle, 0.0, arm, wrist)
        ); 

        new JoystickButton(secondary, Button.kA.value).onTrue(
            new PIDArm(Constants.kArmInitialPosition, arm)
        );

        new POVButton(secondary, 270).onTrue(
            Commands.parallel(
                new PIDArm(Constants.kArmLevel2Angle, arm)
        ));

        new POVButton(secondary, 0).onTrue(
            new PIDArm(Constants.kArmLevel3Angle, arm)
        );

        new POVButton(secondary, 90).onTrue(
            new PIDArmWrist(Constants.kArmHumanPlayerStationAngle, Units.degreesToRadians(0.0), arm, wrist)
        ); 
    
    }

    private void shuffleboardSetup() {
        ShuffleboardTab tab = Shuffleboard.getTab("Encoder Data");

        tab.addNumber("Left Encoder Distance", drivetrain::getLeftEncoderPosition);
        tab.addNumber("Right Encoder Distance", drivetrain::getRightEncoderPosition);
        tab.addNumber("Arm Encoder Radians", arm::armPosition);
        tab.addNumber("Arm Last Set Voltage", arm::getLastSetVoltage);

        tab.addBoolean("Arm Enabled?", arm::isEnabled);
        tab.addBoolean("Arm Feed Forward State", arm::isFeedForwarding);
        tab.addNumber("Arm Feed Forward Volts", arm::getLastSetVoltage);
        tab.addNumber("wrist angle", wrist::getWristAngle);

        tab.addNumber("Wrist Motor Set Value", wrist::getMotorSet);
        tab.addBoolean("Wrist PID Enabled", wrist::isEnabled);
        tab.addBoolean("wrist atsetpoint", wrist::atSetpoint);

        tab.addBoolean("At Setpoint Left", drivetrain::pidReachedLeft);
        tab.addBoolean("At Setpoint Right", drivetrain::pidReachedRight);
        tab.addNumber("navx angle", drivetrain::getGyroAngle);
        tab.addNumber("rightspeed", drivetrain::getRightEncoderVelocity);
        tab.addNumber("leftspeec", drivetrain::getLeftEncoderVelocity);
        
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Stuff");
        autoChooser.setDefaultOption("1 Piece Low", new LowPieceDrive(drivetrain, arm, wrist, manipulator));
        //autoChooser.addOption("1 Piece Balance", new AutoBalance(drivetrain, arm, wrist, manipulator));
        autoChooser.addOption("1 Cone 1 Cube", new ConeCube(drivetrain, arm, wrist, manipulator));
        autoChooser.addOption("1 Cone 1 Cube Hump", new ConeCubeHump(drivetrain, arm, wrist, manipulator));
        autoChooser.addOption("1 Cone High", new OnePieceHigh(arm, wrist, drivetrain, manipulator));
        autoChooser.addOption("PID drive test", new PIDDriveTest(drivetrain));
        //autoChooser.addOption("turn 90 degrees", new PIDTurnAngle(90, drivetrain));
        //autoChooser.addOption("turn 180 degrees", new PIDTurnAngle(180, drivetrain));
        autoChooser.addOption("Do Nothing RIP", null);
        autoChooser.addOption("Positional PID 2 piece", new PositionalPIDBlue(drivetrain, arm, wrist, manipulator));
        autoChooser.addOption("One Cone half cube", new ConeHalfCube(drivetrain, arm, wrist, manipulator));
        //autoChooser.addOption("velocity pid", new PIDVeloTest( drivetrain));


        autoTab.add("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
