package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class Drivetrain extends SubsystemBase {
    private MotorController frontLeft;
    private MotorController rearLeft;
    private MotorController frontRight;
    private MotorController rearRight;

    private MotorControllerGroup left;
    private MotorControllerGroup right;

    private DifferentialDrive dt;

    // I'm only grabbing one Encoder from each side, because
    // unless we really feel the need to average both Encoders
    // on each side, one will probably be enough
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private AHRS navx;
    private double pitchVelo;
    private Timer pitchTimer;
    private double lastPitch = 0;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private SimpleMotorFeedforward driveFeed;

    private PIDController leftVeloController;
    private PIDController rightVeloController;

    public DifferentialDriveKinematics driveKinematics;

    private DifferentialDriveOdometry driveOdometry;

    private RamseteController driveRam;

    private boolean antiTipEnabled = false;

    public Drivetrain() {
        frontLeft = new CANSparkMax(Constants.kDrivetrainFrontLeft, MotorType.kBrushless);
        rearLeft = new CANSparkMax(Constants.kDrivetrainRearLeft, MotorType.kBrushless);
        frontRight = new CANSparkMax(Constants.kDrivetrainFrontRight, MotorType.kBrushless);
        rearRight = new CANSparkMax(Constants.kDrivetrainRearRight, MotorType.kBrushless);

        left = new MotorControllerGroup(frontLeft, rearLeft);
        right = new MotorControllerGroup(frontRight, rearRight);
        right.setInverted(true);

        dt = new DifferentialDrive(left, right);

        leftEncoder = ((CANSparkMax)frontLeft).getEncoder();
        leftEncoder.setPositionConversionFactor(Constants.kDrivetrainEncoderMultiplier);
        leftEncoder.setVelocityConversionFactor(Constants.kDrivetrainEncoderMultiplier/60);

        rightEncoder = ((CANSparkMax)frontRight).getEncoder();
        rightEncoder.setPositionConversionFactor(Constants.kDrivetrainEncoderMultiplier);
        rightEncoder.setVelocityConversionFactor(Constants.kDrivetrainEncoderMultiplier/60);

        navx = new AHRS(SPI.Port.kMXP);
        navx.calibrate();
        pitchTimer = new Timer();
        pitchTimer.reset();

        leftPIDController = new PIDController(Constants.kLeftdtP, Constants.kLeftdtI, Constants.kLeftdtD);
        rightPIDController = new PIDController(Constants.kRightdtP, Constants.kRightdtI, Constants.kRightdtD);
        leftPIDController.setTolerance(Units.inchesToMeters(3));
        rightPIDController.setTolerance(Units.inchesToMeters(3));

        rightVeloController = new PIDController(Constants.kRightdtVeloP, Constants.kRightdtVeloI, Constants.kRightdtVeloD);
        //rightVeloController.setTolerance(0.1);
        leftVeloController = new PIDController(Constants.kLeftdtVeloP, Constants.kLeftdtVeloI, Constants.kLeftdtVeloD);
        //leftVeloController.setTolerance(0.01);


        driveFeed = new SimpleMotorFeedforward(Constants.kDriveFeedS, Constants.kDriveFeedV, Constants.kDriveFeedA);

        driveKinematics = new DifferentialDriveKinematics(Constants.kDriveWheelWidth);

        driveOdometry = new DifferentialDriveOdometry(new Rotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition());

        driveRam = new RamseteController();

    }

    @Override
    public void periodic(){
        driveOdometry.update(navx.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition());

        pitchVelo = (lastPitch-navx.getPitch())/pitchTimer.get();
        pitchTimer.reset();
        lastPitch = navx.getPitch();
        
        if(antiTipEnabled && (navx.getPitch() > Constants.ktipThreshold || navx.getPitch() < -Constants.ktipThreshold)){
            tankDrive(Math.sin(navx.getPitch() * (Math.PI / 180.0))*-1, Math.sin(navx.getPitch() * (Math.PI / 180.0))*-1);
        }
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void resetGyro() {
        navx.reset();
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderPosition() {
        return -rightEncoder.getPosition();
    }

    public double getLeftEncoderVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getRightEncoderVelocity() {
        return -rightEncoder.getVelocity();
    }

    public double getGyroAngle() {
        return navx.getAngle();
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        dt.tankDrive(leftSpeed, rightSpeed);
    }

    public void arcadeDrive(double forward, double turn) {
        dt.arcadeDrive(forward, turn);
    }

    public void slowMode(double forward, double turn){
        dt.arcadeDrive(forward, turn);
    }

    public void pidDrive(double leftSetPoint, double rightSetPoint){
        outputVolts(MathUtil.clamp(leftPIDController.calculate(getLeftEncoderPosition(), leftSetPoint), -4, 4), 
            MathUtil.clamp( rightPIDController.calculate(getRightEncoderPosition(), rightSetPoint), -4, 4));
    }

    public boolean pidReached(){
        return rightPIDController.atSetpoint() || leftPIDController.atSetpoint();
    }

    public boolean pidReachedRight(){
        return rightPIDController.atSetpoint();
    }

    public boolean pidReachedLeft(){
        return leftPIDController.atSetpoint();
    }

    public void PIDVeloDrive(double leftVelo, double rightVelo){
        outputVolts(leftVeloController.calculate(leftEncoder.getVelocity(), leftVelo) + driveFeed.calculate(leftVelo), rightVeloController.calculate(-rightEncoder.getVelocity(), rightVelo) + driveFeed.calculate(rightVelo));
    }

    public void stop() {
        tankDrive(0, 0);
    }

    public void outputVolts(double leftVolts, double rightVolts){
        left.setVoltage(MathUtil.clamp(leftVolts, -12, 12));
        right.setVoltage(MathUtil.clamp(rightVolts, -12, 12));
        dt.feed();
        
    }
    
    public Pose2d getPose(){
        return driveOdometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), -rightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose){
        navx.reset();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        driveOdometry.resetPosition(navx.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition(), pose);
    }


    public void chargeStationBalance(){

        if(navx.getPitch()>3){
            tankDrive(0.4, 0.4);
        }else if(Math.abs(pitchVelo)>15){
            tankDrive(0, 0);
        }else if(navx.getPitch()<3){
            tankDrive(-0.4, -0.4);
        }
    }
    

    // shamelessly copied from pathplanner example code
    public Command followTrajectory(PathPlannerTrajectory traj, boolean isFirstPath){
        
        return new SequentialCommandGroup(
            new InstantCommand(() -> {
              // Reset odometry for the first path you run during auto
              if(isFirstPath){
                  this.resetOdometry(traj.getInitialPose());
              }
            }),
            new PPRamseteCommand(
                traj, 
                this::getPose, // Pose supplier
                this.driveRam,
                this.driveFeed,
                this.driveKinematics, // DifferentialDriveKinematics
                this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
                this.leftVeloController, // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                this.rightVeloController, // Right controller (usually the same values as left controller)
                this::outputVolts, // Voltage biconsumer
                true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
                this // Requires this drive subsystem
                
            )
        );
    }
}
