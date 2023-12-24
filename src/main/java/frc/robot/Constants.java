package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int kDrivetrainFrontLeft = 4;
    public static final int kDrivetrainRearLeft = 3;
    public static final int kDrivetrainFrontRight = 1;
    public static final int kDrivetrainRearRight = 2;
    public static final int kArm = 5;
    public static final int kWrist = 6;

    public static final int kPrimaryControllerUSB = 0;
    public static final int kSecondaryControllerUSB = 1;

    public static final int kSourceChannelWristEncoder1 = 2;
    public static final int kSourceChannelWristEncoder2 = 3;
    public static final int kSourceChannelArmEncoder1 = 0;
    public static final int kSourceChannelArmEncoder2 = 1;
    public static final int kSourceChannelArmSwitch = 6;
    public static final int kSourceChannelWristSwitch = 7;

    public static final double kDrivetrainEncoderMultiplier = Units.inchesToMeters(6 * Math.PI / 8.45);

    //THIS VALUE MUST BE IN RADIANS!
    public static final double kArmEncoderMultiplier = Math.toRadians(360.0/2048.0);
    public static final double kWristEncoderMultiplier = Math.toRadians((360.0/2048.0)*(16.0/44.0));

    public static final double kTurnAngleP = 0.058188;
    public static final double kTurnAngleI = 0;
    public static final double kTurnAngleD = 0.0010891;
    //0.0010891

    public static final double kLeftdtP = 44.524;//47.928; (without weight)
    public static final double kLeftdtI = 0;
    public static final double kLeftdtD = 3.6295;//5.5782; (without weight)

    public static final double kRightdtP = 44.524;//47.928; (without weigth)
    public static final double kRightdtI = 0;
    public static final double kRightdtD = 3.6295;//5.5782; (without weigth)

    public static final double kRightdtVeloP = 3.5152;//1.4884;  (without weight)
    public static final double kRightdtVeloI = 0;
    public static final double kRightdtVeloD = 0;

    public static final double kLeftdtVeloP = 3.5152;//1.4884;  (without weight)
    public static final double kLeftdtVeloI = 0;
    public static final double kLeftdtVeloD = 0;


    public static final double kChargeStatP = 0;
    public static final double kChargeStatI = 0;
    public static final double kChargeStatD = 0;

    public static final double kDriveFeedS = 0.18776;//0.29385;  (without weight)
    public static final double kDriveFeedV = 1.0112;//2.3949;  (without weight)
    public static final double kDriveFeedA = 0.39156;//0.73784;  (without weight)

    public static final double kDriveWheelWidth = Units.inchesToMeters(21.419);

    public static final int kClawPCMOut = 0;
    public static final int kClawPCMIn = 1;


    public static final double kArmP = 8.1409; //11.873;
    //8.1409
    public static final double kArmI = 0;
    public static final double kArmD = 0.27; //1.3071;
    //6.5677

    public static final double kArmFeedS = 0.011333; //0.15323;
    public static final double kArmFeedG = 0.25802; //0.34485;
    public static final double kArmFeedV = 5.5397; //5.0245;
    public static final double kArmFeedA = 0.83731; //0.23018;



/*  public static final double kWristFeedS = 2.2473;
    public static final double kWristFeedG = 2.236;
    public static final double kWristFeedV = 3.3042;
    public static final double kWristFeedA = 0.34277;
*/

    public static final double kWristFeedS = -1.3874;
    public static final double kWristFeedG = 3.5836;
    public static final double kWristFeedV = 4.8122;
    public static final double kWristFeedA = 0.33904; 

    //ALL POSITIONAL VALUES PROVIDED TO THE ARM MUST BE IN RADIANS!
    //The below values are (obviously) all guesses
    //kArmInitialPosition must be specified as an offset, in RADIANS, assuming the position 0
    //is the arm being parallel with the floor. See the documentation for ArmFeedforward.calculate
    //for slightly more information
    public static final double kArmInitialPosition = Math.toRadians(-84.75);
    public static final double kArmMaxVelocitySecond = Math.toRadians(118*(4.0/5.0)); //118
    public static final double kArmMaxAccelRadPerSecond2 = Math.toRadians(215*(0.7)); //215
    public static final double kArmFloorAngle = Math.toRadians(-70);
    public static final double kArmLevel1Angle = Math.toRadians(-55);
    public static final double kArmLevel2Angle = Math.toRadians(-16);
    public static final double kArmLevel3Angle = Math.toRadians(-1);
    public static final double kArmHumanPlayerStationAngle = Math.toRadians(-17.5);

    public static final double kWristMaxVelocityPerSecond = Math.toRadians(90);
    public static final double kWristMaxAccelerationPerSecond2 = Math.toRadians(78);
    public static final double kWristStowedAngle = 2.8587;
    public static final double kWristFloorAngle = 0;//Math.toRadians(60);
    public static final double kWristLevel1Angle = Math.toRadians(55);
    public static final double kWristLevel2Angle = Math.toRadians(16);
    public static final double kWristLevel3Angle = Math.toRadians(1);
    public static final double kWristHumanPlayerStationAngle = 0;

    //p = 7.3191
    //d = 5.1308
    public static final double kWristP = 8.614;
    public static final double kWristI = 0;
    public static final double kWristD = 1.0482;


    public static final double kAutoStart2StationEdge = Units.inchesToMeters(96.75-12+32);

    public static final double ktipThreshold = 15;
}
