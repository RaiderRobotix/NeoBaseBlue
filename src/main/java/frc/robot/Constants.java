package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.COTSNeoSwerveConstants.driveGearRatios;
import edu.wpi.first.math.Matrix;

public class Constants {
    public static class Vision {
        public static final String kCameraName = "Aruducam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
        public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
                new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout
                .loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect
        // correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final COTSNeoSwerveConstants chosenModule = COTSNeoSwerveConstants
            .SDSMK4i(driveGearRatios.SDSMK4i_L2);
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double stickDeadband = 0.1;

    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    public static final double kWheelBase = Units.inchesToMeters(20);
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final class ModuleConstants {
        public static final int kDrivingMotorPinionTeeth = 14;

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
                / kDrivingMotorReduction;
    }

    public static final class Mod0 {

        public static final int drivermotorID0 = 10;
        public static final int anglemotorID0 = 20;
        public static final int camcoderID = 0;
        public static final double offset0 = 0;

    }

    public static final class Mod1 {

        public static final int drivermotorID1 = 11;
        public static final int anglemotorID1 = 21;
        public static final int camcoderID = 1;
        public static final double offset1 = 0;
    }

    public static final class Mod2 {

        public static final int drivermotorID2 = 12;
        public static final int anglemotorID2 = 22;
        public static final int camcoderID = 2;
        public static final double offset2 = 0;
    }

    public static final class Mod3 {

        public static final int drivermotorID3 = 13;
        public static final int anglemotorID3 = 23;
        public static final int camcoderID = 3;
        public static final double offset3 = 0;
    }

    public static final class NeoMotorConstants {
        public static final double kFreeSpeedRpm = 5676;
    }
}