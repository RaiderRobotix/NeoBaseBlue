package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

    private final MAXSwerveModule[] swerveMods = {
            new MAXSwerveModule(
                    Constants.Mod0.drivermotorID0,
                    Constants.Mod0.anglemotorID0,
                    Constants.Mod0.camcoderID,
                    Constants.Mod0.offset0),
            new MAXSwerveModule(Constants.Mod1.drivermotorID1,
                    Constants.Mod1.anglemotorID1,
                    Constants.Mod1.camcoderID,
                    Constants.Mod1.offset1),
            new MAXSwerveModule(Constants.Mod2.drivermotorID2,
                    Constants.Mod2.anglemotorID2,
                    Constants.Mod2.camcoderID,
                    Constants.Mod2.offset2),
            new MAXSwerveModule(Constants.Mod3.drivermotorID3,
                    Constants.Mod3.anglemotorID3,
                    Constants.Mod3.camcoderID,
                    Constants.Mod3.offset3)
    };

    public AHRS m_gyro;
    private final SwerveDrivePoseEstimator poseEstimator;

    public Swerve() {
        m_gyro = new AHRS(NavXComType.kUSB1);

        // Define the standard deviations for the pose estimator, which determine how
        // fast the pose
        // estimate converges to the vision measurement. This should depend on the
        // vision measurement
        // noise
        // and how many or how frequently vision measurements are applied to the pose
        // estimator.
        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.kDriveKinematics,
                getYaw(),
                getModulePositions(),
                new Pose2d(),
                stateStdDevs,
                visionStdDevs);
    }

    @Override
    public void periodic() {
        // Update the pose estimator with the latest gyro heading and module positions
        poseEstimator.update(getYaw(), getModulePositions());
        logToDashboard();
    }

    private void logToDashboard() {
        String table = "Drive/";
        Pose2d pose = getPose();
        SmartDashboard.putNumber(table + "X", pose.getX());
        SmartDashboard.putNumber(table + "Y", pose.getY());
        SmartDashboard.putNumber(table + "Heading", pose.getRotation().getDegrees());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed in the x direction (forward).
     * @param ySpeed        Speed in the y direction (sideways).
     * @param rot           Angular rate of rotation.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * Constants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * Constants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * Constants.kMaxAngularSpeed;

        SwerveModuleState[] swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeedDelivered,
                        ySpeedDelivered,
                        rotDelivered,
                        getHeading())
                        : new ChassisSpeeds(
                                xSpeedDelivered,
                                ySpeedDelivered,
                                rotDelivered));

        setModuleStates(swerveModuleStates);
    }

    /**
     * Get the SwerveModulePosition of each swerve module (position, angle). The
     * returned array order
     * matches the kinematics module order.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                swerveMods[0].getPosition(),
                swerveMods[1].getPosition(),
                swerveMods[2].getPosition(),
                swerveMods[3].getPosition()
        };
    }

    /**
     * Command the swerve modules to the desired states. Velocities exceeding the
     * maximum speed will
     * be desaturated (while preserving the ratios between modules).
     */
    public void setModuleStates(
            SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxSpeedMetersPerSecond);
        for (int i = 0; i < swerveMods.length; i++) {
            swerveMods[i].setDesiredState(desiredStates[i]);
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Reset the estimated pose of the swerve drive on the field.
     *
     * @param pose         New robot pose.
     * @param resetSimPose If the simulated robot pose should also be reset. This
     *                     effectively
     *                     teleports the robot and should only be used during the
     *                     setup of the simulation world.
     */
    public void resetPose(Pose2d pose, boolean resetSimPose) {
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}.
     */
    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
    }

    /**
     * See
     * {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}.
     */
    public void addVisionMeasurement(
            Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    /** Raw gyro yaw (this may not match the field heading!). */
    public Rotation2d getYaw() {
        return m_gyro.getRotation2d();
    }

    /** The heading of the swerve drive's estimated pose on the field. */
    public Rotation2d getHeading() {
        return getPose().getRotation();

    }
}
