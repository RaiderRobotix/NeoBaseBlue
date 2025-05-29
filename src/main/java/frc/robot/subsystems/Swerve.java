package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Swerve extends SubsystemBase{
    private final MAXSwerveModule SwerveMod0 = new MAXSwerveModule(
        Constants.Mod0.drivermotorID0,
        Constants.Mod0.anglemotorID0,
        Constants.Mod0.offset0
    );

    private final MAXSwerveModule SwerveMod1 = new MAXSwerveModule(
        Constants.Mod1.drivermotorID1,
        Constants.Mod1.anglemotorID1,
        Constants.Mod1.offset1
    );

    private final MAXSwerveModule SwerveMod2 = new MAXSwerveModule(
        Constants.Mod2.drivermotorID2,
        Constants.Mod2.anglemotorID2,
        Constants.Mod2.offset2
    );

    private final MAXSwerveModule SwerveMod3 = new MAXSwerveModule(
        Constants.Mod3.drivermotorID3,
        Constants.Mod3.anglemotorID3,
        Constants.Mod3.offset3
    );

    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

    SwerveDriveOdometry m_Odometry = new SwerveDriveOdometry(
        Constants.kDriveKinematics, 
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        new SwerveModulePosition[] {
            SwerveMod0.getPosition(),
            SwerveMod1.getPosition(),
            SwerveMod2.getPosition(),
            SwerveMod3.getPosition()
        }
    );

    public Swerve(){

    }

    @Override
    public void periodic(){
        m_Odometry.update(
            Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
            new SwerveModulePosition[] {
                SwerveMod0.getPosition(),
                SwerveMod1.getPosition(),
                SwerveMod2.getPosition(),
                SwerveMod3.getPosition()
            }
        );
    }

    public Pose2d getPose(){
        return m_Odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose){
        m_Odometry.resetPosition(
            Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
            new SwerveModulePosition[] {
                SwerveMod0.getPosition(),
                SwerveMod1.getPosition(),
                SwerveMod2.getPosition(),
                SwerveMod3.getPosition()
            },
            pose);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
        double xSpeedDelivered = xSpeed * Constants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * Constants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * Constants.kMaxAngularSpeed;

        var swerveModuleStates = Constants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative  
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, 
                    Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, Constants.kMaxSpeedMetersPerSecond);
        SwerveMod0.setDesiredState(swerveModuleStates[0]);
        SwerveMod1.setDesiredState(swerveModuleStates[1]);
        SwerveMod2.setDesiredState(swerveModuleStates[2]);
        SwerveMod3.setDesiredState(swerveModuleStates[3]);
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getHeading() {
        return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    }

}
