package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;
import frc.robot.Constants;

public class MAXSwerveModule {
    private SparkMax m_drivingSpark;
    private SparkMax m_turningSpark;

    private RelativeEncoder m_drivingEncoder;
    private RelativeEncoder m_turningEncoder;

    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;

    private CANcoder angleEncoder;

    private double m_chassisAngularOffset = 0;
    private double rotval = 0;

    public MAXSwerveModule(int drivingCANId, int turningCANId, int cancoderId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        angleEncoder = new CANcoder(cancoderId);
        configEncoders();

        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);
        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;

        resetToAbsolute();

    }

    private void configEncoders() {
        angleEncoder.getConfigurator().apply(Configs.MAXSwerveModule.swerveCANcoderconfig);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_drivingEncoder.setPosition(0);

        m_turningEncoder = m_turningSpark.getEncoder();

    }

    public void resetToAbsolute() {
        double absolutePosition = getCanCoder().getDegrees() - m_chassisAngularOffset;
        m_turningEncoder.setPosition(absolutePosition);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(m_drivingEncoder.getPosition(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(m_drivingEncoder.getVelocity(), getAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SmartDashboard.putNumber("Desired Angle", desiredState.angle.getDegrees());
        setAngle(desiredState);
        setSpeed(desiredState, true);
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void setAngle(SwerveModuleState desiredState) {

        if (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kMaxAngularSpeed * 0.01)) {
            m_turningSpark.stopMotor();
            return;
        }
        rotval = desiredState.angle.getRadians();

        m_turningClosedLoopController.setReference(rotval, ControlType.kPosition);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond;
            m_drivingSpark.set(percentOutput);
            return;
        }
        m_drivingClosedLoopController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    }

    public double getRotVal() {
        return rotval;
    }

}
