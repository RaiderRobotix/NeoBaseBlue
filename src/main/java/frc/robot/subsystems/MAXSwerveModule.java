package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.math.Conversions;



import frc.robot.Configs;
import frc.robot.Constants;

public class MAXSwerveModule {
    private SparkMax m_drivingSpark;
    private SparkMax m_turningSpark;

    private RelativeEncoder m_drivingEncoder;
    private RelativeEncoder m_turningEncoder;
    

    private CANcoder angleEncoder;

    private SparkClosedLoopController m_drivingClosedLoopController;
    private SparkClosedLoopController m_turningClosedLoopController;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwerveModule(int drivingCANId, int turningCANId, int cancoderId, double chassisAngularOffset) {
        m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getEncoder();

        m_drivingEncoder.setPosition(0);
        m_turningEncoder.setPosition(0);

        angleEncoder = new CANcoder(cancoderId);
        angleEncoder.getConfigurator().apply(Configs.MAXSwerveModule.swerveCANcoderconfig);
        
        
        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
        m_turningSpark.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        
        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    }

    

    
    public SwerveModulePosition getPosition() { 
        return new SwerveModulePosition(m_drivingEncoder.getPosition(),
        new Rotation2d(angleEncoder.getPosition().getValueAsDouble() - m_chassisAngularOffset));
    }

    

    public SwerveModuleState getState(){
        return new SwerveModuleState(Conversions.rotationsToMeters(m_drivingEncoder.getPosition(), Constants.ModuleConstants.kWheelCircumferenceMeters), Rotation2d.fromRotations(m_turningEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setAngle(desiredState);
        setSpeed(desiredState, true);
        
        /* 
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromDegrees(m_chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(angleEncoder.getPosition().getValueAsDouble()));
        
        m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        m_desiredState = desiredState;
        */
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(m_turningEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    private void setAngle(SwerveModuleState desiredState)
    {
        if(Math.abs(desiredState.speedMetersPerSecond) <= (Constants.kMaxAngularSpeed * 0.01)) 
        {
            m_turningSpark.stopMotor();
            return;

        }
        Rotation2d angle = desiredState.angle; 
       
        
        
        
        double degReference = angle.getDegrees();
     
       
        
        m_turningClosedLoopController.setReference(degReference, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop)
    {
       
        if(isOpenLoop)
        {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.kMaxSpeedMetersPerSecond;
            m_drivingSpark.set(percentOutput);
            return;
        }
 
        double velocity = desiredState.speedMetersPerSecond;
        
        
        m_drivingClosedLoopController.setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        
    }


    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }



}
