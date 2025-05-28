package frc.robot.Commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

public TeleopSwerve(Swerve s_swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
    this.s_swerve = s_swerve;
    addRequirements(s_swerve);



this.translationSup = translationSup;
this.strafeSup = strafeSup;
this.rotationSup = rotationSup;
this.robotCentricSup = robotCentricSup;
}

@Override
public void execute() {

    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

    s_swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.kMaxSpeedMetersPerSecond),
        rotationVal * Constants.kMaxAngularSpeed,



        !robotCentricSup.getAsBoolean(),
        true
    );
}


}

