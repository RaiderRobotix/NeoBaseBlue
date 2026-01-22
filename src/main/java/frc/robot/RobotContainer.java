package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Swerve;

public class RobotContainer {

    private final XboxController controller = new XboxController(0);
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Subsystems */
    private final Swerve s_swerve = new Swerve();
    public Vision vision;

    public RobotContainer() {
        vision = new Vision(s_swerve::addVisionMeasurement);
        configureDriverControls();

        s_swerve.setDefaultCommand(new RunCommand(
                () -> s_swerve.drive(
                        -controller.getRawAxis(translationAxis),
                        -controller.getRawAxis(strafeAxis),
                        -controller.getRawAxis(rotationAxis),
                        true),
                s_swerve));

    }

    private void configureDriverControls() {

        /* Drive Controls */
        // this code sets up driving in teleop.

        // double translationAxis = XboxController.Axis.kLeftY.value;
        // double strafeAxis = XboxController.Axis.kLeftX.value;
        // double rotationalAxis = XboxController.Axis.kRightX.value;

        // JoystickButton zeroGyro = new JoystickButton(controller,
        // XboxController.Button.kY.value);

    }

}
