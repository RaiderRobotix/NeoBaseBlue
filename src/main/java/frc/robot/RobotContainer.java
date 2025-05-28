package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
    

private final XboxController controller;
private final Joystick operator;



/*Subsystems */
private final Swerve s_swerve;


public RobotContainer() {
    controller = new XboxController(0);
    operator = new Joystick(1);

/*Subsystems */

s_swerve = new Swerve();

        // Configure the button bindings ( what a useful comment)
    configureDriverControls();
  //  configureOperatorControls();
}

private void configureDriverControls() {

        /* Drive Controls */
        // this code sets up driving in teleop.

        int translationAxis = XboxController.Axis.kLeftY.value;
        int strafeAxis = XboxController.Axis.kLeftX.value;
        int rotationalAxis = XboxController.Axis.kRightX.value;

        JoystickButton zeroGyro = new JoystickButton(controller, XboxController.Button.kY.value);



}

}
