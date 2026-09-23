package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

public class Robot extends LoggedRobot {
    public Robot() {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }
}
