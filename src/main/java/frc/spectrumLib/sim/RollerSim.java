package frc.spectrumLib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.LinearSystem;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

/**
 * WPILib-backed simulation of a roller (flywheel) mechanism driven by a single Kraken X60 motor.
 * Updates the TalonFX sim state each robot period and animates the roller — including spin-color
 * feedback — in a {@link Mechanism2d} canvas. Implements {@link Mountable} so the roller axle can
 * follow a parent {@link Mount}.
 */
public class RollerSim implements Mountable {

    private MechanismRoot2d rollerAxle;
    private MechanismLigament2d rollerViz;

    private FlywheelSim rollerSim;
    private TalonFXSimState rollerMotorSim;
    private RollerConfig config;
    private Circle roller;

    /**
     * Creates and registers a roller simulation.
     *
     * @param config physical and display configuration for the roller
     * @param mech the Mechanism2d canvas to draw the roller on
     * @param rollerMotorSim the TalonFX sim state of the motor driving the roller
     * @param name unique name prefix used for Mechanism2d element labels
     */
    public RollerSim(RollerConfig config, Mechanism2d mech, TalonFX motor, String name) {
        this.config = config;
        this.rollerMotorSim = SimMotor.simState(motor, config.isReversedLinkage());
        DCMotor kraken = DCMotor.getKrakenX60Foc(1);
        LinearSystem<N1, N1, N1> flyWheelSystem =
                Models.flywheelFromPhysicalConstants(
                        kraken, config.getSimMOI(), config.getGearRatio());
        rollerSim = new FlywheelSim(flyWheelSystem, kraken);

        rollerAxle = mech.getRoot(name + " Axle", 0.0, 0.0);

        rollerViz =
                rollerAxle.append(
                        new MechanismLigament2d(
                                name + " Roller",
                                Units.inchesToMeters(config.getRollerDiameterInches()) / 2.0,
                                0.0,
                                5.0,
                                new Color8Bit(Color.WHITE)));

        roller =
                new Circle(
                        config.getBackgroundLines(),
                        config.getRollerDiameterInches(),
                        name,
                        rollerAxle,
                        mech);

        SimLoop.register(this::update);
    }

    /**
     * Advances the flywheel physics simulation by one robot period, updates the TalonFX rotor
     * velocity and position, moves the axle to its current mount position, and updates the
     * Mechanism2d color to reflect the roller's spin direction.
     */
    public void update(double dt) {
        rollerSim.setInput(rollerMotorSim.getMotorVoltage());
        rollerSim.update(dt);

        // FlywheelSim reports mechanism-side velocity; the rotor spins gearRatio times faster.
        double rotorRotationsPerSecond =
                rollerSim.getAngularVelocity() / (2.0 * Math.PI) * config.getGearRatio();
        rollerMotorSim.setRotorVelocity(rotorRotationsPerSecond);
        rollerMotorSim.addRotorPosition(rotorRotationsPerSecond * dt);

        if (config.isMounted()) {
            rollerAxle.setPosition(getUpdatedX(config), getUpdatedY(config));
        } else {
            rollerAxle.setPosition(config.getInitialX(), config.getInitialY());
        }

        // Scale down the angular velocity so we can actually see what is happening
        double rpm = (rollerSim.getAngularVelocity() * 60.0 / (2 * Math.PI)) / 2;
        rollerViz.setAngle(rollerViz.getAngle() + Math.toDegrees(rpm) * dt * 0.1);

        if (rollerSim.getAngularVelocity() < -1) {
            roller.setHalfBackground(config.getRevColor(), config.getOffColor());
        } else if (rollerSim.getAngularVelocity() > 1) {
            roller.setBackgroundColor(config.getFwdColor());
        } else {
            roller.setBackgroundColor(config.getOffColor());
        }
    }
}
