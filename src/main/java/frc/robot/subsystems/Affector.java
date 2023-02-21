package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

/**
 *
 */
public class Affector extends SubsystemBase {
    private Encoder encoder;
    private PWMVictorSPX motor;

    /**
    *
    */
    public Affector() {
        encoder = new Encoder(0, 3, false, EncodingType.k4X);
        addChild("encoder", encoder);
        encoder.setDistancePerPulse(1.0);

        motor = new PWMVictorSPX(0);
        addChild("motor", motor);
        motor.setInverted(false);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}
