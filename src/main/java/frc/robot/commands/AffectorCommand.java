package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Affector;

/**
 *
 */
public class AffectorCommand extends InstantCommand {
    private Affector m_affector;
    private double m_activate;

    public AffectorCommand(double activate, Affector subsystem) {
        m_activate = activate;
        m_affector = subsystem;
        addRequirements(m_affector);
    }

    // Called once when this command runs
    @Override
    public void initialize() {
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
