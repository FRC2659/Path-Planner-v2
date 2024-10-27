package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SuperStructure;

public class IntakeCommand extends Command {
    SuperStructure structure;

    public IntakeCommand(SuperStructure structure) {
        this.structure = structure;
    }

    @Override
    public void initialize() {
        structure.goHome();
        structure.intake();
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return !structure.isIntaking();
    }
}
