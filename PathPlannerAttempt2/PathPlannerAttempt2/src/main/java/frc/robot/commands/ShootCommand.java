package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.SuperStructure;

public class ShootCommand extends Command {
    SuperStructure structure;

    public ShootCommand(SuperStructure structure) {
        this.structure = structure;
        structure.setReleased(false);
    }

    @Override
    public void initialize() {
        structure.targetShot(true, 0);
    }
    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted){
        structure.goHome();
    }
    
    public boolean isFinished() {
        return structure.isReleased();
    }
}
