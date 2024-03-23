package org.first5924.frc2024.commands.leds;

import org.first5924.frc2024.subsystems.leds.Leds;

import edu.wpi.first.wpilibj2.command.Command;

public class SetLeds extends Command {
    private final Leds leds;

    public SetLeds(Leds leds) {
    this.leds = leds;
    
    //addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.GreenLEDs();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.EndLEDs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    
}
