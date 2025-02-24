package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class MagicTimer {
  private double initialTime;

  public MagicTimer() {
    initialTime = -1;
  }

  public double getInitialTime() {
    return initialTime;
  }

  public double getTimePassedSec() {
    if (initialTime != -1) {
      return Timer.getFPGATimestamp() - initialTime;
    } else {
      return -1;
    }
  }

  public void startTimer() {
    if (initialTime == -1) {
      initialTime = Timer.getFPGATimestamp();
    }
  }

  public void resetTimer() {
    initialTime = -1;
  }
}
