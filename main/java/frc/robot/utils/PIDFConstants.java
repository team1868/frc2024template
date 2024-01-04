package frc.robot.utils;

import com.ctre.phoenix6.configs.Slot0Configs;

public class PIDFConstants extends PIDConstants {
  public final double f;
  public PIDFConstants(double p, double i, double d, double f) {
    super(p, i, d);
    this.f = f;
  }

  public Slot0Configs toCTRESlot0Configuration() {
    Slot0Configs configured = new Slot0Configs();
    configured.kP = p;
    configured.kI = i;
    configured.kD = d;
    // configured.kF = f;
    return configured;
  }
}
