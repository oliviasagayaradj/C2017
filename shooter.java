package com.frc1678.c2017.subsystems;

import com.frc1678.c2017.Constants;
import com.frc1678.c2017.loops.ILooper;
import com.frc1678.c2017.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.util.TimeDelayedBoolean;

import java.util.ArrayList;

public class Shooter extends Subsystem {
    public static double kShootingVoltage = 2.0;

    private static Shooter mInstance;

    public enum WantedAction {
        NONE, IDLE, SHOOT,
    }

    private enum State {
        NONE, SHOOTING, DONE,
    }

    private State mState = State.NONE;
    private boolean mRunningManual = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private CarriageCanifier mCanifier = CarriageCanifier.getInstance();

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case SHOOT:
            mState = State.SHOOTING;
            break;
        }
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case NONE:
            break;
        case SHOOTING:
            mPeriodicIO.shooting_voltage = kShootingVoltage;
            mPeriodicIO.forks_solenoid = true;
            mPeriodicIO.pins_solenoid = true;
            mPeriodicIO.drop_solenoid = true;
            break;
        case IDLE:
            mPeriodicIO.shooting_voltage = 0.0;
            mPeriodicIO.forks_solenoid = false;
            mPeriodicIO.pins_solenoid = false;
            break;
        }

    }

    @Override
    public synchronized void stop() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
    }

    @Override
    public void readPeriodicInputs() {
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mState = State.NONE;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Shooter.this) {
                    runStateMachine(true);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.NONE;
            }
        });
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;

        // OUTPUTS
        public double demand;
        public double shooter_voltage;
        public boolean forks_solenoid;
        public boolean pins_solenoid;
        public boolean drop_solenoid;
    }
}
