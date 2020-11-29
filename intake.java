package com.frc1678.c2017.subsystems;

import com.frc1678.c2017.Constants;
import com.frc1678.c2017.loops.ILooper;
import com.frc1678.c2017.loops.Loop;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Solenoid;

import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.ReflectingCSVWriter;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.util.TimeDelayedBoolean;

import java.util.ArrayList;

public class Intake extends Subsystem {
    // Intaking is positive
    public static double kIntakeVoltage = 6.0;
    public static double kHoldingVoltage = 6.0;
    public static double kOuttakeVoltage = 6.0;

    private static Intake mInstance;

    public enum WantedAction {
        NONE, INTAKE, OUTTAKE,
    }

    private enum State {
        INTAKING, OUTTAKING, HOLDING,
    }

    private State mState = State.HOLDING;

    private TimeDelayedBoolean mLastSeenBall = new TimeDelayedBoolean();
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private final TalonSRX mMaster;

    private Intake() {
        mPopoutSolenoid = Constants.makeSolenoidForId(Constants.kIntakePopoutSolenoidId);

        mMaster = TalonSRXFactory.createDefaultTalon(Constants.kIntakeRollerId);
    }

    public synchronized static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }
    
    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    if (mRunningManual) {
                        runStateMachine(false);
                        return;
                    } else {
                        runStateMachine(true);
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                mRunningManual = false;
                mState = State.HOLDING;
            }
        }
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case INTAKING:
            if (hasBalls()) {
                mPeriodicIO.demand = kHoldingVoltage;
                mPeriodicIO.pop_out_solenoid = false;
                mState = State.HOLDING;
                break;
            }
            
            if (modifyOutputs) {
                mPeriodicIO.demand = kIntakeVoltage;
            }
            break;
        case OUTTAKING:
            if (modifyOutputs) {
                mPeriodicIO.demand = kOuttakeVoltage;
            } else if (hasBall()) {
                mState = State.HOLDING;
            }
            break;
        case HOLDING:
            if (modifyOutputs) {
                mPeriodicIO.demand = hasCargo() ? kHoldingVoltage : 0.0;
                if (hasCargo()) {
                    mPeriodicIO.pop_out_solenoid = false;
                }
            }
            break;
    }
   
   public void forceIntakeIn() {
        mPeriodicIO.pop_out_solenoid = false;
    }
    
    public void setState(WantedAction wanted_state) {
        mRunningManual = false;
        switch (wanted_state) {
        case NONE:
            mState = State.HOLDING;
            break;
        case INTAKE:
            mState = State.INTAKING;
            break;
        case OUTTAKE:
            mState = State.OUTTAKING;
            break;
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.current = mMaster.getOutputCurrent();
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
        }
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand / 12.0);
        if (Wrist.getInstance().getWantsPassThrough()) {
            forceIntakeIn();
        }
    }

    public static class PeriodicIO {
        // INPUTS
        public double timestamp;
        public double current;

        // OUTPUTS
        public double demand;
    }
}
