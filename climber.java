package com.frc1678.c2017.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.frc1678.c2017.Constants;
import com.frc1678.c2017.loops.ILooper;
import com.frc1678.c2017.loops.Loop;
import com.frc1678.c2017.states.SuperstructureConstants;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXFactory;
import com.team254.lib.util.TimeDelayedBoolean;
import edu.wpi.first.wpilibj.Solenoid;



public class Climber extends Subsystem {
    public static double kClimberVoltage = 6.0;

    private final TalonSRX mCrawler;
    private final Solenoid mPinsSolenoid, mForksSolenoid, mDropSolenoid;

    private static Climber mInstance;

    public enum State {
        CLIMBING, NONE, IDLE, DROPPED, SETTLING,
    }

    public enum WantedAction {
        NONE, DROP, CLIMB, DONE,
    }

    private State mState = State.NONE;
    private boolean mRunningManual = false;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private CarriageCanifier mCanifier = CarriageCanifier.getInstance();

    private Climber() {
        mCrawler = new TalonSRX(Constants.kCrawlerId);
        mCrawler.configFactoryDefault();
        mPinsSolenoid = Constants.makeSolenoidForId(Constants.kPinsSolenoidId);
        mForksSolenoid = Constants.makeSolenoidForId(Constants.kForksSolenoidId);
        mDropSolenoid = Constants.makeSolenoidForId(Constants.kDropSolenoidId);
    }

    public synchronized static Climber getInstance() {
        if (mInstance == null) {
            mInstance = new Climber();
        }
        return mInstance;
    }

    public void setState(WantedAction wanted_state) {
        switch (wanted_state) {
        case NONE:
            mState = State.IDLE;
            break;
        case DROP:
            mState = State.DROPPED;
            break;
        case CRAWL:
            mState = State.CLIMBING;
            break;
        case DONE:
            mState = State.SETTLING;
            break;
        }
    }

    public void runStateMachine(boolean modifyOutputs) {
        switch (mState) {
        case NONE:
            break;
        case CLIMBING:
            mPeriodicIO.crawler_voltage = kCrawlerVoltage;
            mPeriodicIO.forks_solenoid = false;
            mPeriodicIO.pins_solenoid = false;
            mPeriodicIO.drop_solenoid = true;
            break;
        case IDLE:
            mPeriodicIO.crawler_voltage = 0.0;
            mPeriodicIO.forks_solenoid = false;
            mPeriodicIO.pins_solenoid = false;
            break;
        case DROPPED:
            mPeriodicIO.crawler_voltage = 0.0;
            mPeriodicIO.forks_solenoid = false;
            mPeriodicIO.pins_solenoid = false;
            mPeriodicIO.drop_solenoid = true;
            break;
        case SETTLING:
            mPeriodicIO.crawler_voltage = 0.0;
            mPeriodicIO.forks_solenoid = false;
            mPeriodicIO.pins_solenoid = true;
            break;
        default:
            System.out.println("Fell through on Climb states!");
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
    public void writePeriodicOutputs() {
        
        if (Elevator.getInstance().getInchesOffGround() >= SuperstructureConstants.kCrawlerHeight - 2) {
            mDropSolenoid.set(mPeriodicIO.drop_solenoid);
        } else {
            mDropSolenoid.set(false);
        }
        
        if (Elevator.getInstance().getInchesOffGround() <= 5.0) {
            mClimber.set(ControlMode.PercentOutput, mPeriodicIO.crawler_voltage / 12.0);
        } else {
            mCrawler.set(ControlMode.PercentOutput, 0.0);
        }

        mPinsSolenoid.set(mPeriodicIO.pins_solenoid);

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
                synchronized (Climber.this) {
                    runStateMachine(true);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mState = State.NONE;
            }
        });
    }

    private static class PeriodicIO {
        // OUTPUTS
        public double crawler_voltage;
        public boolean forks_solenoid;
        public boolean pins_solenoid;
        public boolean drop_solenoid;
    }
}
