package se.oru.coordination.coordination_oru.ourproject.algorithms;

import java.util.Calendar;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.ourproject.models.Vehicle;
import se.oru.coordination.coordination_oru.ourproject.algorithms.Derivative;
import se.oru.coordination.coordination_oru.ourproject.models.State;

public class ConstantAccelerationForwardModel {
		
	private double maxAccel, maxVel;
	private double temporalResolution = -1;
	private int controlPeriodInMillis = -1;
	private boolean atCP = false;
	private boolean slowingDown = false;
	private static int trackingPeriodInMillis = 0;
	private static int MAX_TX_DELAY = 0;
	private State state;
	
	public ConstantAccelerationForwardModel(Vehicle v, double temporalResolution) {
		this.maxAccel = v.getAccMAx();
		this.maxVel = v.getVelMax();	
		this.temporalResolution = temporalResolution;
		this.controlPeriodInMillis = v.getTc();
		state = new State(v.getDistanceTraveled(), v.getVelocity());
	}

	public int getPathIndex(PoseSteering[] path, State state) {
		if (state == null) return -1;
		Pose pose = null;
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		for (int i = 0; i < path.length-1; i++) {
			double deltaS = path[i].getPose().distanceTo(path[i+1].getPose());
			accumulatedDist += deltaS;
			if (accumulatedDist > state.getPosition()) {
				double ratio = 1.0-(accumulatedDist-state.getPosition())/deltaS;
				pose = path[i].getPose().interpolate(path[i+1].getPose(), ratio);
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = path.length-1;
			pose = path[currentPathIndex].getPose();
		}
		return currentPathIndex;
	}
	
	// fornisce il path index sul quale ci si fermerà date le condizioni attuali
	public int getEarliestStoppingPathIndex(Vehicle v) {
		State auxState = new State(v.getDistanceTraveled(), v.getVelocity());
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		// il periodo stesso lo considero xk ho acc di prima, al successivo già freno
		long lookaheadInMillis = (this.controlPeriodInMillis + MAX_TX_DELAY + trackingPeriodInMillis);
//		if (lookaheadInMillis > 0) {
//			while (time*temporalResolution < lookaheadInMillis) {	// non frenata
//				integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel*1.1);
//				time += deltaTime;
//			}
//		}
//		while (auxState.getVelocity() > 0) {	// frenata
//			integrateRK4(auxState, time, deltaTime, true, maxVel, 1.0, maxAccel*0.9);
//			time += deltaTime;
//		}
		while (auxState.getVelocity() > 0) {
			while (time*temporalResolution < lookaheadInMillis) {	// non frenata
				integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel*1.1);
				time += deltaTime;
			}
		// frenata
			integrateRK4(auxState, time, deltaTime, true, maxVel, 1.0, maxAccel*0.9);
			time += deltaTime;
		}
		return getPathIndex(v.getWholePath(), auxState);
	}
	
	// from integrateRK4 in TrajectoryEnvelopeTrackerRK4: line 337
	public static void integrateRK4(State state, double time, double deltaTime, boolean slowDown, double MAX_VELOCITY, double MAX_VELOCITY_DAMPENING_FACTOR, double MAX_ACCELERATION) {
			Derivative a = Derivative.evaluate(state, time, 0.0, new Derivative(), slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative b = Derivative.evaluate(state, time, deltaTime/2.0, a, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
			Derivative c = Derivative.evaluate(state, time, deltaTime/2.0, b, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR,MAX_ACCELERATION);
			Derivative d = Derivative.evaluate(state, time, deltaTime, c, slowDown, MAX_VELOCITY, MAX_VELOCITY_DAMPENING_FACTOR, MAX_ACCELERATION);
	
			double dxdt = (1.0f / 6.0f) * ( a.getVelocity() + 2.0f*(b.getVelocity() + c.getVelocity()) + d.getVelocity() ); 
		    double dvdt = (1.0f / 6.0f) * ( a.getAcceleration() + 2.0f*(b.getAcceleration() + c.getAcceleration()) + d.getAcceleration() );
			
		    state.setPosition(state.getPosition()+dxdt*deltaTime);
		    state.setVelocity(state.getVelocity()+dvdt*deltaTime);
	}
	
	//from run in TrajectoryEnvelopeTrackerRK4: line 548
	public boolean updateState(Vehicle v, double elapsedTrackingTime) {
		state.setPosition(v.getDistanceTraveled());
		state.setVelocity(v.getVelocity());		// state attuale
		boolean skipIntegration = false;
		boolean arrived = false;
		if (state.getPosition() >= v.getSlowingPoint() && state.getVelocity() <= 0.0) {
			if (v.getCriticalPoint() == -1 && !atCP) {
				//set state to final position, just in case it didn't quite get there (it's certainly close enough)
				state = new State(v.getDistanceTraveled(), 0.0);
				arrived = true;
			}
							
			//Vel < 0 hence we are at CP, thus we need to skip integration
			int pathIndex = v.getPathIndex();
			if (!atCP && pathIndex != v.getWholePath().length-1) {
				System.out.println("At critical point (" + v.getTrajectoryEnvelope().getComponent() + "): " + v.getCriticalPoint( )+ " (" + pathIndex + ")");
				if (pathIndex > v.getCriticalPoint()) 
					System.out.println("* ATTENTION! STOPPED AFTER!! *");
				atCP = true;
			}
			skipIntegration = true;
		}
			
		if (!skipIntegration) {
			if (atCP) {
				System.out.println("Resuming from critical point (" + v.getTrajectoryEnvelope().getComponent() + ")");
				atCP = false;
			}
			slowingDown = false;
			if (state.getPosition() >= v.getSlowingPoint()) slowingDown = true;
			integrateRK4(state, elapsedTrackingTime, v.getTc()*Vehicle.mill2sec, slowingDown, maxVel, 1.0, maxAccel);
			if (state.getVelocity() < 0) state.setVelocity(0.0);
		}
		
		v.setDistanceTraveled(state.getPosition());
		v.setVelocity(state.getVelocity());
		v.setPathIndex(getPathIndex(v.getWholePath(), state));
		return arrived;
	}
	
	
	

}
