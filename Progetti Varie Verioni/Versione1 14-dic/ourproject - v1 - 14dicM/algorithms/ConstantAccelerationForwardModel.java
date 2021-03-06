package se.oru.coordination.coordination_oru.ourproject.algorithms;

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
	private int trackingPeriodInMillis = 0;
	private int controlPeriodInMillis = -1;
	private static int MAX_TX_DELAY = 0;


	public ConstantAccelerationForwardModel(Vehicle v, double temporalResolution, int trackingPeriodInMillis) {
		this.maxAccel = v.getAccMAx();
		this.maxVel = v.getVelMax();	
		this.temporalResolution = temporalResolution;
		this.controlPeriodInMillis = v.getTc();
		this.trackingPeriodInMillis = trackingPeriodInMillis;
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
		double deltaTime =v.getTc()*Vehicle.mill2sec;
		// long lookaheadInMillis = 2*(this.controlPeriodInMillis + MAX_TX_DELAY + trackingPeriodInMillis);
		// if (lookaheadInMillis > 0) {
		// 	while (time*temporalResolution < lookaheadInMillis) {	// non frenata
		// 		integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel*1.1);
		// 		time += deltaTime;
		// 	}
		// }
		while (auxState.getVelocity() > 0) {	// frenata
			integrateRK4(auxState, time, deltaTime, true, maxVel, 1.0, maxAccel*0.9);
			time += deltaTime;
			//System.out.println("<5>" + auxState.getVelocity() + " " + maxVel);
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
	public State updateState(Vehicle v, double elapsedTrackingTime) {
		State state = new State(v.getDistanceTraveled(), v.getVelocity());	// state attuale
		boolean skipIntegration = false;
		// modificata tanto tanto
		if (v.getPathIndex() == v.getCriticalPoint() ) {
			skipIntegration = true;
		}	
		if (!skipIntegration) {
			boolean slowingDown = false;
			if (v.getPathIndex() >= v.getSlowingPoint()) slowingDown = true; //brutto 
			//System.out.println("pos " + state.getPosition()+ "   velo " + state.getVelocity());
			integrateRK4(state, elapsedTrackingTime, v.getTc()*Vehicle.mill2sec, slowingDown, v.getVelMax(), 1.0, v.getAccMAx());
			
			//saturazioni velocità
			if (state.getVelocity() < 0.0) state.setVelocity(0.0);
			int cp = v.getCriticalPoint();
			if (cp == -1) cp = v.getWholePath().length;
			if (v.getPathIndex()> v.getSlowingPoint() && v.getPathIndex()< cp-1 && state.getVelocity()<0.9*v.getAccMAx())
				state.setVelocity(0.9*v.getAccMAx());
			} 
		return state;
	}
	
	
	//public double[] computeTimes() {
	//}
	
	public boolean canStop(TrajectoryEnvelope te, Vehicle v, int targetPathIndex,int StartPathIndex ,boolean useVelocity) {
		if (useVelocity && v.getVelocity() <= 0.0) return true;
		double distance = computeDistance(v.getWholePath(),StartPathIndex , targetPathIndex);
		State state = new State(0.0, v.getVelMax());
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		//long lookaheadInMillis = 2*(this.controlPeriodInMillis + MAX_TX_DELAY + trackingPeriodInMillis);
		// if (lookaheadInMillis > 0) {
		// 	while (time*this.temporalResolution < lookaheadInMillis) {
		// 		integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
		// 		time += deltaTime;
		// 	}
		// }
		//decelerate from maximum to stop
		//String Dist = String.format("%.2f", state.getPosition());
		//String Vel = String.format("%.2f", state.getVelocity());
		while (state.getVelocity() > 0) {
			if (state.getPosition() > distance) return false;
			integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			time += deltaTime;
			// Dist = String.format("%.2f", state.getPosition());
			// Vel = String.format("%.2f", state.getVelocity());
			// System.out.println(StartPathIndex + " " + targetPathIndex + " DISTANZA " + distance +" pos: " + Dist + "  Vel: " + Vel);
	
		}
		return true;
	}

	public static double computeDistance(PoseSteering[] path, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,path.length-1); i++) {
			ret += path[i].getPose().distanceTo(path[i+1].getPose());
		}
		return ret;
	}

}
