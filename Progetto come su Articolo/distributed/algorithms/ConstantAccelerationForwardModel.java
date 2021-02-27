package se.oru.coordination.coordination_oru.distributed.algorithms;

import java.util.ArrayList;
import java.util.HashMap;

import javax.lang.model.util.ElementScanner6;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle.Behavior;
import se.oru.coordination.coordination_oru.distributed.algorithms.Derivative;
import se.oru.coordination.coordination_oru.distributed.models.State;

public class ConstantAccelerationForwardModel {
		
	private double maxAccel, maxVel, alpha;
	
	private int trackingPeriodInMillis = 0;
	private int controlPeriodInMillis = -1;
	private static int MAX_TX_DELAY = 0;
	
	
	
	

	public ConstantAccelerationForwardModel(Vehicle v) {
		this.maxAccel = v.getAccMAx();
		this.maxVel = v.getVelMax();
		this.alpha = v.getalpha();
		
		this.controlPeriodInMillis = v.getTc();
//		this.trackingPeriodInMillis = trackingPeriodInMillis;
	}

	public int getPathIndex(PoseSteering[] path, State state) {
		if (state == null) return -1;

		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		for (int i = 0; i < path.length-1; i++) {
			double deltaS = path[i].getPose().distanceTo(path[i+1].getPose());
			accumulatedDist += deltaS;
			if (accumulatedDist > state.getPosition()) {
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = path.length-1;
		}
		return currentPathIndex;
	}
	
	public int getPathIndex(PoseSteering[] path, double position) {
		int currentPathIndex = -1;
		double accumulatedDist = 0.0;
		for (int i = 0; i < path.length-1; i++) {
			double deltaS = path[i].getPose().distanceTo(path[i+1].getPose());
			accumulatedDist += deltaS;
			if (accumulatedDist > position) {
				currentPathIndex = i;
				break;
			}
		}
		if (currentPathIndex == -1) {
			currentPathIndex = path.length-1;
		}
		return currentPathIndex;
	}
	
	// compute either stopping point braking from next period, or the decision point
	public int getEarliestStoppingPathIndex(Vehicle v,boolean lookForward) {
		State auxState = new State(v.getDistanceTraveled(), v.getVelocity());
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		double future = 0;
		if (lookForward) future = 1+alpha;


		double lookaheadInMillis = (1+future)*(v.getTc()*Vehicle.mill2sec);
		if (lookaheadInMillis > 0 && v.getBehavior() == Behavior.moving ) {
			while (time < lookaheadInMillis) {
				integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel);
				time += deltaTime;
			}
		}
		
		// braking
		while (auxState.getVelocity() > v.getTc()*Vehicle.mill2sec*v.getAccMAx()) {
			integrateRK4(auxState, time, deltaTime, true, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		return getPathIndex(v.getWholePath(), auxState);
	}
	
	// from integrateRK4 in TrajectoryEnvelopeTrackerRK4 of Centralized CoordinationOru project: line 337
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




	public double computeDistance(PoseSteering[] path, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,path.length-1); i++)
			ret += path[i].getPose().distanceTo(path[i+1].getPose());
		return ret;
	}

	// compute estimation of future trajectory, taking saturation into account
	public  HashMap<Integer,Double> computeTs(Vehicle v) {

		HashMap<Integer,Double> times = new HashMap<Integer, Double>();

		int currentPathIndex =  v.getPathIndex();
		int cp = v.getCriticalPoint();
		double sp = v.getSlowingPoint();
		if (cp != 0 && cp == v.getPathIndex() + v.getSpatialEnvelope().getPath().length-1){
			cp = v.getWholeSpatialEnvelope().getPath().length-1;
			sp = v.setSlowingPointTimes();
		}

		double distanceToCp = computeDistance(v.getWholePath(), 0, cp);

		State state = new State(v.getDistanceTraveled(),v.getVelocity()+0.01);
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		
		times.put(currentPathIndex, time);
		
		while (true) {

			if (state.getPosition() >= distanceToCp || state.getVelocity() <= 0) break;
			if (state.getPosition() >= sp) {
				if (state.getVelocity() < deltaTime*v.getAccMAx())
					integrateRK4(state, time, deltaTime, false, deltaTime*v.getAccMAx(), 1.0, v.getAccMAx()*0.8);
				else
					integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			}
			else {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);				
			}
			time += deltaTime;
		
			//create a hashMap(PathIndex,Time) where to insert computed data
			currentPathIndex = getPathIndex(v.getWholePath(), state);
			if (!times.containsKey(currentPathIndex)) {
				times.put(currentPathIndex, time);
				
				// adding time estimation of possibly skipped path indices
				int i = 1;
				while (i > 0){
					if (!times.containsKey(currentPathIndex-i) && (currentPathIndex-i)!=0) {
						times.put(currentPathIndex-i, time-deltaTime/(2*i));
						i = i+1;
					}
					else  i = -1; //break
				}
			}
		}
		double dist = computeDistance(v.getWholePath(),v.getPathIndex(), currentPathIndex);
		while (dist <= v.getMyDistanceToSend()+1 && currentPathIndex <= v.getWholePath().length-1 ){
			if (!times.containsKey(currentPathIndex)) {
				times.put(currentPathIndex, -1.0);
				}
			currentPathIndex += 1;
			dist = computeDistance(v.getWholePath(),v.getPathIndex(), currentPathIndex);
		}

		return times;
		
	}
	


	public Boolean isInsideRadius(Vehicle v , int pathIndex) {

		double x = v.getPose().getX();
		double y = v.getPose().getY();

		double px = v.getWholePath()[pathIndex].getPose().getX();
		double py = v.getWholePath()[pathIndex].getPose().getY();
		double dist = Math.sqrt(Math.pow((x - px), 2.0) + Math.pow((y - py), 2.0));

		if (dist  < v.getRadius()) return true;
		else return false;
	}


}
