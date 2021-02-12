package se.oru.coordination.coordination_oru.distributed.algorithms;

import java.util.ArrayList;
import java.util.HashMap;

import javax.lang.model.util.ElementScanner6;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle;
import se.oru.coordination.coordination_oru.distributed.algorithms.Derivative;
import se.oru.coordination.coordination_oru.distributed.models.State;

public class ConstantAccelerationForwardModel {
		
	private double maxAccel, maxVel;
	private double temporalResolution = -1;
	private int trackingPeriodInMillis = 0;
	private int controlPeriodInMillis = -1;
	private static int MAX_TX_DELAY = 0;
	
	public static enum Behavior {
		start,moving,slowing,minVel,stop,waiting,reached,
	};
	
	private Behavior robotBehavior = Behavior.start;

	public ConstantAccelerationForwardModel(Vehicle v, double temporalResolution) {
		this.maxAccel = v.getAccMAx();
		this.maxVel = v.getVelMax();	
		this.temporalResolution = temporalResolution;
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
	
	// fornisce il path index sul quale ci si fermerà date le condizioni attuali
	public int getEarliestStoppingPathIndex(Vehicle v) {
		State auxState = new State(v.getDistanceTraveled(), v.getVelocity());
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;

		// considero ritardi dovuti a periodo di controllo e tempo di aggiornamento del ciclo //
		long lookaheadInMillis = 1*(this.controlPeriodInMillis);// + MAX_TX_DELAY + trackingPeriodInMillis);
		//avanzamento nel periodo di ritardo
		if (lookaheadInMillis > 0 && robotBehavior == Behavior.moving) {
			while (time*temporalResolution < lookaheadInMillis) {
				integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel);
				time += deltaTime;
			}
		}
		
		//Frenata
		while (auxState.getVelocity() > v.getTc()*Vehicle.mill2sec*v.getAccMAx()) {
			integrateRK4(auxState, time, deltaTime, true, maxVel, 1.0, maxAccel);
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
		public State updateState(Vehicle v, double elapsedTrackingTime,boolean FreeAccess) {
		
			State state = new State(v.getDistanceTraveled(), v.getVelocity());	// state attuale
			double velMaxL = v.getVelMax();
			double deltaT =  v.getTc()*Vehicle.mill2sec;
			boolean skipIntegration = false;
			
			if (v.getPathIndex() >= v.getCriticalPoint() && state.getVelocity() <= 0.0 ) { //-3
				skipIntegration = true;
				robotBehavior = Behavior.stop; // sono fermo
			}	
			
			if (!skipIntegration) {
	 
	
				//saturazioni velocità
				if ( ((state.getVelocity() <= deltaT*v.getAccMAx() && robotBehavior == Behavior.slowing) 
					|| robotBehavior == Behavior.minVel)
					&& v.getDistanceTraveled() >= v.getSlowingPoint()){
					
					if (v.getPathIndex()>= v.getWholePath().length-2 ){
						state.setVelocity(0.0);
						robotBehavior = Behavior.reached;
						state.setPosition(computeDistance(v.getWholePath(), 0, v.getWholePath().length-1));
					}
					else if (v.getPathIndex()>= v.getCriticalPoint()-1 && v.getPathIndex()<= v.getCriticalPoint()){
						state.setVelocity(0.0);
						robotBehavior = Behavior.stop;
						state.setPosition(computeDistance(v.getWholePath(), 0, v.getCriticalPoint()));
					}
					else if ( v.getPathIndex()< v.getCriticalPoint()){
						robotBehavior = Behavior.minVel ;
						integrateRK4(state, elapsedTrackingTime, deltaT, false, deltaT*v.getAccMAx(), 1.0, v.getAccMAx()*0.8); 
					}
					else{
						state.setVelocity(0.0);
						robotBehavior = Behavior.stop; // fermo
						state.setPosition(v.getDistanceTraveled());
					}
					
	
				} 
	
				
				else{
					// caso accelerazione - vMax
					boolean slowingDown = false;
					robotBehavior = Behavior.moving;
				
					// caso Frenata
					if(v.getDistanceTraveled() >= v.getSlowingPoint()) {
						slowingDown = true; 
						robotBehavior = Behavior.slowing; 
						
					}
					integrateRK4(state, elapsedTrackingTime, deltaT, slowingDown, velMaxL, 1.0, v.getAccMAx());
				}	
	
			} 
			if(state.getVelocity()< 0.0){
				state.setVelocity(0.0);
				state.setPosition(v.getDistanceTraveled());
			}
			if (robotBehavior == Behavior.stop && !FreeAccess)robotBehavior = Behavior.waiting;
			return state;
		}


		

	public double computeDistance(PoseSteering[] path, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,path.length-1); i++)
			ret += path[i].getPose().distanceTo(path[i+1].getPose());
		return ret;
	}

	// calcola una previsione dei tempi della traiettoria futura, considera anche le saturazioni di velocità
	public  HashMap<Integer,Double> computeTs(Vehicle v) {

		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
		int csEnd;
		int currentPathIndex =  v.getPathIndex();
		
		double distanceToCp = computeDistance(v.getWholePath(), 0, v.getCriticalPoint());

		State state = new State(v.getDistanceTraveled(),v.getVelocity()+0.01);
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		
		times.put(currentPathIndex, time);
		
		while (true) {

			if (state.getPosition() >= distanceToCp || state.getVelocity() <= 0) break;
			if (state.getPosition() >= v.getSlowingPoint()) {
				if (state.getVelocity() < deltaTime*v.getAccMAx())
					integrateRK4(state, time, deltaTime, false, deltaTime*v.getAccMAx(), 1.0, v.getAccMAx()*0.8);
				else
					integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			}
			else {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);				
			}
			time += deltaTime;
		
			//inserisco la previsione fatta in un hashMap(PathiIndex,TIme)
			currentPathIndex = getPathIndex(v.getWholePath(), state);
			if (!times.containsKey(currentPathIndex)) {
				times.put(currentPathIndex, time);
				
				//inserisco anche gli eventuali pathIndex saltati
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
	
	public Behavior getRobotBehavior(){
		return robotBehavior;
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