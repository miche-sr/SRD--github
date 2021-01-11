package se.oru.coordination.coordination_oru.ourproject.algorithms;

import java.util.ArrayList;
import java.util.HashMap;

import javax.lang.model.util.ElementScanner6;

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
	
	public static enum Behavior {
		stop,moving,slowing,minVelocity,reached,start
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
		long lookaheadInMillis = 0*(this.controlPeriodInMillis);// + MAX_TX_DELAY + trackingPeriodInMillis);
		//avanzamento nel periodo di ritardo
		if (lookaheadInMillis > 0 && robotBehavior == Behavior.moving) {
			while (time*temporalResolution < lookaheadInMillis) {	
				integrateRK4(auxState, time, deltaTime, false, maxVel, 1.0, maxAccel*1.1);
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
	public State updateState(Vehicle v, double elapsedTrackingTime) {
		
		State state = new State(v.getDistanceTraveled(), v.getVelocity());	// state attuale
		double velMaxL = v.getVelMax();
		
		boolean skipIntegration = false;
		
		if (v.getPathIndex() >= v.getCriticalPoint() && state.getVelocity() <= 0.0 ) {
			skipIntegration = true;
			robotBehavior = Behavior.stop; // sono fermo
		}	
		if (!skipIntegration) {
 
			// // // caso Vmin solo goal
			// if ( v.getDistanceTraveled() >= v.getSlowingPoint()
			// 	&& state.getVelocity()< v.getTc()*Vehicle.mill2sec*v.getAccMAx()){
			// 	robotBehavior = Behavior.minVelocity; // vel minima
			// 	velMaxL = v.getTc()*Vehicle.mill2sec*v.getAccMAx();
			// 	slowingDown = false; 
			// 	if (v.getPathIndex()>= v.getWholePath().length-1 ){
			// 	slowingDown = true; 
			// 	}
			// }
			

			//saturazioni velocità
			if (state.getVelocity() < v.getTc()*Vehicle.mill2sec*v.getAccMAx()&& robotBehavior==Behavior.slowing){
				state.setVelocity(0.0);
				if (v.getPathIndex()>= v.getWholePath().length-2 ){
					robotBehavior = Behavior.reached;
					state.setPosition(computeDistance(v.getWholePath(), 0, v.getWholePath().length-1));
				}
				else{
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
				integrateRK4(state, elapsedTrackingTime, v.getTc()*Vehicle.mill2sec, slowingDown, velMaxL, 1.0, v.getAccMAx());
			}	

		} 
		return state;
	}
		
	/*
	public boolean canStop(Vehicle v, int targetPathIndex,int StartPathIndex ,boolean useVelocity) {
		// dati due pathIndex relativi ad un path, considerando un certo stato di partenza, 
		//calcolo se partendo dal primo è possibile fermarsi prima del secondo
		if (useVelocity && v.getVelocity() <= 0.0) return true;
		double distance = computeDistance(v.getWholePath(),StartPathIndex , targetPathIndex);
		State state = new State(0.0, v.getVelMax());
		double time = 0.0;
		double deltaTime = v.getTc()*Vehicle.mill2sec;
		
		// considero ritardi dovuti a periodo di controllo e tempo di aggiornamento del ciclo //
		long lookaheadInMillis = (this.controlPeriodInMillis + MAX_TX_DELAY + trackingPeriodInMillis);
		if (lookaheadInMillis > 0) {
			while (time*this.temporalResolution < lookaheadInMillis) {
				integrateRK4(state, time, deltaTime, false, maxVel, 1.0, maxAccel);
				time += deltaTime;
			}
		}
		//decelerate from maximum to stop

		while (state.getVelocity() > 0) {
			if (state.getPosition() > distance) return false;
			integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
			time += deltaTime;
		}
		return true;
	}
*/
	public double computeDistance(PoseSteering[] path, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,path.length-1); i++)
			ret += path[i].getPose().distanceTo(path[i+1].getPose());
		return ret;
	}

	// calcola una previsione dei tempi della traiettoria futura, considera anche le saturazioni di velocità
	public  HashMap<Integer,Double> computeTs(Vehicle v) {

		HashMap<Integer,Double> times = new HashMap<Integer, Double>();
//		HashMap<Integer,Double> times = v.getTimes();
//		times.clear();
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
				integrateRK4(state, time, deltaTime, true, maxVel, 1.0, maxAccel);
				//saturazioni velocità
//				if (state.getVelocity() < v.getTc()*Vehicle.mill2sec*v.getAccMAx()){
//					state.setVelocity(0.0);
//					state.setPosition(v.getDistanceTraveled());
//				}
				// if (state.getPosition()<  distanceToCp && state.getVelocity()< 0.9*v.getAccMAx()){
				// 	state.setVelocity(0.9*v.getAccMAx());
				// }
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
			
		//inserisco anche il pathIndex relativo al CP
		currentPathIndex  = getPathIndex(v.getWholePath(), state);
		if (!times.containsKey(currentPathIndex)) {
			times.put(currentPathIndex, time);
		}

		//inserisco anche PathIndx tra CP e fine SC inserndo come tempo -1
		if (v.getCs().size() != 0)
			csEnd = v.getCs().last().getTe1End();
		else csEnd = -1;
			currentPathIndex += 1;
		while (currentPathIndex <= csEnd+1 && currentPathIndex != v.getWholePath().length-1){
			times.put(currentPathIndex, -1.0);
			currentPathIndex += 1;
		}

		return times;
		
	}
	
	public Behavior getRobotBehavior(){
		return robotBehavior;
	}
}