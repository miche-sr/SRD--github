package se.oru.coordination.coordination_oru.distributed.models;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import se.oru.coordination.coordination_oru.distributed.algorithms.*;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle.Behavior;


public class Traker implements Runnable {

   
    private boolean run = true;

    private double maxAcc, maxVel;
    private double controlPeriodInMillis = 0;
	private double temporalResolution = -1;
    private SpatialEnvelope wholeSe = null;
    private PoseSteering[] path;
    private double elapsedTrackingTime = 0;

	
    // INIT //
    private Vehicle v;
    private Behavior robotBehavior = Behavior.start;
    private State state = new State(0,0);
	private int pathIndex = 0;
    private int criticalPoint = 0;
    private double slowingPoint = 0;
    private boolean FreeAccess = true;

    public Traker(Vehicle v,double VelMax, double AccMAx, int Tc, double temporalResolution,SpatialEnvelope wholeSe ) {
		this.v=v;
		this.maxVel = VelMax;	
		this.maxAcc = AccMAx;
        this.controlPeriodInMillis = Tc;
        this.temporalResolution = temporalResolution;
        this.wholeSe = wholeSe;
        this.path = wholeSe.getPath();
		
	}

    public double computeDistance(PoseSteering[] path, int startIndex, int endIndex) {
		double ret = 0.0;
		for (int i = startIndex; i < Math.min(endIndex,path.length-1); i++)
			ret += path[i].getPose().distanceTo(path[i+1].getPose());
		return ret;
	}

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

    public void updateState( double elapsedTrackingTime) {

        double distanceTraveled = state.getPosition();
        double deltaT = temporalResolution *0.001;
        double deltaTc = controlPeriodInMillis*0.001;
        
        boolean skipIntegration = false;
        
        if (pathIndex >= criticalPoint && state.getVelocity() <= 0.0 ) { //-3
            skipIntegration = true;
            robotBehavior = Behavior.stop; // robot is still
        }	
        
        if (!skipIntegration) {
 

            //velocity saturation
            if ( ((state.getVelocity() <= 1.1*deltaTc*maxAcc && robotBehavior == Behavior.slowing) 
                || robotBehavior == Behavior.minVel)
                && state.getPosition() >= slowingPoint){
                
                if (pathIndex >= path.length-2 ){
                    state.setVelocity(0.0);
                    robotBehavior = Behavior.reached;
                    state.setPosition(computeDistance(path, 0,path.length-1));
                }
                else if (pathIndex>= criticalPoint-1 && pathIndex<= criticalPoint){
                    state.setVelocity(0.0);
                    robotBehavior = Behavior.stop;
                    state.setPosition(computeDistance(path, 0, criticalPoint));
                }
                else if ( pathIndex< criticalPoint){
                    
                    robotBehavior = Behavior.minVel ;
                    integrateRK4(state, elapsedTrackingTime, deltaT, false,1.5*deltaTc*maxAcc, 1.0, maxAcc*0.8); 
                }
                else{
                    state.setVelocity(0.0);
                    robotBehavior = Behavior.stop; // still
                    state.setPosition(distanceTraveled);
                }
                

            } 

            else{
                // accelerating case - vMax
                boolean slowingDown = false;
                robotBehavior = Behavior.moving;
            
                // braking case
                if(state.getPosition() >= slowingPoint) {
                    slowingDown = true; 
                    robotBehavior = Behavior.slowing; 
                    
                }
                integrateRK4(state, elapsedTrackingTime, deltaT, slowingDown, maxVel, 1.0, maxAcc);
            }	

        } 
        if(state.getVelocity()< 0.0){
            state.setVelocity(0.0);
            state.setPosition(distanceTraveled);
        }
        if (robotBehavior == Behavior.stop && !FreeAccess)robotBehavior = Behavior.waiting;
        
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
    
    public void run()  {
        
        double start;
        double end;
        String infoCs = robotBehavior.toString();
        System.out.println("traker R" + v.getID());
        try{
            while (robotBehavior != Behavior.reached && run) {
                start = System.currentTimeMillis();

                updateState(elapsedTrackingTime);
                pathIndex = getPathIndex(path, state);
                
                infoCs = robotBehavior.toString();
                
                v.getVisualization().displayRealRobotState(wholeSe.getFootprint(), v,pathIndex,path,infoCs);
                
                end = System.currentTimeMillis();
                Thread.sleep( (long) temporalResolution); // sleep
                elapsedTrackingTime += temporalResolution*Vehicle.mill2sec;
            }
        }
        catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}	



    }

    public void getState() {
        v.setPathIndex(pathIndex);
        v.setPose(path[pathIndex].getPose());
        v.setDistanceTraveled(state.getPosition());
		v.setVelocity(state.getVelocity());
        v.setBehavior(robotBehavior);
    }


    public void setWholeSe(SpatialEnvelope wholeSe) {
        this.wholeSe = wholeSe;
    }

    public void setPath(PoseSteering[] path) {
        this.path = path;
    }


    public void setCriticalPoint(int criticalPoint) {
        this.criticalPoint = criticalPoint;
        setSlowingPoint();
        // State st = new State(slowingPoint,0);
        // v.getVisualization().displayPoint(v, getPathIndex(path, st) ,"#457d00");

        if(! v.isTrakerEnable()){
           
            updateState(elapsedTrackingTime);
            pathIndex = getPathIndex(path, state);
            String infoCs = robotBehavior.toString();
            v.getVisualization().displayRealRobotState(wholeSe.getFootprint(), v,pathIndex,path,infoCs);
            elapsedTrackingTime =elapsedTrackingTime +v.getTc();
        }

    }


    public void setSlowingPoint(){

		double distanceToCpAbsolute = computeDistance(path, 0, criticalPoint);
		double decMax = maxAcc;
        double brakingFromVelMax = Math.pow(maxVel,2.0)/(decMax*2);
        double braking;
        double traveledInTc;
    	
		braking = brakingFromVelMax;
    	traveledInTc = 2*maxVel*controlPeriodInMillis*Vehicle.mill2sec;
        
		this.slowingPoint =Math.max(0, (distanceToCpAbsolute-(braking+traveledInTc)));
	}

    public void setSlowingPointNew() {

        double distanceToCpAbsolute = computeDistance(path, 0, criticalPoint);
        double distanceToCpRelative = computeDistance(path, pathIndex,criticalPoint);
    
        double v0 = state.getVelocity();
        double decMax = maxAcc*0.9;
        double timeToVelMax = (maxVel - v0)/maxAcc;
        double distToVelMax = v0*timeToVelMax + maxAcc*Math.pow(timeToVelMax,2.0)/2;
        double brakingFromVelMax = Math.pow(maxVel,2.0)/(decMax*2);

        double braking;
        double traveledInTc = 0;
        if (distToVelMax + brakingFromVelMax > distanceToCpRelative){	// triangular profile
            // braking = brak1 (from NowVel to zero) + brak2 (from velReached to NowVel)
            double brak1 = Math.pow(v0,2.0)/(maxAcc*2);
            double brak2 = (distanceToCpRelative - brak1)/2;
            braking = brak1 + brak2;
            
            double timeToTopVel = -v0/maxAcc + Math.sqrt(Math.pow(v0/maxAcc, 2)+2*brak2/maxAcc);
            double topVel = v0 + maxAcc*timeToTopVel;
            traveledInTc = 2*topVel*controlPeriodInMillis*Vehicle.mill2sec;//- Math.pow(Tc*mill2sec,2.0)*accMax/2;
        }
        else {
            braking = brakingFromVelMax;
            traveledInTc = 2*maxVel*controlPeriodInMillis*Vehicle.mill2sec;
        }
        this.slowingPoint =Math.max(0, (distanceToCpAbsolute-(braking+traveledInTc)));
    
    }


    public void setFreeAccess(boolean freeAccess) {
        FreeAccess = freeAccess;
    }


}
