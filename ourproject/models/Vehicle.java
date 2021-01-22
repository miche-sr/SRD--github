package se.oru.coordination.coordination_oru.ourproject.models;

import se.oru.coordination.coordination_oru.ourproject.algorithms.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import java.util.*;

public class Vehicle {

	public static enum Category {
		CAR, AMBULANCE
	};

	private static final double sideCar = 1;
	private static final double sideAmb = 2;
	private static final Coordinate[] fpCar = { new Coordinate(0, 0), new Coordinate(sideCar, 0), 
												new Coordinate(sideCar, 1), new Coordinate(0, 1) };
	private static final Coordinate[] fpAmb = { new Coordinate(0, 0), new Coordinate(sideAmb, 0), 
												new Coordinate(sideAmb, 1), new Coordinate(0, 1) };

	// CONSTANT
	public static double mill2sec = 0.001;

	// VARIABILI FISICHE E PROPRIE DEL VEICOLO
	private int ID = -1;
	private int priority = -1;
	private double radius = -1.0;
	private double secForSafety = -1.0; // length of trajectory to share
	private int Tc; // [ ms ]
	private double velocity = 0.0; // [ m/s ]
	private double velMax = 0.0; // [ m/s ]
	private double accMax = 0.0; // [ m/s^2 ]
	private Coordinate[] footprint;
	private double side;
	private double myDistanceToSend;

	// VARIABILI DI PERCORSO E TRAIETTORIA
	ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	private int pathIndex = 0; // index of the last pose passed
	private Pose pose;
	private Pose start;
	private Pose[] goal;
	private PoseSteering[] path;
	private double distanceTraveled = 0.0;
	private ArrayList<PoseSteering> truncatedPath = new ArrayList<PoseSteering>();
	private SpatialEnvelope se = null;
	private SpatialEnvelope wholeSe = null;
	private HashMap<Integer, Double> times = new HashMap<Integer, Double>();
	private HashMap<Integer, Double> truncateTimes = new HashMap<Integer, Double>();

	// VARIABILI PER LE SEZIONI CRITICHE
	private int criticalPoint = -1; // -1 if no critical point
	private boolean csTooClose = false;
	private int stoppingPoint = -1; // punto di fermata, a ogni ciclo: al quale mi fermo da dove sono
	private double slowingPoint = -1; // punto di frenata, unico: per fermarsi prima del p. critico
	private TreeSet<CriticalSection> cs = new TreeSet<CriticalSection>();
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();

	// DA USARE PER I VICINI
	private ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private ArrayList<Vehicle> vehicleNear = new ArrayList<Vehicle>();
	private ArrayList<TrafficLights> trafficLightsList = new ArrayList<TrafficLights>();
	private ArrayList<TrafficLights> trafficLightsNear = new ArrayList<TrafficLights>();
	private HashMap<Integer, RobotReport> mainTable;

	private ConstantAccelerationForwardModel forward;
	private BrowserVisualizationDist viz;
	private PrecedencesFounder prec = new PrecedencesFounder();

	// COSTRUTTORE
	public Vehicle(int ID, Category category, Pose start, Pose[] goal, String yamlFile) {
		this.ID = ID;
		this.pose = start;
		this.start = start;
		this.goal = goal;

		switch (category) {
			case CAR:
				this.velMax = 2;
				this.accMax = 1.0;
				this.priority = 1;
				this.Tc = 350;
				this.side = sideCar;
				this.footprint = fpCar;
				break;

			case AMBULANCE:
				this.velMax = 3.0;
				this.accMax = 2.0;
				this.priority = 2;
				this.Tc = 150;
				this.side = sideAmb;
				this.footprint = fpAmb;
				break;

			default:
				System.out.println("Unknown vehicle");
		}
		double brakingDistanceMax = Math.pow(this.velMax,2.0) / (2*this.accMax);
		this.myDistanceToSend= (2 * this.Tc * mill2sec ) * this.velMax + brakingDistanceMax + side;
		this.radius= this.myDistanceToSend;
		this.path = createWholePath(yamlFile);
		this.forward = new ConstantAccelerationForwardModel(this, 1000); // ???

	}


	/**********************************************
	 ** SET & GET PER VARIABILI FISICHE E PROPRIE **
	 ***********************************************/
	public int getID() {
		return ID;
	}

	public int getRobotID() {
		return ID;
	}

	public int getTc() {
		return Tc;
	}

	public double getRadius() {
		return radius;
	}

	public void setRadius(double radius) {
		this.radius = radius;
	}

	public double getSecForSafety() {
		return secForSafety;
	}

	public void setSecForSafety(double secForSafety) {
		this.secForSafety = secForSafety;
	}

	public int getPriority() {
		return priority;
	}

	public double getVelocity() {
		return velocity;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public double getVelMax() {
		return velMax;
	}

	public double getAccMAx() {
		return accMax;
	}

	public Coordinate[] getFootprint() {
		return footprint;
	}



	/*****************************************
	 ** SET & GET PER PERCORSO E TRAIETTORIA **
	 ******************************************/
	 public Pose getPose() {
		return pose;
	}

	public void setPose(double x, double y, double theta) {
		this.pose = new Pose(x, y, theta);
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public Pose[] getGoal() {
		return goal;
	}

	public ReedsSheppCarPlanner getMotionPlanner() {
		return rsp;
	}
	
	public PoseSteering[] createWholePath(String yamlFile) {
		if (yamlFile != null) rsp.setMap(yamlFile);
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(this.footprint);
		rsp.setStart(this.start);
		rsp.setGoals(this.goal);
		if (!rsp.plan())
			throw new Error("No path between " + this.start + " and " + this.goal);
		wholeSe = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), this.footprint);
		return rsp.getPath();
	}
	
	public void setNewWholePath(RobotReport r) {
		this.path = intersect.rePlanPath(this, r);
		wholeSe = TrajectoryEnvelope.createSpatialEnvelope(this.path, this.footprint);
	}

	public PoseSteering[] getWholePath() {
		return path;
	}

	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	public void setDistanceTraveled(double distanceTraveled) {
		this.distanceTraveled = distanceTraveled;
	}

	public SpatialEnvelope getWholeSpatialEnvelope() {
		return wholeSe;
	}

	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}

	// CALCOLO PATH TRAIETTORIA TRONCATA //
	public void setSpatialEnvelope() {
		this.truncatedPath.clear();
		this.truncateTimes.clear();
		int csEnd;

		// from cp to Pathndex //
		if (cs.size() != 0)
			csEnd = this.cs.last().getTe1End();
		else
			csEnd = -1;

		int i = 0;
		// trasmetto solo traiettoria all'interno del raggio(in tempi) e comunque sempre fino alla fine della prima sezione critica //
		//System.out.print(this.getRobotID() + "SPatial " + pathIndex+ "\n"+times );
		while ( times.get(pathIndex + i) <= secForSafety || pathIndex + i <= csEnd+1 ) {
			this.truncateTimes.put(pathIndex + i, times.get(pathIndex + i));
			
			this.truncatedPath.add(path[pathIndex + i]);
			i++;
			//System.out.print("\n "+this.getRobotID()+" path+i  " + (pathIndex+i) + "\n" + times);
			if (!times.containsKey(pathIndex + i)){
				break;// questo forse serve per l'ultimo path index?
			}
		}
		PoseSteering[] truncatedPathArray = truncatedPath.toArray(new PoseSteering[truncatedPath.size()]);
		se = TrajectoryEnvelope.createSpatialEnvelope(truncatedPathArray, footprint);
		
	}


	public void setSpatialEnvelope2(Boolean FreeAcces) {
		this.truncatedPath.clear();
		this.truncateTimes.clear();


		double dist = 0.0;
		int i = 0;
		int Maxdist = path.length-1;
		if (!FreeAcces) Maxdist = getCriticalPoint()+1;
		while (dist < this.myDistanceToSend && (pathIndex+i)<= Maxdist){
			
			dist = forward.computeDistance(path, pathIndex, pathIndex+i);
			this.truncateTimes.put(pathIndex + i, times.get(pathIndex + i));
			this.truncatedPath.add(path[pathIndex + i]);
			i++;

			if (!times.containsKey(pathIndex + i)){  
				System.out.println("R"+ID +"perchè?");
				break;}
		}
		
		PoseSteering[] truncatedPathArray = truncatedPath.toArray(new PoseSteering[truncatedPath.size()]);
		//SpatialEnvelope se2 = TrajectoryEnvelope.createSpatialEnvelope(truncatedPathArray, footprint);
		//viz.addEnvelope(se2.getPolygon(), this, "#ffffff");
		se = TrajectoryEnvelope.createSpatialEnvelope(truncatedPathArray, footprint);
	}

	public int getPathIndex() {
		return pathIndex;
	}

	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}

	public ConstantAccelerationForwardModel getForwardModel(){
		return forward;
	}
	// AGGIORNAMENTO POSIZIONE //
	public void setPathIndex(double elapsedTrackingTime) {
		State next_state = forward.updateState(this, elapsedTrackingTime);  //calcolo nuova velocità e posizione
		setDistanceTraveled(next_state.getPosition());
		setVelocity(next_state.getVelocity());
		this.pathIndex = forward.getPathIndex(this.path, next_state);

	}

	public HashMap<Integer, Double> getTimes() {
		return times;
	}

	public void setTimes() {
		times = forward.computeTs(this);

	}

	public HashMap<Integer, Double> getTruncateTimes() {
		return truncateTimes;
	}

	/**************************************
	 ** SET & GET PER LE SEZIONI CRITICHE **
	 ***************************************/
	public TreeSet<CriticalSection> getCs() {
		return cs;
	}

	// CALCOLO NUOVE SEZIONI CRITICHE //
	public void appendCs(RobotReport v2) {
		CriticalSection[] cs = intersect.findCriticalSections(this, v2);
		for (CriticalSection c : cs)
			this.cs.add(c);
	}

	public void clearCs() {
		this.cs.clear();
	}

	public int getCriticalPoint() {
		return criticalPoint;
	}

	public void setCriticalPoint(int criticalPoint) {
		this.criticalPoint = criticalPoint;
	}

	public void setCriticalPoint(CriticalSection cs) {
		if (cs.getTe1Start()== 0) this.criticalPoint = 0;
		else this.criticalPoint = cs.getTe1Start()-1;
	}

	public double getSlowingPoint() {
		return slowingPoint;
	}

	public void setSlowingPoint(double slowingPoint) {
		this.slowingPoint = slowingPoint;
	}

	public boolean isCsTooClose() {
		return csTooClose;
	}

	public void setCsTooClose(boolean csTooClose) {
		this.csTooClose = csTooClose;
	}



/*
	public void setSlowingPoint() {
		boolean stop = false;
		int i = this.criticalPoint;
		int cp = this.criticalPoint;
		
		
		if (this.criticalPoint == -1) {
			i = this.path.length;
			cp = this.path.length;
		}

		// itero per tentativi partendo da cp-1 fino a che non trovo il punto dal quale frenando riesco a fermarmi
		while (!stop && i > 0) {
			i -= 1;
			stop = forward.canStop(this, cp, i, false);
		}
		this.slowingPoint = i;
	}
*/
	public void setSlowingPointNew(){

        int cp = this.criticalPoint;
        if (cp == -1) cp = path.length-1;
        
		double distanceToCpAbsolute = forward.computeDistance(path, 0, cp);
		double distanceToCpRelative = forward.computeDistance(path, pathIndex,cp);
        // We compute the distance traveled:
        // - accelerating up to vel max from current vel (distToVelMax)
        // - decelerating up to zero vel from vel max (brakingVelMax)

		double v0 = velocity;
		double decMax = this.accMax*0.9;//new
        //double timeToVelMax = (velMax - v0)/accMax;
        //double distToVelMax = v0*timeToVelMax + accMax*Math.pow(timeToVelMax,2.0)/2;
        double brakingFromVelMax = Math.pow(velMax,2.0)/(decMax*2);

        // If sum of the two is lower than distanceToCp, than the move profile is trapezoidal.
        // If higher, than the profile is triangular, but not reaching maximum speed.
        // For triangular: accelerating distance == decelerating distance
        double braking;
        double traveledInTc = 0;
        // if (distToVelMax + brakingFromVelMax > distanceToCpRelative){	// triangular profile
        //     // braking = brak1 (from NowVel to zero) + brak2 (from velReached to NowVel)
        //     double brak1 = Math.pow(v0,2.0)/(accMax*2);
        //     double brak2 = (distanceToCpRelative - brak1)/2;
        //     braking = brak1 + brak2;
            
        //     double timeToTopVel = -v0/accMax + Math.sqrt(Math.pow(v0/accMax, 2)+2*brak2/accMax);
        //     double topVel = v0 + accMax*timeToTopVel;
        //     traveledInTc = topVel*Tc*mill2sec; //- Math.pow(Tc*mill2sec,2.0)*accMax/2;
		// }
        // else {
        	braking = brakingFromVelMax;
        	traveledInTc = velMax*Tc*mill2sec;
        //}
        this.slowingPoint = distanceToCpAbsolute-(braking+traveledInTc);
//        System.out.println("braking"+braking);
        /*
		//System.out.println("SP NEW: "+(distanceToCp - braking));
        State slowpoint = new State(distanceToCpAbsolute-(braking+traveledInTc), 0.0); //arbitrary vel, not used
        //System.out.println("SP NEW: "+forward.getPathIndex(path, slowpoint));
		this.slowingPoint = forward.getPathIndex(path, slowpoint);
		double distanceToSlow = forward.computeDistance(path, 0, this.slowingPoint);
		//System.out.println("distToSlowNew: "+distanceToSlow);
		 * 
		 */
	}

	public int getStoppingPoint() {
		return stoppingPoint;
	}

	public void setStoppingPoint() {
		this.stoppingPoint = forward.getEarliestStoppingPathIndex(this);
	}

		/********************************
	** SET & GET PER LE PRECEDENZE **
	*********************************/
	public Boolean ComputePrecedences(CriticalSection cs) {
		return prec.ComputePrecedences(cs);
	}
	

	/*******************************
	 ** SET & GET PER LISTA VICINI **
	 ********************************/
	public void setVehicleList(ArrayList<Vehicle> vehicleList) {
		this.vehicleList = vehicleList;
	}

	public void setTrafficLightsList( ArrayList<TrafficLights>  trafficLightsList) {
		this.trafficLightsList = trafficLightsList;
	}
	public ArrayList<TrafficLights> getTrafficLightsList() {
		return trafficLightsList;
	}



	
	public void setMainTable(HashMap<Integer, RobotReport> mainTable) {
		this.mainTable = mainTable;
	}
	public HashMap<Integer, RobotReport> getMainTable() {
		return mainTable;
	}Pose start3 = new Pose(75, 35, Math.PI); Pose[] goal3 = { new Pose(5, 8, Math.PI) };


	// CALCOLO QUALI SONO I ROBOT VICINI //
	public ArrayList<Vehicle> getNears() {
		this.vehicleNear.clear();
		double vhX, vhY, dist;
		double x = this.getPose().getX();
		double y = this.getPose().getY();
		for (Vehicle vh : this.vehicleList) {
			vhX = vh.getPose().getX();
			vhY = vh.getPose().getY();
			dist = Math.sqrt(Math.pow((x - vhX), 2.0) + Math.pow((y - vhY), 2.0));
			if (dist <=2*this.radius && dist > 0) {
				this.vehicleNear.add(vh);
			}
		}
		return vehicleNear;
	}
	public Boolean checkCollision(Vehicle vh) {
		
		SpatialEnvelope pose1 = TrajectoryEnvelope.createSpatialEnvelope(new PoseSteering[] { path[pathIndex] }, footprint);
		Geometry shape1 = pose1.getPolygon();

		SpatialEnvelope pose2 = TrajectoryEnvelope.createSpatialEnvelope(new PoseSteering[] { vh.getSpatialEnvelope().getPath()[0] }, vh.getFootprint());
		Geometry shape2 = pose2.getPolygon();

		if (shape1.intersects(shape2)) 
			return true;
		else
			return false;
	}

	public ArrayList<TrafficLights> getTrafficLightsNears() {
		//this.trafficLightsNear.clear();
		double sm1X, sm1Y, dist1;
		double sm2X, sm2Y, dist2;
		double x = this.getPose().getX();
		double y = this.getPose().getY();
		for (TrafficLights TL : this.trafficLightsList) {
			sm1X = TL.getSemaphore1().getX();
			sm1Y = TL.getSemaphore1().getY();
			dist1 = Math.sqrt(Math.pow((x - sm1X), 2.0) + Math.pow((y - sm1Y), 2.0));
			if (dist1 <=2*this.radius && dist1 > 0) {
				if (!this.trafficLightsNear.contains(TL)) 
					this.trafficLightsNear.add(TL);
			}
			sm2X = TL.getSemaphore2().getX();
			sm2Y = TL.getSemaphore2().getY();
			dist2 = Math.sqrt(Math.pow((x - sm2X), 2.0) + Math.pow((y - sm2Y), 2.0));
			if (dist2 <=2*this.radius && dist2 > 0) {
				if (!this.trafficLightsNear.contains(TL)) 
					this.trafficLightsNear.add(TL);	
			}


		}
		return trafficLightsNear;
	}



	public void sendNewRr() {
		HashMap<Integer,Double> TruTim = (HashMap<Integer,Double>) truncateTimes.clone();
		RobotReport rr = new RobotReport(this.ID, this.priority, this.pathIndex, 
				this.se, TruTim, this.stoppingPoint,this.isCsTooClose());
		
		mainTable.put(ID, rr);
		
	}
	

	/**************************************
	 ** SET & GET PER LA VISUALIZZAZIONE **
	 ***************************************/

	public void setVisualization(BrowserVisualizationDist viz){
		this.viz = viz;
	}

	public BrowserVisualizationDist getVisualization(){
		return this.viz;
	}

}
