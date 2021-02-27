package se.oru.coordination.coordination_oru.distributed.models;

import se.oru.coordination.coordination_oru.distributed.algorithms.*;

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

	// CONSTANTS
	public static enum Category {
		CAR, AMBULANCE
	};
	private static final double front = 0.5;
	private static final double sideCar = front;
	private static final double sideAmb = 2 * front;
	private static final Coordinate[] fpCar = { new Coordinate(-sideCar, front), new Coordinate(sideCar, front),
			new Coordinate(sideCar, -front), new Coordinate(-sideCar, -front) };
	private static int tcCar = 350;
	private static final Coordinate[] fpAmb = { new Coordinate(-sideAmb, front), new Coordinate(sideAmb, front),
			new Coordinate(sideAmb, -front), new Coordinate(-sideAmb, -front) };
	private static int tcAmb = 150;
	public static double mill2sec = 0.001;
	public static enum Behavior {
		start, moving, slowing, minVel, stop, waiting, reached,
	};

	// DISTINCTIVE VARIABLES 
	private int ID = -1;
	private int priority = -1;
	private double radius = -1.0;
	private int Tc; // [ ms ]
	private double alpha;
	private double velocity = 0.0; // [ m/s ]
	private double velMax = 0.0; // [ m/s ]
	private double accMax = 0.0; // [ m/s^2 ]
	private Coordinate[] footprint;
	private double side;


	// VARIABLES OF CHUNK & TRAJECTORY
	private double secForSafety = -1.0; // length of trajectory to share
	private double myDistanceToSend;
	private int PoseResolution;
	private int TcTraker;

	ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
	private Behavior behavior = Behavior.start;
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

	// VARIABLES OF CRITICAL SECTIONS
	private int criticalPoint = 0;
	private boolean csTooClose = false;
	private int stoppingPoint = 0; // decision point
	private double slowingPoint = 0; // point where to slow in order to stop at critical point
	private TreeSet<CriticalSection> cs = new TreeSet<CriticalSection>();
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();
	private Traker traker;
	
	// VARIABLES USED FOR NEIGHBORHOOD COMMUNICATION
	private ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private ArrayList<Integer> vehicleNear = new ArrayList<Integer>();
	private ArrayList<TrafficLights> trafficLightsList = new ArrayList<TrafficLights>();
	private ArrayList<TrafficLights> trafficLightsNear = new ArrayList<TrafficLights>();
	private HashMap<Integer, RobotReport> mainTable;

	private ConstantAccelerationForwardModel forward;
	private BrowserVisualizationDist viz;
	private PrecedencesFounder prec = new PrecedencesFounder();
	private ArrayList<Integer> ListAllCS = new ArrayList<Integer>();
	private boolean filterCs = true;
	private boolean trakerEnable = true;


	public Vehicle(int ID, Category category, Pose start, Pose[] goal,int TcTraker, int PoseResolution,Boolean mrs, String yamlFile) {
		this.ID = ID;
		this.pose = start;
		this.start = start;
		this.goal = goal;
		this.TcTraker = TcTraker;
		this.PoseResolution = PoseResolution;


		switch (category) {
			case CAR:
				this.velMax = 1.5;//3;
				this.accMax = 0.6;//1;
				this.priority = 1;
				this.Tc = tcCar;
				this.side = sideCar;
				this.footprint = fpCar;
				break;

			case AMBULANCE:
				this.velMax = 2.5;
				this.accMax = 1.0;
				this.priority = 2;
				this.Tc = tcAmb;
				this.side = sideAmb;
				this.footprint = fpAmb;
				break;

			default:
				System.out.println("Unknown vehicle");
		}
		if(mrs)
			alpha = Math.max(tcCar, tcAmb) / this.Tc;
		else alpha = 1;
		
		double brakingDistanceMax = Math.pow(this.velMax, 2.0) / (2 * this.accMax);
		this.radius = ((2 + alpha) * this.Tc * mill2sec) * this.velMax + brakingDistanceMax + 3 * side;
		this.myDistanceToSend = this.radius;
		this.path = createWholePath(yamlFile);

		this.forward = new ConstantAccelerationForwardModel(this);

	}

	

	public void Init(double rMax, double tMax, ArrayList<Vehicle> vehicleList,
			HashMap<Integer,
								 RobotReport> mainTable,BrowserVisualizationDist viz){
		setRadius(rMax);
		setSecForSafety(tMax);
		setVehicleList(vehicleList);
		setMainTable(mainTable);
		setCriticalPoint(path.length-1);
		this.se = wholeSe;
		setTimes();
		setChunk(true,0);
		getNears();
		sendNewRr(0);
		setVisualization(viz);
		initViz();
		this.traker = new Traker(this, velMax, accMax, Tc,TcTraker, wholeSe);
		
	}

	/**********************************************
	 ** SET & GET OF DISTINCTIVE VARIABLES  **
	 ***********************************************/
	public int getID() {
		return ID;
	}

	public int getRobotID() {
		return ID;
	}

	public double getalpha(){
		return alpha;
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

	public double getMyDistanceToSend(){
		return myDistanceToSend;
	}

	public double getSide(){
		return this.side;
	}

	/************************************************
	 ** SET & GET OF CHUNK AND TRAJECTORY VARIABLES **
	 ************************************************/
	 public void setBehavior(Behavior robotBehavior){
		this.behavior=robotBehavior;
	 }
	 public Behavior getBehavior(){
		return this.behavior;
	 }

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
		rsp.setDistanceBetweenPathPoints(side/PoseResolution); 
		rsp.setFootprint(this.footprint);
		rsp.setStart(this.start);
		rsp.setGoals(this.goal);
		if (!rsp.plan())
			throw new Error("No path between " + this.start + " and " + this.goal);
		wholeSe = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), this.footprint);
		return rsp.getPath();
	}
	
	public void setNewWholePath() {
		this.path = intersect.rePlanPath(this, mainTable , getNears());
		wholeSe = TrajectoryEnvelope.createSpatialEnvelope(this.path, this.footprint);
		viz.addEnvelope(wholeSe.getPolygon(),this,"#adadad"); 
		double allCs = 0;
		for (Vehicle vh : vehicleList){
			allCs = allCs + vh.countAllcsReplan();
		}
		System.out.println("\n New number of all Cs : " + allCs/2);
		traker.setWholeSe(wholeSe);
		traker.setPath(path);
		setSpatialEnvelope2(true,0);
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

	// COMPUTE CHUNK ENVELOPE //
	public void setChunk(Boolean FreeAcces, int smStopIndex) {
		this.truncatedPath.clear();
		this.truncateTimes.clear();
		

		this.truncateTimes.put(pathIndex , times.get(pathIndex));
		this.truncatedPath.add(path[pathIndex]);
		double dist = 0.0;
		int i = 1;
		int Maxdist = path.length-1;
		if (!FreeAcces) Maxdist = smStopIndex+1;
		while (dist < this.myDistanceToSend && (pathIndex+i)<= Maxdist){
			if (!times.containsKey(pathIndex + i)) break;
			
			
			this.truncateTimes.put(pathIndex + i, times.get(pathIndex + i));
			this.truncatedPath.add(path[pathIndex + i]);
			dist = path[pathIndex].getPose().distanceTo(path[pathIndex+i].getPose());
			i++;
		}
		
		PoseSteering[] truncatedPathArray = truncatedPath.toArray(new PoseSteering[truncatedPath.size()]);
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



	public void updateState(){
		traker.getState();
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

	/**********************************************
	 ** SET & GET OF CRITICAL SECTIONS' VARIABLES **
	 **********************************************/
	public TreeSet<CriticalSection> getCs() {
		return cs;
	}

	// COMPUTE NEW CS
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
		else this.criticalPoint = cs.getTe1Start()-PoseResolution; 
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



	public double setSlowingPointTimes(){

		double distanceToCpAbsolute = forward.computeDistance(path, 0, criticalPoint);
		double distanceToCpRelative = forward.computeDistance(path, pathIndex,criticalPoint);
		double v0 = velocity;
		double decMax = this.accMax*0.9;//new
		double timeToVelMax = (velMax - v0)/accMax;
		double distToVelMax = v0*timeToVelMax + accMax*Math.pow(timeToVelMax,2.0)/2;
		double brakingFromVelMax = Math.pow(velMax,2.0)/(decMax*2);
		double braking;
		double traveledInTc = 0;
		
		if (distToVelMax + brakingFromVelMax > distanceToCpRelative){	
	
			double brak1 = Math.pow(v0,2.0)/(accMax*2);
			double brak2 = (distanceToCpRelative - brak1)/2;
			braking = brak1 + brak2;
			
			double timeToTopVel = -v0/accMax + Math.sqrt(Math.pow(v0/accMax, 2)+2*brak2/accMax);
			double topVel = v0 + accMax*timeToTopVel;
			traveledInTc = 2*topVel*Tc*mill2sec;
		}
		else {
			braking = brakingFromVelMax;
			traveledInTc = 2*velMax*Tc*mill2sec;
		}
        
		return Math.max(0, (distanceToCpAbsolute-(braking+traveledInTc)));
	}

	public int getStoppingPoint() {
		return stoppingPoint;
	}

	public void setStoppingPoint() {
		this.stoppingPoint = forward.getEarliestStoppingPathIndex(this,false);
	}
	public int getFutureStoppingPoint() {
		return forward.getEarliestStoppingPathIndex(this,true);
	}


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
	}


	public ArrayList<Integer> getNears() {
		this.vehicleNear.clear();
		double vhX, vhY, dist;
		double x = this.getPose().getX();
		double y = this.getPose().getY();
		for (Vehicle vh : this.vehicleList) {
			vhX = vh.getPose().getX();
			vhY = vh.getPose().getY();
			dist = Math.sqrt(Math.pow((x - vhX), 2.0) + Math.pow((y - vhY), 2.0));
			if (dist <=2*this.radius && dist > 0) {
				this.vehicleNear.add(vh.getID());
			}
		}
		return vehicleNear;
	}
	public Boolean checkCollision(Integer v) {
		RobotReport vh = mainTable.get(v);
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



	public void sendNewRr(int spFuture) {
		HashMap<Integer,Double> TruTim = (HashMap<Integer,Double>) truncateTimes.clone();
		//int sp = Math.min(spFuture, criticalPoint);
		RobotReport rr = new RobotReport(this.ID, this.priority,this.footprint, this.pathIndex, 
				this.se, TruTim, spFuture,this.isCsTooClose(),behavior);
		
		mainTable.put(ID, rr);
	}

	/**************************************
	 ** SET & GET FOR VISUALIZATION **
	 ***************************************/

	public void setVisualization(BrowserVisualizationDist viz){
		this.viz = viz;
	}

	public BrowserVisualizationDist getVisualization(){
		return this.viz;
	}

	public void setReplan(boolean replan){
		prec.setReplan(replan);
	}

	public void setFilterCs(boolean filterCS){
		this.filterCs = filterCS;
	}
	public boolean IsFilterCS(){
		return filterCs;
	}

	public void initViz(){
		String infoCs = behavior.toString();
		viz.displayRobotState(wholeSe.getFootprint(), this,infoCs);
		viz.addEnvelope(wholeSe.getPolygon(),this,"#adadad"); 
		//viz.addEnvelope(se.getPolygon(), this, "#000000");
	}

	public int countAllcs(){
		
		ListAllCS  = intersect.findCriticalSectionsAll(this, vehicleList);
		System.out.println("ROBOT R"+ID+" - 1° ListCS start: " + ListAllCS.size());
		return ListAllCS.size();
	}
	public int countAllcsReplan(){
		ArrayList<Integer> List = new ArrayList<Integer>();
		ArrayList<Integer> ListNew = new ArrayList<Integer>();
		for(Integer i : ListAllCS){
			if(i<pathIndex){
				List.add(i);
			}
		}
		ListNew = intersect.findCriticalSectionsAll(this, vehicleList);
		for(Integer i : ListNew){
			if(i>pathIndex){
				List.add(i);
			}
		}
		//System.out.println("ROBOT R"+ID+" - ListCS start: " + ListAllCS.size());
		ListAllCS.clear();
		ListAllCS=List;
		//System.out.println("ROBOT R"+ID+" - ListCS start: " + ListAllCS.size() +" "+List.size());
		return List.size();
	}

	public int AllCs(){
		System.out.println("ROBOT R"+ID+" - ListCS End: " + ListAllCS.size());
		return ListAllCS.size();
	}

	public void startTraker(){
		Thread thread = new Thread(traker);
		thread.start();
	}
	public Traker getTraker(){
		return traker;
	}
	public boolean isTrakerEnable() {
		return trakerEnable;
	}

	public void setTrakerEnable(boolean trakerEnable) {
		this.trakerEnable = trakerEnable;
	}
}
