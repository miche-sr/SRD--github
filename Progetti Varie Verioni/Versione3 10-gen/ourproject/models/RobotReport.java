package se.oru.coordination.coordination_oru.ourproject.models;

import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

public class RobotReport {
	private int ID = -1;
	private int priority = -1;
	private int pathIndex = 0;
	private SpatialEnvelope se = null;
	private HashMap<Integer, Double> truncateTimes = new HashMap<Integer, Double>();
	private int stoppingPoint = -1;
	
	
//	public RobotReport(int ID, int priority, int pathIndex, SpatialEnvelope se, 
//			HashMap<Integer, Double> truncateTimes,	int stoppingPoint) {
//		this.ID = ID;
//		this.priority = priority;
//		this.pathIndex = pathIndex;
//		this.se = se;
//		this.truncateTimes = truncateTimes;
//		this.stoppingPoint = stoppingPoint;
//	}
//	
//	public RobotReport() {
//		// TODO Auto-generated constructor stub
//	}

	public SpatialEnvelope getSe() {
		return se;
	}
	public void setSe(SpatialEnvelope se) {
		this.se = se;
	}
	public void setID(int iD) {
		ID = iD;
	}
	public void setPriority(int priority) {
		this.priority = priority;
	}
	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}
	public void setTruncateTimes(HashMap<Integer, Double> truncateTimes) {
		this.truncateTimes = truncateTimes;
	}
	public void setStoppingPoint(int stoppingPoint) {
		this.stoppingPoint = stoppingPoint;
	}
	public int getID() {
		return ID;
	}
	public int getPriority() {
		return priority;
	}
	public int getPathIndex() {
		return pathIndex;
	}
	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}
	public HashMap<Integer, Double> getTruncateTimes() {
		return truncateTimes;
	}
	public int getStoppingPoint() {
		return stoppingPoint;
	}
	
    public static RobotReport deepcopy(int ID, int priority, int pathIndex, SpatialEnvelope se, 
			HashMap<Integer, Double> truncateTimes, int stoppingPoint) {
    	RobotReport newRr = new RobotReport();
    	newRr.ID = ID;
    	newRr.priority = priority;
    	newRr.pathIndex = pathIndex;
    	newRr.se = se;
    	newRr.truncateTimes = truncateTimes;
    	newRr.stoppingPoint = stoppingPoint;
    	return newRr;
   }
	
}
