package se.oru.coordination.coordination_oru.distributed.models;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.ArrayList;

import com.vividsolutions.jts.geom.Coordinate;

public class TrafficLights {
    private Pose semaphore1;
    private Pose semaphore2;
    private Boolean s1Access = true;
    private Boolean s2Access = true;
    private SpatialEnvelope corridorPath;
    private int counter = 0;
    private double size = 0.0;
    private int ID;
    private ArrayList<Integer> robotInsideCorridor = new ArrayList<Integer>();

    private String yamlFile = null; 
    ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
    private BrowserVisualizationDist viz;

    public TrafficLights(Pose semaphore1, Pose semaphore2, SpatialEnvelope corridorPath) {
        this.semaphore1 = semaphore1;
        this.semaphore2 = semaphore2;
        this.corridorPath = corridorPath;

    }


    

    public TrafficLights(int ID, Pose semaphore1, Pose semaphore2, double size, BrowserVisualizationDist viz) {
        this.ID = ID;
        this.semaphore1 = semaphore1;
        this.semaphore2 = semaphore2;
        this.size = size;
        this.viz = viz;

        Coordinate[] fp = { new Coordinate(0, 0), new Coordinate(0.5, 0), new Coordinate(0.5, size),
			new Coordinate(0, size) };
        this.createWholePath(fp);
        
        viz.addCorridor(corridorPath.getPolygon(), ID);
        viz.displayLight(semaphore1, ID, 1, s1Access);
        viz.displayLight(semaphore2, ID, 2, s2Access);
    }

    public TrafficLights(int ID, Pose semaphore1, Pose[] semaphore2, double size, BrowserVisualizationDist viz, String yamlFile) {
        this.ID = ID;
        this.semaphore1 = semaphore1;
        this.semaphore2 = semaphore2[semaphore2.length-1];
        this.size = size;
        this.viz = viz;
        this.yamlFile = yamlFile;

        Coordinate[] fp = { new Coordinate(0, 0), new Coordinate(1, 0), new Coordinate(1, size),
			new Coordinate(0, size) };
        this.createWholePath(fp);
        
        viz.addCorridor(corridorPath.getPolygon(), ID);
        viz.displayLight(semaphore1, ID, 1, s1Access);
        viz.displayLight(this.semaphore2, ID, 2, s2Access);
    }

    public void createWholePath(Coordinate[] fp) {
        if (yamlFile!= null) rsp.setMap(yamlFile);
        rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(fp);
		rsp.setStart(semaphore1);
		rsp.setGoals(semaphore2);
		if (!rsp.plan())
			throw new Error("No path between " + semaphore1 + " and " + semaphore2);
        corridorPath = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), fp);
	}

    public Pose getSemaphore1() {
        return semaphore1;
    }

    public SpatialEnvelope getCorridorPath() {
        return corridorPath;
    }

    public void setCorridorPath(SpatialEnvelope corridorPath) {
        this.corridorPath = corridorPath;
    }

    public Boolean getS2Access() {
        return s2Access;
    }

    public void setS2Access(Boolean s2Access) {
        this.s2Access = s2Access;
    }

    public Boolean getS1Access() {
        return s1Access;
    }

    public void setS1Access(Boolean s1Access) {
        this.s1Access = s1Access;
    }

    public Pose getSemaphore2() {
        return semaphore2;
    }

    public void setSemaphore2(Pose semaphore2) {
        this.semaphore2 = semaphore2;
    }

    public void setSemaphore1(Pose semaphore1) {
        this.semaphore1 = semaphore1;
    }

    public int getCounter() {
        return counter;
    }

    public void setCounter(int counter) {
        this.counter = counter;
    }

    public ArrayList<Integer> getRobotInsideCorridor() {
        return robotInsideCorridor;
    }

    public void setRobotInsideCorridor(ArrayList<Integer> robotInsideCorridor) {
        this.robotInsideCorridor = robotInsideCorridor;
        }

    public boolean checkSemophore(Vehicle v){
        double sm1X, sm1Y, dist1;
		double sm2X, sm2Y, dist2;
		double x = v.getPose().getX();
		double y = v.getPose().getY();
		
        sm1X = semaphore1.getX();
        sm1Y = semaphore1.getY();
        dist1 = Math.sqrt(Math.pow((x - sm1X), 2.0) + Math.pow((y - sm1Y), 2.0));

        sm2X = semaphore2.getX();
        sm2Y = semaphore2.getY();
        dist2 = Math.sqrt(Math.pow((x - sm2X), 2.0) + Math.pow((y - sm2Y), 2.0));

        if (dist1 == Math.min(dist1,dist2)) return s1Access;
        else return s2Access;
    }

    public void changeSemophoreColor(Vehicle v){
        double sm1X, sm1Y, dist1;
		double sm2X, sm2Y, dist2;
		double x = v.getPose().getX();
		double y = v.getPose().getY();
		
        sm1X = semaphore1.getX();
        sm1Y = semaphore1.getY();
        dist1 = Math.sqrt(Math.pow((x - sm1X), 2.0) + Math.pow((y - sm1Y), 2.0));

        sm2X = semaphore2.getX();
        sm2Y = semaphore2.getY();
        dist2 = Math.sqrt(Math.pow((x - sm2X), 2.0) + Math.pow((y - sm2Y), 2.0));

        if (dist1 == Math.min(dist1,dist2)) s2Access = false;
        else s1Access = false;
    }


    public void addVehicleInside(Vehicle v){
        robotInsideCorridor.add(v.getID());
        if (robotInsideCorridor.size() == 1){
            changeSemophoreColor(v);
            viz.displayLight(semaphore1, ID, 1, s1Access);
            viz.displayLight(semaphore2, ID, 2, s2Access);
        }

    }

    public void removeVehicleInside(Vehicle v){
        robotInsideCorridor.remove(Integer.valueOf(v.getID()));
        if (robotInsideCorridor.size() == 0){
            s2Access = true;
            s1Access = true;
            viz.displayLight(semaphore1, ID, 1, s1Access);
            viz.displayLight(semaphore2, ID, 2, s2Access);
        }
        
    }
}
    
