package se.oru.coordination.coordination_oru.ourproject.models;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import com.vividsolutions.jts.geom.Coordinate;

public class trafficLights {
    private Pose semaphore1;
    private Pose semaphore2;
    private Boolean s1Access = true;
    private Boolean s2Access = true;
    private SpatialEnvelope corridorPath;
    private int counter = 0;
    private double size = 0.0;
    private int ID;


    ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
    private BrowserVisualizationDist viz;

    public trafficLights(Pose semaphore1, Pose semaphore2, SpatialEnvelope corridorPath) {
        this.semaphore1 = semaphore1;
        this.semaphore2 = semaphore2;
        this.corridorPath = corridorPath;

    }

    public trafficLights(int ID,Pose semaphore1, Pose semaphore2, double size,BrowserVisualizationDist viz) {
        this.ID = ID;
        this.semaphore1 = semaphore1;
        this.semaphore2 = semaphore2;
        this.size = size;
        this.viz = viz;

        Coordinate[] fp = { new Coordinate(0, 0), new Coordinate(size, 0), new Coordinate(size, size),
			new Coordinate(0, size) };
        this.createWholePath(fp);
        
        viz.addCorridor(corridorPath.getPolygon(), ID);
        viz.displayLight(semaphore1, ID, 1, s1Access);
        viz.displayLight(semaphore2, ID, 2, s2Access);
    }


    public void createWholePath(Coordinate[] fp) {
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




    
}
