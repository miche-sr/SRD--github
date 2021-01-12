package se.oru.coordination.coordination_oru.ourproject;

import se.oru.coordination.coordination_oru.ourproject.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestCorridoio {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

		/* corridioio */
		Pose start1 = new Pose(5.0,5.0,Math.PI);
		Pose goal11 = new Pose(10.0,7.0,Math.PI);
		Pose goal12 = new Pose(25.0,7.0,Math.PI);
		Pose goal13 = new Pose(30.0,5.0,Math.PI);
		Pose[] goal1 = { goal11, goal12, goal13};
		
		Pose start2 = new Pose(30.0,8.0,0);
		Pose goal21 = new Pose(25.0,6.0,0);
		Pose goal22 = new Pose(10.0,6.0,0);
		Pose goal23 = new Pose(3.0,8.0,0);
		Pose[] goal2 = { goal21, goal22, goal23};
		
				

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);


		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(25, 12, 15);
		try {
			TimeUnit.SECONDS.sleep(5);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	
	double rMax = -1; double tMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) {
			rMax = r;
			tMax = r/vh.getVelMax();
		}
	}
	for (Vehicle vh : vehicleList){
		vh.setRadius(rMax);
		vh.setSecForSafety(tMax);
		vh.setVehicleList(vehicleList);
		vh.setMainTable(mainTable);
		vh.setSlowingPointNew();
		vh.setTimes();
		vh.setSpatialEnvelope();
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
	}
	System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
	thread2.start();

	}
}