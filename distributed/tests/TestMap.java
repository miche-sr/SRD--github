package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestMap {

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

		/* Mappa */
		yamlFile = "maps/map-partial-2.yaml";	
		Pose start1 = new Pose(8, 20, Math.PI); Pose[] goal1 = { new Pose(75, 20, Math.PI) };
		Pose start2 = new Pose(8, 15, Math.PI); Pose[] goal2 = { new Pose(75, 10, Math.PI) };
		Pose start3 = new Pose(75, 35, Math.PI); Pose[] goal3 = { new Pose(5, 8, Math.PI) };
		Pose start4 = new Pose(75, 15, Math.PI); Pose[] goal4 = { new Pose(5, 35, Math.PI) };
		Pose start5 = new Pose(8, 10, 0); Pose[] goal5 = {new Pose(75, 15,Math.PI)};
		Pose start6 = new Pose(75, 25, -Math.PI/2); Pose[] goal6 = {new Pose(8, 20,Math.PI)};
		Pose start7 = new Pose(8, 25, 0); Pose[] goal7 = {new Pose(75, 30,Math.PI)};
		Pose start8 = new Pose(8, 35, Math.PI); Pose[] goal8 = { new Pose(75, 5, Math.PI) };

	

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		Thread thread4 = initThread(4, c, start4, goal4);
		Thread thread5 = initThread(5, a, start5, goal5);
		Thread thread6 = initThread(6, c, start6, goal6);
		Thread thread7 = initThread(7, a, start7, goal7);
		Thread thread8 = initThread(8, c, start8, goal8);

		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(20, 2, 3);
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
		vh.setSpatialEnvelope2(true,0);
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
		vh.setFilterCs(false);
	}
	System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
	thread2.start();
	thread3.start();
	thread4.start();
	thread5.start();
	thread6.start();
	thread7.start();
	thread8.start();	
	}
}