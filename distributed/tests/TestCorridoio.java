package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;


import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestCorridoio {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static ArrayList<TrafficLights> trafficLightsList = new ArrayList<TrafficLights>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;
	
	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		viz.setInitialTransform(40, 5, 10);
		
		
		Thread.sleep(2000);

		Pose CorrStart = new Pose(9.0,5.0,0);
		Pose CorrEnd  = new Pose(25.5,5.0,0);
		TrafficLights corridoio = new TrafficLights(1, CorrStart, CorrEnd, 2, viz);
		trafficLightsList.add(corridoio);

		Thread.sleep(3000);

		/* corridioio */
		Pose start1 = new Pose(2.0,4.0,Math.PI);
		Pose goal11 = new Pose(10.0,6.0,Math.PI);
		Pose goal12 = new Pose(25.0,6.0,Math.PI);
		Pose goal13 = new Pose(35.0,4.0,Math.PI);
		Pose[] goal1 = { goal11, goal12, goal13};
		
		Pose start2 = new Pose(35.0,9.0,0);
		Pose goal21 = new Pose(25.0,6.0,0);
		Pose goal22 = new Pose(10.0,6.0,0);
		Pose goal23 = new Pose(2.0,9.0,0);
		Pose[] goal2 = { goal21, goal22, goal23};
		
		Pose start3 = new Pose(2.0,6.0,Math.PI);
		Pose goal31 = new Pose(10.0,6.0,Math.PI);
		Pose goal32 = new Pose(25.0,6.0,Math.PI);
		Pose goal33 = new Pose(35.0,6.0,Math.PI);
		Pose[] goal3 = { goal31, goal32, goal33};		

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, a, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);


	
	
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
		vh.setTrafficLightsList(trafficLightsList);
		vh.setMainTable(mainTable);
		vh.setSlowingPointNew();
		vh.setTimes();
		vh.setSpatialEnvelope2(true,0);
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
		vh.initViz();
	}
	System.out.println("\n" + "Radius "  + rMax );
	
	
	
	Thread.sleep(3000);


	thread1.start();
	thread2.start();
	thread3.start();

	}
}