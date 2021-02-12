package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestCerchio {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;
	private static ArrayList<TrafficLights> trafficLightsList = new ArrayList<TrafficLights>();
	private static ArrayList<Thread> threads = new ArrayList<Thread>();

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		threads.add(thread);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;
		
		int NUMBER_ROBOTS = 90;
		double radius = 40 ;


		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(12,radius+3, radius+1);

		double theta = 0.0;
		
		for (int i = 10; i < NUMBER_ROBOTS; i++) {
			//In case deadlocks occur, we make the coordinator capable of re-planning on the fly (experimental, not working properly yet)
			
			//Place robots.
			double alpha = theta + i*Math.PI/NUMBER_ROBOTS;
			Pose startPose = new Pose(radius*Math.cos(alpha), radius*Math.sin(alpha), alpha);
			Pose[] goalPose ={ new Pose(radius*Math.cos(alpha+Math.PI), radius*Math.sin(alpha+Math.PI), alpha)};

			Thread thread = initThread(NUMBER_ROBOTS-i, c, startPose, goalPose);
			
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
			vh.setTrafficLightsList(trafficLightsList);
			vh.setVehicleList(vehicleList);
			vh.setMainTable(mainTable);
			vh.setSlowingPointNew();
			vh.setTimes();
			vh.setSpatialEnvelope2(true,0);
			vh.getNears();
			vh.sendNewRr();
			vh.setVisualization(viz);
			vh.setReplan(false);
			vh.initViz();
			vh.setFilterCs(true);
		}
		Thread.sleep(1200);
		System.out.println("\n" + "Radius "  + rMax );
		System.out.println("\n" + "Radius "  + rMax );
		int all = 0;
		for (Vehicle vh : vehicleList){
			all = all + vh.countAllcs();
		}
		System.out.println("\n" + "All CS "  + all/2 );
		
		double start = System.currentTimeMillis();
		for(Thread tr : threads){
			tr.start();
			try {
				TimeUnit.MILLISECONDS.sleep(5);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
		}
		for(Thread tr : threads){
			tr.join();}
		
		double finish = System.currentTimeMillis();
		double timeElapsed = (finish - start)/1000;
		System.out.println("\n Numero Robot  - " + NUMBER_ROBOTS );
		System.out.println("\n Time Elapsed Complessivo - " + timeElapsed + "  ovvero in minuti: "+timeElapsed/60 );
	
	}
}