package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestBasement {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;
	private static ArrayList<TrafficLights> trafficLightsList = new ArrayList<TrafficLights>();


	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal, 50, 3, true,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {


		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;
		yamlFile = "maps/basement.yaml";
		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(25, 1, 1);
		Thread.sleep(2000);

		Pose CorrStart = new Pose(1.5,29,-Math.PI/2);
		Pose[] CorrEnd  = {new Pose(1.5,34,-2*Math.PI/3), new Pose(29,33,-Math.PI)};
		TrafficLights corridor = new TrafficLights(1, CorrStart, CorrEnd, 1.4, viz,yamlFile);
		trafficLightsList.add(corridor);
		Thread.sleep(2000);
		
		Pose start1 = new Pose(1.8, 11, -Math.PI/2); 	Pose[] goal1 = {new Pose(20, 24.5, Math.PI)};
		Pose start2 = new Pose(1.8, 13.5, -Math.PI/2); 	Pose[] goal2 = {new Pose(11, 2, Math.PI/2)};
		Pose start3 = new Pose(1.8, 15, -Math.PI/2); 	Pose[] goal3 = {new Pose(33, 33, Math.PI)};
		Pose start4 = new Pose(16, 4, -Math.PI/2); 	Pose[] goal4 = {new Pose(13.5, 28.5, -Math.PI/2)};
		Pose start5 = new Pose(24, 14, 0); 		Pose[] goal5 = {new Pose(2.5, 2, Math.PI/2)};
		Pose start6 = new Pose(34, 14, Math.PI/2); 	Pose[] goal6 = {new Pose(33, 3, Math.PI)};
		Pose start7 = new Pose(32, 3, 0); 		Pose[] goal7 = {new Pose(7, 2.5, Math.PI/2)};
		Pose start8 = new Pose(32, 6, 0); 		Pose[] goal8 = {new Pose(33, 11, -Math.PI/2) };
		Pose start9 = new Pose(34, 11.5, Math.PI/2);	Pose[] goal9 = {new Pose(11.5, 28, Math.PI)};
		//Pose start10 = new Pose(34, 15.5, Math.PI/2);		Pose[] goal10 = {new Pose(22, 19, Math.PI)};
		
		Thread thread1 = initThread(1, a, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		Thread thread4 = initThread(4, c, start4, goal4);
		Thread thread5 = initThread(5, c, start5, goal5);
		Thread thread6 = initThread(6, c, start6, goal6);
		Thread thread7 = initThread(7, a, start7, goal7);
		Thread thread8 = initThread(8, c, start8, goal8);
		Thread thread9 = initThread(9, a, start9, goal9);
		//Thread thread10 = initThread(10, c, start10, goal10);

		try {
			TimeUnit.SECONDS.sleep(1);
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
		vh.Init(rMax, tMax, vehicleList, mainTable, viz);
		vh.setFilterCs(false);
		vh.setTrafficLightsList(trafficLightsList);
	}
	System.out.println("\n" + "Radius "  + rMax );
	Thread.sleep(2000);

	System.out.println("\n" + "Radius "  + rMax );
	int all = 0;
	for (Vehicle vh : vehicleList){
		all = all + vh.countAllcs();
	}
	System.out.println("\n" + "All CS "  + all/2 );

	double start = System.currentTimeMillis();
	thread1.start();
	TimeUnit.MILLISECONDS.sleep(35);
	thread2.start();
	thread3.start();
	TimeUnit.MILLISECONDS.sleep(35);
	thread4.start();
	thread5.start();
	thread6.start();
	TimeUnit.MILLISECONDS.sleep(35);
	thread7.start();
	thread8.start();
	TimeUnit.MILLISECONDS.sleep(35);
	thread9.start();
	//thread10.start();	


	thread1.join();
	thread2.join();
	thread3.join();
	// thread4.join();
	// thread5.join();
	// thread6.join();
	// thread7.join();
	// thread8.join();
	// thread9.join();

	System.out.println("\n" + "Radius "  + rMax );
	double all2 = 0;
	for (Vehicle vh : vehicleList){
		all2 = all2 + vh.countAllcs();
	}
	System.out.println("\n" + "All CS end "  + all2/2 );
	double finish = System.currentTimeMillis();
	double timeElapsed = (finish - start)/1000;
	System.out.println("Total Time Elapsed - " + timeElapsed);

	}
}
