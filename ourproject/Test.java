package se.oru.coordination.coordination_oru.ourproject;

import se.oru.coordination.coordination_oru.ourproject.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test {

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



		/* pose libere */
		Pose start1 = new Pose(1, 12.5, Math.PI); 	Pose[] goal1 = { new Pose(16, 12.5, Math.PI) };	
		Pose start2 = new Pose(1, 2.5, Math.PI); 	Pose[] goal2 = { new Pose(16, 2.5, Math.PI) };
		Pose start3 = new Pose(1, 5, Math.PI); 		Pose[] goal3 = { new Pose(16, 5, Math.PI) };
		Pose start4 = new Pose(1, 7.5, Math.PI); 	Pose[] goal4 = { new Pose(16, 7.5, Math.PI) };
		Pose start5 = new Pose(1, 10, Math.PI); 	Pose[] goal5 = { new Pose(16, 10, Math.PI) };	

		Pose start6 = new Pose(15, 0.2, 0); 		Pose[] goal6 = { new Pose(0, 0.2, 0) };
		Pose start7 = new Pose(15, 2.7,0) ; 		Pose[] goal7 = { new Pose(0, 2.7, 0) };
		Pose start8 = new Pose(15, 5.2, 0); 		Pose[] goal8 = { new Pose(0, 5.2, 0) };
		Pose start9 = new Pose(15, 7.7, 0); 		Pose[] goal9 = { new Pose(0, 7.7, 0) };
		Pose start10 = new Pose(15, 10.2, 0); 		Pose[] goal10 = { new Pose(0, 10.2, 0) };

		Pose start11 = new Pose(12.7,0, -Math.PI/2); 	Pose[] goal11 = { new Pose(12.7,15, -Math.PI/2) };
		Pose start12 = new Pose(2.7,0,-Math.PI/2) ; 	Pose[] goal12 = { new Pose(2.7,15, -Math.PI/2) };
		Pose start13 = new Pose(5.2, 0, -Math.PI/2); 	Pose[] goal13 = { new Pose(5.2,15, -Math.PI/2) };
		Pose start14 = new Pose(7.7, 0, -Math.PI/2); 	Pose[] goal14 = { new Pose(7.7,15, -Math.PI/2) };
		Pose start15 = new Pose(10.2,0, -Math.PI/2); 	Pose[] goal15 = { new Pose(10.2,15, -Math.PI/2) };
		
		Pose start16 = new Pose(12.5,14, Math.PI/2); 	Pose[] goal16 = { new Pose(12.5,-1, Math.PI/2) };
		Pose start17 = new Pose(2.5, 14, Math.PI/2) ; 	Pose[] goal17 = { new Pose(2.5,-1,  Math.PI/2) };
		Pose start18 = new Pose(5, 14, Math.PI/2); 		Pose[] goal18 = { new Pose(5,-1,  Math.PI/2) };
		Pose start19 = new Pose(7.5, 14, Math.PI/2); 	Pose[] goal19 = { new Pose(7.5,-1,   Math.PI/2) };
		Pose start20 = new Pose(10,14, Math.PI/2); 		Pose[] goal20 = { new Pose(10,-1, Math.PI/2) };

		Pose start100 = new Pose(21, 12.5, Math.PI); 	Pose[] goal100 = { new Pose(36, 12.5, Math.PI) };	
		Pose start200 = new Pose(21, 2.5, Math.PI); 	Pose[] goal200 = { new Pose(36, 2.5, Math.PI) };
		Pose start300 = new Pose(21, 5, Math.PI); 		Pose[] goal300 = { new Pose(36, 5, Math.PI) };
		Pose start400 = new Pose(21, 7.5, Math.PI); 	Pose[] goal400 = { new Pose(36, 7.5, Math.PI) };
		Pose start500 = new Pose(21, 10, Math.PI); 	Pose[] goal500 = { new Pose(36, 10, Math.PI) };	

		Pose start600 = new Pose(35, 0.2, 0); 		Pose[] goal600 = { new Pose(20, 0.2, 0) };
		Pose start700 = new Pose(35, 2.7,0) ; 		Pose[] goal700 = { new Pose(20, 2.7, 0) };
		Pose start800 = new Pose(35, 5.2, 0); 		Pose[] goal800 = { new Pose(20, 5.2, 0) };
		Pose start900 = new Pose(35, 7.7, 0); 		Pose[] goal900 = { new Pose(20, 7.7, 0) };
		Pose start101 = new Pose(35, 10.2, 0); 		Pose[] goal101 = { new Pose(20, 10.2, 0) };

		Pose start110 = new Pose(32.7,0, -Math.PI/2); 	Pose[] goal110 = { new Pose(32.7,15, -Math.PI/2) };
		Pose start120 = new Pose(22.7,0,-Math.PI/2) ; 	Pose[] goal120 = { new Pose(22.7,15, -Math.PI/2) };
		Pose start130 = new Pose(25.2, 0, -Math.PI/2); 	Pose[] goal130 = { new Pose(25.2,15, -Math.PI/2) };
		Pose start140 = new Pose(27.7, 0, -Math.PI/2); 	Pose[] goal140 = { new Pose(27.7,15, -Math.PI/2) };
		Pose start150 = new Pose(30.2,0, -Math.PI/2); 	Pose[] goal150 = { new Pose(30.2,15, -Math.PI/2) };
		
		Pose start160 = new Pose(32.5,14, Math.PI/2); 	Pose[] goal160 = { new Pose(32.5,-1, Math.PI/2) };
		Pose start170 = new Pose(22.5, 14, Math.PI/2) ; 	Pose[] goal170 = { new Pose(22.5,-1,  Math.PI/2) };
		Pose start180 = new Pose(25, 14, Math.PI/2); 		Pose[] goal180 = { new Pose(25,-1,  Math.PI/2) };
		Pose start190 = new Pose(27.5, 14, Math.PI/2); 	Pose[] goal190 = { new Pose(27.5,-1,   Math.PI/2) };
		Pose start201 = new Pose(30,14, Math.PI/2); 		Pose[] goal201 = { new Pose(30,-1, Math.PI/2) };

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		Thread thread4 = initThread(4, c, start4, goal4);
		Thread thread5 = initThread(5, c, start5, goal5);
		Thread thread6 = initThread(6, c, start6, goal6);
		Thread thread7 = initThread(7, c, start7, goal7);
		Thread thread8 = initThread(8, c, start8, goal8);
		Thread thread9 = initThread(9, c, start9, goal9);
		Thread thread10 = initThread(10, c, start10, goal10);
		Thread thread11 = initThread(11, c, start11, goal11);
		Thread thread12 = initThread(12, c, start12, goal12);
		Thread thread13 = initThread(13, c, start13, goal13);
		Thread thread14 = initThread(14, c, start14, goal14);
		Thread thread15 = initThread(15, c, start15, goal15);
		Thread thread16 = initThread(16, c, start16, goal16);
		Thread thread17 = initThread(17, c, start17, goal17);
		Thread thread18 = initThread(18, c, start18, goal18);
		Thread thread19 = initThread(19, c, start19, goal19);
		Thread thread20 = initThread(20, c, start20, goal20);
		
		Thread thread100 = initThread(100, c, start100, goal100);
		Thread thread200 = initThread(200, c, start200, goal200);
		Thread thread300 = initThread(300, c, start300, goal300);
		Thread thread400 = initThread(400, c, start400, goal400);
		Thread thread500 = initThread(500, c, start500, goal500);
		Thread thread600 = initThread(600, c, start600, goal600);
		Thread thread700 = initThread(700, c, start700, goal700);
		Thread thread800 = initThread(800, c, start800, goal800);
		Thread thread900 = initThread(900, c, start900, goal900);
		Thread thread101 = initThread(101, c, start101, goal101);
		Thread thread110 = initThread(110, c, start110, goal110);
		Thread thread120 = initThread(120, c, start120, goal120);
		Thread thread130 = initThread(130, c, start130, goal130);
		Thread thread140 = initThread(140, c, start140, goal140);
		Thread thread150 = initThread(150, c, start150, goal150);
		Thread thread160 = initThread(160, c, start160, goal160);
		Thread thread170 = initThread(170, c, start170, goal170);
		Thread thread180 = initThread(180, c, start180, goal180);
		Thread thread190 = initThread(190, c, start190, goal190);
		Thread thread201 = initThread(201, c, start201, goal201);


		Pose start122 = new Pose(1, 32.5, Math.PI); 	Pose[] goal122 = { new Pose(16, 32.5, Math.PI) };	
		Pose start222 = new Pose(1, 22.5, Math.PI); 	Pose[] goal222 = { new Pose(16, 22.5, Math.PI) };
		Pose start322 = new Pose(1, 25, Math.PI); 		Pose[] goal322 = { new Pose(16, 25, Math.PI) };
		Pose start422 = new Pose(1, 27.5, Math.PI); 	Pose[] goal422 = { new Pose(16, 27.5, Math.PI) };
		Pose start522 = new Pose(1, 30, Math.PI); 	Pose[] goal522 = { new Pose(16, 30, Math.PI) };	
		Pose start622 = new Pose(15, 20.2, 0); 		Pose[] goal622 = { new Pose(0, 20.2, 0) };
		Pose start722 = new Pose(15, 22.7,0) ; 		Pose[] goal722 = { new Pose(0, 22.7, 0) };
		Pose start822 = new Pose(15, 25.2, 0); 		Pose[] goal822 = { new Pose(0, 25.2, 0) };
		Pose start922 = new Pose(15, 27.7, 0); 		Pose[] goal922 = { new Pose(0, 27.7, 0) };
		Pose start102 = new Pose(15, 30.2, 0); 		Pose[] goal102 = { new Pose(0, 30.2, 0) };

		Pose start112 = new Pose(12.7,20, -Math.PI/2); 	Pose[] goal112 = { new Pose(12.7,35, -Math.PI/2) };
		Pose start123 = new Pose(2.7,20,-Math.PI/2) ; 	Pose[] goal123 = { new Pose(2.7,35, -Math.PI/2) };
		Pose start132 = new Pose(5.2, 20, -Math.PI/2); 	Pose[] goal132 = { new Pose(5.2,35, -Math.PI/2) };
		Pose start142 = new Pose(7.7, 20, -Math.PI/2); 	Pose[] goal142 = { new Pose(7.7,35, -Math.PI/2) };
		Pose start152 = new Pose(10.2,20, -Math.PI/2); 	Pose[] goal152 = { new Pose(10.2,35, -Math.PI/2) };
		Pose start162 = new Pose(12.5,34, Math.PI/2); 	Pose[] goal162 = { new Pose(12.5,19, Math.PI/2) };
		Pose start172 = new Pose(2.5, 34, Math.PI/2) ; 	Pose[] goal172 = { new Pose(2.5,19,  Math.PI/2) };
		Pose start182 = new Pose(5, 34, Math.PI/2); 	Pose[] goal182 = { new Pose(5,19,  Math.PI/2) };
		Pose start192 = new Pose(7.5, 34, Math.PI/2); 	Pose[] goal192 = { new Pose(7.5,19,   Math.PI/2) };
		Pose start203 = new Pose(10,34, Math.PI/2); 	Pose[] goal203 = { new Pose(10,19, Math.PI/2) };


		Pose start104 = new Pose(21, 32.5, Math.PI); 	Pose[] goal104 = { new Pose(36, 32.5, Math.PI) };	
		Pose start204 = new Pose(21, 22.5, Math.PI); 	Pose[] goal204 = { new Pose(36, 22.5, Math.PI) };
		Pose start304 = new Pose(21, 25, Math.PI); 		Pose[] goal304 = { new Pose(36, 25, Math.PI) };
		Pose start404 = new Pose(21, 27.5, Math.PI); 	Pose[] goal404 = { new Pose(36, 27.5, Math.PI) };
		Pose start504 = new Pose(21, 30, Math.PI); 	Pose[] goal504 = { new Pose(36, 30, Math.PI) };	
		Pose start604 = new Pose(35, 20.2, 0); 		Pose[] goal604 = { new Pose(20, 20.2, 0) };
		Pose start704 = new Pose(35, 22.7,0) ; 		Pose[] goal704 = { new Pose(20, 22.7, 0) };
		Pose start804 = new Pose(35, 25.2, 0); 		Pose[] goal804 = { new Pose(20, 25.2, 0) };
		Pose start904 = new Pose(35, 27.7, 0); 		Pose[] goal904 = { new Pose(20, 27.7, 0) };
		Pose start105 = new Pose(35, 30.2, 0); 		Pose[] goal105 = { new Pose(20, 30.2, 0) };

		Pose start115 = new Pose(32.7,20, -Math.PI/2); 	Pose[] goal115 = { new Pose(32.7,35, -Math.PI/2) };
		Pose start125 = new Pose(22.7,20,-Math.PI/2) ; 	Pose[] goal125 = { new Pose(22.7,35, -Math.PI/2) };
		Pose start135 = new Pose(25.2,20, -Math.PI/2); 	Pose[] goal135 = { new Pose(25.2,35, -Math.PI/2) };
		Pose start145 = new Pose(27.7, 20, -Math.PI/2); 	Pose[] goal145 = { new Pose(27.7,35, -Math.PI/2) };
		Pose start155 = new Pose(30.2,20, -Math.PI/2); 	Pose[] goal155 = { new Pose(30.2,35, -Math.PI/2) };
		Pose start165 = new Pose(32.5,34, Math.PI/2); 	Pose[] goal165 = { new Pose(32.5,19, Math.PI/2) };
		Pose start175 = new Pose(22.5, 34, Math.PI/2) ; 	Pose[] goal175 = { new Pose(22.5,19,  Math.PI/2) };
		Pose start185 = new Pose(25, 34, Math.PI/2); 		Pose[] goal185 = { new Pose(25,19,  Math.PI/2) };
		Pose start195 = new Pose(27.5, 34, Math.PI/2); 	Pose[] goal195 = { new Pose(27.5,19,   Math.PI/2) };
		Pose start205 = new Pose(30,34, Math.PI/2); 		Pose[] goal205 = { new Pose(30,19, Math.PI/2) };

		Thread thread122 = initThread(122, c, start122, goal122);
		Thread thread222 = initThread(222, c, start222, goal222);
		Thread thread322 = initThread(322, c, start322, goal322);
		Thread thread422 = initThread(422, c, start422, goal422);
		Thread thread522 = initThread(522, c, start522, goal522);
		Thread thread622 = initThread(622, c, start622, goal622);
		Thread thread722 = initThread(722, c, start722, goal722);
		Thread thread822 = initThread(822, c, start822, goal822);
		Thread thread922 = initThread(922, c, start922, goal922);
		Thread thread102 = initThread(102, c, start102, goal102);
		
		Thread thread112 = initThread(112, c, start112, goal112);
		Thread thread123 = initThread(123, c, start123, goal123);
		Thread thread132 = initThread(132, c, start132, goal132);
		Thread thread142 = initThread(142, c, start142, goal142);
		Thread thread152 = initThread(152, c, start152, goal152);
		Thread thread162 = initThread(162, c, start162, goal162);
		Thread thread172 = initThread(172, c, start172, goal172);
		Thread thread182 = initThread(182, c, start182, goal182);
		Thread thread192 = initThread(192, c, start192, goal192);
		Thread thread203 = initThread(203, c, start203, goal203);
		
		Thread thread104 = initThread(104, c, start104, goal104);
		Thread thread204 = initThread(204, c, start204, goal204);
		Thread thread304 = initThread(304, c, start304, goal304);
		Thread thread404 = initThread(404, c, start404, goal404);
		Thread thread504 = initThread(504, c, start504, goal504);
		Thread thread604 = initThread(604, c, start604, goal604);
		Thread thread704 = initThread(704, c, start704, goal704);
		Thread thread804 = initThread(804, c, start804, goal804);
		Thread thread904 = initThread(904, c, start904, goal904);
		Thread thread105 = initThread(105, c, start105, goal105);
		Thread thread115 = initThread(115, c, start115, goal115);
		Thread thread125 = initThread(125, c, start125, goal125);
		Thread thread135 = initThread(135, c, start135, goal135);
		Thread thread145 = initThread(145, c, start145, goal145);
		Thread thread155 = initThread(155, c, start155, goal155);
		Thread thread165 = initThread(165, c, start165, goal165);
		Thread thread175 = initThread(175, c, start175, goal175);
		Thread thread185 = initThread(185, c, start185, goal185);
		Thread thread195 = initThread(195, c, start195, goal195);
		Thread thread205 = initThread(205, c, start205, goal205);



		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(25, 12, 15);
		//viz.setInitialTransform(15, -10, 5);
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
		vh.setSpatialEnvelope2(true);
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
	}
	//System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
	thread2.start();
	thread3.start();
	thread4.start();
	thread5.start();
	thread6.start();
	thread7.start();
	thread8.start();	
	thread9.start();	
	thread10.start();
	thread11.start();
	thread12.start();	
	thread13.start();	
	thread14.start();
	thread15.start();		
	thread16.start();
	thread17.start();	
	thread18.start();	
	thread19.start();
	thread20.start();
	
	thread100.start();
	thread200.start();
	thread300.start();
	thread400.start();
	thread500.start();
	thread600.start();
	thread700.start();
	thread800.start();	
	thread900.start();	
	thread101.start();
	thread110.start();
	thread120.start();	
	thread130.start();	
	thread140.start();
	thread150.start();		
	thread160.start();
	thread170.start();	
	thread180.start();	
	thread190.start();
	thread201.start();


	thread122.start();
	thread222.start();
	thread322.start();
	thread422.start();
	thread522.start();
	thread622.start();
	thread722.start();
	thread822.start();	
	thread922.start();	
	thread102.start();
	thread112.start();
	thread123.start();	
	thread132.start();	
	thread142.start();
	thread152.start();		
	thread162.start();
	thread172.start();	
	thread182.start();	
	thread192.start();
	thread203.start();
	
	thread104.start();
	thread204.start();
	thread304.start();
	thread404.start();
	thread504.start();
	thread604.start();
	thread704.start();
	thread804.start();	
	thread904.start();	
	thread105.start();
	thread115.start();
	thread125.start();	
	thread135.start();	
	thread145.start();
	thread155.start();		
	thread165.start();
	thread175.start();	
	thread185.start();	
	thread195.start();
	thread205.start();

	}
}