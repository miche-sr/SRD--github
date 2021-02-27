package se.oru.coordination.coordination_oru.distributed.models;

import static org.junit.Assert.fail;


import java.util.*;
import java.nio.*;


import cern.colt.buffer.IntBuffer;
import se.oru.coordination.coordination_oru.distributed.algorithms.CriticalSectionsFounder;
import se.oru.coordination.coordination_oru.distributed.models.Vehicle.Behavior;


public class VehicleThread implements Runnable {
	
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();

	private boolean run = true;
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int oldCp = -1;
	private int cp = 0;
	private TreeSet<CriticalSection> analysedCs = new TreeSet<CriticalSection>(); 
	private ArrayList<Integer> analysedVehiclesI = new ArrayList<Integer>();
	private HashMap<Integer, RobotReport> rrNears = new HashMap<Integer, RobotReport>();
	private Boolean prec = true;
	private CriticalSection csOld;
	private boolean FreeAccess = true;
	private int smStopIndex = -1;
	private String List = " ";
	private int spF = 0;
	

	

	private static String colorTruEnv ="#000000"; //#efe007";
	private static String colorStp = "#047d00"; //"#0008f6"; //"#ffffff";
	private static String colorCrp = "#a30202"; //"#29f600";

	private ArrayList<Double> timesCs = new ArrayList<Double>();
	private ArrayList<Double> timesPrec = new ArrayList<Double>();
	private ArrayList<Double> timesTc = new ArrayList<Double>();


	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	/************************** 
	****  MAIN ALGORITHM   ****
	***************************/

	public void run() {
		double start = System.currentTimeMillis();
		if(v.isTrakerEnable()) v.startTraker();
		// int alpha =(int) v.getalpha();
		// int[] chunkArray = new int[alpha];
		// for (int k=0;k==alpha;k++){
		// 	chunkArray[k]=0;
		// }
		
		try{
			while(v.getBehavior() != Behavior.reached && run){
				long startTc = System.currentTimeMillis();
				/*********************
				 * RECEIVE MESSAGES *
				 ********************/
				rrNears.clear();
				List = " ";
				for (Integer vh : v.getNears()){
					rrNears.put(vh,v.getMainTable().get(vh));
					List = (List + vh + " " );
					if (v.checkCollision(vh) ) {
						System.out.println("\u001B[31m" + "ATTENTION! COLLISION OCCURRED  R" + v.getID() +" E R" +vh + "\u001B[0m");
						printLog(List, prec);
						run = false;
					}
				}
				
				
				
				/**************************
				 ** 	UPDATE STATE	  ** 
				 ***************************/
				v.updateState();
				
				
				// v.setTimes();
				v.setChunk(FreeAccess,smStopIndex);
				v.setStoppingPoint();
				

				
				/****************************
				 * FIND CRITICAL SECTIONS *
				 ****************************/
				double startCs = System.currentTimeMillis();

	
				v.clearCs();
				for (RobotReport vh : rrNears.values())
					v.appendCs(vh);

				double finishCs = System.currentTimeMillis();
				if(v.getCs().size() != 0) timesCs.add((finishCs - startCs));

				/**********************************
				 		** SEMAPHORE **
				 ++++++++++++++++++++++++++++++++*/
				trafficLights();

				/*******************************
				 ** COMPUTE THE PRECEDENCES ** 
				 *******************************/
				
				//  v.setCriticalPoint(chunkArray[0]);
				//  	for (int k=1;k<(alpha);k++){
				//  	chunkArray[k-1]=chunkArray[k];
				//  }
				// chunkArray[alpha-1] = (v.getPathIndex() + v.getSpatialEnvelope().getPath().length-1);
				v.setCriticalPoint(v.getPathIndex() + v.getSpatialEnvelope().getPath().length-1); 
				
				precedence();

				/************************************
				 ** 	SET CRITICAL AISLE	 ** 
				 ************************************/

				if(FreeAccess== false && v.getStoppingPoint() != -1 && smStopIndex >= 0) // Aisle management
					v.setCriticalPoint( Math.min(v.getCriticalPoint(),smStopIndex) );

				
				/*****************************************
				 ** FORWARD CP TO LOW LEVELE CONTROLLER ** 
				 *****************************************/
				v.getTraker().setCriticalPoint(v.getCriticalPoint());
				v.getTraker().setFreeAccess(FreeAccess);

				/*************************************
				 ** ESTIMATE TEMPORAL PROFILE	 ** 
				 *************************************/
				v.setTimes();


				/*****************************************
				 ** FUTURE STOPPING POINT AND BROADCAST	 ** 
				 ***************************************/
				v.updateState();
				v.setChunk(FreeAccess,smStopIndex);
				
				
				
				 spF = v.getFutureStoppingPoint();
				v.sendNewRr(spF);

				/***********************************
				 ****** VISUALIZATION AND PRINT ****
				 ***********************************/
				//printLog(List, prec);
				visualization();

				/********************************
				 ****** SLEEPING TIME****
				 ********************************/
				double finishTc = System.currentTimeMillis();
				double timeElapsedTc = (finishTc - startTc);
				timesTc.add(timeElapsedTc);

				Thread.sleep(v.getTc() ); //sleep
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			
			} //END OF PERIOD //
			

			
			/********************************
			 ****** THREAD CLOSURE ****
			********************************/
			v.setStoppingPoint();
			v.setTimes();
			v.setChunk(FreeAccess,smStopIndex);
			visualization();
			v.sendNewRr(spF);
			double finish = System.currentTimeMillis();
			double timeElapsed = (finish - start)/1000;
		

			String dataCs = getTimeData(timesCs, false);
			String dataPrec = getTimeData(timesPrec, false);
			String dataTc = getTimeData(timesTc, true);

			System.out.println("\u001B[34m"+"R" + this.v.getID() + " " +  String.format("%.3f", timeElapsed)+ 
			" " + dataCs +  dataPrec + dataTc +"" + timesTc.size()+"\u001B[0m");
		

		}	
		catch (InterruptedException e) {
			System.out.println("Thread interrupted");
		}	
	}







	public void visualization(){
		if(v.getCriticalPoint() >= v.getWholePath().length-1 || v.getCriticalPoint()==-1)
			cp = (v.getPathIndex() + v.getSpatialEnvelope().getPath().length-1 ) ;
		else cp = v.getCriticalPoint();

		
		v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,colorTruEnv);
		v.getVisualization().displayPoint(v, cp, colorCrp); 
		v.getVisualization().displayPoint(v, v.getStoppingPoint(), colorStp);
		//String infoCs = v.getBehavior().toString();
		//v.getVisualization().displayRobotState(v.getSpatialEnvelope().getFootprint(), v,infoCs);
		


	}

	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		String Dist = String.format("%.2f", v.getDistanceTraveled());
		String Vel = String.format("%.2f", v.getVelocity());
		String Slow = String.format("%.2f", v.getSlowingPoint());
		String infoCs = v.getBehavior().toString();
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Critical Section" +
				"\t Mine: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + 
				"\t Precedence:" + cs.isPrecedenza() + "\n";
				i += 1;
			}
		}
		else	CsString = ("0 Critical Sections\n");
				
		System.out.println(
			"============================================================\n"+
			"Info R" + this.v.getID() + " : \n" + 
			"Last Index: "+ (v.getWholePath().length-1) + "\t \t " + infoCs + "\n" + 
			"Neighbors: "  + List + "\t \t Precedence: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"Slowing Point: " + Slow +"\t Critical Point: " + v.getCriticalPoint()+ "\n" + 
			CsString + "\n" +
			"Current Trajectory \n" +v.getTruncateTimes() + "\n"+
			"============================================================"
			);
	}
	
	public String getTimeData(ArrayList<Double> times, boolean Cex){
		double sum = 0;
		double tmax = 0;
		int count = 0;
		int countEx = 0;
		for(double t : times){
			if ( t != 0.0 ){
			sum = sum + t;
			count = count +1;}
			if(t > v.getTc()) countEx = countEx +1;
			if(t > tmax) tmax = t;
		}
		double mean = sum/ count ;

		String string = " " + String.format("%.2f", mean)+ " " + String.format("%.2f", tmax) + " " ;
		if (Cex == true) string = string  + countEx + " ";
		return string;
	}



	public void trafficLights(){
		for (TrafficLights TL : v.getTrafficLightsNears()){
			synchronized(TL){
				if (v.getWholeSpatialEnvelope().getPolygon().intersects(TL.getCorridorPath().getPolygon())){

					if (!TL.getRobotInsideCorridor().contains(v.getID()) ){
						if (TL.checkSemophore(v) ){ // entering the corridor
								FreeAccess = true;				v.clearCs();
								for (RobotReport vh : rrNears.values())
									v.appendCs(vh);
								if(v.getSpatialEnvelope().getPolygon().intersects(TL.getCorridorPath().getPolygon()))
									TL.addVehicleInside(v);
						}
						else { // red semaphore
							
							if (FreeAccess == true) smStopIndex = intersect.SmStopIndex(v, TL) - 18; //30pathIndex = 3 footPrint
							FreeAccess = false;
						}
					}
				}
				if (!v.getSpatialEnvelope().getPolygon().intersects(TL.getCorridorPath().getPolygon())) {
					if (FreeAccess == true) { // exiting the corridor
						if (TL.getRobotInsideCorridor().contains(v.getID()))
							TL.removeVehicleInside(v);
					}
					else if (v.getPathIndex() >= intersect.SmStopIndex(v, TL)) FreeAccess = true; // robot is outside
			
				}
				
			}
		}

	}


	public void precedence(){
		double startPrec = System.currentTimeMillis();
				prec = true;
				
				//v.setCriticalPoint(v.getWholePath().length-1); 
				v.setCriticalPoint(v.getPathIndex() + v.getSpatialEnvelope().getPath().length-1); 
				csOld = null;
				if (v.getCs().size() <= 1)  v.setCsTooClose(false);
				for (CriticalSection cs : this.v.getCs()){
					
					prec =v.ComputePrecedences(cs);
					if (csOld != null) v.setCsTooClose(intersect.csTooClose(v, csOld, cs));
					if (prec == false && v.isCsTooClose() && csOld != null){		//calculate precedence as long as I have precedence
						v.setCriticalPoint(csOld);
						v.getVisualization().displayDependency(v, cs.getVehicle2(),"waiting"+v.getID());
						break;
					}
					else if (prec == false){
						v.setCriticalPoint(cs);
						v.getVisualization().displayDependency(v, cs.getVehicle2(),"waiting"+v.getID());
						break;
					}
					if (v.isCsTooClose() == false || csOld == null) 
						csOld = cs;
				}

				double endPrec = System.currentTimeMillis();
				if(v.getCs().size() != 0) timesPrec.add(( endPrec  - startPrec));
	}

	

}
