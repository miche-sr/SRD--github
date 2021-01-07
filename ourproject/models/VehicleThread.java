package se.oru.coordination.coordination_oru.ourproject.models;

//import java.lang.reflect.Array;
//import java.sql.Time;
import java.util.ArrayList;
import java.util.TreeSet;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.ourproject.algorithms.ConstantAccelerationForwardModel.Behavior;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int cp = -2;
	private int sp = -1;
	private TreeSet<CriticalSection> analysedCs = new TreeSet<CriticalSection>(); 
	private ArrayList<Vehicle> analysedVehicles = new ArrayList<Vehicle>();
	private ArrayList<RobotReport> rrNears = new ArrayList<RobotReport>();
	private Boolean prec = true;
	

	public VehicleThread(Vehicle v){
		this.v = v;
	}
	
	/************************** 
	****  MAIN ALGORITHM   ****
	***************************/

	public void run() {
		String List;

		try{
			while(v.getForwardModel().getRobotBehavior() != Behavior.reached){
				
				//// UNPACK MESSAGES ////
				rrNears.clear();
				for (Vehicle vh : v.getNears()){
					rrNears.add(v.getMainTable().get(vh.getID()));
				}
				for (RobotReport rr : rrNears)
					System.out.println(rr.getID()+" ");
		
				///// MEMORY OF CRITICAL SECTIONS ////
				// if a cs has already been found and no one is inside the cs then I don't recalculate the cs //
				analysedVehicles.clear();
				this.analysedCs.clear();
				for (CriticalSection analysedCs : v.getCs()){
					if (v.getPathIndex() < analysedCs.getTe1Start() 
							&& analysedCs.getVehicle2().getPathIndex() < analysedCs.getTe2Start()
								&& !analysedCs.isCsTruncated()) {
						this.analysedCs.add(analysedCs);
						analysedVehicles.add(analysedCs.getVehicle2());
					}
				if (analysedCs.isCsTruncated()) System.out.println("troncato");
				}
				
				// re-add the cs already analysed and find the cs of other vehicles
				v.clearCs();
				for (CriticalSection analysedCs : this.analysedCs)
					v.getCs().add(analysedCs);
				
				List = "";
				boolean newPossibleCs = false;
				for (Vehicle vh : this.v.getNears()){
					if (!analysedVehicles.contains(vh)) {
						v.appendCs(vh);
						newPossibleCs = true;
					}
					else System.out.println("skip");
					List = (List + vh.getID() + " " );
				}
				
				//// CALCULATE THE PRECEDENCES ////
				if ((newPossibleCs == true && v.getCs().size() != 0)  //per nuova commenta qui e aggiung ")"
						||(v.getCs().size() == 0 && v.getCriticalPoint() != -1) 
							|| cp == -2){
					System.out.println("ricalcolo");
					prec = true;
					v.setCriticalPoint(-1);
					for (CriticalSection cs : this.v.getCs()){
						prec = cs.ComputePrecedences();
						if (prec == false)								//calculate precedence as long as I have precedence
							v.setCriticalPoint(cs);						//calcultate critical Point
							break;
					}
					//v.setSlowingPoint(); // altrimenti non calcola ultimo punto critico
					v.setSlowingPointNew();
					cp = v.getCriticalPoint();
					if (cp == -1) cp = v.getWholePath().length;

					sp = v.getForwardModel().getPathIndex(v.getWholePath(), v.getSlowingPoint());
					if (sp == 0) sp = 1;
				}
				//else System.out.print("\nFUNZIONA skip calcolo prec \n");

				//// UPDATE VALUES ////
				v.setTimes();
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();
				v.setSpatialEnvelope();

				//// SEND NEW ROBOT REPORT ////
				v.sendNewRr();
				
				/***********************************
				 ****** VISUALIZATION AND PRINT ****
				 ***********************************/
				printLog(List, prec);
				
				v.getVisualization().addEnvelope(v.getWholeSpatialEnvelope().getPolygon(),v,"#f600f6");
				v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,"#efe007");
				v.getVisualization().displayPoint(v, cp-1, "#29f600"); //-1 perche array parte da zero
				v.getVisualization().displayPoint(v, sp-1, "#0008f6");
				v.getVisualization().displayPoint(v, v.getStoppingPoint(), "#ffffff");

				String infoCs = v.getForwardModel().getRobotBehavior().toString();
				v.getVisualization().displayRobotState(v.getSpatialEnvelope().getFootprint(), v,infoCs);

				/// SLEEPING TIME ////
				Thread.sleep(v.getTc());
				this.elapsedTrackingTime += v.getTc()*Vehicle.mill2sec;
			}
			System.out.println("\n R" + this.v.getID() + " : GOAL RAGGIUNTO" );
		}
		
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");

		}	
	}


	/// FUNCTION FOR PRINTING INFORMATIONS ////	
	public void printLog(String List, Boolean prec) {
		String CsString = "";
		String Dist = String.format("%.2f", v.getDistanceTraveled());
		String Vel = String.format("%.2f", v.getVelocity());
		//String infoCs = this.infoCs();
		String infoCs = v.getForwardModel().getRobotBehavior().toString();
		if (v.getCs().size() != 0){
			int i = 0;
			for (CriticalSection cs : v.getCs()) {
				CsString = CsString + (i+1 +"° Sezione Critica" +
				"\t Mia: " + cs.getTe1Start()+"-"+cs.getTe1End() +
				"\t R" + cs.getVehicle2().getID() + ":" +
				"\t" + cs.getTe2Start()+"-"+cs.getTe2End()) + "\n";
				i += 1;
			}
		}
		else	CsString = ("0 Sezioni Critiche");
				
		System.out.println(
			"============================================================\n"+
			"Info R" + this.v.getID() + " : \n" + 
			"Last Index: "+ (v.getWholePath().length-1) + "\t \t " + infoCs + "\n" + 
			"Vicini: "  + List + "\t \t Precedenza: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"SLowing Point: " + v.getSlowingPoint() +"\t Critical Point:" + v.getCriticalPoint()+ "\n" + 
			CsString + "\n " +
			"Traiettoria Trasmessa \n" +v.getTruncateTimes() +   "\n"+
			"============================================================"
			);
	}
	



}