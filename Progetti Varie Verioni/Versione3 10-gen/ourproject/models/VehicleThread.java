package se.oru.coordination.coordination_oru.ourproject.models;

//import java.lang.reflect.Array;
//import java.sql.Time;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TreeSet;

import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import se.oru.coordination.coordination_oru.ourproject.algorithms.ConstantAccelerationForwardModel.Behavior;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	private double elapsedTrackingTime = 0.0;
	private int sp = -1;
	private int oldCp = -1;
	private TreeSet<CriticalSection> analysedCs = new TreeSet<CriticalSection>(); 
	private ArrayList<RobotReport> analysedVehicles = new ArrayList<RobotReport>();
	private HashMap<Integer, RobotReport> rrNears = new HashMap<Integer, RobotReport>();
//	private ArrayList<RobotReport> rrNears = new ArrayList<RobotReport>();
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
					rrNears.put(vh.getID(),v.getMainTable().get(vh.getID()));
					//if(v.getID()==1) System.out.println("pathIndex3rr "+rrNears.get(3).getPathIndex());
				}
				//if(v.getID()==1) System.out.println("pathIndex3tb "+v.getMainTable().get(3).getPathIndex());

			/****************************
			 * FILTER CRITICAL SECTIONS *
			 ****************************/
			// if a cs has already been found and no one is inside it then I don't recalculate it //
				analysedVehicles.clear();
				this.analysedCs.clear();
				for (CriticalSection analysedCs : v.getCs()){
					int v2Id = analysedCs.getVehicle2().getID();
					System.out.println(v.getID()+": Index altrui: "+rrNears.get(v2Id).getPathIndex());
					if (v.getPathIndex() < analysedCs.getTe1Start() 
							&& rrNears.get(v2Id).getPathIndex() < analysedCs.getTe2Start()
								&& !analysedCs.isCsTruncated()) {
						this.analysedCs.add(analysedCs);
						analysedVehicles.add(analysedCs.getVehicle2());
					}
//					if (analysedCs.isCsTruncated()) System.out.println("troncato");
				}
				
				// re-add the cs already analysed and find the cs of other vehicles
				v.clearCs();
				for (CriticalSection analysedCs : this.analysedCs)
					v.getCs().add(analysedCs);
				
				List = "";
				boolean newPossibleCs = false;
				for (RobotReport vh : rrNears.values()){
					if (!analysedVehicles.contains(vh)) {
						v.appendCs(vh);
						newPossibleCs = true;
					}
					else System.out.println(v.getID()+": skip");
					List = (List + vh.getID() + " " );
				}
				
				/*******************************
				 ** CALCULATE THE PRECEDENCES ** 
				 *******************************/
				prec = true;
				v.setCriticalPoint(v.getWholePath().length-1); // ex -1
				for (CriticalSection cs : this.v.getCs()){
					prec = cs.ComputePrecedences();
					if (prec == false)		//calculate precedence as long as I have precedence
						v.setCriticalPoint(cs);
						break;
				}
				if (oldCp != v.getCriticalPoint()) {
					v.setSlowingPointNew();
					sp = v.getForwardModel().getPathIndex(v.getWholePath(), v.getSlowingPoint());
					oldCp = v.getCriticalPoint();
				}

				//// UPDATE VALUES ////
				printLog(List, prec);
				
				v.setPathIndex(elapsedTrackingTime);
				v.setPose(v.getWholePath()[v.getPathIndex()].getPose());
				v.setStoppingPoint();

				v.setTimes();
				v.setSpatialEnvelope();

				//// SEND NEW ROBOT REPORT ////
				v.sendNewRr();
//				if(v.getID()==1) System.out.println(v.getMainTable().get(3).getPathIndex());
//				if(v.getID()==1 && rrNears.containsKey(3)) System.out.println(rrNears.get(3));

				
				/***********************************
				 ****** VISUALIZATION AND PRINT ****
				 ***********************************/
//				/*if(v.getID()==1)*/ printLog(List, prec);
				
				v.getVisualization().addEnvelope(v.getWholeSpatialEnvelope().getPolygon(),v,"#f600f6");
				v.getVisualization().addEnvelope(v.getSpatialEnvelope().getPolygon(),v,"#efe007");
				v.getVisualization().displayPoint(v, v.getCriticalPoint(), "#29f600"); //-1 perche array parte da zero
				v.getVisualization().displayPoint(v, sp, "#0008f6");
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
		String Slow = String.format("%.2f", v.getSlowingPoint());
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
		else	CsString = ("0 Sezioni Critiche\n");
				
		System.out.println(
			"============================================================\n"+
			"Info R" + this.v.getID() + " : \n" + 
			"Last Index: "+ (v.getWholePath().length-1) + "\t \t " + infoCs + "\n" + 
			"Vicini: "  + List + "\t \t Precedence: " + prec +  "\n" + 
			"Path Index: " 	+ v.getPathIndex() + "\t \t Stopping Point: " + v.getStoppingPoint() + "\n" +
			"Distance: "+ Dist  + "\t \t Velocity: " + Vel + "\n" +
			"Slowing Point: " + Slow +"\t Critical Point: " + v.getCriticalPoint()+ "\n" + 
			CsString + "\n" +
			"Traiettoria Trasmessa \n" +v.getTruncateTimes() + "\n"+
			"============================================================"
			);
	}
	



}