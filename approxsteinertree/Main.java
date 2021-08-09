/*************************************************************************************
 * Allocations and Algorithms
 * Main.java
 * Authors:
 * Meet Shah - i6196781
 * Max van den Broek - i6161647
 * Runs code to calculate Approximate Steiner Tree for a given instance
 * Check in code comments for more information
 * Requires classes: Graph and MelhornAlgo.
 *************************************************************************************/

package approxsteinertree;

public class Main {

	public static void main(String[] args) {
		/**Set Instance in following piece of code to get solution for the instance**/
		/**Assumes working directory is steiner_tree_approx**/

		String instance = "001" ; //Fill instance number here
		String filepath1 = "./instances/instance" + instance + ".gr";
		String filepath2 = "./solvedInstances/instance" + instance + ".sol";
		Graph instanceWorkedUpon = new Graph(filepath1);
		MehlhornAlgorithm cycleChecker = new MehlhornAlgorithm(instanceWorkedUpon);
		Graph finalGraph = cycleChecker.execute(false, filepath2); //set to true if solution is wanted in Console and not in .sol file.
		System.out.println("Is Valid Steiner Tree:" + finalGraph.isValidSteinerTree(instanceWorkedUpon)); //Checker which checks if solution is valid.



		/**Uncomment underlying code & Comment code above in order to get solutions for each instance of problems given**/
		/**Assumes working directory is the folder steiner_tree_approx**/


	/*	String filepath1 = "./Instances/instance";
		String filepath2 = "./solvedInstances/instance";
		String fileStart = filepath1+"141.gr";
		String fileEnd = filepath2 + "141.sol";
		for(int i = 1; i < 200; i+=2) {
			if (i < 10) {
				String instance = "00" + i;
				fileStart = filepath1 + instance + ".gr";
				fileEnd = filepath2+instance+".sol";
			}
			else if (i >= 10 && i< 100) {
				String instance = "0" + i;
				fileStart = filepath1 + instance + ".gr";
				fileEnd = filepath2+instance+".sol";
			}
			else {
				String instance = Integer.toString(i);
				fileStart = filepath1 + instance + ".gr";
				fileEnd = filepath2+instance+".sol";
			}
			Graph instanceWorkedUpon = new Graph(fileStart);
			MehlhornAlgorithm Approx2Algo = new MehlhornAlgorithm(instanceWorkedUpon);
			Approx2Algo.execute(false, fileEnd);
		} */
	}
}
