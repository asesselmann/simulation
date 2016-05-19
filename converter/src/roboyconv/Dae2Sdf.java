package roboyconv;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.UnsupportedEncodingException;
import java.util.LinkedList;

import javax.swing.JTextArea;

public class Dae2Sdf {

	File daeFile;
	JTextArea daeLog;
	static private final String newline = "\n";
	String rest;
	boolean debug = false;

	public Dae2Sdf(File daeFile, JTextArea daeLog) {
		this.rest = "";
		this.daeFile = daeFile;
		this.daeLog = daeLog;

		daeLog.removeAll();

		try {
			createSDF();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (UnsupportedEncodingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	private static double[] transformationMatrix2EulerAngles(double[][] tfMatrix)
			throws Exception {

		if (tfMatrix.length != 4) {
			throw new Exception("Dimension of transformation matrix wrong.");
		}
		if (tfMatrix[0].length != 4 || tfMatrix[1].length != 4
				|| tfMatrix[2].length != 4 || tfMatrix[3].length != 4) {
			throw new Exception("Dimension of transformation matrix wrong.");
		}

		double[] eulerAngles = new double[6];

		eulerAngles[0] = tfMatrix[0][3];
		eulerAngles[1] = tfMatrix[1][3];
		eulerAngles[2] = tfMatrix[2][3];

		eulerAngles[3] = Math.atan2(tfMatrix[2][1], tfMatrix[2][2]);
		eulerAngles[4] = Math.atan2(
				-tfMatrix[2][0],
				Math.sqrt(tfMatrix[2][1] * tfMatrix[2][1] + tfMatrix[2][2]
						* tfMatrix[2][2]));
		eulerAngles[5] = Math.atan2(tfMatrix[1][0], tfMatrix[0][0]);
		return eulerAngles;
	}

	private static void printEuler(double[] euler) {

		if (euler.length != 3) {
			return;
		}

		String outputString = "[" + euler[0];
		for (int i = 1; i < euler.length; i++) {
			outputString += ", " + euler[i];
		}
		outputString += "]";
		System.out.println(outputString);
	}

	private static boolean isValidXMLTag(String tag) {
		if (tag.startsWith("<") && tag.startsWith(">")) {
			return true;
		} else {
			return false;
		}
	}

	private static String getXMLcontent(String tag) {
		return tag.substring(1, tag.length() - 1);
	}

	private static boolean containsXMLTag(String line) {
		if (line.contains("<") && line.contains(">")) {
			return true;
		} else {
			return false;
		}
	}

	private String getNextXMLTag(String line) {
		String foo = line.substring(line.indexOf("<"), line.indexOf(">") + 1);
		rest = line.substring(line.indexOf(">") + 1, line.length());
		return foo;
	}

	private void debugPrint(String string) {
		if (debug) {
			System.out.println(string);
		}
	}

	/**
	 * Main method for creating a SDF file out of the dae
	 * 
	 * @throws FileNotFoundException
	 * @throws UnsupportedEncodingException
	 */
	public String createSDF() throws FileNotFoundException,
			UnsupportedEncodingException {

		// unity matrix as basic transformation
		double[][] currentTransformation = { { 1.0, 0.0, 0.0, 0.0 },
				{ 0.0, 1.0, 0.0, 0.0 }, { 0.0, 0.0, 1.0, 0.0 },
				{ 0.0, 0.0, 0.0, 1.0 } };

		try {
			BufferedReader br = new BufferedReader(new FileReader(daeFile));
			String line;

			// creating some "head" object, used later
			XMLObject head = new XMLObject("head", null);
			XMLObject currentParent = head;

			// a list of alle XML objects found, used for step 2
			LinkedList<XMLObject> XMLList = new LinkedList<XMLObject>();

			// Step 1: read file line for line
			while (true) {
				String debugString = "";
				// case for multiple xml tags in one line
				if (containsXMLTag(rest)) {
					line = rest;
				} else {
					line = br.readLine();
				}

				if (line == null)
					break;

				// parse the next xml tag
				String currentTag = getNextXMLTag(line);

				// ignore headers
				if (currentTag.contains("<?")
						|| currentTag.toLowerCase().contains("<collada")
						|| currentTag.toLowerCase().contains("<library_nodes")
						|| currentTag.toLowerCase().contains("</library_nodes")
						|| currentTag.toLowerCase().contains("</collada")) {
					continue;
				}

				daeLog.append(currentTag + newline);

				if (currentTag.toLowerCase().contains("/matrix")) {
					continue;
				}
				if (currentTag.startsWith("</")) {
					// closed Tag
					currentParent = currentParent.getParent();
					debugString += "Curr Parent is now: "
							+ currentParent.getTag();
					debugPrint(debugString);
					continue;
				} else if (currentTag.endsWith("/>")) {
					if (currentTag.contains("instance_node")) {
						XMLObject tmp = new XMLObject(currentTag, currentParent);
						currentParent.addContent(tmp);
					}
					// TODO material
					continue;
				} else {
					// new Tag

					// if matrix detected, parse it
					if (line.contains("<matrix>")) {
						for (int i = 0; i < 4; i++) {
							String matLine = br.readLine();
							matLine = matLine.trim();
							String[] values = matLine.split(" ");
							for (int j = 0; j < 4; j++) {
								currentTransformation[i][j] = Double
										.parseDouble(values[j]);
							}
						}
						// we need euler angles
						currentParent.pose = transformationMatrix2EulerAngles(currentTransformation);
						debugString += "Matrix detected";
						debugPrint(debugString);
						continue;
					}

					debugString += "Curr Parent: " + currentParent.getTag()
							+ " Curr Tag: " + currentTag;

					XMLObject foo = new XMLObject(currentTag, currentParent);
					currentParent.addContent(foo);
					currentParent = foo;
					debugString += " is now current parent ";
					XMLList.add(foo);
					debugString += " added to List";
				}
				debugPrint(debugString);
			}
			// now the content of the library_nodes.dae file is parsed and in a
			// hierarchical structure
			// Step 2: now replace the node_instances

			br.close();
			head.replaceInstances(XMLList);
			return head.toSDFString("");

		} catch (Exception e) {
			e.printStackTrace();
		}
		return "error";
	}
}
