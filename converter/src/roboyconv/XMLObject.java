package roboyconv;

import java.util.LinkedList;

public class XMLObject {

	static private final String newline = "\n";
	static private final String tab = "\t";

	private String tag;
	private LinkedList<XMLObject> content;
	private XMLObject parent;
	public double[] pose;

	public XMLObject() {
		this.pose = new double[6];
		this.tag = "";
		this.parent = null;
		this.content = new LinkedList<XMLObject>();
	}

	public XMLObject(String tag, XMLObject parent) {
		this.pose = new double[6];
		this.tag = tag;
		this.parent = parent;
		this.content = new LinkedList<XMLObject>();
	}

	// cloning
	public XMLObject(XMLObject source, XMLObject parent) {
		this.pose = source.pose;
		this.tag = source.tag;
		this.parent = parent;
		this.content = new LinkedList<XMLObject>();
		for (XMLObject xmlObject : source.content) {
			content.add(new XMLObject(xmlObject, this));
		}
	}

	public void addContent(XMLObject object) {
		content.add(object);
	}

	public XMLObject getParent() {
		return this.parent;
	}

	public String getTag() {
		return this.tag;
	}

	private String poseString() {
		String foo = "<pose>" + pose[0];
		for (int i = 1; i < pose.length; i++) {
			foo += " " + pose[i];
		}
		foo += "</pose>";
		return foo;
	}

	private String getName() {
		String outputString = "";

		String currentTag = tag.replace("<", "");
		currentTag = currentTag.replace(">", "");
		String[] values = currentTag.split(" ");

		for (String element : values) {
			if (element.contains("name=\"")) {
				outputString = element
						.substring(element.indexOf("name=\"") + 6);
				outputString = outputString.replace("\"", "");
			}
		}

		return outputString;
	}

	private String getUrl() {
		String outputString = "";

		String currentTag = tag.replace("<", "");
		currentTag = currentTag.replace(">", "");
		String[] values = currentTag.split(" ");

		for (String element : values) {
			if (element.contains("url=\"")) {
				outputString = element.substring(element.indexOf("url=\"") + 5);
				outputString = outputString.replace("\"", "");
				outputString = outputString.replace("/", "");
				outputString = outputString.replace("#", "");
			}
		}

		if (outputString.contains(".dae")) {
			String[] bar = outputString.split(".dae");
			outputString = bar[0] + ".dae";
		}

		return outputString;
	}

	private String getType() {
		return tag.substring(1, tag.length() - 1).split(" ")[0];
	}

	public void replaceInstances(LinkedList<XMLObject> XMLList) {
		// System.out.println("Tag: " + this.tag + " Name: " + this.getName() +
		// " URL:" + this.getUrl() + " Content:" + this.content.size());
		for (int i = 0; i < content.size(); i++) {
			XMLObject child = content.get(i);
			// System.out.println(child.tag + " " + child.getUrl());
			if (child.tag.contains("instance_node")) {
				for (XMLObject xmlObject : XMLList) {
					if (child.getUrl().equals(xmlObject.getName())) {
						content.remove(child);
						content.add(new XMLObject(xmlObject, this));
						i--;
					}
				}
			}
		}

		for (XMLObject child : content) {
			// recursion
			child.replaceInstances(XMLList);
		}
	}

	/**
	 * 
	 * This method handles the parsing from .dea to .sdf, different cases
	 * (distinction by xml value, e.g. "instance_geometry") are implemented
	 * 
	 * @param offsetString
	 * 
	 *            The method is working recursive, offsetString is a series of
	 *            tab for spacing
	 * 
	 * @return
	 */
	public String toSDFString(String offsetString) {

		// head is the "main" model xml tag, defining the beginning of the model

		String outputString = "";

		if (tag.equals("head")) {

			outputString += content.getFirst().toSDFString("");
			return outputString;
		}

		// everything that has to do with geometry

		if (getType().toLowerCase().equals("instance_geometry")) {
			outputString = offsetString + "<collision name='" + getUrl() + "'>"
					+ newline + offsetString + tab + "<geometry>" + newline
					+ offsetString + tab + tab + "<mesh>" + newline
					+ offsetString + tab + tab + tab
					+ "<uri>model://roboy/meshes/" + getUrl() + "</uri>"
					+ newline + offsetString + tab + tab + "</mesh>" + newline
					+ offsetString + tab + "</geometry>" + newline
					+ offsetString + "</collision>" + newline;
			outputString += offsetString + "<visual name='" + getUrl() + "'>"
					+ newline + offsetString + tab + "<geometry>" + newline
					+ offsetString + tab + tab + "<mesh>" + newline
					+ offsetString + tab + tab + tab
					+ "<uri>model://roboy/meshes/" + getUrl() + "</uri>"
					+ newline + offsetString + tab + tab + "</mesh>" + newline
					+ offsetString + tab + "</geometry>" + newline
					+ offsetString + "</visual>";
			return outputString;
		}

		// nodes: how to handle children nodes, printed recursive

		if (getType().toLowerCase().equals("node")) {

			if (this.parent.parent == null) {
				outputString = "<?xml version='1.0'?>" + newline
						+ "<sdf version='1.4'>" + newline + "<model name=\""
						+ this.getName() + "\">" + newline
						+ "<static>true</static>" + newline;
				outputString += offsetString + tab + poseString();

				outputString += newline
						+ content.getFirst().toSDFString(offsetString + tab);
				outputString += newline + offsetString + "</model>" + newline
						+ offsetString + "</sdf>";

			} else {

				outputString = offsetString + "<link name='" + getName() + "'>";
				outputString += newline + offsetString + tab + poseString();
				for (XMLObject element : content) {
					outputString += newline
							+ element.toSDFString(offsetString + tab);
				}
				outputString += newline + offsetString + "</link>";
				return outputString;
			}
		}

		return outputString;

	}
}
