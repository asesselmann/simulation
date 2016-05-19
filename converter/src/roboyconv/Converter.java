package roboyconv;

import java.awt.BorderLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import javax.swing.JButton;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.filechooser.FileNameExtensionFilter;

@SuppressWarnings("serial")
public class Converter extends JPanel implements ActionListener {

	JButton openButton, saveButton;
	JTextArea daeLog;
	// JTextArea sdfLog;
	JFileChooser fc;
	Dae2Sdf myConverter;
	boolean fileOpened = false;

	public static void main(String[] args) {

		// // einheit
		// double[][] m1 = { { 1.0, 0.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0, 0.0 },
		// { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } };
		// // drehung x 90°
		// double[][] m2 = { { 1.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, -1.0, 0.0 },
		// { 0.0, 1.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } };
		// // drehung x 15°
		// double[][] m3 = { { 1.0, 0.0, 0.0, 0.0 },
		// { 0.0, 0.96593, -0.25882, 0.0 },
		// { 0.0, 0.25882, 0.96593, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } };
		// // drehung z 180°
		// double[][] m4 = { { 1.0, 0.0, 0.0, 0.0 }, { 0.0, -1.0, 0.0, 0.0 },
		// { 0.0, 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0, 1.0 } };
		//
		// try {
		// printEuler(transformationMatrix2EulerAngles(m1));
		// printEuler(transformationMatrix2EulerAngles(m2));
		// printEuler(transformationMatrix2EulerAngles(m3));
		// printEuler(transformationMatrix2EulerAngles(m4));
		// } catch (Exception e) {
		// e.printStackTrace();
		// }

		SwingUtilities.invokeLater(new Runnable() {
			public void run() {
				// Turn off metal's use of bold fonts
				UIManager.put("swing.boldMetal", Boolean.FALSE);
				createAndShowGUI();
			}
		});

	}

	public Converter() {

		super(new BorderLayout());
		// Create the log first, because the action listeners
		// need to refer to it.
		daeLog = new JTextArea(25, 75);
		daeLog.setMargin(new Insets(5, 5, 5, 5));
		daeLog.setEditable(false);
		JScrollPane daeLogScrollPane = new JScrollPane(daeLog);

		// sdfLog = new JTextArea(25, 75);
		// sdfLog.setMargin(new Insets(5, 5, 5, 5));
		// sdfLog.setEditable(false);
		// JScrollPane sdfLogScrollPane = new JScrollPane(sdfLog);

		// Create a file chooser
		fc = new JFileChooser();

		// Uncomment one of the following lines to try a different
		// file selection mode. The first allows just directories
		// to be selected (and, at least in the Java look and feel,
		// shown). The second allows both files and directories
		// to be selected. If you leave these lines commented out,
		// then the default mode (FILES_ONLY) will be used.
		//
		// fc.setFileSelectionMode(JFileChooser.DIRECTORIES_ONLY);
		// fc.setFileSelectionMode(JFileChooser.FILES_AND_DIRECTORIES);

		// Create the open button. We use the image from the JLF
		// Graphics Repository (but we extracted it from the jar).
		openButton = new JButton("Open a File...");
		openButton.addActionListener(this);

		// Create the save button. We use the image from the JLF
		// Graphics Repository (but we extracted it from the jar).
		saveButton = new JButton("Export to File...");
		saveButton.addActionListener(this);

		// For layout purposes, put the buttons in a separate panel
		JPanel buttonPanel = new JPanel(); // use FlowLayout
		buttonPanel.add(openButton);
		buttonPanel.add(saveButton);

		// Add the buttons and the log to this panel.
		add(buttonPanel, BorderLayout.PAGE_START);
		add(daeLogScrollPane, BorderLayout.CENTER);
		// add(sdfLogScrollPane, BorderLayout.EAST);
	}

	@Override
	public void actionPerformed(ActionEvent e) {

		// Handle open button action.
		if (e.getSource() == openButton) {
			fc.setFileFilter(new FileNameExtensionFilter(
					"Collada files (.dae)", "dae"));
			int returnVal = fc.showOpenDialog(Converter.this);

			if (returnVal == JFileChooser.APPROVE_OPTION) {

				File file = fc.getSelectedFile();
				myConverter = new Dae2Sdf(file, daeLog);
				fileOpened = true;
			}
			daeLog.setCaretPosition(daeLog.getDocument().getLength());

			// Handle save button action.
		} else if (e.getSource() == saveButton) {
			if (fileOpened) {
				fc.setFileFilter(new FileNameExtensionFilter("SDF file (.sdf)",
						"sdf"));
				fc.setSelectedFile(new File("model.sdf"));
				int returnVal = fc.showSaveDialog(Converter.this);
				if (returnVal == JFileChooser.APPROVE_OPTION) {
					File file = fc.getSelectedFile();
					// for writing into target file
					try {
						PrintWriter writer = new PrintWriter(file, "UTF-8");
						writer.append(myConverter.createSDF());
						writer.flush();
						writer.close();
					} catch (FileNotFoundException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					} catch (UnsupportedEncodingException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					// This is where a real application would save the file.
					// sdfLog.append("Saving: " + file.getName() + "." +
					// newline);
				} else {
					// sdfLog.append("Save command cancelled by user." +
					// newline);
				}
				daeLog.setCaretPosition(daeLog.getDocument().getLength());
			}
		}
	}

	private static void createAndShowGUI() {
		// Create and set up the window.
		JFrame frame = new JFrame("Roboy Dae to SDF converter");
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

		// Add content to the window.
		frame.add(new Converter());

		// Display the window.
		frame.pack();
		frame.setVisible(true);
	}
}
