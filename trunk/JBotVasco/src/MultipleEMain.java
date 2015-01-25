import java.io.File;

import javax.swing.JFrame;

import simulation.util.Arguments;
import evolutionaryrobotics.JBotEvolver;
import gui.evolution.EvolutionGui;

public class MultipleEMain extends Thread {
	public static String FOLDER = "to_run";
	public static String CONFIG_FILES_START_NAME = "faultDetection_simple_";
	public static String EXTENSION = ".conf";

	public static String fileToExclude = "faultDetection_base.conf";

	private String filename = null;

	public static void main(String[] args) throws Exception {
		File folder = new File(FOLDER);
		File[] listOfFiles = folder.listFiles();

		for (int i = 0; i < listOfFiles.length; i++) {
			if (listOfFiles[i].isFile() && 
				listOfFiles[i].getName().startsWith(CONFIG_FILES_START_NAME) && 
				!listOfFiles[i].equals(fileToExclude)) {
				System.out.println("Created EMain Thread! ["+i+"]");
				new MultipleEMain(FOLDER + "/" + listOfFiles[i].getName()).start();
			}
		}

	}

	public MultipleEMain(String filename) {
		this.filename = filename;
	}

	public void createEvolverInstance() {
		try {
			String[] args = new String[] { filename };
			JBotEvolver jBotEvolver = new JBotEvolver(args);
			EvolutionGui evo = new EvolutionGui(jBotEvolver, new Arguments(""));
			JFrame frame = new JFrame();
			frame.add(evo);
			frame.setSize(1000, 600);
			frame.setVisible(true);
			frame.setLocationRelativeTo(null);
			frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
			evo.init();
			evo.executeEvolution();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void run() {
		createEvolverInstance();
	}
}
