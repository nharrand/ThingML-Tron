package testJavaSwing;


import javax.swing.SwingUtilities;

public class TronMonitor {
	public static void main(String[] args){
		
		TronFrame fenetre = new TronFrame();
		fenetre.setVisible(true);
		fenetre.paintSquare(0, 0, 0);
		fenetre.paintSquare(1, 0, 1);
		fenetre.paintSquare(0, 1, 2);
		fenetre.paintSquare(1, 1, 0);
		
	}
	
}