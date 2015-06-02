
package JavaMonitor;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Point;
import java.util.ArrayList;
import java.util.Iterator;

import javax.swing.JPanel;

public class PaintingPanel extends JPanel{
	
	private ArrayList<SnakeCell> Snakes;
	private Color[] colors = new Color[3];
	
	public PaintingPanel () {
		super();
		
		colors[0] = new Color(255, 0, 0);
		colors[1] = new Color(0, 0, 255);
		colors[2] = new Color(0, 255, 0);
		Snakes = new ArrayList<SnakeCell>();
	}
	
	public void addCell(int x, int y, int id) {
		this.Snakes.add(new SnakeCell(x,y,id));
	}
	
	@Override
	public void paint(Graphics g) {
		super.paint(g);
		
		g.setColor(Color.black);
		g.fillRect(0, 0, 424, 524);
		g.setColor(Color.white);
		g.drawRect(1, 1, 421, 521);
		
		
		for (SnakeCell snakeCell : Snakes) {
			g.setColor(colors[snakeCell.id]);
			g.fillRect(snakeCell.getC().x * 10 + 2, snakeCell.getC().y * 10 + 2, 9, 9);
		}
	}
	
	
	

}
