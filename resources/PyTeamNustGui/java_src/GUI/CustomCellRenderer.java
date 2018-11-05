package GUI;

import java.awt.Color;
import java.awt.Component;

import javax.swing.JTable;
import javax.swing.table.DefaultTableCellRenderer;

public class CustomCellRenderer extends DefaultTableCellRenderer 
{
private static final long serialVersionUID = 6703872492730589499L;

    public Component getTableCellRendererComponent(JTable table, Object value, boolean isSelected, boolean hasFocus, int row, int column)
    {
        Component cellComponent = super.getTableCellRendererComponent(table, value, isSelected, hasFocus, row, column);

        if( ((String)table.getValueAt(row, column)).startsWith("*")){
            cellComponent.setBackground(new Color(0, 230, 0)) ;
        } else{
            cellComponent.setBackground(Color.LIGHT_GRAY);
        }
        return cellComponent;
    }
}