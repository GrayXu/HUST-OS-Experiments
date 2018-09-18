package GUI;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;

import org.jb2011.lnf.beautyeye.BeautyEyeLNFHelper;
import org.jb2011.lnf.beautyeye.ch3_button.BEButtonUI;

import java.awt.*;
import java.awt.event.MouseEvent;

public class MainGui {

    JPanel jMainPanel;
    private JTabbedPane tabPanel;
    private JPanel infoPanel;
    private JTextArea infoTA;
    private JPanel chartPanel;
    private JPanel graphPanel;
    private JScrollPane scrollPanel;
    private JTable jTable;
    private JFrame frame;
    private int delete_row_id = -1;
    private JPopupMenu jPopupMenu;

    private static int SCREEN_WIDTH;
    private static int SCREEN_HEIGHT;

    public static void main(String[] args) {

        try {
            org.jb2011.lnf.beautyeye.BeautyEyeLNFHelper.launchBeautyEyeLNF();
            BeautyEyeLNFHelper.frameBorderStyle = BeautyEyeLNFHelper.FrameBorderStyle.translucencyAppleLike;
            org.jb2011.lnf.beautyeye.BeautyEyeLNFHelper.launchBeautyEyeLNF();
        } catch (Exception e) {
            e.printStackTrace();
        }
        UIManager.put("RootPane.setupButtonVisible", false);
        setFontForBeautyEye();

        Dimension dim = Toolkit.getDefaultToolkit().getScreenSize();
        SCREEN_WIDTH = dim.width;
        SCREEN_HEIGHT = dim.height;

        JFrame frame = new JFrame("Monitor");
        MainGui mainGui = new MainGui();
        frame.setContentPane(mainGui.jMainPanel);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(500,600);
        frame.setVisible(true);

        mainGui.init();
    }

    private void init(){
        infoTA.setOpaque(false);
        createPopupMenu();
        scrollPanel.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
    }

    private void initChart(){
        jTable.getTableHeader().setReorderingAllowed(false);
        DefaultTableModel tableModel = new DefaultTableModel();
        jTable.setModel(tableModel);

        jTable.addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                mouseRightButtonClick(e, jTable);
            }
        });

    }

    //判断是否为鼠标的BUTTON3按钮，BUTTON3为鼠标右键
    private void mouseRightButtonClick(MouseEvent evt, JTable jTable) {

        if (evt.getButton() == java.awt.event.MouseEvent.BUTTON3) {
            //通过点击位置找到点击为表格中的行
            int focusedRowIndex = jTable.rowAtPoint(evt.getPoint());
            if (focusedRowIndex == -1) {
                return;
            }
            delete_row_id = focusedRowIndex;
            //将表格所选项设为当前右键点击的行
            jTable.setRowSelectionInterval(focusedRowIndex, focusedRowIndex);
            //弹出菜单
            if (jPopupMenu != null) {
                jPopupMenu.show(jTable, evt.getX(), evt.getY());
            }
        }
    }

    //右键菜单
    private void createPopupMenu() {
        jPopupMenu = new JPopupMenu();

        JMenuItem delMenuItem = new JMenuItem();
        delMenuItem.setText("Test1");
        delMenuItem.addActionListener(evt -> {
//            String key = (String) jTable.getValueAt(delete_row_id, 0);
//            try {
//                DataBase.getInstance().deleteRow(PANEL_MODE, key);
//                tableModel.removeRow(delete_row_id);
//            } catch (SQLException e) {
//                e.printStackTrace();
//                noticeMsg("删除失败");
//            }
        });


        JMenuItem addMenuItem = new JMenuItem();
        addMenuItem.setText("Kill");
//        addMenuItem.addActionListener(evt -> setAddRowDialog());

        jPopupMenu.add(delMenuItem);
        jPopupMenu.add(addMenuItem);
    }

    /**
     * 修复字体发虚
     */
    private static void setFontForBeautyEye() {
        String[] DEFAULT_FONT = new String[]{
                "Table.font"
                , "TableHeader.font"
                , "CheckBox.font"
                , "Tree.font"
                , "Viewport.font"
                , "ProgressBar.font"
                , "RadioButtonMenuItem.font"
                , "ToolBar.font"
                , "ColorChooser.font"
                , "ToggleButton.font"
                , "Panel.font"
                , "TextArea.font"
                , "Menu.font"
                , "TableHeader.font"
                , "OptionPane.font"
                , "MenuBar.font"
                , "Button.font"
                , "Label.font"
                , "PasswordField.font"
                , "ScrollPane.font"
                , "MenuItem.font"
                , "ToolTip.font"
                , "List.font"
                , "EditorPane.font"
                , "Table.font"
                , "TabbedPane.font"
                , "RadioButton.font"
                , "CheckBoxMenuItem.font"
                , "TextPane.font"
                , "PopupMenu.font"
                , "TitledBorder.font"
                , "ComboBox.font"
        };

        for (String aDEFAULT_FONT : DEFAULT_FONT) {
            UIManager.put(aDEFAULT_FONT, new Font("微软雅黑", Font.PLAIN, 12));
        }
    }

    /**
     * 窗口放置桌面中央
     *
     * @param c component waited to be reset
     */
    private static void setCenter(Component c) {
        c.setLocation((SCREEN_WIDTH - c.getWidth()) / 2, (SCREEN_HEIGHT - c.getHeight()) / 2);
        c.setVisible(true);
    }
}
