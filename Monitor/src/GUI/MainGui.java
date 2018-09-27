package GUI;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import javax.swing.table.TableModel;
import javax.swing.table.TableRowSorter;

import Data.DataUtils;
import org.jb2011.lnf.beautyeye.BeautyEyeLNFHelper;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.StandardChartTheme;


import java.awt.*;
import java.awt.event.MouseEvent;
import java.io.*;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Vector;

public class MainGui {

    JPanel jMainPanel;
    private JTabbedPane tabPanel;
    private JPanel infoPanel;
    private JTextArea infoTA;
    private JPanel tablePanel;
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
        frame.setSize(600, 600);
        frame.setVisible(true);
        frame.setResizable(false);
        mainGui.init();
    }

    private void init() {
        infoTA.setOpaque(false);
        createPopupMenu();
        scrollPanel.setVerticalScrollBarPolicy(JScrollPane.VERTICAL_SCROLLBAR_ALWAYS);
        initInfoPanel();
        initTable();
        initGraph();
    }


    private void initGraph() {
        StandardChartTheme standardChartTheme = new StandardChartTheme("CN");
        standardChartTheme.setExtraLargeFont(new Font("微软雅黑", Font.BOLD, 20));
        standardChartTheme.setRegularFont(new Font("微软雅黑", Font.PLAIN, 15));
        standardChartTheme.setLargeFont(new Font("微软雅黑", Font.PLAIN, 15));

        ChartFactory.setChartTheme(standardChartTheme);
        MemChart memChart = new MemChart("Memory Usage", "Memory Usage", "%");
        CpuChart cpuChart = new CpuChart("Memory Usage", "CPU Usage", "%");
        graphPanel.add(memChart, BorderLayout.NORTH);
        graphPanel.add(cpuChart, BorderLayout.SOUTH);

        memChart.setPreferredSize(new Dimension(200, 200));
        cpuChart.setPreferredSize(new Dimension(200, 200));

//        memChart.setSize(100,100);
//        cpuChart.setSize(100,100);
        (new Thread(memChart)).start();
        (new Thread(cpuChart)).start();
    }


    private void initInfoPanel() {
        infoTA.setText("");
        infoTA.append("Hostname:\t" + DataUtils.getAllString("/proc/sys/kernel/hostname") + "\n");
        infoTA.append("Version\t:\t" + DataUtils.getAllString("/proc/version").split("b")[0] + "\n");
        infoTA.append("CPU\t:\t" + DataUtils.getValue("/proc/cpuinfo", "model name") + "\n");
        infoTA.append("Cores\t:\t" + DataUtils.getValue("/proc/cpuinfo", "cpu cores") + "\n");
        infoTA.append("L3 Cache:\t" + DataUtils.getValue("/proc/cpuinfo", "cache size") + "\n");
        infoTA.append("Memory\t:" + DataUtils.getValue("/proc/meminfo", "MemTotal") + "\n");
        infoTA.append("Disk\t:" + DataUtils.getPartitions() + "\n");
    }

    private void initTable() {
        jTable.getTableHeader().setReorderingAllowed(false);

        Vector<String> columns = new Vector<>(Arrays.asList("Pid", "Name", "PPid", "Memory(kB)", "Priority", "CPU(%)"));
        Vector<Vector<String>> data = DataUtils.getProcessData();

        DefaultTableModel tableModel = new DefaultTableModel(data, columns) {
            @Override
            public boolean isCellEditable(int i, int i1) {
                return false;
            }
        };
        jTable.setModel(tableModel);

        RowSorter<TableModel> sorter = new TableRowSorter<TableModel>(tableModel);

        Comparator<Object> comparator = new Comparator<Object>() {
            @Override
            public int compare(Object o, Object t1) {
                try {
                    int a = Integer.parseInt(o.toString());
                    int b = Integer.parseInt(t1.toString());
                    return a - b;
                } catch (NumberFormatException e1) {
                    try {

                        float a = Float.parseFloat(o.toString());
                        float b = Float.parseFloat(t1.toString());

                        if (a > b) return 1;
                        else if (a < b) return -1;
                        else return 0;

                    } catch (Exception e2) {
                        return String.valueOf(o).compareTo(String.valueOf(t1));
                    }

                }
            }
        };

        ((TableRowSorter<TableModel>) sorter).setComparator(0, comparator);
        ((TableRowSorter<TableModel>) sorter).setComparator(2, comparator);
        ((TableRowSorter<TableModel>) sorter).setComparator(3, comparator);
        ((TableRowSorter<TableModel>) sorter).setComparator(4, comparator);
        ((TableRowSorter<TableModel>) sorter).setComparator(5, comparator);
        jTable.setRowSorter(sorter);

        jTable.addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                mouseRightButtonClick(e, jTable);
            }
        });

        (new Thread(new TableUpdater(tableModel, jTable))).start();
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

        JMenuItem addMenuItem = new JMenuItem();
        addMenuItem.setText("Kill");

        addMenuItem.addActionListener(evt -> {
            String pid = (String) jTable.getValueAt(delete_row_id, 0);
            killProcess(pid);
        });

        jPopupMenu.add(addMenuItem);
    }

    private void killProcess(String pid) {
        String command = "bash run.sh";
        //chage file to fit the requirement

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter("run.sh"));

            writer.write("sudo -S kill "+pid+" << EOF \n" +
                    "XIANG1569348\n" +
                    "EOF");

            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }


        new Thread(() -> {
            try {
                System.out.println(command);
                Process ps = Runtime.getRuntime().exec(command);
                int com = ps.waitFor();
                if (com == 0) {
                    System.out.println("success");
                } else {
                    System.out.println("fail");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }).start();
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
