package GUI;

import Data.DataUtils;

import javax.swing.*;
import javax.swing.table.DefaultTableModel;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Vector;

public class TableUpdater implements Runnable {

    JTable table;
    DefaultTableModel tableModel;

    public TableUpdater(DefaultTableModel inTableModel, JTable jTable) {
        table = jTable;
        tableModel = inTableModel;
    }

    int count = 0;

    @Override
    public void run() {
        while (true) {
            System.out.println("update!");
            Vector<Vector<String>> data = DataUtils.getProcessData();

            Vector<Vector<String>> oldData = tableModel.getDataVector();
            updateDataVec(oldData, data);
            tableModel.fireTableDataChanged();


            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }

    }

    private void updateDataVec(Vector<Vector<String>> oldData, Vector<Vector<String>> newData){
        ArrayList<String> pidList = new ArrayList<>();
        for(Vector<String> oldVec:oldData){
            pidList.add(oldVec.get(0));
        }

        for(Vector<String> oldVec:oldData){
            String pid = oldVec.get(0);
            for (Vector<String> newVec:newData){
                if (pid.equals(newVec.get(0))){
                    for (int i = 1; i < oldVec.size(); i++) {
                        oldVec.set(i,newVec.get(i));
                    }
                    newData.remove(newVec);
                    pidList.remove(pid);
                    break;
                }
            }
        }
        if (!newData.isEmpty()){
            for (Vector<String> newVec:newData){
                oldData.add(newVec);
            }
        }

        ArrayList<Vector<String>> wait2delVecs = new ArrayList<>();
        if (!pidList.isEmpty()){
            for(Vector<String> oldVec:oldData){
                if (pidList.contains(oldVec.get(0))){
                    wait2delVecs.add(oldVec);
                }
            }
        }
        for (Vector<String> delVec:wait2delVecs){
            oldData.remove(delVec);
        }
    }

}
