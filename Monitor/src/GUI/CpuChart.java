package GUI;

import Data.DataUtils;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.data.time.Millisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;

public class CpuChart extends ChartPanel implements Runnable {


    private static final long serialVersionUID = 1L;
    private static TimeSeries timeSeries;

    public CpuChart(String chartContent, String title, String yAxisName) {
        super(createChart(chartContent, title, yAxisName));
    }

    private static JFreeChart createChart(String chartContent, String title, String yAxisName) {
        timeSeries = new TimeSeries(chartContent);
        TimeSeriesCollection timeseriescollection = new TimeSeriesCollection(timeSeries);
        JFreeChart jfreechart = ChartFactory.createTimeSeriesChart(title, null, yAxisName, timeseriescollection, false, true, false);
        jfreechart.getXYPlot().getRangeAxis().setRange(0,100);

        return jfreechart;
    }


    int count = 0;

    @Override
    public void run() {
        while (true) {
            try {
                timeSeries.add(new Millisecond(), DataUtils.getCpuUsage()*100);
                if (count == 20) {
                    timeSeries.delete(0, 0);
                } else {
                    count++;
                }
                Thread.sleep(1000);
            } catch (InterruptedException e) {
            }
        }
    }
}
