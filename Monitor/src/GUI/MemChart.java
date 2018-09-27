package GUI;

import Data.DataUtils;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.ValueAxis;
import org.jfree.data.time.Millisecond;
import org.jfree.data.time.TimeSeries;
import org.jfree.data.time.TimeSeriesCollection;

public class MemChart extends ChartPanel implements Runnable {


    private static final long serialVersionUID = 1L;
    private static TimeSeries timeSeries;

    public MemChart(String chartContent, String title, String yAxisName) {
        super(createChart(chartContent, title, yAxisName));
    }

    private static JFreeChart createChart(String chartContent, String title, String yAxisName) {
        timeSeries = new TimeSeries(chartContent);
        TimeSeriesCollection timeseriescollection = new TimeSeriesCollection(timeSeries);
        JFreeChart jfreechart = ChartFactory.createTimeSeriesChart(title, null, yAxisName, timeseriescollection, false, false, false);
        jfreechart.getXYPlot().getRangeAxis().setRange(0,100);
//        ValueAxis valueaxis = jfreechart.getXYPlot().getDomainAxis();
//        valueaxis.setRange(0,1);
//        valueaxis.setLowerBound(0);
//        valueaxis.setUpperBound(10);
//        valueaxis.setAutoRange(true);
        return jfreechart;
    }


    int count = 0;

    @Override
    public void run() {
        while (true) {
            try {
                if (count == 20) {
                    timeSeries.delete(0, 0);
                } else {
                    count++;
                }
                timeSeries.add(new Millisecond(), DataUtils.getMemUsage()*100);
                Thread.sleep(1500);
            } catch (InterruptedException e) {
            }
        }
    }


}
