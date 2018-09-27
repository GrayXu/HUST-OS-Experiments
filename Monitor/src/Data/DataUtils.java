package Data;

import java.io.*;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

public class DataUtils {

    public static String getValue(String path, String key) {
        BufferedReader reader = null;
        try {
            reader = new BufferedReader(new FileReader(path));
            String line = "";
            String returnStr = null;
            while ((line = reader.readLine()) != null) {
                if (line.contains(key)) {
                    returnStr = line.split(":")[1];
                    break;
                }
            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

            return returnStr;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return null;
    }

    public static String getAllString(String path) {
        StringBuilder sb = new StringBuilder();
        BufferedReader reader = null;
        try {
            reader = new BufferedReader(new FileReader(path));
            String line;

            while ((line = reader.readLine()) != null) {
                sb.append(line);
            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return String.valueOf(sb);
    }

    public static String getPartitions() {
        BufferedReader reader = null;
        int blocks = 0;
        try {
            reader = new BufferedReader(new FileReader("/proc/partitions"));
            String line;
            int count = 0;
            while ((line = reader.readLine()) != null) {
                if (count >= 2) {
//                    System.out.println(line.split(" ")[2]);
                    blocks += Integer.valueOf(
                            line.replaceAll("\\s+", ",").split(",")[3]
                    );
                }
                count++;
            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return String.valueOf("\t" + blocks + " blocks");
    }


    public static float getMemUsage() {
        BufferedReader reader = null;
        float result = 0;
        float use = 0;
        float free = 0;
        try {
            reader = new BufferedReader(new FileReader("/proc/meminfo"));
            String line;
            int count = 0;
            while ((line = reader.readLine()) != null) {

                if (count == 0) {
                    line = line.replaceAll("\\s+", ",");
                    String[] strs = line.split(",");
                    use = Float.parseFloat(strs[strs.length - 2]);
                } else if (count == 1) {

                } else if (count == 2) {
                    line = line.replaceAll("\\s+", ",");
                    String[] strs = line.split(",");
                    free = Float.parseFloat(strs[strs.length - 2]);
                } else {
                    result = 1 - free / use;
                    break;
                }

                count++;
            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
//            System.out.println(use+" "+free+" "+result);
            return result;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return 0;

    }

    public static float getCpuUsage() {
//        System.out.println("in get cpu usage func");
        BufferedReader reader = null;
        float result = 0;
        float total = 0;
        float idle = 0;
        try {
            reader = new BufferedReader(new FileReader("/proc/stat"));
            String line;
            while ((line = reader.readLine()) != null) {
                line = line.replaceAll("\\s+", ",");
                String[] data = line.split(",");
                for (int i = 1; i <= 9; i++) {
                    total += Integer.valueOf(data[i]);
                }

                idle = Integer.valueOf(data[4]);
//                System.out.print(total + " " + idle + " ");
                break;
            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

            //get delta
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            reader = new BufferedReader(new FileReader("/proc/stat"));

            while ((line = reader.readLine()) != null) {
                line = line.replaceAll("\\s+", ",");
                String[] data = line.split(",");
                float newTotal = 0;
                for (int i = 1; i <= 9; i++) {
                    newTotal += Integer.valueOf(data[i]);
                }
                total = newTotal - total;
                idle = Integer.valueOf(data[4]) - idle;
//                System.out.print(total + " " + idle);
                break;

            }

            try {
                reader.close();
            } catch (IOException e) {
                e.printStackTrace();
            }

            result = 1 - idle / total;
//            System.out.println(" " + result);
            return result;

        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }

        return 0;
    }

    public static Vector<Vector<String>> getProcessData() {
        Vector<Vector<String>> data = new Vector<>();

        File file = new File("/proc");
        File[] fileList = file.listFiles();

        for (File f : fileList) {
            try {
                Integer.parseInt(f.getName());

                Vector<String> vector = new Vector<>();
                vector.add(f.getName());
                data.add(vector);
            } catch (NumberFormatException e) {

            }
        }

        //to save cpu time info to calculate process cpu usage
        HashMap<String, float[]> map = new HashMap<>();
//        for (Vector<String> vec : data) {
        for (int i = 0; i < data.size(); i++) {
            Vector<String> vec = data.get(i);
            String path = "/proc/" + vec.get(0) + "/status";
            String name = getValue(path, "Name");
            if (name == null){//removed
                data.remove(vec);
                i--;
                continue;
            }

            String ppid = getValue(path, "PPid");
            ppid = ppid.replaceAll("\t","");

            vec.add(name);
            vec.add(ppid);

            String vmrss = getValue(path, "VmRSS");
//            vec.add(priority);
            if (vmrss != null) {
                vmrss = vmrss.replaceAll("\t", "");
                vmrss = vmrss.replaceAll("\\s+", "");
                vmrss = vmrss.replaceAll("kB", "");
                vec.add(vmrss);
            } else {
                vec.add("");
            }

            String priority = getAllString("/proc/" + vec.get(0) + "/stat").replaceAll("\\s+",",").split(",")[17];
            if (priority != null){
                priority = priority.replaceAll("\t", "");
                vec.add(priority);
            }else{
                vec.add("");
            }



            path = "/proc/" + vec.get(0) + "/stat";
            float[] formmerTimes = getProcessCpuTime(path);

            map.put(vec.get(0), formmerTimes);


        }

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        for (int i = 0; i < data.size(); i++) {
            Vector<String> vec = data.get(i);
            String path = "/proc/" + vec.get(0) + "/stat";
            float[] latterTimes = getProcessCpuTime(path);
            if (latterTimes == null) {
                data.remove(i);
                i--;
                continue;
            }

            float[] formmerTimes = map.get(vec.get(0));

            float percentage = (latterTimes[0] - formmerTimes[0]) / (latterTimes[1] - formmerTimes[1]);
            percentage = Math.round(percentage*100*10)/10;
            vec.add(String.valueOf(percentage));
        }


        return data;
    }


    private static float[] getProcessCpuTime(String processStatPath) {
        String temp = getAllString(processStatPath);
        if (temp.equals("") || temp == null){
            return null;
        }
        String[] processStr = temp.replaceAll("\\s+", ",").split(",");
        float processTime = Integer.valueOf(processStr[13]) +
                Integer.valueOf(processStr[14]) +
                Integer.valueOf(processStr[15]);

        BufferedReader reader = null;
        float toatlTime = 0;

        try {
            reader = new BufferedReader(new FileReader("/proc/stat"));
            String line = reader.readLine();
            line = line.replaceAll("\\s+", ",");
            String[] data = line.split(",");
            for (int i = 1; i <= 9; i++) {
                toatlTime += Integer.valueOf(data[i]);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            return null;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }


        float[] times = new float[]{processTime, toatlTime};

        return times;

    }

}
