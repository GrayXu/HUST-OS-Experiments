import java.io.IOException;

import net.tinyos.message.*;
import net.tinyos.packet.*;
import net.tinyos.util.*;

public class SensorClient implements MessageListener {
    private MoteIF moteIF;

    public SensorClient(MoteIF moteIF) {
        this.moteIF = moteIF;
        this.moteIF.registerListener(new SensorMsg(), this);
    }

    public static void main(String[] args) throws Exception {
        String source = null;
        if (args.length == 2) {
            if (!args[0].equals("-comm")) {
                usage();
                System.exit(1);
            }
            source = args[1];
        } else if (args.length != 0) {
            usage();
            System.exit(1);
        }

        PhoenixSource phoenix;

        if (source == null) {
            phoenix = BuildSource.makePhoenix(PrintStreamMessenger.err);
        } else {
            phoenix = BuildSource.makePhoenix(source, PrintStreamMessenger.err);
        }

        MoteIF mif = new MoteIF(phoenix);
        SensorClient serial = new SensorClient(mif);
    }

    public void messageReceived(int to, Message message) {
        SensorMsg msg = (SensorMsg) message;
        int type = msg.get_kind();
        double tempature;
        double humidity;
        double photo;

        switch (type) {
        case 0:
            tempature = -40.1 + 0.01 * msg.get_data();
            System.out.println("Temperature:" + tempature + "℃");
            break;
        case 1:
            humidity = -4 + 0.0405 * msg.get_data() + (-2.8 / 1000000) * msg.get_data() * msg.get_data();
            System.out.println("Humidity:" + humidity + "%");
            break;
        case 2:
            photo = msg.get_data() * 1.5 / 4096 / 10000;
            photo = 0.625 * 1000000 * photo * 1000;
            System.out.println("Photo:" + photo + "Lux");
            break;
        default:
            System.out.println("Unknow data:" + msg.get_data());
            break;
        }

       // try {
         //   Thread.sleep(1000);
        //} catch (Exception e) {}

    }

    private static void usage() {
        System.err.println("usage: SensorClient [-comm <source>]");
    }

}
