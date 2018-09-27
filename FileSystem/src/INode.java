import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;

/**
 * One file links muti-blocks
 */
public class INode implements Serializable {

    //TODO: authority & history
    private static final long serialVersionUID = 1L;

    private String name;

    public INode(String name) {
        this.name = name;
    }

    public INode(String name, ArrayList<DiskBlock> blocklist) {
        this(name);
        this.blocklist = blocklist;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    private ArrayList<DiskBlock> blocklist = new ArrayList<DiskBlock>();

    public ArrayList<DiskBlock> getBlocklist() {
        return blocklist;
    }

    public void setBlocklist(ArrayList<DiskBlock> blocklist) {
        this.blocklist = blocklist;
    }

    /**
     * delete all blocks
     */
    public void remove() {
        for (DiskBlock one : blocklist) {
            SuperBlock.getInstance().deleteBlock(one.getId());
        }
        blocklist.clear();
    }

    /**
     * copy file
     *
     * @param file
     * @return
     */
    public static INode copy(INode file) {
        ArrayList<DiskBlock> newlist = new ArrayList<>();
        for (DiskBlock one : file.getBlocklist()) {
            newlist.add(one.copy());
        }
        return new INode(file.getName(), newlist);
    }

    /**
     * add a new block to save big files
     *
     * @param d
     */
    public void addBlock(DiskBlock d) {
        this.blocklist.add(d);
    }

    /**
     * get file size( actually it's get blocks' size occupied by this file  )
     *
     * @return file size
     */
    public int getSize() {
        return blocklist.size() * DiskBlock.maxSize;
    }

    public void reWrite(StringBuffer sb) {
        this.remove();
        int blockCount = (int) Math.ceil( (float)(sb.length())/DiskBlock.maxSize);
        for (int i = 0; i < blockCount-1; i++) {
            String s = sb.substring(i*32,i*32+31);
            DiskBlock diskBlock = new DiskBlock(new StringBuffer(s));
            blocklist.add(diskBlock);
            SuperBlock.getInstance().addBlock(diskBlock);
        }
        DiskBlock diskBlock = new DiskBlock(new StringBuffer(sb.substring((blockCount-1)*32)));
        blocklist.add(diskBlock);
        SuperBlock.getInstance().addBlock(diskBlock);
    }

    /**
     * get all string storing in this file
     *
     * @return string buffer
     */
    public StringBuffer read() {
        StringBuffer stringBuffer = new StringBuffer();
        Iterator<DiskBlock> iterator = this.blocklist.iterator();
        while (iterator.hasNext()) {
            DiskBlock diskBlock = iterator.next();
            stringBuffer.append(diskBlock.getStringBuffer());
        }
        return stringBuffer;
    }

}
