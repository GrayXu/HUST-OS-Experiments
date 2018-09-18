import java.io.Serializable;
import java.util.HashMap;
import java.util.Iterator;

public class SuperBlock implements Serializable {

    private static SuperBlock fileSys = new SuperBlock();//hungry singleton
    private static final long serialVersionUID = 1L;

    //it's just a only-add number
    int nextBlock = 0;//points the next empty block id

    public void setUsedMap(HashMap<Integer, DiskBlock> usedMap) {
        this.usedMap = usedMap;
    }

    public HashMap<Integer, DiskBlock> getUsedMap() {
        return usedMap;
    }

    private HashMap<Integer, DiskBlock> usedMap;
    public int restCount = 100;

    public void setRestCount(int restCount) {
        this.restCount = restCount;
    }

    public int getRestCount() {
        return restCount;
    }

    public SuperBlock() {
        usedMap = new HashMap<>();
    }

    public static SuperBlock getInstance() {
        return fileSys;
    }

    /**
     * add a disk block into a dist instance
     *
     * @param diskBlock
     */
    public void addBlock(DiskBlock diskBlock) {
        diskBlock.setId(nextBlock);
        usedMap.put(nextBlock, diskBlock);
        restCount--;
        nextBlock++;
    }

    //TODO: nextBlock as a pointer should get this info to recycle space
    public void deleteBlock(int id) {
        restCount++;
        usedMap.remove(id);
    }

    public void printUsedIDs() {
        Iterator<Integer> it = usedMap.keySet().iterator();
        while (it.hasNext()) {
            System.out.print(usedMap.get(it.next()).getId() + ",");
        }
        System.out.println("");
    }


}
