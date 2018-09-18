import java.io.Serializable;

public class DiskBlock implements Serializable {

    public static int maxSize = 32;
    private int id;
    private StringBuffer stringBuffer;
    private static final long serialVersionUID = 1L;


    public DiskBlock(){
        stringBuffer = new StringBuffer();
    }

    public DiskBlock(StringBuffer sb){
        stringBuffer = sb;
    }

    public DiskBlock(int id, StringBuffer stringBuffer) {
        this.id = id;
        this.stringBuffer = stringBuffer;
    }

    public int getId() {
        return id;
    }

    public StringBuffer getStringBuffer() {
        return stringBuffer;
    }

    public void setId(int id) {
        this.id = id;
    }

    public void setStringBuffer(StringBuffer stringBuffer) {
        this.stringBuffer = stringBuffer;
    }

    public int getContentSize(){
        return stringBuffer.length();
    }

    public boolean isFull(){
        if (stringBuffer.length()>=maxSize) return true;
        else return false;
    }

    //TODO: recycle!
    public void delete(){
        SuperBlock.getInstance().deleteBlock(id);
    }

    public DiskBlock copy(){
        SuperBlock superBlock = SuperBlock.getInstance();
        int tempPoint = superBlock.nextBlock;
        StringBuffer tempSB = new StringBuffer(stringBuffer);
        DiskBlock tempBlock = new DiskBlock(tempPoint,stringBuffer);
        superBlock.getUsedMap().put(tempPoint, tempBlock);
        superBlock.restCount--;
        superBlock.nextBlock++;
        return tempBlock;
    }
}
