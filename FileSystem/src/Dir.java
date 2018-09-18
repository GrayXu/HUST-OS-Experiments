import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Iterator;

public class Dir implements Serializable {

    private static final long serialVersionUID = 1L;

    private String name;

    private Dir fatherDir;
//    private String owner;

    private ArrayList<Dir> dirlist = new ArrayList<>();
    private ArrayList<INode> filelist = new ArrayList<>();

    public Dir(String name, ArrayList<Dir> dirlist, ArrayList<INode> filelist) {
        this.name = name;
        this.dirlist = dirlist;
        this.filelist = filelist;
    }

    public ArrayList<Dir> getDirlist() {
        return dirlist;
    }

    public ArrayList<INode> getFilelist() {
        return filelist;
    }

//    public void setOwner(String owner) {
//        this.owner = owner;
//    }

    public static Object cloneObject(Object obj) {
        Object objx = null;
        try {
            ByteArrayOutputStream byteOut = new ByteArrayOutputStream();
            ObjectOutputStream out = new ObjectOutputStream(byteOut);
            out.writeObject(obj);
            ByteArrayInputStream byteIn = new ByteArrayInputStream(byteOut.toByteArray());
            ObjectInputStream in = new ObjectInputStream(byteIn);
            objx = in.readObject();

        } catch (Exception e) {
            System.out.println("Failed...");
        }
        return objx;

    }

    public Dir copy(Dir save) {
        Dir temp = copyRecur(this, save);
        getDirlist().add(temp);
        return temp;
    }

    private Dir copyRecur(Dir father, Dir now) {
        if (now.isEmpty()) {
            return new Dir(now.getName());
        } else {
            //copy all files in this layer
            ArrayList<INode> files = now.getFilelist();
            ArrayList<INode> newFiles = new ArrayList<>();
            for (INode oneFile : files) {
                String filename = oneFile.getName();
                INode file = oneFile;

                ArrayList<DiskBlock> blockList = file.getBlocklist();
                ArrayList<DiskBlock> newBlocks = new ArrayList<>();
                for (DiskBlock oneBlock : blockList) {
                    DiskBlock tempSave = oneBlock.copy();
                    newBlocks.add(tempSave); // copy
                }
                INode newFile = new INode(filename, newBlocks); // 重建新文件
                newFiles.add(newFile);
            }

            //copy all dirs in this layer
            ArrayList<Dir> dirs = now.getDirlist();
            ArrayList<Dir> newDirs = new ArrayList<>();
            for (Dir oneDir : dirs) {
                newDirs.add(copyRecur(now, oneDir));
            }

            //create new father-dir for this layer
            Dir realNew = new Dir(now.getName(), newDirs, newFiles);
            for (Dir oneDir : newDirs) {
                oneDir.setFatherDir(realNew); // 设置父目录
            }
            now.setFatherDir(father);
            return realNew;
        }
    }

    public boolean isEmpty() {
        if (this.dirlist.isEmpty() && this.filelist.isEmpty()) {
            return true;
        } else
            return false;
    }

    public Dir(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public Dir getFatherDir() {
        return fatherDir;
    }

    public void setFatherDir(Dir fatherDir) {
        this.fatherDir = fatherDir;
    }

    /**
     * get full size in this directory
     *
     * @return
     */
    public static int getSize(Dir now) {
        if (now.isEmpty()) {
            return 0;
        } else {
            int sum = 0;
            ArrayList<INode> files = now.getFilelist();

            for (INode file : files) {
                sum += file.getSize();
            }

            ArrayList<Dir> dirs = now.getDirlist();
            for (Dir kid : dirs) {
                sum += getSize(kid);
            }
            return sum;
        }
    }

    /**
     */
    public static void write(String path, Dir now) {

        if (now.isEmpty()) {
            return;
        } else {
            ArrayList<INode> files = now.getFilelist();
            for (INode file : files) {
                File f = new File(path + "/" + now.getName(), file.getName());
                if (!f.exists()) {
                    try {
                        f.createNewFile();
                        StringBuffer sb = new StringBuffer();
                        for (DiskBlock block : file.getBlocklist()) {
                            sb.append(block.getStringBuffer());
                        }
                        BufferedWriter bw = new BufferedWriter(new FileWriter(f));
                        bw.write(sb.toString());
                        bw.flush();
                        bw.close();
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            ArrayList<Dir> dirs = now.getDirlist();
            for (Dir kid : dirs) {
                String tempPath = path + "/" + now.getName() + "/" + kid.getName();
                File f = new File(tempPath);
                f.mkdirs();
                write(tempPath, kid);
            }
        }
    }

//    remove
//    public static void remove(Dir now) {
//        if (now.isEmpty()) {
//            return;
//        } else {
//            HashMap<String, INode> files = now.getFilelist();
//            for (Entry<String, INode> oneFile : files.entrySet()) {
//                INode file = oneFile.getValue();
//                file.remove();
//            }
//            files.clear();
//            HashMap<String, Dir> dirs = now.getDirlist();
//            for (Entry<String, Dir> onedir : dirs.entrySet()) {
//                Dir kid = onedir.getValue();
//                remove(kid);
//            }
//            dirs.clear();
//        }
//    }

    /**
     * print all file and dir in this directory
     */
    public void list() {
        Iterator<INode> a = filelist.iterator();
        while (a.hasNext()) {
            INode inst = a.next();
            System.out.print(inst.getName() + " ,size:" + inst.getSize() + "\n");
        }
        Iterator<Dir> b = dirlist.iterator();
        while (b.hasNext()) {
            Dir inst = b.next();
            System.err.print(inst.getName() + " ,size:" + getSize(inst) + "\n");
        }
        if (filelist.isEmpty() && dirlist.isEmpty()) System.out.println("no file");
        else System.out.println();
    }

    /**
     * get a child-dir
     */
    public Dir getDir(String name) {
        Iterator<Dir> b = dirlist.iterator();
        while (b.hasNext()) {
            Dir inst = b.next();
            if (inst.name.equals(name)) return inst;
        }
        return null;
    }

    /**
     * get father dir
     *
     * @return
     */
    public Dir cdReturn() {
        return fatherDir;
    }

    public Dir cd(String name) {
        Iterator<Dir> i = dirlist.iterator();
        while (i.hasNext()){
            Dir dir = i.next();
            if (name.equals(dir.name)){
                return dir;
            }
        }
        return null;
    }

    /**
     * add a dir in dirlist
     *
     * @param a instance of Dir
     */
    public void addDir(Dir a) {
        if (dirlist.contains(a))
            System.out.println("same directory name has already exist");
        else
            dirlist.add(a);
    }

    /**
     * delete a dir
     *
     * @param dirname String
     */
    public void deleteDir(String dirname) {
//        dirlist.remove(dirname);
        dirlist.remove(this.getDir(dirname));
    }

    /**
     * get file by filename
     *
     * @param filename 文件名，为String型
     */
    public INode getFile(String filename) {
        Iterator<INode> b = filelist.iterator();
        while (b.hasNext()) {
            INode inst = b.next();
            if (inst.getName().equals(filename)) return inst;
        }
        return null;
    }

    /**
     * Add a new file to this dir
     *
     * @param a
     */
    public void addFile(INode a) {
        if (filelist.contains(a))
            System.out.println("对不起，该目录下已经存在同名文件，操作失败");
        else
            filelist.add(a);
    }

    public void addFile(String filename, StringBuffer realcontent) {

        int point = 0;
        ArrayList<DiskBlock> newFileBlocklist = new ArrayList<>();

        ArrayList<Integer> thenw = new ArrayList<Integer>();
        int blocksize = DiskBlock.maxSize;
        int sizecount = 0;
        if (realcontent.length() > blocksize) {
            for (int i = 0; i < realcontent.length() - blocksize; i = i + blocksize) {
                StringBuffer op = new StringBuffer(realcontent.substring(i, i + blocksize));
                DiskBlock newblock = new DiskBlock(op);
                SuperBlock.getInstance().addBlock(newblock);
                thenw.add(newblock.getId());
                newFileBlocklist.add(newblock);
                point = i;
                sizecount++;
            }
            StringBuffer rest = new StringBuffer(realcontent.substring(point + blocksize, realcontent.length()));
            sizecount++;
            DiskBlock ano = new DiskBlock(rest);
            SuperBlock.getInstance().addBlock(ano); // 修改后的内容保存到磁盘
            thenw.add(ano.getId());
            newFileBlocklist.add(ano); // 保存到文件
        } else {
            StringBuffer shortone = new StringBuffer(realcontent.substring(0, realcontent.length()));
            DiskBlock one = new DiskBlock(shortone);

            sizecount++;
            SuperBlock.getInstance().addBlock(one);
            thenw.add(one.getId());
            newFileBlocklist.add(one);
        }
        INode file = new INode(filename, newFileBlocklist);
        this.getFilelist().add(file);
    }

    /**
     * 删除文件
     *
     * @param filename String
     */
    public void deleteFile(String filename) {
        filelist.remove(filename);
    }

}
