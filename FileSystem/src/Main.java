import java.io.*;

public class Main {

    static String path = "/home/gray/Workplace/FileSystem";
    static SuperBlock superBlockRead;
    public static void main(String[] args) {
        System.out.println("Welcome to file system, you are in \"root\" directory.");
        BufferedReader br = new BufferedReader(new InputStreamReader(System.in));
        
        Dir dirRead = autoloadDir();
        superBlockRead = autoloadSuperBlock();
        
        Dir root;
        if (dirRead == null || superBlockRead == null){
            superBlockRead = SuperBlock.getInstance();
            root = new Dir("root");
        }else{
            root = dirRead;
        }
        Dir nowDir = root;


        help();

        while (true) {
            String line = null;
            try {
                line = br.readLine();
            } catch (IOException e) {
                e.printStackTrace();
            }
            if (line.startsWith("help")) {
                help();
                continue;
            } else if (line.startsWith("ls")) {
                nowDir.list();
                continue;
            } else if (line.startsWith("cd")) {
                String path = line.substring(3);
                if (path.startsWith("..")) {
                    if (nowDir.getFatherDir() != null) {
                        nowDir = nowDir.getFatherDir();
                    } else {
                        System.out.println("Already in root directory");
                    }
                } else {
                    Dir tempDir = getDirByPath(nowDir, path);
                    if (tempDir != null) {
                        nowDir = tempDir;
                    } else {
                        System.out.println("No such directory");
                    }
                }
            } else if (line.startsWith("mkdir")) {
                String dirname = line.substring(6);
                Dir dir = new Dir(dirname);
                dir.setFatherDir(nowDir);
                nowDir.addDir(dir);

            } else if (line.startsWith("rename")) {
                String value = line.substring(7);
                String oldName = value.split(" ")[0];
                String newName = value.split(" ")[1];
                if (getDirByPath(nowDir, oldName) != null) {
                    getDirByPath(nowDir, oldName).setName(newName);
                } else if (getFileByPath(nowDir, oldName) != null) {
                    getFileByPath(nowDir, oldName).setName(newName);
                } else {
                    System.out.println("No such file");
                }
            } else if (line.startsWith("rm")) {
                String value = line.substring(3);
                System.out.println("delete "+value);
//                if (getDirByPath(nowDir, value) != null) {
                if (nowDir.getDir(value) != null) {
                    nowDir.getDir(value).deleteDir(value);
                } else if (nowDir.getFile(value) != null) {
//                    String[] routes = value.split("/");
//                    String fileFatherDirPath = routes[0];
//                    for (int i = 1; i < routes.length - 1; i++) {
//                        fileFatherDirPath = fileFatherDirPath + "/" + routes[i];
//                    }
//                    Dir tempDir = getDirByPath(nowDir, fileFatherDirPath);
                    nowDir.deleteFile(value);
                } else {
                    System.out.println("No such file");
                }
            } else if (line.startsWith("copy")) {//important part
                String value = line.substring(5);
                String oldName = value.split(" ")[0];
                String newName = value.split(" ")[1];
                if (nowDir.getFile(oldName) != null) {
                    Dir newDir = nowDir.copy(nowDir.getDir(oldName));
                    newDir.setName(newName);
                    getDirByPath(nowDir, newName).addDir(newDir);
                } else if (nowDir.getFile(oldName) != null) {
                    INode newINode = INode.copy(nowDir.getFile(oldName));
                    newINode.setName(newName);
                    getDirByPath(nowDir, newName).addFile(newINode);
                } else {
                    System.out.println("No such file");
                }
            } else if (line.startsWith("vim")) {
                String value = line.substring(4);
                INode nowINode = getFileByPath(nowDir, value);
                if (nowINode != null) {
                    StringBuffer stringBuffer = nowINode.read();
                    System.out.println(stringBuffer.toString());
                } else {
                    nowINode = new INode(value);
                    nowDir.addFile(nowINode);
                    System.out.println("created new file");
                }
                System.out.println("input new data ( end with # ) :");
                StringBuffer newSB = new StringBuffer();
                String subLine;
                try {
                    while ((subLine = br.readLine()) != null) {
                        newSB.append(subLine + "\n");
                        if (subLine.endsWith("#")) {
                            break;
                        }
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }

                nowINode.reWrite(new StringBuffer(String.valueOf(newSB).replaceAll("#", "")));

            } else if (line.startsWith("cat")) {
                String value = line.substring(4);
                if (getFileByPath(nowDir, value) != null) {
                    StringBuffer stringBuffer = getFileByPath(nowDir, value).read();
                    System.out.println(stringBuffer.toString());
                } else {
                    System.out.println("No such file");
                }

            } else if (line.startsWith("exit")) {
                break;

            } else {
                System.out.println("Unknown command");
                continue;
            }

            //save to disk
//            File f = new File(path, root.getName());
//            f.mkdirs();
//            Dir.write(path, root);
            try {
                autoave(root);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

    }

    public static void autoave(Dir root) throws IOException {
        FileOutputStream fileOutputStream1 = new FileOutputStream("superblock.ser");
        FileOutputStream fileOutputStream3 = new FileOutputStream("root.ser");
        ObjectOutputStream objectOutputStream1 = new ObjectOutputStream(fileOutputStream1);
        ObjectOutputStream objectOutputStream3 = new ObjectOutputStream(fileOutputStream3);
        objectOutputStream1.writeObject(superBlockRead);
        objectOutputStream3.writeObject(root);
        objectOutputStream1.close();
        objectOutputStream3.close();
    }

    public static SuperBlock autoloadSuperBlock() {
        ObjectInputStream objectInputStream1 = null;
        try {
            objectInputStream1 = new ObjectInputStream(new FileInputStream("superblock.ser"));
            SuperBlock superBlock = ((SuperBlock) objectInputStream1.readObject());
            objectInputStream1.close();
            return superBlock;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }

    }

    public static Dir autoloadDir() {
        ObjectInputStream objectInputStream2 = null;
        try {
            objectInputStream2 = new ObjectInputStream(new FileInputStream("root.ser"));
            Dir dir = ((Dir) objectInputStream2.readObject());
            objectInputStream2.close();
            return dir;
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


    public static void help() {
        System.out.println("Command Information:\n\tls : list all files in this directory\n" +
                "\tcd [path] : open directory\n" +
                "\tmkdir [directory's name] : create a new directory(only in this directory)\n" +
                "\trm [path] : delete a file or a directory \n" +
                "\trename [path] : rename a file or a directory\n" +
                "\tcopy [source path] [target path] : copy a file or a directory (source must be in this directory)\n" +
                "\tvim [path] : edit files (if it doesn't exist, create it first)\n" +
                "\tcat [path] : output all data in this file\n" +
                "\thelp : get help information\n" +
                "\texit : exit\n");
    }

    public static Dir getDirByPath(Dir nowDir, String path) {
        if (path.startsWith("/")) return null;
        String[] routes = path.split("/");
        Dir tempDir = nowDir;
        for (String str : routes) {
            if (str.endsWith("..")) {
                tempDir = tempDir.getFatherDir();
            } else {
                tempDir = tempDir.getDir(str);
            }
            if (tempDir == null) return null;
        }
        return tempDir;
    }

    public static INode getFileByPath(Dir nowDir, String path) {
        if (path.startsWith("/")) return null;
        String[] routes = path.split("/");
        Dir tempDir = nowDir;
        for (int i = 0; i < routes.length - 1; i++) {
            if (routes[i].endsWith("..")) {
                tempDir = tempDir.getFatherDir();
            } else {
                tempDir = tempDir.getDir(routes[i]);
            }
            if (tempDir == null) return null;
        }
        return tempDir.getFile(routes[routes.length - 1]);
    }

}
