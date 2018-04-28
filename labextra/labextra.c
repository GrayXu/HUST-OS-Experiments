#include <dirent.h>
#include <grp.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

int showDir(char* dirname, int mode);
char* getFileInfo(struct stat* sP, char* filename, int* blocks);
char* num2month(int num);
void printInfoList(char** allFileInfos, int indexForInfos);
void printFormatList(char** allFileNames,
                     int maxLengthOfFileName,
                     int indexForFileNames);

int WIDTH = 0;  // num of columns of terminal

int main(int argc, char** argv) {
    char* dir = ".";
    int mode = 0;

    // get width(Columns) of terminal
    struct winsize size;
    ioctl(STDIN_FILENO, TIOCGWINSZ, &size);
    WIDTH = size.ws_col;

    // read arg's info
    if (argc != 1) {
        for (int i = 1; i < argc; i++) {
            if (argv[i][0] == '-') {
                if (strcmp("-lR", argv[i]) == 0) {
                    mode = 2;
                } else {
                    mode = 1;
                }
            } else {
                dir = argv[i];  // change the path
            }
        }
    }

    showDir(dir, mode);

    return 0;
}

void strcat_char(char* a, char b) {
    for (int i = 0; i < strlen(a); i++) {
        if (a[i] == '\0') {
            a[i] = b;
            a[i + 1] = '\0';
            break;
        }
    }
}

// called by showDir
// with '-l' further infomation of files
char* getFileInfo(struct stat* sP, char* filename, int* blocks) {
    char* buf = (char*)malloc(sizeof(char) * 1024);
    struct stat s = *sP;

    switch (s.st_mode & S_IFMT) {
        case S_IFREG:
            sprintf(buf, "-");
            break;
        case S_IFDIR:
            sprintf(buf, "d");
            break;
        case S_IFLNK:
            sprintf(buf, "l");
            break;
        case S_IFBLK:
            sprintf(buf, "b");
            break;
        case S_IFCHR:
            sprintf(buf, "c");
            break;
        case S_IFIFO:
            sprintf(buf, "p");
            break;
        case S_IFSOCK:
            sprintf(buf, "s");
            break;
    }

    for (int i = 8; i >= 0; i--) {
        if (s.st_mode & (1 << i)) {
            switch (i % 3) {
                case 2:
                    strcat(buf,"r");
                    break;
                case 1:
                    strcat(buf,"w");
                    break;
                case 0:
                    strcat(buf,"x");
                    break;
            }
        } else {
            strcat(buf,"-");
        }
    }

    struct passwd* p = getpwuid(s.st_uid);
    struct group* g = getgrgid(s.st_gid);
    
    char temp[128];
    sprintf(temp, " %d %s %s %6ld", (int)s.st_nlink, p->pw_name, g->gr_name,
            s.st_size);
    strcat(buf, temp);

    struct tm* t = localtime(&s.st_ctime);
    sprintf(temp, " %s %02d %02d:%02d", num2month(t->tm_mon + 1), t->tm_mday,
            t->tm_hour, t->tm_min);
    strcat(buf, temp);

    sprintf(temp, " %s\n", filename);
    strcat(buf, temp);
    if (s.st_size % 4096 == 0) {
        *blocks = *blocks + s.st_size / 1028;
    } else {
        *blocks = *blocks + (s.st_size / 4096 + 1) * 4;
    }

    return buf;
}

// mode 1-> with '-l' input
int showDir(char* dirname, int mode) {
    if (mode == 2) {
        printf("%s:\n", dirname);
    }
    int blocks = 0;

    DIR* dir = opendir(dirname);
    struct dirent* dirDescribe;
    struct stat st;
    char nowDirnameBuf[1024];

    /*for formatting*/
    int indexForFileNames = 0;  // index for allFileNames
    int maxLengthOfFileName = 0;
    char** allFileNames = NULL;  // save all file's name to make sure format
    if (mode == 0) {
        allFileNames = (char**)malloc(sizeof(char*) * 2048);
    }

    /*for keeping file info (kind of delay for us to get "total block")*/
    int indexForInfos = 0;
    char** allFileInfos = NULL;
    if (mode != 0) {
        allFileInfos = (char**)malloc(sizeof(char*) * 2048);
    }

    /*for recursion*/
    int indexForDirNames = 0;
    char** allDirNames = NULL;
    if (mode == 2) {
        allDirNames = (char**)malloc(sizeof(char*) * 2048);
    }

    while ((dirDescribe = readdir(dir)) != NULL) {  // reading in loop

        strcpy(nowDirnameBuf, dirname);
        strcat(nowDirnameBuf, "/");
        strcat(nowDirnameBuf, dirDescribe->d_name);
        if (stat(nowDirnameBuf, &st)) {
            printf("error\n");
            return -1;
        }

        if (dirDescribe->d_name[0] != '.') {  // hidden files off
            if (mode == 0) {
                char* nameBuf = (char*)malloc(sizeof(char) * 100);
                strcpy(nameBuf, dirDescribe->d_name);
                allFileNames[indexForFileNames] = nameBuf;
                indexForFileNames++;

                if (strlen(nameBuf) > maxLengthOfFileName)
                    maxLengthOfFileName = strlen(nameBuf);

            } else {
                if (mode == 2) {
                    if (S_ISDIR(st.st_mode)) {  // is dir or not
                        char* nameBuf = (char*)malloc(sizeof(char) * 100);
                        strcpy(nameBuf, nowDirnameBuf);
                        allDirNames[indexForDirNames] = nameBuf;
                        indexForDirNames++;
                    }
                }
                char* fileInfo = getFileInfo(&st, dirDescribe->d_name, &blocks);
                allFileInfos[indexForInfos] = fileInfo;
                indexForInfos++;
            }
        }
    }

    // output formatted infomation when mode is 0
    if (mode == 0) {
        printFormatList(allFileNames, maxLengthOfFileName, indexForFileNames);
    } else {
        printf("total:%d\n", blocks);
        printInfoList(allFileInfos, indexForInfos);

        if (mode == 2) {
            printf("\n");
            int i = 0;
            for (i = 0; i < indexForDirNames; i++) {
                showDir(allDirNames[i], 2);  // enter recursion here
            }
        }
    }

    // free all heap space
    if (mode == 0) {
        for (int i = 0; i < indexForFileNames; i++) {
            free(allFileNames[i]);
        }
        free(allFileNames);
    } else {
        free(allFileInfos);
        if (mode == 2) {
            for (int i = 0; i < indexForDirNames; i++) {
                free(allDirNames[i]);
            }
            free(allDirNames);
        }
    }

    closedir(dir);
    return 0;
}

char* num2month(int num) {
    switch (num) {
        case 1:
            return "Jan";
        case 2:
            return "Feb";
        case 3:
            return "Mar";
        case 4:
            return "Apr";
        case 5:
            return "May";
        case 6:
            return "Jun";
        case 7:
            return "Jul";
        case 8:
            return "Aug";
        case 9:
            return "Sep";
        case 10:
            return "Oct";
        case 11:
            return "Nov";
        case 12:
            return "Dec";
        default:
            return "";
    }
}

void printFormatList(char** allFileNames,
                     int maxLengthOfFileName,
                     int indexForFileNames) {
    int i = 0;
    int num = WIDTH / (maxLengthOfFileName + 2);
    // little trick here (nerver go over 99, or i gonna fuck♂you
    int temp = maxLengthOfFileName;
    char format[10];
    strcpy(format, "%-");
    if (temp > 9) {
        format[2] = temp / 10 + 48;
        format[3] = temp % 10 + 48;
        format[4] = '\0';
    } else {
        format[2] = temp + 48;
        format[3] = '\0';
    }
    strcat(format, "s  ");

    for (i = 0; i < indexForFileNames; i++) {
        printf(format, allFileNames[i]);

        if ((i + 1) % num == 0) {
            printf("\n");
        }
    }
    printf("\n");
}

void printInfoList(char** allFileInfos, int indexForInfos) {
    for (int i = 0; i < indexForInfos; i++) {
        printf("%s", allFileInfos[i]);
    }
}

/*reference*/
// struct dirDescribe {
// #ifndef __USE_FILE_OFFSET64
//     __ino_t d_ino;
//     __off_t d_off;
// #else
//     __ino64_t d_ino;
//     __off64_t d_off;
// #endif
//     unsigned short int d_reclen;
//     unsigned char d_type;
//     char d_name[256]; /* We must not include limits.h! */
// };

// struct stat {
//     unsigned long   st_dev;        /* Device.  */
//     unsigned long   st_ino;        /* File serial number.  */
//     unsigned int    st_mode;       /* File mode.  */
//     unsigned int    st_nlink;      /* Link count.  */
//     unsigned int    st_uid;        /* User ID of the file's owner.  */
//     unsigned int    st_gid;        /* Group ID of the file's group. */
//     unsigned long   st_rdev;       /* Device number, if device.  */
//     unsigned long   __pad1;
//     long            st_size;       /* Size of file, in blocks.  */
//     int             st_blksize;    /* Optimal block size for I/O.  */
//     int             __pad2;
//     long            st_blocks;     /* Number 512-byte blocks allocated. */
//     int             st_atime;      /* Time of last access.  */
//     unsigned int    st_atime_nsec;
//     int             st_mtime;      /* Time of last modification.  */
//     unsigned int    st_mtime_nsec;
//     int             st_ctime;      /* Time of last status change.  */
//     unsigned int    st_ctime_nsec;
//     unsigned int    __unused4;
//     unsigned int    __unused5;
// };