/**
 ****************************************************************************************************
 * @file        audioplay.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2025-01-01
 * @brief       音乐播放器 应用代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原ESP32-S3 开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include "audioplay.h"


__audiodev g_audiodev;          /* 音乐播放控制器 */

/**
 * @brief       开始音频播放
 * @param       无
 * @retval      无
 */
void audio_start(void)
{
    g_audiodev.status = 3 << 0;
    i2s_trx_start();
}

/**
 * @brief       停止音频播放
 * @param       无
 * @retval      无
 */
void audio_stop(void)
{
    g_audiodev.status = 0;
    i2s_trx_stop();
}

/**
 * @brief       得到path路径下，目标文件的总数
 * @param       path : 文件路径
 * @retval      有效文件总数
 */
uint16_t audio_get_tnum(uint8_t *path)
{
    uint8_t res;
    uint16_t rval = 0;
    FF_DIR tdir;
    FILINFO *tfileinfo;
    
    tfileinfo = (FILINFO*)malloc(sizeof(FILINFO));
    
    res = f_opendir(&tdir, (const TCHAR*)path);
    
    if ((res == FR_OK) && tfileinfo)
    {
        while (1)
        {
            res = f_readdir(&tdir, tfileinfo);
            
            if ((res != FR_OK) || (tfileinfo->fname[0] == 0))
            {
                break;
            }

            res = exfuns_file_type(tfileinfo->fname);
            
            if ((res & 0xF0) == 0x40)
            {
                rval++; /* 有效文件数增加1 */
            }
        }
    }
    
    free(tfileinfo);
    
    return rval;
}

/**
 * @brief       显示曲目索引
 * @param       index : 当前索引
 * @param       total : 总文件数
 * @retval      无
 */
void audio_index_show(uint16_t index, uint16_t total)
{
    spilcd_show_num(30 + 0, 90, index, 3, 16, RED);
    spilcd_show_char(30 + 24, 90, '/', 16, 0, RED);
    spilcd_show_num(30 + 32, 90, total, 3, 16, RED);
}

/**
 * @brief       显示播放时间,比特率 信息
 * @param       totsec : 音频文件总时间长度
 * @param       cursec : 当前播放时间
 * @param       bitrate: 比特率(位速)
 * @retval      无
 */
void audio_msg_show(uint32_t totsec, uint32_t cursec, uint32_t bitrate)
{
    static uint16_t playtime = 0xFFFF;
    
    if (playtime != cursec)
    {
        playtime = cursec;
        
        spilcd_show_xnum(30, 70, playtime / 60, 2, 16, 0X80, RED);
        spilcd_show_char(30 + 16, 70, ':', 16, 0, RED);
        spilcd_show_xnum(30 + 24, 70, playtime % 60, 2, 16, 0X80, RED);
        spilcd_show_char(30 + 40, 70, '/', 16, 0, RED);
        
        spilcd_show_xnum(30 + 48, 70, totsec / 60, 2, 16, 0X80, RED);
        spilcd_show_char(30 + 64, 70, ':', 16, 0, RED);
        spilcd_show_xnum(30 + 72, 70, totsec % 60, 2, 16, 0X80, RED);
        
        spilcd_show_num(30 + 110, 70, bitrate / 1000, 4, 16, RED);
        spilcd_show_string(30 + 110 + 32 , 70, 200, 16, 16, "Kbps", RED);
    }
}

/**
 * @brief       转换
 * @param       fs:文件系统对象
 * @param       clst:转换
 * @retval      =0:扇区号，0:失败
 */
static LBA_t atk_clst2sect(FATFS* fs,DWORD clst)
{
    clst -= 2;          /* Cluster number is origin from 2 */

    if (clst >= fs->n_fatent - 2)
    {
        return 0;       /* Is it invalid cluster number? */
    }

    return fs->database + (LBA_t)fs->csize * clst;  /* Start sector number of the cluster */
}

/**
 * @brief       偏移
 * @param       dp:指向目录对象
 * @param       Offset:目录表的偏移量
 * @retval      FR_OK(0):成功，!=0:错误
 */
FRESULT atk_dir_sdi(FF_DIR* dp,DWORD ofs)
{
    DWORD clst;
    FATFS *fs = dp->obj.fs;

    if (ofs >= (DWORD)((FF_FS_EXFAT && fs->fs_type == FS_EXFAT) ? 0x10000000 : 0x200000) || ofs % 32)
    {
        /* Check range of offset and alignment */
        return FR_INT_ERR;
    }

    dp->dptr = ofs;         /* Set current offset */
    clst = dp->obj.sclust;  /* Table start cluster (0:root) */

    if (clst == 0 && fs->fs_type >= FS_FAT32)
    {	/* Replace cluster# 0 with root cluster# */
        clst = (DWORD)fs->dirbase;

        if (FF_FS_EXFAT)
        {
            dp->obj.stat = 0;
        }
        /* exFAT: Root dir has an FAT chain */
    }

    if (clst == 0)
    {	/* Static table (root-directory on the FAT volume) */
        if (ofs / 32 >= fs->n_rootdir)
        {
            return FR_INT_ERR;  /* Is index out of range? */
        }

        dp->sect = fs->dirbase;

    }
    else
    {   /* Dynamic table (sub-directory or root-directory on the FAT32/exFAT volume) */
        dp->sect = atk_clst2sect(fs, clst);
    }

    dp->clust = clst;   /* Current cluster# */

    if (dp->sect == 0)
    {
        return FR_INT_ERR;
    }

    dp->sect += ofs / fs->ssize;             /* Sector# of the directory entry */
    dp->dir = fs->win + (ofs % fs->ssize);   /* Pointer to the entry in the win[] */

    return FR_OK;
}

/**
 * @brief       播放音乐
 * @param       无
 * @retval      无
 */
void audio_play(void)
{
    uint8_t res;
    FF_DIR wavdir;
    FILINFO *wavfileinfo;
    uint8_t *pname;
    uint16_t totwavnum;
    uint16_t curindex;
    uint8_t key;
    uint32_t temp;
    uint32_t *wavoffsettbl;
    uint32_t *song_breakpoints; /* 新增：用于记录每首歌断点位置的数组 */

    while (f_opendir(&wavdir, "0:/MUSIC"))
    {
        text_show_string(30, 190, 240, 16, "MUSIC文件夹错误!", 16, 0, BLUE);
        vTaskDelay(200);
        spilcd_fill(30, 190, 240, 206, WHITE);
        vTaskDelay(200);
    }

    totwavnum = audio_get_tnum((uint8_t *)"0:/MUSIC");          /* 得到总有效文件数 */
    
    while (totwavnum == 0)
    {
        text_show_string(30, 190, 240, 16, "没有音乐文件!", 16, 0, BLUE);
        vTaskDelay(200);
        spilcd_fill(30, 190, 240, 146, WHITE);
        vTaskDelay(200);
    }
    
    wavfileinfo = (FILINFO*)malloc(sizeof(FILINFO));
    pname = malloc(255 * 2 + 1);
    wavoffsettbl = malloc(4 * totwavnum);
    song_breakpoints = malloc(4 * totwavnum); /* 新增：分配断点记忆数组内存 */
    
    while (!wavfileinfo || !pname || !wavoffsettbl || !song_breakpoints)
    {
        text_show_string(30, 190, 240, 16, "内存分配失败!", 16, 0, BLUE);
        vTaskDelay(200);
        spilcd_fill(30, 190, 240, 146, WHITE);
        vTaskDelay(200);
    }
    
    memset(song_breakpoints, 0, 4 * totwavnum); /* 新增：初始化所有断点为0 */

    res = f_opendir(&wavdir, "0:/MUSIC");
    uint32_t total_index = 0; /* 记录总索引数 */
    if (res == FR_OK)
    {
        curindex = 0;                                           /* 当前索引为0 */
        while (1)
        {
            temp = wavdir.dptr;                                 /* 记录当前index */
            res = f_readdir(&wavdir, wavfileinfo);              /* 读取目录下的一个文件 */
            if ((res != FR_OK) || (wavfileinfo->fname[0] == 0)) break;

            res = exfuns_file_type(wavfileinfo->fname);
            if ((res & 0xF0) == 0x40)
            {
                wavoffsettbl[curindex] = temp;                   /* 记录索引 */
                curindex++;
                total_index++;
            }
        }
    }

    //遍历索引表，显示文件名和索引
    for(int i = 0; i < total_index; i++){
        atk_dir_sdi(&wavdir, wavoffsettbl[i]); /* 定位到索引位置 */
        f_readdir(&wavdir, wavfileinfo); /* 读取文件信息 */
        text_show_string(30, 100+ i*20, 300, 16, (char *)wavfileinfo->fname, 16, 0, BLUE);
        //限制显示数量，只显示4首歌
        if ((i>=3)) break;
    }   

    curindex = 0;                                               /* 从0开始显示 */
    res = f_opendir(&wavdir, (const TCHAR*)"0:/MUSIC");

    while (res == FR_OK)                                        /* 打开目录 */
    {
        atk_dir_sdi(&wavdir, wavoffsettbl[curindex]);           /* 改变当前目录索引 */
        res = f_readdir(&wavdir, wavfileinfo);                  /* 读取文件 */
        
        if ((res != FR_OK) || (wavfileinfo->fname[0] == 0)) break;
        
        strcpy((char *)pname, "0:/MUSIC/");
        strcat((char *)pname, (const char *)wavfileinfo->fname);
        spilcd_fill(30, 190, spilcddev.width, spilcddev.height, WHITE);
        audio_index_show(curindex + 1, totwavnum);
        text_show_string(30, 50, 300, 16, (char *)wavfileinfo->fname, 16, 0, BLUE);
        
        /* 修改：将对应的断点指针传入播放函数 */
        key = audio_play_song(pname, &song_breakpoints[curindex]);

        if (key == KEY1_PRES)       /* 上一首 */
        {
            if (curindex) curindex--;
            else curindex = totwavnum - 1;
        }
        else if (key == KEY0_PRES)  /* 下一首 */
        {
            curindex++;
            if (curindex >= totwavnum) curindex = 0;
        }
        else
        {
            break;
        }
    }

    free(wavfileinfo);
    free(pname);
    free(wavoffsettbl);
    free(song_breakpoints); /* 新增：释放断点数组内存 */
}


/**
 * @brief       播放某个音频文件
 * @param       fname : 文件名
 * @param       breakpoint : 当前歌曲的断点记忆位置指针
 * @retval      按键值
 */
uint8_t audio_play_song(uint8_t *fname, uint32_t *breakpoint)
{
    uint8_t res;  
    
    res = exfuns_file_type((char *)fname); 

    switch (res)
    {
        case T_WAV:
            /* 修改：将断点指针传递给wav播放函数 */
            res = wav_play_song(fname, breakpoint); 
            break;
        case T_MP3:
            /* 自行实现 mp3_play_song(fname, breakpoint); */
            break;

        default:            /* 其他文件,自动跳转到下一曲 */
            printf("can't play:%s\r\n", fname);
            res = KEY0_PRES;
            break;
    }
    return res;
}

/**
//  * @brief       显示歌曲列表
//  * @param       path : 存放歌曲的路径 (如 "0:/MUSIC")
//  * @retval      无
//  */
// void audio_show_song_list(uint8_t *path)
// {
//     uint8_t res;
//     uint16_t file_count = 0;
//     FF_DIR dir;
//     FILINFO *fileinfo;
    
//     fileinfo = (FILINFO*)malloc(sizeof(FILINFO));
//     if (!fileinfo) return;

//     res = f_opendir(&dir, (const TCHAR*)path);
    
//     if (res == FR_OK)
//     {
//         /* 清屏，准备显示列表 */
//         spilcd_clear(WHITE);
//         text_show_string(30, 50, 200, 16, "--- 歌曲列表 ---", 16, 0, BLUE);

//         while (1)
//         {
//             res = f_readdir(&dir, fileinfo);
            
//             /* 如果读取失败或文件名为结束符，则退出 */
//             if ((res != FR_OK) || (fileinfo->fname[0] == 0)) break;

//             /* 检查文件类型 (0x40 通常代表音频文件组) */
//             res = exfuns_file_type(fileinfo->fname);
//             if ((res & 0xF0) == 0x40)
//             {
//                 file_count++;
                
//                 /* 在屏幕上显示序号和文件名 */
//                 /* 每一行 y 坐标增加 20 像素 */
//                 char show_buf[100];
//                 snprintf(show_buf, sizeof(show_buf), "%02d. %s", file_count, fileinfo->fname);
                
//                 // 限制显示数量，防止超出屏幕底部
//                 if (20 + (file_count * 20) < spilcddev.height - 20)
//                 {
//                     text_show_string(10, 20 + (file_count * 20), 300, 16, show_buf, 16, 0, BLACK);
//                 }
//             }
//         }
//     }
    
//     if (file_count == 0)
//     {
//         text_show_string(30, 50, 200, 16, "未找到音乐文件", 16, 0, RED);
//     }

//     free(fileinfo);
// }