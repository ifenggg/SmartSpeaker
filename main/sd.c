#include "sd.h"
#include <dirent.h>   // 用于目录操作
#include <sys/stat.h> // 用于文件状态判断
#include <string.h>   // 用于字符串操作

#define EXAMPLE_MAX_CHAR_SIZE    64     //定义字符最大长度
static const char *SD_TAG = "SD卡";
#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO  GPIO_NUM_19
#define PIN_NUM_MOSI  GPIO_NUM_23
#define PIN_NUM_CLK   GPIO_NUM_18
#define PIN_NUM_CS    GPIO_NUM_5

// 判断文件是否为MP3格式（不区分大小写）
static bool is_mp3_file(const char *filename) {
    if (filename == NULL) return false;
    
    // 查找最后一个'.'的位置
    const char *ext = strrchr(filename, '.');
    if (ext == NULL) return false; // 没有扩展名
    
    // 比较扩展名（不区分大小写）
    return (strcasecmp(ext, ".mp3") == 0);
}

// 递归搜索目录中的所有MP3文件
static esp_err_t search_mp3_files(const char *base_path) {
    DIR *dir = opendir(base_path);
    if (dir == NULL) {
        ESP_LOGE(SD_TAG, "无法打开目录");
        return ESP_FAIL;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // 跳过当前目录(.)和上级目录(..)
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
            continue;
        }

        // 构建完整路径
        char full_path[512];
        snprintf(full_path, sizeof(full_path), "%s/%s", base_path, entry->d_name);

        // 获取文件状态
        struct stat file_stat;
        if (stat(full_path, &file_stat) != 0) {
            //ESP_LOGE(SD_TAG, "无法获取文件状态: %s", full_path);
            continue;
        }

        // 如果是目录，递归搜索
        if (S_ISDIR(file_stat.st_mode)) {
            esp_err_t ret = search_mp3_files(full_path);
            if (ret != ESP_OK) {
                closedir(dir);
                return ret;
            }
        }
        // 如果是文件，检查是否为MP3
        else if (S_ISREG(file_stat.st_mode)) {
            if (is_mp3_file(entry->d_name)) {
                ESP_LOGI(SD_TAG, "找到MP3文件: %s (大小: %lld字节)", 
                         full_path, (long long)file_stat.st_size);
                uart_send("t2.txt=\"%s\"",full_path);
            }
        }
    }

    closedir(dir);
    return ESP_OK;
}

//文件写入函数
static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(SD_TAG, "正在打开文件 %s", path);
    FILE *f = fopen(path, "w");     //"w"模式打开文件
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "无法打开文件写入");
        return ESP_FAIL;
    }
    fprintf(f, data);   //把data写入
    fclose(f);      //关闭文件
    ESP_LOGI(SD_TAG, "文件已写入");

    return ESP_OK;
}

//文件读取函数
static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(SD_TAG, "正在读取文件 %s", path);
    FILE *f = fopen(path, "r");     //"r"模式打开文件
    if (f == NULL) {
        ESP_LOGE(SD_TAG, "无法打开文件读取");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);   //读取一行数据到 line 中
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n'); //查找 line 中的换行符 '\n'
    if (pos) {      //如果有'\n'
        *pos = '\0';    //替换成'\0'
    }
    ESP_LOGI(SD_TAG, "从文件读取: '%s'", line);

    return ESP_OK;
}

const char mount_point[] = MOUNT_POINT;     //挂载路径
sdmmc_card_t *card;
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
void sd_init(void)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
//挂载失败时格式化SD卡
        .format_if_mount_failed = false,
        .max_files = 5,     //同时可打开的最大文件数
        .allocation_unit_size = 16 * 1024   //文件系统的分配单元大小
    };
    
    ESP_LOGI(SD_TAG, "初始化 SD 卡");

    ESP_LOGI(SD_TAG, "使用SPI外设");

    host.max_freq_khz = 1000;   //1MHz

//初始化SPI总线
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "无法初始化SPI总线");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();  //默认配置
    slot_config.gpio_cs = PIN_NUM_CS;   //片选脚
    slot_config.host_id = host.slot;    //SPI主机槽位

    ESP_LOGI(SD_TAG, "挂载文件系统");
    //挂载文件系统
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "挂载文件系统失败 "
                     "如果你想格式化卡，请设置CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED菜单配置选项");
        } else {
            ESP_LOGE(SD_TAG, "无法初始化卡(%s). "
                     "确保SD卡引脚有上拉电阻", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(SD_TAG, "文件系统已挂载");

    // //打印 SD 卡的信息（容量、速度等级）
    // sdmmc_card_print_info(stdout, card);

    // search_mp3_files(MOUNT_POINT);

    // //创建文件
    // const char *file_hello = MOUNT_POINT"/hello.txt";
    // char data[EXAMPLE_MAX_CHAR_SIZE];
    // snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);     //把写入数据格式化
    // ret = s_example_write_file(file_hello, data);   //写入文件
    // if (ret != ESP_OK) {
    //     return;
    // }

    // const char *file_foo = MOUNT_POINT"/foo.txt";

    // struct stat st;
    // //检查文件是否存在
    // if (stat(file_foo, &st) == 0) {
    //     //若存在则删除
    //     unlink(file_foo);
    // }

    // //重命名文件
    // ESP_LOGI(SD_TAG, "重命名文件 %s to %s", file_hello, file_foo);
    // if (rename(file_hello, file_foo) != 0) {
    //     ESP_LOGE(SD_TAG, "重命名失败");
    //     return;
    // }

    // ret = s_example_read_file(file_foo);    //读取文件
    // if (ret != ESP_OK) {
    //     return;
    // }

    // const char *file_nihao = MOUNT_POINT"/nihao.txt";   //创建新文件
    // memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);     //把数组清零
    // snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", card->cid.name);  //格式化要写入的数据
    // ret = s_example_write_file(file_nihao, data);   //把数据写入文件
    // if (ret != ESP_OK) {
    //     return;
    // }

    // //读取文件
    // ret = s_example_read_file(file_nihao);
    // if (ret != ESP_OK) {
    //     return;
    // }

     // 【新增】开始搜索MP3文件
    ESP_LOGI(SD_TAG, "开始搜索所有MP3文件...");
    esp_err_t search_ret = search_mp3_files(MOUNT_POINT);
    if (search_ret != ESP_OK) {
        ESP_LOGE(SD_TAG, "MP3文件搜索失败");
    } else {
        ESP_LOGI(SD_TAG, "MP3文件搜索完成");
    }


}

void sd_deinit(void)
{
        //卸载挂载在 mount_point 路径的文件系统，并禁用 SPI 外设
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(SD_TAG, "SD卡已卸载");
    // // 仅卸载 VFS 层和 FAT 文件系统，不释放 SPI 总线和外设
    // esp_vfs_fat_unmount(mount_point);

    //释放 SPI 总线
    spi_bus_free(host.slot);
    ESP_LOGI("SD","sd_la");
}

void sd_pau(void)
{
    ESP_LOGI("SD","sd_pau");
}

void sd_pla(void)
{
    ESP_LOGI("SD","sd_pla");
}

void sd_ne(void)
{
    ESP_LOGI("SD","sd_ne");
}

void sd_la(void)
{
    ESP_LOGI("SD","sd_la");
}
