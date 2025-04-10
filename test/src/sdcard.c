#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/ext2.h>
#include <zephyr/sys/check.h>
#include "sdcard.h"

LOG_MODULE_REGISTER(sdcard, CONFIG_LOG_DEFAULT_LEVEL);

#define DISK_DRIVE_NAME "SDMMC"
#define DISK_MOUNT_PT "/ext"
#define FS_RET_OK 0

static const struct device *const sdcard = DEVICE_DT_GET(DT_NODELABEL(sdhc0));
static const struct gpio_dt_spec sd_en = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(sdcard_en_pin), gpios, {0});

static struct fs_mount_t mount_point = {
	.type = FS_EXT2,
	.flags = FS_MOUNT_FLAG_NO_FORMAT,
	.storage_dev = (void *)DISK_DRIVE_NAME,
	.mnt_point = DISK_MOUNT_PT,
};

uint8_t file_count = 0;

#define MAX_PATH_LENGTH 256
static char current_full_path[MAX_PATH_LENGTH];
static char read_buffer[MAX_PATH_LENGTH];
static char write_buffer[MAX_PATH_LENGTH];

uint32_t file_num_array[2];    

static const char *disk_mount_pt = DISK_MOUNT_PT;

bool sd_enabled = false;
static bool is_mounted = false;

static int sd_enable_power(bool enable)
{
	int ret;
	// Make sure the GPIO is valid before configuring
	if (!device_is_ready(sd_en.port)) {
		LOG_ERR("SD enable GPIO port not ready");
		return -ENODEV;
	}
	
	ret = gpio_pin_configure_dt(&sd_en, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure SD enable pin: %d", ret);
		return ret;
	}
	
	if (enable)
	{
		LOG_INF("Enabling SD card power");
		ret = gpio_pin_set_dt(&sd_en, 1);
		if (ret < 0) {
			LOG_ERR("Failed to set SD enable pin: %d", ret);
			return ret;
		}
		
		// Give the SD card time to power up
		k_msleep(50);
		
		ret = pm_device_action_run(sdcard, PM_DEVICE_ACTION_RESUME);
		sd_enabled = true;
	} 
	else
	{
		LOG_INF("Disabling SD card power");
		ret = pm_device_action_run(sdcard, PM_DEVICE_ACTION_SUSPEND);
		
		// Wait for device to suspend
		k_msleep(10);
		
		ret = gpio_pin_set_dt(&sd_en, 0);
		if (ret < 0) {
			LOG_ERR("Failed to clear SD enable pin: %d", ret);
			return ret;
		}
		
		sd_enabled = false;
	}
	return ret;
}

int mount_sd_card(void)
{
    //initialize the sd card enable pin
    int ret = sd_enable_power(true);
    if (ret < 0) {
        LOG_ERR("Failed to power on SD card (%d)", ret);
        return ret;
    }

    // Give SD card time to power up and stabilize
    k_msleep(100);

    //initialize the sd card
    const char *disk_pdrv = DISK_DRIVE_NAME;  
	int err = disk_access_init(disk_pdrv); 
    LOG_INF("disk_access_init: %d", err);
    if (err) 
    {   
        // For -22 (EINVAL), check hardware connections
        if (err == -22) {
            LOG_ERR("Invalid argument in disk_access_init, check SD card hardware");
        }
        
        //reattempt with longer delay
        LOG_INF("Retrying SD card initialization...");
        k_msleep(1000);
        err = disk_access_init(disk_pdrv); 
        if (err) 
        {
            LOG_ERR("disk_access_init failed with error %d", err);
            sd_enable_power(false);
            return -1;
        }
    }

    // Verify disk is present
    uint32_t sector_count = 0;
    uint32_t sector_size = 0;
    err = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &sector_count);
    if (err) {
        LOG_ERR("Unable to get sector count (err=%d)", err);
        sd_enable_power(false);
        return -1;
    }
    
    err = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &sector_size);
    if (err) {
        LOG_ERR("Unable to get sector size (err=%d)", err);
        sd_enable_power(false);
        return -1;
    }
    
    LOG_INF("SD card detected: %u sectors of %u bytes", sector_count, sector_size);

    if (is_mounted) {
        LOG_INF("Disk already mounted");
        return 0;
    }

    int res = fs_mount(&mount_point);
    if (res == FS_RET_OK) 
    {
        LOG_INF("SD card mounted successfully");
        is_mounted = true;
    } 
    else 
    {
        LOG_INF("File system not found, creating file system...");
        res = fs_mkfs(FS_EXT2, (uintptr_t)mount_point.storage_dev, NULL, 0);
        if (res != 0) {
            LOG_ERR("Error formatting filesystem [%d]", res);
            sd_enable_power(false);
            return res;
        }

        res = fs_mount(&mount_point);
        if (res != FS_RET_OK) {
            LOG_ERR("Error mounting disk %d", res);
            sd_enable_power(false);
            return res;
        }
        is_mounted = true;
        LOG_INF("SD card mounted successfully after formatting");
    }
    
    res = fs_mkdir(DISK_MOUNT_PT "/audio");

    if (res == 0) 
    {
        LOG_INF("audio directory created successfully");
        initialize_audio_file(1);
    }
    else if (res == -EEXIST) 
    {
        LOG_INF("audio directory already exists");
    }
    else 
    {
        LOG_INF("audio directory creation failed: %d", res);
    }

    struct fs_dir_t audio_dir_entry;
    fs_dir_t_init(&audio_dir_entry);
    err = fs_opendir(&audio_dir_entry, DISK_MOUNT_PT "/audio");
    if (err) 
    {
        LOG_ERR("error while opening directory %d", err);
        return -1;
    }
    LOG_INF("result of opendir: %d", err);
    initialize_audio_file(1);
    struct fs_dirent file_count_entry;
    file_count = get_file_contents(&audio_dir_entry, &file_count_entry);
    file_count = 1;
    if (file_count < 0) 
    {
        LOG_ERR("error getting file count");
        return -1;
    }

    fs_closedir(&audio_dir_entry);
    LOG_INF("new num files: %d", file_count);

    res = move_write_pointer(file_count); 
    if (res) 
    {
        LOG_ERR("error while moving the write pointer");
        return -1;
    }

    move_read_pointer(file_count);

    if (res) 
    {
        LOG_ERR("error while moving the reader pointer");
        return -1;
    }
    LOG_INF("file count: %d", file_count);
   
    struct fs_dirent info_file_entry; //check if the info file exists. if not, generate new info file
    const char *info_path = DISK_MOUNT_PT "/info.txt";
    res = fs_stat(info_path, &info_file_entry); //for later
    if (res) 
    {
        res = create_file("info.txt");
        save_offset(0);
        LOG_INF("result of info.txt creation: %d", res);
    }
    
    LOG_INF("result of check: %d", res);

	return 0;
}

uint32_t get_file_size(uint8_t num)
{
    char *ptr = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s/%s", disk_mount_pt, ptr);
    k_free(ptr);
    struct fs_dirent entry;
    int res = fs_stat(current_full_path, &entry);
    if (res)
    {
        LOG_ERR("invalid file in get file size");
        return 0;  
    }
    return (uint32_t)entry.size;
}

int move_read_pointer(uint8_t num) 
{
    char *read_ptr = generate_new_audio_header(num);
    snprintf(read_buffer, sizeof(read_buffer), "%s/%s", disk_mount_pt, read_ptr);
    k_free(read_ptr);
    struct fs_dirent entry; 
    int res = fs_stat(read_buffer, &entry);
    if (res) 
    {
        LOG_ERR("invalid file in move read ptr");
        return -1;  
    }
    return 0;
}

int move_write_pointer(uint8_t num) 
{
    char *write_ptr = generate_new_audio_header(num);
    snprintf(write_buffer, sizeof(write_buffer), "%s/%s", disk_mount_pt, write_ptr);
    k_free(write_ptr);
    struct fs_dirent entry;
    int res = fs_stat(write_buffer, &entry);
    if (res) 
    {
        LOG_ERR("invalid file in move write pointer");  
        return -1;  
    }
    return 0;   
}

int create_file(const char *file_path)
{
    int ret = 0;
    snprintf(current_full_path, sizeof(current_full_path), "%s/%s", disk_mount_pt, file_path);
	struct fs_file_t data_file;
	fs_file_t_init(&data_file);
	ret = fs_open(&data_file, current_full_path, FS_O_WRITE | FS_O_CREATE);
	if (ret) 
	{
        LOG_ERR("File creation failed %d", ret);
		return -2;
	} 
    fs_close(&data_file);
    return 0;
}

int read_audio_data(uint8_t *buf, int amount, int offset) 
{
    struct fs_file_t read_file;
   	fs_file_t_init(&read_file); 
    uint8_t *temp_ptr = buf;

	int rc = fs_open(&read_file, read_buffer, FS_O_READ | FS_O_RDWR);
    rc = fs_seek(&read_file, offset, FS_SEEK_SET);
    rc = fs_read(&read_file, temp_ptr, amount);
  	fs_close(&read_file);

    return rc;
}

int write_to_file(uint8_t *data, uint32_t length)
{
    struct fs_file_t write_file;
	fs_file_t_init(&write_file);
    uint8_t *write_ptr = data;
   	fs_open(&write_file, write_buffer, FS_O_WRITE | FS_O_APPEND);
	fs_write(&write_file, write_ptr, length);
    fs_close(&write_file);
    return 0;
}
    
int initialize_audio_file(uint8_t num) 
{
    char *header = generate_new_audio_header(num);
    if (header == NULL) 
    {
        return -1;
    }
    int res = create_file(header);
    k_free(header);
    return res;
}

char* generate_new_audio_header(uint8_t num) 
{
    if (num > 99) return NULL;
    char *ptr_ = k_malloc(14);
    ptr_[0] = 'a';
    ptr_[1] = 'u';
    ptr_[2] = 'd';
    ptr_[3] = 'i';
    ptr_[4] = 'o';
    ptr_[5] = '/';
    ptr_[6] = 'a';
    ptr_[7] = 48 + (num / 10);
    ptr_[8] = 48 + (num % 10);
    ptr_[9] = '.';
    ptr_[10] = 't';
    ptr_[11] = 'x';
    ptr_[12] = 't';
    ptr_[13] = '\0';

    return ptr_;
}

int get_file_contents(struct fs_dir_t *zdp, struct fs_dirent *entry) 
{
   if (fs_readdir(zdp, entry)) 
   {
    return -1;
   }
   if (entry->name[0] == 0) 
   {
    return 0;
   }
   int count = 0;  
   file_num_array[count] = entry->size;
   LOG_INF("file numarray %d %d", count, file_num_array[count]);
   LOG_INF("file name is %s", entry->name);
   count++;
   while (fs_readdir(zdp, entry) == 0) 
   {
        if (entry->name[0] == 0)
        {
            break;
        }
        file_num_array[count] = entry->size;
        LOG_INF("file numarray %d %d", count, file_num_array[count]);
        LOG_INF("file name is %s", entry->name);
        count++;
   }
   return count;
}

int clear_audio_file(uint8_t num) 
{
    char *clear_header = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s/%s", disk_mount_pt, clear_header);
    k_free(clear_header);
    int res = fs_unlink(current_full_path);
    if (res) 
    {
        LOG_ERR("error deleting file");
        return -1;
    }

    char *create_file_header = generate_new_audio_header(num);
    k_msleep(10);
    res = create_file(create_file_header);
    k_free(create_file_header);
    if (res) 
    {
        LOG_ERR("error creating file");
        return -1;
    }

    return 0;
}

int delete_audio_file(uint8_t num) 
{
    char *ptr = generate_new_audio_header(num);
    snprintf(current_full_path, sizeof(current_full_path), "%s/%s", disk_mount_pt, ptr);
    k_free(ptr);
    int res = fs_unlink(current_full_path);
    if (res) 
    {
        LOG_ERR("error deleting file in delete");
        return -1;
    }

    return 0;
}

int clear_audio_directory() 
{
    if (file_count == 1) 
    {
        return 0;
    }
    
    int res = 0;
    for (uint8_t i = file_count; i > 0; i--) 
    {
        res = delete_audio_file(i);
        k_msleep(10);
        if (res) 
        {
            LOG_ERR("error on %d", i);
            return -1;
        }  
    }
    res = fs_unlink(DISK_MOUNT_PT "/audio");
    if (res) 
    {
        LOG_ERR("error deleting file");
        return -1;
    }
    res = fs_mkdir(DISK_MOUNT_PT "/audio");
    if (res) 
    {
        LOG_ERR("failed to make directory");
        return -1;
    }
    res = create_file("audio/a01.txt");
    if (res) 
    {
        LOG_ERR("failed to make new file in directory files");
        return -1;
    }
    LOG_INF("done with clearing");

    file_count = 1;  
    move_write_pointer(1);
    return 0;
}

int save_offset(uint32_t offset)
{
    uint8_t buf[4] = {
	offset & 0xFF,
	(offset >> 8) & 0xFF,
	(offset >> 16) & 0xFF, 
	(offset >> 24) & 0xFF 
    };

    struct fs_file_t write_file;
    fs_file_t_init(&write_file);
    int res = fs_open(&write_file, DISK_MOUNT_PT "/info.txt", FS_O_WRITE | FS_O_CREATE);
    if (res) 
    {
        LOG_ERR("error opening file %d", res);
        return -1;
    }
    res = fs_write(&write_file, &buf, 4);
    if (res < 0)
    {
        LOG_ERR("error writing file %d", res);
        return -1;
    }
    fs_close(&write_file);
    return 0;
}

int get_offset()
{
    uint8_t buf[4];
    struct fs_file_t read_file;
    fs_file_t_init(&read_file);
    int rc = fs_open(&read_file, DISK_MOUNT_PT "/info.txt", FS_O_READ | FS_O_RDWR);
    if (rc < 0)
    {
        LOG_ERR("error opening file %d", rc);
        return -1;
    }
    rc = fs_seek(&read_file, 0, FS_SEEK_SET);
    if (rc < 0)
    {
        LOG_ERR("error seeking file %d", rc);
        return -1;
    }
    rc = fs_read(&read_file, &buf, 4);
    if (rc < 0)
    {
        LOG_ERR("error reading file %d", rc);
        return -1;
    }
    fs_close(&read_file);
    uint32_t *offset_ptr = (uint32_t*)buf;
    LOG_INF("get offset is %d", offset_ptr[0]);
    return offset_ptr[0];
}

void sd_off()
{
    sd_enable_power(false);
}

void sd_on()
{
    sd_enable_power(true);
}

bool is_sd_on()
{
    return sd_enabled;
}


