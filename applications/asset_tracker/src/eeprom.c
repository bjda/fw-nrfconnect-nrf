#include <zephyr.h>
#include <stdio.h>
#include <drivers/i2c.h>
#include <logging/log.h>
LOG_MODULE_REGISTER(eeprom, 3);

static struct device *i2c_dev;
#define EEPROM_ADDR 0x50
#define BOARD_ID_MAGIC 0x24519142

struct board_id {
	u32_t magic;
	u32_t pca;
	u32_t version;
	u32_t checksum;
};

int board_id_make_checksum(struct board_id *board) {
	if (board == NULL) {
		return EINVAL;
	}

	board->checksum = board->magic ^ board->pca ^ board->version;
	return 0;
}

int app_i2c_init(void){
	u32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_MASTER;

	i2c_dev = device_get_binding(DT_NORDIC_NRF_TWIM_I2C_2_LABEL);
	return i2c_configure(i2c_dev,i2c_cfg);
}

bool eeprom_is_present(void){
	u8_t dummy_buf[1];
	int err;

	err = i2c_read(i2c_dev,dummy_buf,sizeof(dummy_buf),EEPROM_ADDR);
	if (err == 0){
		return true;
	}
	return false;
}

int eeprom_verify(u16_t addr, const void *buf, size_t len, bool *verified){
	int ret;
	u8_t read_byte;
	u8_t buf_addr[] = {(u8_t)((addr >> 8) & 0xFF), (u8_t)((addr) & 0xFF)};

	*verified = false;

	/* Set adress pointer */
	ret = i2c_write(i2c_dev, buf_addr, sizeof(buf_addr), EEPROM_ADDR);
	if (ret) {
		LOG_ERR("I2C write failed: %d.",ret);
		return ret;
	}

	for (size_t i = 0; i < len; i++){
		i2c_read(i2c_dev, &read_byte, 1, EEPROM_ADDR);
		if (ret) {
			LOG_ERR("I2C read failed: %d.",ret);
			return ret;
		}

		if (read_byte != ((u8_t*)buf)[i]){
			LOG_INF("EEPROM verification mismatch. 0x%x has 0x%x, should be 0x%x",addr+i,read_byte,((u8_t*)buf)[i]);
			return 0;
		}

		LOG_INF("Byte verified: 0x%x has 0x%x",addr+i,read_byte);

	}

	*verified = true;
	return 0;
}

int eeprom_write(u16_t addr, const void *buf, size_t len, bool verify){
	/* struct i2c_msg msg[2]; */
	u8_t tx_buffer[34]; /* Separate buffers unusable with Zephyr ATM? */
	bool verified;
	int ret;

	u8_t buf_addr[] = {(u8_t)((addr >> 8) & 0xFF), (u8_t)((addr) & 0xFF)};
	
	/* Separate buffers unusable with Zephyr ATM? */
	/*
	msg[0].buf = buf_addr;
	msg[0].len = sizeof(buf_addr);
	msg[0].flags = I2C_MSG_WRITE;

	msg[1].buf = buf;
	msg[1].len = len;
	msg[1].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

	ret = i2c_transfer(i2c_dev, msg, 2, EEPROM_ADDR);
	*/

	memcpy(tx_buffer, buf_addr, 2);
	memcpy(tx_buffer + 2, buf, len);
	ret = i2c_write(i2c_dev, tx_buffer, len + 2, EEPROM_ADDR);
	if (ret) {
		LOG_ERR("I2C transfer failed: %d.",ret);
		return ret;
	}

	k_sleep(6); /* 5 ms is the maximum time to complete a write cycle */

	if (verify == false){
		return 0;
	}

	ret = eeprom_verify(addr, buf, len, &verified);
	if (ret) {
		LOG_ERR("EEPROM verification failed: %d.",ret);
		return ret;
	}

	if (verified == 0){
		LOG_ERR("EEPROM mismatch on verify after write");
		return -EIO;
	}

	return 0;
}

int eeprom_read(u16_t addr, void *buf, size_t len){
	u8_t buf_addr[] = {(u8_t)((addr >> 8) & 0xFF), (u8_t)((addr) & 0xFF)};
	return i2c_write_read(i2c_dev, EEPROM_ADDR, buf_addr, sizeof(addr), buf, len);
}

int eeprom_write_board_id(void){
	struct board_id board = {
		.magic = BOARD_ID_MAGIC,
		.pca = 20035,
		.version = 0x01040000,
	};
	int ret;

	board_id_make_checksum(&board);
	
	ret = eeprom_write(0x07E0, &board, sizeof(board), true);
	if (ret){
		LOG_ERR("Writing board ID failed: %d", ret);
		return ret;
	}

	return 0;
}

int eeprom_unprotect(void){
	u8_t buf = 0x40;
	int ret;
	
	ret = eeprom_write(0x8000, &buf, 1, false);
	if (ret){
		LOG_ERR("Unprotect failed: %d", ret);
		return ret;
	}

	ret = eeprom_read(0x8000, &buf, 1);
	if (ret){
		LOG_ERR("Unprotect verification failed: %d", ret);
		return ret;
	}

	
	if (buf != 0x00){
		LOG_ERR("Unprotect verification mismatch. Read value: 0x%x",buf);
		return -EIO;
	}

	return 0;
}

int eeprom_protect(void){
	const u8_t protect_upper_quarter = 0x48;
	const u8_t protect_upper_quarter_and_lock = 0x69;
	u8_t config_read;
	int ret;
	
	ret = eeprom_write(0x8000, &protect_upper_quarter, 1, false);
	if (ret){
		LOG_ERR("Protect failed: %d", ret);
		return ret;
	}

	ret = eeprom_read(0x8000, &config_read, 1);
	if (ret){
		LOG_ERR("Protect verification failed: %d", ret);
		return ret;
	}

	
	if (config_read != 0x08){ // 0x09 if locked
		LOG_ERR("Protect verification mismatch. Read value: 0x%x",config_read);
		return -EIO;
	}

	return 0;
}

int main(void){
	int ret;

	LOG_INF("EEPROM fw started");

	ret = app_i2c_init();
	LOG_INF("I2C Init->%d",ret);

	bool present = eeprom_is_present();
	LOG_INF("EEPROM Present: %d",present);

	ret = eeprom_write_board_id();
	LOG_INF("EEPROM write board ID->%d",ret);

	ret = eeprom_protect();
	LOG_INF("EEPROM protect->%d",ret);
}