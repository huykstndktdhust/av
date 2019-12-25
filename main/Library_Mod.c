/**
  ******************************************************************************
  *   @file Library_Mod.c
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.5
  *   @date 09-05-2015
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Library_Mod.h"
#include "ExternVariablesFunctions.h"

/* Exported functions ------------------------------------------------------- */

static uint8_t I2C_MasterSelectRegister(uint8_t dev, uint8_t reg){
    esp_err_t error_esp = ESP_OK;
    i2c_cmd_handle_t cmd;
    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, dev | I2C_MASTER_WRITE, ACK_CHECK_EN));//(dev << 1)
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    I2C_ESP_ERROR_CHECK(error_esp, i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
    if(error_esp != ESP_OK)return LL_ERROR;
    return LL_OK;
}

uint8_t I2C_MasterMemoryRead(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    esp_err_t error_esp = ESP_OK;
    i2c_cmd_handle_t cmd;
    error_esp = I2C_MasterSelectRegister(devAddr, regAddr);
    if(error_esp != LL_OK)return LL_ERROR;

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, devAddr | I2C_MASTER_READ, ACK_CHECK_EN));//(devAddr << 1)

    if(length > 1){
      ESP_ERROR_CHECK(i2c_master_read(cmd, data, length-1, ACK_VAL));}

    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+length-1, NACK_VAL));

    ESP_ERROR_CHECK(i2c_master_stop(cmd));
    I2C_ESP_ERROR_CHECK(error_esp, i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    if(error_esp != ESP_OK)return LL_ERROR;
    return LL_OK;
}

uint8_t I2C_MasterMemoryWrite(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
  esp_err_t error_esp = ESP_OK;
  i2c_cmd_handle_t cmd;

  cmd = i2c_cmd_link_create();
  ESP_ERROR_CHECK(i2c_master_start(cmd));
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, devAddr | I2C_MASTER_WRITE, ACK_CHECK_EN));//(devAddr << 1)
  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN));

  //if(length > 1) {
    ESP_ERROR_CHECK(i2c_master_write(cmd, data, length-1, ACK_CHECK_EN));//}

  ESP_ERROR_CHECK(i2c_master_write_byte(cmd, data[length-1], ACK_CHECK_EN));
  ESP_ERROR_CHECK(i2c_master_stop(cmd));
  I2C_ESP_ERROR_CHECK(error_esp, i2c_master_cmd_begin(I2C_NUM, cmd, 1000/portTICK_PERIOD_MS));
  i2c_cmd_link_delete(cmd);

  if(error_esp != ESP_OK)return LL_ERROR;
  return LL_OK;
}
/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
    esp_err_t error_esp = ESP_OK;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;//GPIO_PULLUP_ENABLE;//
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;//GPIO_PULLUP_ENABLE;//
    conf.master.clk_speed = 400000;
    I2C_ESP_ERROR_CHECK(error_esp, i2c_param_config(I2C_NUM, &conf));
    if(error_esp != ESP_OK)return error_esp;
    I2C_ESP_ERROR_CHECK(error_esp, i2c_driver_install(I2C_NUM, conf.mode,
                                       I2C_MASTER_RX_BUF_DISABLE,
                                       I2C_MASTER_TX_BUF_DISABLE, 0));
    return error_esp;
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
