/***************************************************************************//**
 *   @file   LTC268X.c
 *   @brief  Implementation of LTC2686/8 Driver.
 *   @author Mircea Caprioru (mircea.caprioru@analog.com)
 *   @port to C++ and CS controlled by I2C by Lautaro Narvaez (lautaro@caltech.edu)
 */

 

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include <SPI.h>
#include <Wire.h>
#include "ltc268x.h" /* LTC268X definitions. */

const struct ltc268x_span_tbl ltc268x_span_tbl[] = {
  [LTC268X_VOLTAGE_RANGE_0V_5V] = {0, 5},
  [LTC268X_VOLTAGE_RANGE_0V_10V] = {0, 10},
  [LTC268X_VOLTAGE_RANGE_M5V_5V] = {-5, 5},
  [LTC268X_VOLTAGE_RANGE_M10V_10V] = {-10, 10},
  [LTC268X_VOLTAGE_RANGE_M15V_15V] = {-15, 15}
};
/******************************************************************************/
/************************ Functions Definitions *******************************/
/******************************************************************************/


ltc268x::ltc268x(   enum ltc268x_device_id       dev_id, 
                    uint16_t                    pwd_dac_setting, 
                    uint16_t                    dither_toggle_en, 
                    bool                        dither_mode[16],
                    enum ltc268x_voltage_range  crt_range[16],
                    enum ltc268x_dither_phase   dither_phase[16],
                    enum ltc268x_dither_period  dither_period[16],
                    enum ltc268x_clk_input      clk_input[16],
                    enum ltc268x_a_b_register   reg_select[16],
                    int                         address,
                    TwoWire                     *bus,
                    int                         boardNum   ){
                      
  uint8_t channel = 0;
  int ret;
  this->boardN = boardNum;
  SPI.begin();
  //ret = no_os_spi_init(&dev->spi_desc, &init_param.spi_init);
  //if (ret < 0)
  //  goto error;
   BoardSel = new PCA9557(address, bus);
   if (!BoardSel->pinMode(PCA9557busDAC_CS,OUTPUT)) {
          sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", boardN);
          Serial.print(_err);
          //rv = -1;
      }
      if (!BoardSel->pinMode(PCA9557busDAC_CLR,OUTPUT)) {
          sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", boardN);
          Serial.print(_err);
          //rv = -1;
      }
      if (!BoardSel->pinMode(PCA9557busDAC_LDAC,OUTPUT)) {
          sprintf(_err, "Error setting pin mode for address 0x%1x ADC. Aborting...\n", boardN);
          Serial.print(_err);
          //rv = -1;
      }

      // Set the address pin HIGH so we don't talk to multiple chips at once. 

      if (!BoardSel->digitalWrite(PCA9557busDAC_CS,HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", boardN);
          Serial.print(_err);
          //rv = -1;
      }      

      if (!BoardSel->digitalWrite(PCA9557busDAC_CLR, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", boardN);
          Serial.print(_err);
          //rv = -1;
      }
      if (!BoardSel->digitalWrite(PCA9557busDAC_LDAC, HIGH)) {
          sprintf(_err, "Error setting address pin for board %i HIGH\n", boardN);
          Serial.print(_err);
      }
      
  ret = this->software_reset();
  if (ret < 0)
    goto error;

  delay(100);

  this->dev_id = dev_id;
  if (dev_id == LTC2686) {
    this->num_channels = 8;
  } else if (dev_id == LTC2688) {
    this->num_channels = 16;
  }

  /* Powerdown/up channels */
  ret = this->set_pwr_dac(pwd_dac_setting);
  if (ret < 0)
    goto error;

  /* Enable dither/toggle */
  ret = this->set_dither_toggle(dither_toggle_en);
  if (ret < 0)
    goto error;

  for (channel = 0; channel < this->num_channels; channel++) {
    /* Setup channel span */
    ret = this->set_span(channel, crt_range[channel]);
    if (ret < 0)
      goto error;
     /* Serial.print("ctr_range i :");
    Serial.println( crt_range[channel]);*/
    /* Set dither phase */
    ret = this->set_dither_phase(channel, dither_phase[channel]);
    if (ret < 0)
      goto error;

    /* Set dither period */
    ret = this->set_dither_period(channel,
            dither_period[channel]);
    if (ret < 0)
      goto error;

    ret = this->set_dither_mode(channel, dither_mode[channel]);
    if (ret < 0)
      goto error;

    /* Set toggle/dither clock */
    ret = this->select_tg_dith_clk(channel, clk_input[channel]);
    if (ret < 0)
      goto error;
  }

  /* Update all dac channels */
  //init at midscale
  ret = this->spi_write(LTC268X_CMD_UPDATE_ALL, 0);
  //ret = this->spi_write(LTC268X_CMD_SETTING_UPDATE_ALL, 0x8000);
  if (ret < 0)
    goto error;


  //BoardSel->digitalWrite(PCA9557busDAC_LDAC, HIGH);
  Serial.print("LTC268X successfully initialized\n");


error:
  Serial.print("LTC268X initialization error ");
  Serial.println(ret);

  }

int32_t ltc268x::spi_write(uint8_t reg, uint16_t data){
	uint8_t buf[3];
	int32_t ret;
  //uint8_t rx_array[4];

	buf[0] = reg;
  buf[1] = (data & 0xFF00) >> 8;
	buf[2] = data & 0x00FF;
  
  ret=0;
	//ret = no_os_spi_write_and_read(dev->spi_desc, buf, 3);
  BoardSel->digitalWrite(PCA9557busDAC_CS, LOW);
  SPI.beginTransaction(SPISettings(SPIBAUD, MSBFIRST, SPIMODE));
  //for (int i=2;  i >= 0; i--){
  for (int i=0;  i < 3; i++){
    //DEBUG_PRINTELN("data sent over SPI " + String(buf[i],HEX));
    //rx_array[i] = 
    SPI.transfer(buf[i]);    //! 2) Read and send byte array
  }
  SPI.endTransaction();
  BoardSel->digitalWrite(PCA9557busDAC_CS, HIGH);
	return ret;
}

/**
 * SPI read from device.
 * @param dev - The device structure.
 * @param reg - The register address.
 * @param data - The register data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::spi_read(uint8_t reg, uint16_t *data)
{
	uint8_t buf[3] = {0, 0, 0};
	int32_t ret;
  uint8_t rx_array[3];

	this->spi_write(reg | LTC268X_READ_OPERATION, 0x0000);

	buf[0] = LTC268X_CMD_NOOP;
	//ret = no_os_spi_write_and_read(dev->spi_desc, buf, 3);
  BoardSel->digitalWrite(PCA9557busDAC_CS, LOW);
  SPI.beginTransaction(SPISettings(SPIBAUD, MSBFIRST, SPIMODE));
  //for (int i=2;  i >= 0; i--){
  for (int i=0;  i < 3; i++){
    rx_array[i] = SPI.transfer(buf[i]);    //! 2) Read and send byte array
  }
  SPI.endTransaction();
  BoardSel->digitalWrite(PCA9557busDAC_CS, HIGH);
	*data = (rx_array[1] << 8) | rx_array[2];

	return ret;
}

/**
 * SPI readback register from device.
 * @param dev - The device structure.
 * @param reg - The register address.
 * @param mask - The register mask.
 * @param val - The updated value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::spi_update_bits(uint8_t reg,	uint16_t mask, uint16_t val)
{
	uint16_t regval;
	int32_t ret;

	ret = this->spi_read(reg, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return this->spi_write(reg, regval);
}

/**
 * Power down the selected channels.
 * @param dev - The device structure.
 * @param setting - The setting.
 *		    Accepted values: LTC268X_PWDN(x) | LTC268X_PWDN(y) | ...
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_pwr_dac(uint16_t setting)
{
	int32_t ret;

	ret = this->spi_write(LTC268X_CMD_POWERDOWN_REG, setting);
	if (ret < 0)
		return ret;

	this->pwd_dac_setting = setting;

	return 0;
}

/**
 * Enable dither/toggle for selected channels.
 * @param dev - The device structure.
 * @param setting - The setting.
 *		    Accepted values: LTC268X_DITH_EN(x) | LTC268X_DITH_EN(y) | ...
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_dither_toggle(uint16_t setting)
{
	int32_t ret;

	ret = this->spi_write(LTC268X_CMD_TOGGLE_DITHER_EN_REG, setting);
	if (ret < 0)
		return ret;

	this->dither_toggle_en = setting;

	return 0;
}

/**
 * Set channel to dither mode.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param en - enable or disable dither mode
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_dither_mode(uint8_t channel,bool en){
	uint16_t val = 0;
	int32_t ret;

	if (channel >= this->num_channels)
		return -ENOENT;

	if (en)
		val = LTC268X_CH_MODE;

	ret = this->spi_update_bits(LTC268X_CMD_CH_SETTING(channel,
				       this->dev_id),
				       LTC268X_CH_MODE, val);
	if (ret < 0)
		return ret;

	this->dither_mode[channel] = en;

	return 0;
}

/**
 * Set channel span.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param range - Voltage range.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_span(uint8_t channel, enum ltc268x_voltage_range range){
	int32_t ret;

	if (channel >= this->num_channels)
		return -ENOENT;

	ret = this->spi_update_bits(LTC268X_CMD_CH_SETTING(channel, this->dev_id), LTC268X_CH_SPAN_MSK, LTC268X_CH_SPAN(range));
	if (ret < 0)
		return ret;

	this->crt_range[channel] = range;

	return 0;
}

/**
 * Set channel dither phase.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param phase - Dither phase.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_dither_phase(uint8_t channel, enum  ltc268x_dither_phase phase){
	int32_t ret;
	if (channel >= this->num_channels)
		return -ENOENT;

	ret = this->spi_update_bits(LTC268X_CMD_CH_SETTING(channel,this->dev_id), LTC268X_CH_DIT_PH_MSK, LTC268X_CH_DIT_PH(phase));
	if (ret < 0)
		return ret;

	this->dither_phase[channel] = phase;

	return 0;
}

/**
 * Set channel dither period.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param period - Dither period.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::set_dither_period(uint8_t channel, enum  ltc268x_dither_period period){
	int32_t ret;

	if (channel >= this->num_channels)
		return -ENOENT;

	ret = this->spi_update_bits(LTC268X_CMD_CH_SETTING(channel, this->dev_id), LTC268X_CH_DIT_PER_MSK, LTC268X_CH_DIT_PER(period));
	if (ret < 0)
		return ret;

	this->dither_period[channel] = period;

	return 0;
}

/**
 * Select register A or B for value.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param sel_reg - Select register A or B to store DAC output value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::select_reg(uint8_t channel, enum  ltc268x_a_b_register sel_reg){
	int32_t ret;

	if (channel >= this->num_channels)
		return -ENOENT;

	ret = ltc268x::spi_update_bits(LTC268X_CMD_A_B_SELECT_REG, NO_OS_BIT(channel), sel_reg << channel);
	if (ret < 0)
		return ret;

	this->reg_select[channel] = sel_reg;

	return 0;
}

/**
 * Select dither/toggle clock input.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @param clk_input - Select the source for the clock input.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::select_tg_dith_clk(uint8_t channel, enum  ltc268x_clk_input clk_input){
	int32_t ret;

	if (channel >= this->num_channels)
		return -ENOENT;

	ret = ltc268x::spi_update_bits(LTC268X_CMD_CH_SETTING(channel,this->dev_id), LTC268X_CH_TD_SEL_MSK, LTC268X_CH_TD_SEL(clk_input));
	if (ret < 0)
		return ret;

	this->clk_input[channel] = clk_input;

	return 0;
}

/**
 * Toggle the software source for dither/toggle.
 * @param dev - The device structure.
 * @param channel - The channel for which to change the mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::software_toggle(uint8_t channel){
	int32_t ret;
	uint16_t regval;

	if (channel >= this->num_channels)
		return -ENOENT;

	ret = this->spi_read(LTC268X_CMD_SW_TOGGLE_REG, &regval);
	if (ret < 0)
		return ret;

	regval ^= NO_OS_BIT(channel);

	return this->spi_write(LTC268X_CMD_SW_TOGGLE_REG, regval);
}

/**
 * Software reset the device.
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ltc268x::software_reset()
{
	return this->spi_update_bits(LTC268X_CMD_CONFIG_REG, LTC268X_CONFIG_RST, LTC268X_CONFIG_RST);
}

/**
 *  Sets the output voltage of a channel.
 *
 * @param dev     - The device structure.
 * @param channel - Channel option.
 *
 * @param voltage - Value to be outputted by the DAC(Volts).
 *
 * @return The actual voltage value that can be outputted by the channel.
 */
int32_t ltc268x::set_voltage(uint8_t channel, float voltage){
	uint16_t _offset, _gain, code;
	int32_t range_offset, v_ref, ret;

	/* Get the offset, gain and range of the selected channel. */
	ret = this->spi_read(LTC268X_CMD_CH_OFFSET(channel, this->dev_id),
				&_offset);
	if (ret < 0)
		return ret;

	ret = this->spi_read(LTC268X_CMD_CH_GAIN(channel, this->dev_id), &_gain);
	if (ret < 0)
		return ret;

	range_offset = ltc268x_span_tbl[this->crt_range[channel]].min;
	v_ref = ltc268x_span_tbl[this->crt_range[channel]].max -
		ltc268x_span_tbl[this->crt_range[channel]].min;
   
  /* Serial.print("channel: ");Serial.println(channel);
   Serial.print("voltage: ");Serial.println(voltage);
   Serial.print("range offset: ");Serial.println(range_offset);
   Serial.print("v_ref: ");Serial.println(v_ref);*/
	/* Compute the binary code from the value(mA) provided by user. */
	code = (uint32_t)((voltage - range_offset) * (1l << 16) / v_ref);
	if(code > 0xFFFF)
		code = 0xFFFF;

	this->dac_code[channel] = code;

	/* Write to the Data Register of the DAC. */
	return this->spi_write(LTC268X_CMD_CH_CODE_UPDATE(channel, this->dev_id), code);
}

/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 *		       parameters.
 * @return 0 in case of success, negative error code otherwise.
 */

	


/**
 * @brief Free the resources allocated by ltc268x_init().
 * @param dev - The device structure.
 * @return 0 in case of success, negative error code otherwise.
 */
