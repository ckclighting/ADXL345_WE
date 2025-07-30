/********************************************************************
* This is a library for the ADXL345 accelerometer.
*
* You'll find an example which should enable you to use the library. 
*
* You are free to use it, change it or build on it. In case you like 
* it, it would be cool if you give it a star.
* 
* If you find bugs, please inform me!
* 
* Written by Wolfgang (Wolle) Ewald
* https://wolles-elektronikkiste.de/adxl345-teil-1 (German)
* https://wolles-elektronikkiste.de/en/adxl345-the-universal-accelerometer-part-1 (English)
*
*********************************************************************/

#include "ADXL345_WE.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "Utils/Dimming/DimmingParam.h"
#include "Utils/Dimming/Dimming.h"


static QueueHandle_t  Int_Semaphore;//中断信号量



//中断服务函数
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = *(uint32_t*)arg;

    if(gpio_num == GPIO_INT_INPUT_IO)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(Int_Semaphore, &xHigherPriorityTaskWoken);//发布信号量
        if(xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR();//呼叫等待的任务
        }
    }
}


/************ Basic settings ************/
    
bool ADXL345_WE::init(){    

    if(useSPI){
        if(mosiPin == 999){
            _spi->begin();
        }
#ifdef ESP32
        else{
            _spi->begin(sckPin, misoPin, mosiPin, csPin);
        }
#endif
        mySPISettings = SPISettings(5000000, MSBFIRST, SPI_MODE3);
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
    }
    else
    {
        I2C_Init();
    }

    Int_Semaphore = xSemaphoreCreateBinary();//创建信号量

    rangeFactor = 1.0;
    corrFact.x = 1.0;
    corrFact.y = 1.0;
    corrFact.z = 1.0;
    offsetVal.x = 0.0;
    offsetVal.y = 0.0;
    offsetVal.z = 0.0;
    angleOffsetVal.x = 0.0;
    angleOffsetVal.y = 0.0;
    angleOffsetVal.z = 0.0;

    unsigned char DveID=0;
    DveID = this->getADXL345DeviceID();
    // ESP_LOGW("ADXL345DeviceID","0x%x",DveID);

    //清除寄存器
    writeRegister(ADXL345_POWER_CTL, 0);
    writeRegister(ADXL345_DATA_FORMAT, 0);
    writeRegister(ADXL345_INT_ENABLE, 0);
    writeRegister(ADXL345_INT_MAP, 0);
    writeRegister(ADXL345_THRESH_ACT, 0);
    writeRegister(ADXL345_TIME_INACT, 0);
    writeRegister(ADXL345_THRESH_INACT, 0);
    writeRegister(ADXL345_ACT_INACT_CTL, 0);
    writeRegister(ADXL345_DUR, 0);
    writeRegister(ADXL345_LATENT, 0);
    writeRegister(ADXL345_THRESH_TAP, 0);
    writeRegister(ADXL345_TAP_AXES, 0);
    writeRegister(ADXL345_WINDOW, 0);
    uint8_t IntSource =readAndClearInterrupts();
    // ESP_LOGW("ADXL345_IntSrc","0x%x",IntSource);
    writeRegister(ADXL345_FIFO_CTL, 0);
    writeRegister(ADXL345_FIFO_STATUS, 0);


    // //设置寄存器
    setMeasureMode(true);//开启测量模式
    setFullRes(true);//全分辨率
    setRange(ADXL345_RANGE_2G);//2G量程
    setDataRate(ADXL345_DATA_RATE_25);
    setInactivityParameters(ADXL345_DC_MODE, ADXL345_XYZ, 0.3, 100);
    setFreeFallThresholds(0.3, 100);//设置自由落体加速度阈值和时间
    setInterruptPolarity(ADXL345_ACT_LOW);
    setInterrupt(ADXL345_FREEFALL, INT_PIN_1);//开启freeFall中断

    uint8_t IntEnable =readRegister8(ADXL345_INT_ENABLE);
    // ESP_LOGW("ADXL345_IntEn","0x%x",IntEnable);

    xTaskCreatePinnedToCore(ADXL345_IntHandle_Task, "ADXL345_IntHandle_Task", 4096, this, 20, &Test_taskHandle, APP_CPU_NUM);

    return true;
}

void ADXL345_WE::setSPIClockSpeed(unsigned long clock){
    mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE3);
}

unsigned char ADXL345_WE::getADXL345DeviceID(void)
{
    return readRegister8(ADXL345_DEVID);
}

void ADXL345_WE::setCorrFactors(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    corrFact.x = UNITS_PER_G / (0.5 * (xMax - xMin));
    corrFact.y = UNITS_PER_G / (0.5 * (yMax - yMin));
    corrFact.z = UNITS_PER_G / (0.5 * (zMax - zMin));
    offsetVal.x = (xMax + xMin) * 0.5;
    offsetVal.y = (yMax + yMin) * 0.5;
    offsetVal.z = (zMax + zMin) * 0.5;
}

void ADXL345_WE::setDataRate(adxl345_dataRate rate){
    regVal |= readRegister8(ADXL345_BW_RATE);
    regVal &= 0xF0;
    regVal |= rate;
    writeRegister(ADXL345_BW_RATE, regVal);
}
    
adxl345_dataRate ADXL345_WE::getDataRate(){
    return (adxl345_dataRate)(readRegister8(ADXL345_BW_RATE) & 0x0F);
}

String ADXL345_WE::getDataRateAsString(){
    adxl345_dataRate dataRate = (adxl345_dataRate)(readRegister8(ADXL345_BW_RATE) & 0x0F);
    String returnString = "";
    
    switch(dataRate) {
        case ADXL345_DATA_RATE_3200: returnString = "3200 Hz"; break;
        case ADXL345_DATA_RATE_1600: returnString = "1600 Hz"; break;
        case ADXL345_DATA_RATE_800:  returnString = "800 Hz";  break;
        case ADXL345_DATA_RATE_400:  returnString = "400 Hz";  break;
        case ADXL345_DATA_RATE_200:  returnString = "200 Hz";  break;
        case ADXL345_DATA_RATE_100:  returnString = "100 Hz";  break;
        case ADXL345_DATA_RATE_50:   returnString = "50 Hz";   break;
        case ADXL345_DATA_RATE_25:   returnString = "25 Hz";   break;
        case ADXL345_DATA_RATE_12_5: returnString = "12.5 Hz"; break;
        case ADXL345_DATA_RATE_6_25: returnString = "6.25 Hz"; break;
        case ADXL345_DATA_RATE_3_13: returnString = "3.13 Hz"; break;
        case ADXL345_DATA_RATE_1_56: returnString = "1.56 Hz"; break;
        case ADXL345_DATA_RATE_0_78: returnString = "0.78 Hz"; break;
        case ADXL345_DATA_RATE_0_39: returnString = "0.39 Hz"; break;
        case ADXL345_DATA_RATE_0_20: returnString = "0.20 Hz"; break;
        case ADXL345_DATA_RATE_0_10: returnString = "0.10 Hz"; break;
    }
    
    return returnString;
}

uint8_t ADXL345_WE::getPowerCtlReg(){
    return readRegister8(ADXL345_POWER_CTL);
}

void ADXL345_WE::setRange(adxl345_range range){
    uint8_t regVal = readRegister8(ADXL345_DATA_FORMAT);
    if(adxl345_lowRes){
        switch(range){
            case ADXL345_RANGE_2G:  rangeFactor = 1.0;  break;
            case ADXL345_RANGE_4G:  rangeFactor = 2.0;  break;
            case ADXL345_RANGE_8G:  rangeFactor = 4.0;  break;
            case ADXL345_RANGE_16G: rangeFactor = 8.0;  break;  
        }
    }
    else{
        rangeFactor = 1.0;
    }
    regVal &= 0b11111100;
    regVal |= range;
    writeRegister(ADXL345_DATA_FORMAT, regVal);
}

adxl345_range ADXL345_WE::getRange(){
    regVal = readRegister8(ADXL345_DATA_FORMAT);
    regVal &= 0x03; 
    return adxl345_range(regVal);
}

void ADXL345_WE::setFullRes(bool full){
    regVal = readRegister8(ADXL345_DATA_FORMAT);
    if(full){
        adxl345_lowRes = false;
        rangeFactor = 1.0;
        regVal |= (1<<ADXL345_FULL_RES);
    }
    else{
        adxl345_lowRes = true;
        regVal &= ~(1<<ADXL345_FULL_RES);
        setRange(getRange());
    }
    writeRegister(ADXL345_DATA_FORMAT, regVal);
}

String ADXL345_WE::getRangeAsString(){
    String rangeAsString = "";
    adxl345_range range = getRange();
    switch(range){
        case ADXL345_RANGE_2G:  rangeAsString = "2g";   break;
        case ADXL345_RANGE_4G:  rangeAsString = "4g";   break;
        case ADXL345_RANGE_8G:  rangeAsString = "8g";   break;
        case ADXL345_RANGE_16G: rangeAsString = "16g";  break;
        
    }
    return rangeAsString;
}

/************ x,y,z results ************/

xyzFloat ADXL345_WE::getRawValues(){
    uint8_t rawData[6]; 
    xyzFloat rawVal = {0.0, 0.0, 0.0};
    readMultipleRegisters(ADXL345_DATAX0, 6, rawData);
    rawVal.x = (static_cast<int16_t>((rawData[1] << 8) | rawData[0])) * 1.0;
    rawVal.y = (static_cast<int16_t>((rawData[3] << 8) | rawData[2])) * 1.0;
    rawVal.z = (static_cast<int16_t>((rawData[5] << 8) | rawData[4])) * 1.0;
    
    return rawVal;
}

xyzFloat ADXL345_WE::getCorrectedRawValues(){
    uint8_t rawData[6]; 
    xyzFloat rawVal = {0.0, 0.0, 0.0};
    readMultipleRegisters(ADXL345_DATAX0, 6, rawData);
    int16_t xRaw = ((int16_t)rawData[1] << 8) | (int16_t)rawData[0];
    int16_t yRaw = ((int16_t)rawData[3] << 8) | (int16_t)rawData[2];
    int16_t zRaw = ((int16_t)rawData[5] << 8) | (int16_t)rawData[4];

    // ESP_LOGI("xRaw","0x%x",xRaw);
    // ESP_LOGI("yRaw","0x%x",yRaw);
    // ESP_LOGI("zRaw","0x%x",zRaw);

    rawVal.x = xRaw * 1.0 - (offsetVal.x / rangeFactor);
    rawVal.y = yRaw * 1.0 - (offsetVal.y / rangeFactor);
    rawVal.z = zRaw * 1.0 - (offsetVal.z / rangeFactor);
    
    return rawVal;
}

xyzFloat ADXL345_WE::getGValues(){
    xyzFloat rawVal = getCorrectedRawValues();
    xyzFloat gVal = {0.0, 0.0, 0.0};
    gVal.x = rawVal.x * MILLI_G_PER_LSB * rangeFactor * corrFact.x / 1000.0;
    gVal.y = rawVal.y * MILLI_G_PER_LSB * rangeFactor * corrFact.y / 1000.0;
    gVal.z = rawVal.z * MILLI_G_PER_LSB * rangeFactor * corrFact.z / 1000.0;
    return gVal;
}

xyzFloat ADXL345_WE::getAngles(){
    xyzFloat gVal = getGValues();
    xyzFloat angleVal = {0.0, 0.0, 0.0};
    if(gVal.x > 1){
        gVal.x = 1;
    }
    else if(gVal.x < -1){
        gVal.x = -1;
    }
    angleVal.x = (asin(gVal.x)) * 57.296;
    
    if(gVal.y > 1){
        gVal.y = 1;
    }
    else if(gVal.y < -1){
        gVal.y = -1;
    }
    angleVal.y = (asin(gVal.y)) * 57.296;
    
    if(gVal.z > 1){
        gVal.z = 1;
    }
    else if(gVal.z < -1){
        gVal.z = -1;
    }
    angleVal.z = (asin(gVal.z)) * 57.296;
    
    return angleVal;
}

xyzFloat ADXL345_WE::getCorrAngles(){
    xyzFloat corrAnglesVal = getAngles();
    corrAnglesVal.x -= angleOffsetVal.x;
    corrAnglesVal.y -= angleOffsetVal.y;
    corrAnglesVal.z -= angleOffsetVal.z;
        
    return corrAnglesVal;
}

/************ Angles and Orientation ************/ 

void ADXL345_WE::measureAngleOffsets(){
    angleOffsetVal = getAngles();
}

xyzFloat ADXL345_WE::getAngleOffsets(){
    return angleOffsetVal;
}

void ADXL345_WE::setAngleOffsets(xyzFloat aos){
    angleOffsetVal = aos;
}

adxl345_orientation ADXL345_WE::getOrientation(){
    adxl345_orientation orientation = FLAT;
    xyzFloat angleVal = getAngles();
    if(abs(angleVal.x) < 45){      // |x| < 45
        if(abs(angleVal.y) < 45){      // |y| < 45
            if(angleVal.z > 0){          //  z  > 0
                orientation = FLAT;
            }
            else{                        //  z  < 0
                orientation = FLAT_1;
            }
        }
        else{                         // |y| > 45 
            if(angleVal.y > 0){         //  y  > 0
                orientation = XY;
            }
            else{                       //  y  < 0
                orientation = XY_1;   
            }
        }
    }
    else{                           // |x| >= 45
        if(angleVal.x > 0){           //  x  >  0
            orientation = YX;       
        }
        else{                       //  x  <  0
            orientation = YX_1;
        }
    }
    return orientation;
}

String ADXL345_WE::getOrientationAsString(){
    adxl345_orientation orientation = getOrientation();
    String orientationAsString = "";
    switch(orientation){
        case FLAT:      orientationAsString = "z up";   break;
        case FLAT_1:    orientationAsString = "z down"; break;
        case XY:        orientationAsString = "y up";   break;
        case XY_1:      orientationAsString = "y down"; break;
        case YX:        orientationAsString = "x up";   break;
        case YX_1:      orientationAsString = "x down"; break;
    }
    return orientationAsString;
}

float ADXL345_WE::getPitch(){
    xyzFloat gVal = getGValues();
    float pitch = (atan2(-gVal.x, sqrt(abs((gVal.y*gVal.y + gVal.z*gVal.z))))*180.0)/M_PI;
    return pitch;
}
    
float ADXL345_WE::getRoll(){
    xyzFloat gVal = getGValues();
    float roll = (atan2(gVal.y, gVal.z)*180.0)/M_PI;
    return roll;
}

/************ Power, Sleep, Standby ************/ 

void ADXL345_WE::setMeasureMode(bool measure){
    regVal = readRegister8(ADXL345_POWER_CTL);
    if(measure){
        regVal |= (1<<ADXL345_MEASURE);
    }
    else{
        regVal &= ~(1<<ADXL345_MEASURE);
    }
    writeRegister(ADXL345_POWER_CTL, regVal);
}

void ADXL345_WE::setSleep(bool sleep, adxl345_wUpFreq freq){
    regVal = readRegister8(ADXL345_POWER_CTL);
    regVal &= 0b11111100;
    regVal |= freq;
    if(sleep){
        regVal |= (1<<ADXL345_SLEEP);
    }
    else{
        setMeasureMode(false);  // it is recommended to enter Stand Mode when clearing the Sleep Bit!
        regVal &= ~(1<<ADXL345_SLEEP);
        regVal &= ~(1<<ADXL345_MEASURE);
    }
    writeRegister(ADXL345_POWER_CTL, regVal);
    if(!sleep){
        setMeasureMode(true);
    }
}

void ADXL345_WE::setSleep(bool sleep){
    regVal = readRegister8(ADXL345_POWER_CTL);
    if(sleep){
        regVal |= (1<<ADXL345_SLEEP);
    }
    else{
        setMeasureMode(false);  // it is recommended to enter Stand Mode when clearing the Sleep Bit!
        regVal &= ~(1<<ADXL345_SLEEP);
        regVal &= ~(1<<ADXL345_MEASURE);
    }
    writeRegister(ADXL345_POWER_CTL, regVal);
    if(!sleep){
        setMeasureMode(true);
    }
}
    
void ADXL345_WE::setAutoSleep(bool autoSleep, adxl345_wUpFreq freq){
    if(autoSleep){
        setLinkBit(true);
    }
    regVal = readRegister8(ADXL345_POWER_CTL);
    regVal &= 0b11111100;
    regVal |= freq;
    if(autoSleep){
        regVal |= (1<<ADXL345_AUTO_SLEEP);      
    }
    else{
        regVal &= ~(1<<ADXL345_AUTO_SLEEP);
    }
    writeRegister(ADXL345_POWER_CTL, regVal);
}
        
void ADXL345_WE::setAutoSleep(bool autoSleep){
    if(autoSleep){
        setLinkBit(true);
        regVal = readRegister8(ADXL345_POWER_CTL);
        regVal |= (1<<ADXL345_AUTO_SLEEP);
        writeRegister(ADXL345_POWER_CTL, regVal);
    }
    else{
        regVal = readRegister8(ADXL345_POWER_CTL);
        regVal &= ~(1<<ADXL345_AUTO_SLEEP);
        writeRegister(ADXL345_POWER_CTL, regVal);
    }
        
}

bool ADXL345_WE::isAsleep(){
    return readRegister8(ADXL345_ACT_TAP_STATUS) & (1<<ADXL345_ASLEEP);
}

void ADXL345_WE::setLowPower(bool lowpwr){
    regVal = readRegister8(ADXL345_BW_RATE);
    if(lowpwr){
        regVal |= (1<<ADXL345_LOW_POWER);
    }
    else{
        regVal &= ~(1<<ADXL345_LOW_POWER);
    }
    writeRegister(ADXL345_BW_RATE, regVal);
}

bool ADXL345_WE::isLowPower(){
    return readRegister8(ADXL345_BW_RATE) & (1<<ADXL345_LOW_POWER);
}
            
/************ Interrupts ************/


void ADXL345_WE::setInterrupt(adxl345_int type, uint8_t pin){
    regVal = readRegister8(ADXL345_INT_ENABLE);
    regVal |= (1<<type);
    writeRegister(ADXL345_INT_ENABLE, regVal);
    regVal = readRegister8(ADXL345_INT_MAP);
    if(pin == INT_PIN_1){
        regVal &= ~(1<<type);
    }
    else {
        regVal |= (1<<type);
    }
    writeRegister(ADXL345_INT_MAP, regVal);
}

void ADXL345_WE::setInterruptPolarity(uint8_t pol){
    regVal = readRegister8(ADXL345_DATA_FORMAT);
    if(pol == ADXL345_ACT_HIGH){
        regVal &= ~(0b00100000);

        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //interrupt of falling edge
        io_conf.intr_type = GPIO_INTR_POSEDGE;//上升沿触发
        //bit mask of the pins
        io_conf.pin_bit_mask = GPIO_INT_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);

        gpio_install_isr_service(0);

        //hook isr handler for specific gpio pin
        Int_data = GPIO_INT_INPUT_IO;
        gpio_isr_handler_add(GPIO_INT_INPUT_IO, 
                                gpio_isr_handler, 
                                (void*)&Int_data);
    }
    else if(pol == ADXL345_ACT_LOW){
        regVal |= 0b00100000;

        //zero-initialize the config structure.
        gpio_config_t io_conf = {};
        //interrupt of falling edge
        io_conf.intr_type = GPIO_INTR_NEGEDGE;//下降沿触发
        //bit mask of the pins
        io_conf.pin_bit_mask = GPIO_INT_INPUT_PIN_SEL;
        //set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        //enable pull-up mode
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);

        gpio_install_isr_service(0);

        //hook isr handler for specific gpio pin
        Int_data = GPIO_INT_INPUT_IO;
        gpio_isr_handler_add(GPIO_INT_INPUT_IO, 
                                gpio_isr_handler, 
                                (void*)&Int_data);
    }
    writeRegister(ADXL345_DATA_FORMAT, regVal);
}

void ADXL345_WE::deleteInterrupt(adxl345_int type){
    regVal = readRegister8(ADXL345_INT_ENABLE);
    regVal &= ~(1<<type);
    writeRegister(ADXL345_INT_ENABLE, regVal);  
}

bool ADXL345_WE::WaitIntEvent(TickType_t xTicksToWait)
{
    bool regVal = pdFALSE;
    if(xSemaphoreTake(Int_Semaphore, xTicksToWait))//接收到中断信号量
    {
        regVal = pdTRUE;
    }
    else
        regVal = pdFALSE;

    return regVal;
}

uint8_t ADXL345_WE::readAndClearInterrupts(){
    regVal = readRegister8(ADXL345_INT_SOURCE);
    return regVal;
}

    bool ADXL345_WE::checkInterrupt(uint8_t source, adxl345_int type){
    source &= (1<<type);
    return source;
}

void ADXL345_WE::setLinkBit(bool link){
    regVal = readRegister8(ADXL345_POWER_CTL);
    if(link){
        regVal |= (1<<ADXL345_LINK);
    }
    else{
        regVal &= ~(1<<ADXL345_LINK);
    }
    writeRegister(ADXL345_POWER_CTL, regVal);
}

void ADXL345_WE::setFreeFallThresholds(float ffg, float fft){
    regVal = static_cast<uint8_t>(round(ffg / 0.0625));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_THRESH_FF, regVal);
    regVal = static_cast<uint8_t>(round(fft / 5));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_TIME_FF, regVal);
}

void ADXL345_WE::setActivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold){
    regVal = static_cast<uint8_t>(round(threshold / 0.0625));
    if(regVal<1){
        regVal = 1;
    }
    
    writeRegister(ADXL345_THRESH_ACT, regVal);

    regVal = readRegister8(ADXL345_ACT_INACT_CTL);
    regVal &= 0x0F;
    regVal |= (static_cast<uint8_t>(mode) + static_cast<uint8_t>(axes))<<4;
    writeRegister(ADXL345_ACT_INACT_CTL, regVal);
}

void ADXL345_WE::setInactivityParameters(adxl345_dcAcMode mode, adxl345_actTapSet axes, float threshold, uint8_t inactTime){
    regVal = static_cast<uint8_t>(round(threshold / 0.0625));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_THRESH_INACT, regVal);

    regVal = readRegister8(ADXL345_ACT_INACT_CTL);
    regVal &= 0xF0;
    regVal |= static_cast<uint8_t>(mode) + static_cast<uint16_t>(axes);
    writeRegister(ADXL345_ACT_INACT_CTL, regVal);

    writeRegister(ADXL345_TIME_INACT, inactTime);
}

void ADXL345_WE::setGeneralTapParameters(adxl345_actTapSet axes, float threshold, float duration, float latent){
    regVal = readRegister8(ADXL345_TAP_AXES);
    regVal &= 0b11111000;
    regVal |= static_cast<uint8_t>(axes);
    writeRegister(ADXL345_TAP_AXES, regVal);
    
    regVal = static_cast<uint8_t>(round(threshold / 0.0625));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_THRESH_TAP,regVal);
    
    regVal = static_cast<uint8_t>(round(duration / 0.625));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_DUR, regVal);
    
    regVal = static_cast<uint8_t>(round(latent / 1.25));
    if(regVal<1){
        regVal = 1;
    }
    writeRegister(ADXL345_LATENT, regVal);      
}

void ADXL345_WE::setAdditionalDoubleTapParameters(bool suppress, float window){
    regVal = readRegister8(ADXL345_TAP_AXES);
    if(suppress){
        regVal |= (1<<ADXL345_SUPPRESS);
    }
    else{
        regVal &= ~(1<<ADXL345_SUPPRESS);
    }
    writeRegister(ADXL345_TAP_AXES, regVal);
    
    regVal = static_cast<uint8_t>(round(window / 1.25));
    writeRegister(ADXL345_WINDOW, regVal);
}

uint8_t ADXL345_WE::getActTapStatus(){
    return readRegister8(ADXL345_ACT_TAP_STATUS);
}

String ADXL345_WE::getActTapStatusAsString(){
    uint8_t mask = (readRegister8(ADXL345_ACT_INACT_CTL)) & 0b01110000;
    mask |= ((readRegister8(ADXL345_TAP_AXES)) & 0b00000111);
        
    String returnStr = "";
    regVal = readRegister8(ADXL345_ACT_TAP_STATUS); 
    regVal &= mask;
        
    if(regVal & (1<<ADXL345_TAP_Z)) { returnStr += "TAP-Z "; }
    if(regVal & (1<<ADXL345_TAP_Y)) { returnStr += "TAP-Y "; }
    if(regVal & (1<<ADXL345_TAP_X)) { returnStr += "TAP-X "; }
    if(regVal & (1<<ADXL345_ACT_Z)) { returnStr += "ACT-Z "; }
    if(regVal & (1<<ADXL345_ACT_Y)) { returnStr += "ACT-Y "; }
    if(regVal & (1<<ADXL345_ACT_X)) { returnStr += "ACT-X "; }
    
    return returnStr;
}

/************ FIFO ************/

void ADXL345_WE::setFifoParameters(adxl345_triggerInt intNumber, uint8_t samples){
    regVal = readRegister8(ADXL345_FIFO_CTL);
    regVal &= 0b11000000;
    regVal |= (samples-1);
    if(intNumber == ADXL345_TRIGGER_INT_2){
        regVal |= 0x20;
    }
    writeRegister(ADXL345_FIFO_CTL, regVal);
}

void ADXL345_WE::setFifoMode(adxl345_fifoMode mode){
    regVal = readRegister8(ADXL345_FIFO_CTL);
    regVal &= 0b00111111;
    regVal |= (mode<<6);
    writeRegister(ADXL345_FIFO_CTL,regVal);
}

uint8_t ADXL345_WE::getFifoStatus(){
    return readRegister8(ADXL345_FIFO_STATUS);
}

void ADXL345_WE::resetTrigger(){
    setFifoMode(ADXL345_BYPASS);
    setFifoMode(ADXL345_TRIGGER);
}


/************************************************ 
    private functions
*************************************************/
void ADXL345_WE::I2C_Init(void)
{
    /*IIC 设备注册*/
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = _sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = _scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    i2c_bus.bus = i2c_bus_create(_port, &conf);//创建总线
    i2c_bus.device = i2c_bus_device_create(i2c_bus.bus, i2cAddress, 0);//创建设备
}

uint8_t ADXL345_WE::writeRegister(uint8_t reg, uint8_t val){
    if(!useSPI){
        // _wire->beginTransmission(i2cAddress);
        // _wire->write(reg);
        // _wire->write(val);
  
        // return _wire->endTransmission();

        esp_err_t err=0;
        err = i2c_bus_write_bytes(i2c_bus.device, reg, 1, (unsigned char*)&val);
        return err;
    }
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        _spi->transfer(val);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
        return false; // to be amended
    }

}
  
uint8_t ADXL345_WE::readRegister8(uint8_t reg){
    uint8_t regValue = 0;
    if(!useSPI){    
        // _wire->beginTransmission(i2cAddress);
        // _wire->write(reg);
        // _wire->endTransmission(false);
        // _wire->requestFrom(i2cAddress, static_cast<uint8_t>(1));
        // if(_wire->available()){
        //     regValue = _wire->read();
        // }

        i2c_bus_read_bytes(i2c_bus.device, reg, 1, (unsigned char*)&regValue);
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        regValue = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }

    return regValue;
}

void ADXL345_WE::readMultipleRegisters(uint8_t reg, uint8_t count, uint8_t *buf){
    if(!useSPI){
        // _wire->beginTransmission(i2cAddress);
        // _wire->write(reg);
        // _wire->endTransmission(false);
        // _wire->requestFrom(i2cAddress,count);
        // for(int i=0; i<count; i++){
        //     buf[i] = _wire->read();
        // }    

        i2c_bus_read_bytes(i2c_bus.device, reg, count, buf);
    }
    else{
        reg = reg | 0x80;
        reg = reg | 0x40;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        for(int i=0; i<count; i++){
            buf[i] = _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }

}

/* Free Fall */
void ADXL345_WE::SetFreeFallLog(unsigned int val)
{
    FreeFall_cnt = val;
};
unsigned int ADXL345_WE::ReadFreeFallLog(void)
{
    return FreeFall_cnt;
};
unsigned char ADXL345_WE::FreeFall_UserCallback_Register(FreeFall_UserCallback_t user_callback)
{
    if(user_callback == NULL)return 0;

    this->FreeFall_CallBack = user_callback;

    return 1;
};


/* Y Axis Flip */
unsigned char ADXL345_WE::Y_AxisFlip_UserCallback_Register(Y_AxisFlip_UserCallback_t user_callback)
{
    if(user_callback == NULL)return 0;

    this->Y_AxisFlip_CallBack = user_callback;

    return 1;
};


/*Int Handle*/
void ADXL345_WE::ADXL345_IntHandle_Task(void *parameters)
{
    ADXL345_WE * prt = (ADXL345_WE*)parameters;
    static unsigned short flick=0;
    while(1)
    {
        if(prt->WaitIntEvent(1000))
        {
            unsigned char temp = 0;
            temp = prt->readAndClearInterrupts();

            if(temp & (0x01 << ADXL345_FREEFALL))
            {
                prt->FreeFall_cnt ++ ;
                if(prt->FreeFall_CallBack)
                    prt->FreeFall_CallBack(prt->FreeFall_cnt);
            }

            // if(!flick)
            // {
            //     flick = ~flick;
            // }
            // else
            // {
            //     flick = ~flick;
            // }
            // DimmingParam dimParam;
            // for (size_t i = 0; i < 32; i++)
            // {
            //     for (size_t j = 0; j < Engine::colorNumbers; j++)
            //     {
            //         dimParam.pixels[i].color[j] = flick;
            //     }
            // }
            // dimParam.setIntensity(100.0f);
            // Dimming::instance().setParam(&dimParam);

            // ESP_LOGI("ADXL345Int","0x%X",temp);
        }
        else
        {
            xyzFloat xyz = prt->getAngles();
            if(xyz.y <= -10 )
            {
                prt->Y_Flip = ADXL345_Y_AXIS_NORMAL;
            }
            else if(xyz.y >= 10 )
            {
                prt->Y_Flip = ADXL345_Y_AXIS_FLIP;
            }

            if(prt->Y_AxisFlip_CallBack)
                prt->Y_AxisFlip_CallBack(prt->Y_Flip);

            // ESP_LOGI("x","%f",xyz.x);
            // ESP_LOGI("y","%f",xyz.y);
            // ESP_LOGI("z","%f",xyz.z);

            // if(prt->Y_Flip == 0 )
            // {
            //     flick = 0;
            // }
            // else
            // {
            //     flick = 0xffff;
            // }
            // DimmingParam dimParam;
            // for (size_t i = 0; i < 32; i++)
            // {
            //     for (size_t j = 0; j < Engine::colorNumbers; j++)
            //     {
            //         dimParam.pixels[i].color[j] = flick;
            //     }
            // }
            // dimParam.setIntensity(100.0f);
            // Dimming::instance().setParam(&dimParam);
        }
    }
}

ADXL345_WE   ADXL345_we(ADXL345_DEVICE_DAAR,I2C_NUM_0, GPIO_NUM_39, GPIO_NUM_40);

