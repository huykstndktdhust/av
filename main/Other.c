/**
  ******************************************************************************
  *   @file Other.c
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.6
  *   @date 12-30-2016
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Other.h"
#include "ExternVariablesFunctions.h"

/* Static variables ------------------------------------------------------- */
static char *TAG = "Other.c";

/* Exported functions ------------------------------------------------------- */

int32_t My_Uint32_Abs(int32_t data)
{
  int32_t temp;
  if (data < 0)
    temp = -data;
  else
    temp = data;

  return temp;
}

nvs_handle my_handle;

uint8_t SaveMemoryOffset(VectorInt16 aOffset, VectorInt16 gOffset)
{
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE_OP(TAG, "Error (%s) opening NVS!\n", esp_err_to_name(err));
  }
  else
  {
    //PrintStringI(TAG, "Opening NVS Done\n");
    // Write
    PrintStringI(TAG, "Saving Offset:\n");
    err = nvs_set_i16(my_handle, "aOffset->x", aOffset.x);
    if (err != ESP_OK)
      PrintStringE(TAG, "aOffset->x Failed!\n");

    err = nvs_set_i16(my_handle, "aOffset->y", aOffset.y);
    if (err != ESP_OK)
      PrintStringE(TAG, "aOffset->y Failed!\n");

    err = nvs_set_i16(my_handle, "aOffset->z", aOffset.z);
    if (err != ESP_OK)
      PrintStringE(TAG, "aOffset->z Failed!\n");

    err = nvs_set_i16(my_handle, "gOffset->x", gOffset.x);
    if (err != ESP_OK)
      PrintStringE(TAG, "gOffset->x Failed!\n");

    err = nvs_set_i16(my_handle, "gOffset->y", gOffset.y);
    if (err != ESP_OK)
      PrintStringE(TAG, "gOffset->y Failed!\n");

    err = nvs_set_i16(my_handle, "gOffset->z", gOffset.z);
    if (err != ESP_OK)
      PrintStringE(TAG, "gOffset->z Failed!\n");

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    //PrintStringI(TAG, "Committing updates");
    err = nvs_commit(my_handle);
    //PrintString(TAG, (err != ESP_OK) ? "Failed!\n" : "Done\n");
    if (err != ESP_OK)
      PrintStringE(TAG, "Committing Failed!\n");
  }
  // Close
  nvs_close(my_handle);

  VectorInt16 temp_aOffset;
  VectorInt16 temp_gOffset;
  //Check memory
  PrintStringI(TAG, "Readout and check:\n");
  ReadMemoryOffset(&temp_aOffset, &temp_gOffset);
  int8_t temp_err = memcmp((VectorInt16 *)&temp_aOffset, (VectorInt16 *)&aOffset, 3);
  if (temp_err != 0)
  {
    PrintStringE(TAG, "aOffset Failed!\n");
    return temp_err;
  }
  else
    PrintStringI(TAG, "aOffset OK!\n");

  temp_err = memcmp((VectorInt16 *)&temp_gOffset, (VectorInt16 *)&gOffset, 3);
  if (temp_err != 0)
  {
    PrintStringE(TAG, "gOffset Failed!\n");
    return temp_err;
  }
  else
    PrintStringI(TAG, "gOffset OK!\n");

  return LL_OK;
}

void PutErrorI16(esp_err_t err, int16_t value)
{
  switch (err)
  {
  case ESP_OK:
    //ESP_LOGI_OP(TAG, " = %d\n", value);
    PrintI16(TAG, value);
    break;
  case ESP_ERR_NVS_NOT_FOUND:
    PrintStringE(TAG, "The value is not initialized yet!\n");
    break;
  default:
    ESP_LOGE_OP(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
  }
}
void PutErrorU16(esp_err_t err, uint16_t value)
{
  switch (err)
  {
  case ESP_OK:
    //ESP_LOGI_OP(TAG, " = %d\n", value);
    PrintU16Hex(TAG, value);
    break;
  case ESP_ERR_NVS_NOT_FOUND:
    PrintStringE(TAG, "The value is not initialized yet!\n");
    break;
  default:
    ESP_LOGE_OP(TAG, "Error (%s) reading!\n", esp_err_to_name(err));
  }
}
void ReadMemoryOffset(VectorInt16 *aOffset, VectorInt16 *gOffset)
{
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE_OP(TAG, "Error (%s) opening NVS!\n", esp_err_to_name(err));
  }
  else
  {
    //PrintStringI(TAG, "Opening NVS Done\n");
    PrintStringI(TAG, "Reading:\n");
    //----------------------------------------------------
    PrintStringI(TAG, "aOffset->x");
    err = nvs_get_i16(my_handle, "aOffset->x", &aOffset->x);
    PutErrorI16(err, aOffset->x);

    PrintStringI(TAG, "aOffset->y");
    err = nvs_get_i16(my_handle, "aOffset->y", &aOffset->y);
    PutErrorI16(err, aOffset->y);

    PrintStringI(TAG, "aOffset->z");
    err = nvs_get_i16(my_handle, "aOffset->z", &aOffset->z);
    PutErrorI16(err, aOffset->z);

    //----------------------------------------------------
    PrintStringI(TAG, "gOffset->x");
    err = nvs_get_i16(my_handle, "gOffset->x", &gOffset->x);
    PutErrorI16(err, gOffset->x);

    PrintStringI(TAG, "gOffset->y");
    err = nvs_get_i16(my_handle, "gOffset->y", &gOffset->y);
    PutErrorI16(err, gOffset->y);

    PrintStringI(TAG, "gOffset->z");
    err = nvs_get_i16(my_handle, "gOffset->z", &gOffset->z);
    PutErrorI16(err, gOffset->z);
  }
  nvs_close(my_handle);
}
void ReadMemoryU16(char *name, u16 *data)
{
  esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE_OP(TAG, "Error (%s) opening NVS!\r\n", esp_err_to_name(err));
  }
  else
  {
    //PrintStringI(TAG, "Opening NVS Done\r\n");
    //PrintStringI(TAG, "Reading data");
    //PrintStringI(TAG, name);
    err = nvs_get_u16(my_handle, name, data);
    //PutErrorU16(err, *data);
  }

  nvs_close(my_handle);
}
uint8_t SaveMemoryU16(char *name, u16 data)
{
  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK)
  {
    ESP_LOGE_OP(TAG, "Error (%s) opening NVS!\n", esp_err_to_name(err));
  }
  else
  {
    //PrintStringI(TAG, "Opening NVS Done\n");
    // Write
    PrintStringI(TAG, "Saving:\n");
    PrintStringI(TAG, name);
    PrintU16Hex(TAG, data);

    err = nvs_set_u16(my_handle, name, data);
    if (err != ESP_OK)
      PrintStringE(TAG, "Failed!\n");
    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    //PrintStringI(TAG, "Committing updates");
    err = nvs_commit(my_handle);
    if (err != ESP_OK)
      PrintStringE(TAG, "Committing Failed!\n");
  }
  // Close
  nvs_close(my_handle);

  u16 temp;
  //Check memory
  PrintStringI(TAG, "Readout and check:\n");
  ReadMemoryU16(name, &temp);
  if (temp == data)
  {
    PrintStringI(TAG, "OK!\n");
    return LL_OK;
  }
  else
  {
    //PrintStringI(TAG, name);
    PrintU16Hex(TAG, temp);
    PrintStringE(TAG, "Failed!\n");
  }
  return LL_ERROR;
}
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

uint8_t mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void MPU6050_dmpDataReady(void)
{
  mpuInterrupt = true;
}
void IRAM_ATTR gpio_isr_handler(void *arg)
{
  MPU6050_dmpDataReady();
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void MPU6050_Setup(void)
{
  //  initialize device
  //  Serial.println(F("Initializing I2C devices..."));
  //  mpu.initialize();
  //  PrintStringI("Initializing I2C devices...");
  MPU6050_Init_Loop();

  //  pinMode(INTERRUPT_PIN, INPUT);

  //    verify connection
  //    Serial.println(F("Testing device connections..."));
  //    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //    PrintStringI("Testing device connections...");
  MPU6050_Connection_Loop();
  //    wait for ready
  //    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //    while (Serial.available() && Serial.read()); // empty buffer
  //    while (!Serial.available());                 // wait for data
  //    while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  PrintStringI(TAG, "Initializing DMP...");
  MPU6050_dmpInitializeLoop();

  PrintStringI(TAG, "Init DMP success...");
  PrintStringI(TAG, "Waiting for stable");
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
inline float abs_f(float data)
{
  if (data < 0.0f)
    return -data;
  return data;
}
inline int16_t abs_i16(int16_t data)
{
  if (data < 0)
    return -data;
  return data;
}
inline int32_t abs_i32(int32_t data)
{
  if (data < 0)
    return -data;
  return data;
}
void serialFloatX10PrintDegree(const char *tag, float f, uint8_t name)
{
  UartTxBuffer[0] = name;
  UartTxBuffer[1] = ':';
  int16_t w = roundf(f * 10);
  sprintf((char *)&UartTxBuffer[2], "%4d,%.1d\r\n", w / 10, abs_i16(w % 10));
  ESP_LOGI_OP(tag, "%10s", (char *)&UartTxBuffer[0]);
}
void serialFloatX1000PrintPI(const char *tag, float f, uint8_t name)
{
  UartTxBuffer[0] = name;
  UartTxBuffer[1] = ':';
  int16_t w = roundf(f * 1000);
  sprintf((char *)&UartTxBuffer[2], "%2d,%.1d\r\n", w / 1000, abs_i16(w % 1000));
  ESP_LOGI_OP(tag, "%10s", (char *)&UartTxBuffer[0]);
}
void serialFloatX10Print360(const char *tag, float f, uint8_t name)
{
  UartTxBuffer[0] = name;
  UartTxBuffer[1] = ':';
  int16_t w = roundf(f * 10);
  //move value from -180->180 to 0->360
  if (w < 0)
  {
    //w = -w;
    w += 3600; // 360 degree
  }
  sprintf((char *)&UartTxBuffer[2], "%4d,%.1d\r\n", w / 10, w % 10);
  ESP_LOGI_OP(tag, "%10s", (char *)&UartTxBuffer[0]);
}
inline void PrintStringI(const char *tag, char *b)
{
  ESP_LOGI_OP(tag, "%s", (char *)&b[0]);
}
inline void PrintStringE(const char *tag, char *b)
{
  ESP_LOGE_OP(tag, "%s", (char *)&b[0]);
}
inline void PrintI32(const char *tag, int32_t data)
{
  ESP_LOGI_OP(tag, "%11d", data); //( char *) &
}
inline void PrintU32(const char *tag, uint32_t data)
{
  ESP_LOGI_OP(tag, "%11u", data); //( char *) &
}
inline void PrintI16(const char *tag, int16_t data)
{
  ESP_LOGI_OP(tag, "%06d", data); //( char *) &
}
inline void PrintU16Hex(const char *tag, uint16_t data)
{
  ESP_LOGI_OP(tag, "%04x", data); //( char *) &
}
inline void PrintU16(const char *tag, uint16_t data)
{
  ESP_LOGI_OP(tag, "%06u", data); //( char *) &
}
inline void PrintCharacter(const char *tag, char b)
{
  ESP_LOGI_OP(tag, "%c", b); //( char *) &
}

void PrintTeapot(const char *tag)
{
  // display quaternion values in InvenSense Teapot demo format:
  teapotPacket[2] = fifoBuffer[0];
  teapotPacket[3] = fifoBuffer[1];
  teapotPacket[4] = fifoBuffer[4];
  teapotPacket[5] = fifoBuffer[5];
  teapotPacket[6] = fifoBuffer[8];
  teapotPacket[7] = fifoBuffer[9];
  teapotPacket[8] = fifoBuffer[12];
  teapotPacket[9] = fifoBuffer[13];

  ESP_LOGI_OP(tag, "%14s", (char *)&teapotPacket[0]);

  teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
}

uint8_t MPU6050_dmpReadLoop(void)
{
//  if programming failed, don't try to do anything
//  if (!dmpReady) return; //not need because init loop dmp
//    Repeat_dmpRead:;
//    // wait for MPU interrupt or extra packet(s) available
//    while (!mpuInterrupt && fifoCount < packetSize) {
//
//        // other program behavior stuff here
//        // .
//        // .
//        // .
//        // if you are really paranoid you can frequently test in between other
//        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
//        // while() loop to immediately process the MPU data
//        // .
//        // .
//        // .
//    }
#ifdef USE_INT_PIN
  if (mpuInterrupt && fifoCount < packetSize)
  {
    // try to get out of the infinite loop
    devStatus = MPU6050_getFIFOCount(&fifoCount);
    if (devStatus > 0)
      return devStatus;

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
#else
  {
#endif
    if (Pause_taskMPU6050 == 1)
    {
      // reset so we can continue cleanly
      devStatus = MPU6050_resetFIFO();
      if (devStatus > 0)
        return devStatus;
      else
        return LL_OK;
    }

    devStatus = MPU6050_getIntStatus(&mpuIntStatus);
    if (devStatus > 0)
      return devStatus;

    // get current FIFO count
    devStatus = MPU6050_getFIFOCount(&fifoCount);
    if (devStatus > 0)
      return devStatus;

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || (fifoCount >= 1024))
    {
      // reset so we can continue cleanly
      devStatus = MPU6050_resetFIFO();
      if (devStatus > 0)
        return devStatus;

      devStatus = MPU6050_getFIFOCount(&fifoCount);
      if (devStatus > 0)
        return devStatus;

      //        Serial.println(F("FIFO overflow!"));

      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT))
    {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize)
      {
        devStatus = MPU6050_getFIFOCount(&fifoCount);
        if (devStatus > 0)
          return devStatus;
      }

      // read a packet from FIFO
      devStatus = MPU6050_getFIFOBytes(fifoBuffer, packetSize);
      if (devStatus > 0)
        return devStatus;
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      MPU6050_dmpGetQuaternionQ(&q, fifoBuffer);
#endif

#ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      MPU6050_dmpGetEuler(&euler, &q);
      //degree.yaw   = container_gravity.yaw   * RAD_TO_DEGREES;
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL

#ifdef USE_SET_HOME
      if (_home == true)
      {
        getProductQ(&tmp, q, hq);
        MPU6050_dmpGetGravityV_Float(&gravity, &tmp);
        MPU6050_dmpGetYawPitchRoll((float *)&container_gravity, &tmp, &gravity);
      }
      else
      {
        MPU6050_dmpGetGravityV_Float(&gravity, &q);
        MPU6050_dmpGetYawPitchRoll((float *)&container_gravity, &q, &gravity);
      }
#else
      MPU6050_dmpGetGravityV_Float(&gravity, &q);
      MPU6050_dmpGetYawPitchRoll((float *)&container_gravity, &q, &gravity);
#endif

      //serialFloatX1000PrintPI(TAG, container_gravity.yaw, 'y');
      degree.yaw = container_gravity.yaw * RAD_TO_DEGREES;
      degree.pitch = container_gravity.pitch * RAD_TO_DEGREES;
      degree.roll = container_gravity.roll * RAD_TO_DEGREES;

      if (system_t.ready == false)
      {
        //pop(&system_t.cmd);//clear cmd
        if (abs_f(system_t.last_val - degree.yaw) > 0.05f)
        {
          system_t.last_val = degree.yaw;
          system_t.CountReady = 0;
        }
        else
        {
          if (system_t.CountReady < 10)
          { //1 second
            system_t.CountReady++;
          }
          else
          {
            system_t.ready = true;
            system_t.SetCountLed = 40; //200ms

            //set home( yaw, pith, roll to 0.0f)
            getConjugateQ(&hq, q);
            _home = true;
          }
        }
        PrintStringI(TAG, "waiting...");
      }
      else // if(system_t.ready == true)
      {
        switch (system_t.ModeOutput)
        {
        case 'z':                //set home
          getConjugateQypr('a'); //system_t.cmd);//&hq, q,
          break;
          //                  getConjugateQ(&hq, q);
          //                  _home = true;
          //                  break;
          //                case 'n': //release home
          //                  _home = false;
          //                  break;
          //                case 'y': //return yaw
          //                    getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 0);
          //                    serialFloatX10Print(TAG, degree_rezero.yaw, 'y');
          //                  break;
          //                case 'p': //return pitch
          //                    getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 1);
          //                    serialFloatX10Print(TAG, degree_rezero.pitch, 'p');
          //                  break;
          //                case 'r': //return roll
          //                    getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 2);
          //                    serialFloatX10Print(TAG, degree_rezero.roll, 'r');
          //                  break;
        }
        // put data to uart 0
        //serialFloatX10PrintDegree(TAG, degree.yaw, 'y');

        //              getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 0);
        //              serialFloatX10Print360(TAG, degree_rezero.yaw, 'y');
        //              getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 1);
        //              serialFloatX10Print360(TAG, degree_rezero.pitch, 'p');
        //              getDegreeRezero((float *)&degree_rezero, (float *)&degree , (float *)&degree_mem , 2);
        //              serialFloatX10Print360(TAG, degree_rezero.roll, 'r');

        notify2app_gstr.data.Yaw = degree.yaw;
        notify2app_gstr.data.Pitch = degree.pitch;
        notify2app_gstr.data.Roll = degree.roll;
      }
#endif

#ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      MPU6050_dmpGetAccelVi16(&aa, fifoBuffer);
      MPU6050_dmpGetGravityV_Float(&gravity, &q);
      MPU6050_dmpGetLinearAccel(&aaReal, &aa, &gravity);
#endif

#ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
#ifndef OUTPUT_READABLE_QUATERNION
#error "Not enable read quaternion"
#endif
#ifndef OUTPUT_READABLE_REALACCEL
#error "Not enable read real accel"
#endif
      MPU6050_dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
#endif

#ifdef OUTPUT_TEAPOT
      if (ModeOutput == PRINT_TEAPOT)
      {
        PrintTeapot();
      }
#endif

      // blink LED to indicate activity
      //blinkState = !blinkState;
      //digitalWrite(LED_PIN, blinkState);
    }
  }
  return devStatus;
}
void MPU6050_dmpInitializeLoop(void)
{
  uint8_t err;
  uint8_t temp = 0;
Repeat_dmpInitialize:;
  err = MPU6050_dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (err == 0)
  {
#ifdef USE_SET_OFFSET
    //--------------------new------------------------//
    ReadMemoryOffset(&aOffset, &gOffset);
    if ((aOffset.x == (int16_t)0xFFFF) ||
        (aOffset.y == (int16_t)0xFFFF) ||
        (aOffset.z == (int16_t)0xFFFF) ||
        (aOffset.x == (int16_t)0x0000) ||
        (aOffset.y == (int16_t)0x0000) ||
        (aOffset.z == (int16_t)0x0000))
    {
      aOffset.x = -567;
      aOffset.y = -211;
      aOffset.z = 1393;
      temp = 1;
    }
    if ((gOffset.x == (int16_t)0xFFFF) ||
        (gOffset.y == (int16_t)0xFFFF) ||
        (gOffset.z == (int16_t)0xFFFF) ||
        (gOffset.x == (int16_t)0x0000) ||
        (gOffset.y == (int16_t)0x0000) ||
        (gOffset.z == (int16_t)0x0000))
    {
      gOffset.x = 88;
      gOffset.y = -66;
      gOffset.z = 4;
      temp = 1;
    }
    if (temp == 1)
    {
      temp = 0;
      if (SaveMemoryOffset(aOffset, gOffset))
      {
        TIMEOUT_UserCallback();
      }
    }
    err = MPU6050_setXAccelOffset(aOffset.x);
    if (err > 0)
      goto End_dmpInitialize;
    err = MPU6050_setYAccelOffset(aOffset.y);
    if (err > 0)
      goto End_dmpInitialize;
    //--------------------------------------------//
    err = MPU6050_setZAccelOffset(aOffset.z); //factory default for my test chip
    if (err > 0)
      goto End_dmpInitialize;
    // supply your own gyro offsets here, scaled for min sensitivity
    err = MPU6050_setXGyroOffset(gOffset.x);
    if (err > 0)
      goto End_dmpInitialize;
    err = MPU6050_setYGyroOffset(gOffset.y);
    if (err > 0)
      goto End_dmpInitialize;
    err = MPU6050_setZGyroOffset(gOffset.z);
    if (err > 0)
      goto End_dmpInitialize;
#else
    //--------------------------------------------//
    err = MPU6050_setZAccelOffset(1788); // 1688 factory default for my test chip
    if (err > 0)
      goto End_dmpInitialize;
    // supply your own gyro offsets here, scaled for min sensitivity
    err = MPU6050_setXGyroOffset(220);
    if (err > 0)
      goto End_dmpInitialize;
    err = MPU6050_setYGyroOffset(76);
    if (err > 0)
      goto End_dmpInitialize;
    err = MPU6050_setZGyroOffset(85);
    if (err > 0)
      goto End_dmpInitialize;
#endif
    //turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    //mpu.setDMPEnabled(true);
    err = MPU6050_setDMPEnabled(ENABLE);
    if (err > 0)
      goto End_dmpInitialize;
    //    enable Arduino interrupt detection
    //    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //    Serial.println(F(")..."));
    //    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), MPU6050_dmpDataReady, RISING);// move to main
    err = MPU6050_getIntStatus(&mpuIntStatus);
    if (err > 0)
      goto End_dmpInitialize;

    //    set our DMP Ready flag so the main loop() function knows it's okay to use it
    //    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    //    get expected DMP packet size for later comparison
    packetSize = MPU6050_dmpGetFIFOPacketSize();

    return;
  }
  else
  {
    //      ERROR!
    //      1 = initial memory load failed
    //      5 = DMP configuration updates failed
    //      (if it's going to break, usually the code will be 1)
    //      Serial.print(F("DMP Initialization failed (code "));
    //      Serial.print(err);
    //      Serial.println(F(")"));
    goto End_dmpInitialize;
  }

  // configure LED for output
  //    pinMode(LED_PIN, OUTPUT);
End_dmpInitialize:
  TIMEOUT_UserCallback();
  goto Repeat_dmpInitialize;
}

void MPU6050_Reset_Loop(void)
{
  uint8_t err;
RepeatReset:;
  err = MPU6050_reset();
  if (err == LL_OK)
    return;

  TIMEOUT_UserCallback();
  goto RepeatReset;
}

void MPU6050_Init_Loop(void)
{
  uint8_t err;
  MPU6050(MPU6050_ADDRESS_AD0_LOW);
RepeatInit:;
  err = MPU6050_initialize();
  if (err == LL_OK)
    return;

  TIMEOUT_UserCallback();
  goto RepeatInit;
}

void MPU6050_Connection_Loop(void)
{
  uint8_t err;
RepeatConnection:;
  // verify connection
  //   Serial.println("Testing device connections...");
  //   Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  err = MPU6050_testConnection();
  if (err == LL_OK)
    return;

  TIMEOUT_UserCallback();
  goto RepeatConnection;
}

/**
  * @}
  */

/**
  * @}
  */

/*****************************END OF FILE****/
