/**
  ******************************************************************************
  *   @file IMU_Zero.c
  *   @editor SangTN@fsoft.com.vn - FPT Company
  *   @version V1.3
  *   @date 12-30-2016
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "IMU_Zero.h"
#include "ExternVariablesFunctions.h"

/* Static variables ------------------------------------------------------- */
static  char *TAG = "IMU_Zero.c";
#define OLD_SET
/* Exported functions ------------------------------------------------------- */
void ForceHeader (void) {
  LinesOut = 99;
  }
 uint8_t GetSmoothedLedCount = 0;
void GetSmoothed (void)
{
   int16_t RawValue[6];
   int32_t Sums[6];
  for (uint8_t i = iAx; i <= iGz; i++)
  { Sums[i] = 0; }
//  unsigned long Start = micros();
//  Initialise the xLastWakeTime variable with the current time.

  for (int16_t i = 1; i <= N; i++)
  {
    //delay_us(usDelay);
    delay_ms(usDelay/1000);//Perform an action every 5 ticks //200Hz
    GetSmoothedLedCount++;
    if(GetSmoothedLedCount >= 10)//50ms
    {
      GetSmoothedLedCount = 0;
      LED_taskMPU6050 ^= 1;
    }
    // get sums
    MPU6050_getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                       &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
    if ((i % 500) == 0)//2.5second
    PrintCharacter(TAG, PERIOD);

    for (uint8_t j = iAx; j <= iGz; j++)
      Sums[j] = Sums[j] + RawValue[j];
  }// get sums
//  unsigned long usForN = micros() - Start;
//  Serial.print(" reading at ");
//  Serial.print(1000000/((usForN+N/2)/N));
//  Serial.println(" Hz");
  for (uint8_t i = iAx; i <= iGz; i++)
  { Smoothed[i] = (Sums[i] + N/2) / N ; }
} // GetSmoothed

void SetOffsets (int32_t *TheOffsets)
  { MPU6050_setXAccelOffset(TheOffsets [iAx]);
    MPU6050_setYAccelOffset(TheOffsets [iAy]);
    MPU6050_setZAccelOffset(TheOffsets [iAz]);
    MPU6050_setXGyroOffset(TheOffsets [iGx]);
    MPU6050_setYGyroOffset(TheOffsets [iGy]);
    MPU6050_setZGyroOffset(TheOffsets [iGz]);
  } // SetOffsets

void ShowProgress (void)
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        //PrintStringI("\tXAccel\t\t\t\tYAccel\t\t\t\tZAccel\t\t\t\tXGyro\t\t\t\tYGyro\t\t\t\tZGyro\r\n\r\n");
        LinesOut = 0;
      } // show header
//    PrintChar(BLANK);
//    for (uint8_t i = iAx; i <= iGz; i++)
//      { PrintChar(LBRACKET);
//        PrintInt(LowOffset[i]),
//        PrintChar(COMMA);
//        PrintInt(HighOffset[i]);
//        PrintStringI("] --> [");
//        PrintInt(LowValue[i]);
//        PrintChar(COMMA);
//        PrintInt(HighValue[i]);
//        if (i == iGz)
//          { PrintChar(RBRACKET); }
//        else
//          { PrintStringI("]\t"); }
//      }
    LinesOut++;
  } // ShowProgress
 int32_t NewOffset[6];
void PullBracketsIn (void)
  {
     bool AllBracketsNarrow;
     bool StillWorking;
    //int16_t NewOffset[6];

//    PrintStringI("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (uint8_t i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (uint8_t i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working

  } // PullBracketsIn

void PullBracketsOut (void)
  {
     bool Done = false;
     int32_t NextLowOffset[6];
     int32_t NextHighOffset[6];

    //PrintStringI("expanding:");
    ForceHeader();

    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (uint8_t i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (uint8_t i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (uint8_t i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging (int16_t NewN)
  { N = NewN;
    PrintStringI(TAG, "averaging ");
    PrintI16(TAG, N);
    PrintStringI(TAG, " readings each time");
   } // SetAveraging

uint8_t SetupCalib (void)
{
      PrintStringI(TAG, "Disabling DMP (you turn it on later)...\r\n");
       uint8_t err = MPU6050_setDMPEnabled(false);
      if(err > 0) return err;

      PrintStringI(TAG, "Resetting FIFO and clearing INT status one last time...\r\n");
      err = MPU6050_resetFIFO();
      if(err > 0) return err;

       uint8_t int_status;
      err = MPU6050_getIntStatus(&int_status);
      if(err > 0) return err;

    for (uint8_t i = iAx; i <= iGz; i++)
      { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);

    PullBracketsOut();
    PullBracketsIn();

    PrintStringI(TAG, "-------------- done --------------");
    aOffset.x = NewOffset[0];
    aOffset.y = NewOffset[1];
    aOffset.z = NewOffset[2];
    gOffset.x = NewOffset[3];
    gOffset.y = NewOffset[4];
    gOffset.z = NewOffset[5];
//    PrintStringI(TAG, "\r\n-------------- aOffset.x --------------\r\n");
//    PrintI16(TAG, aOffset.x);
//    PrintStringI(TAG, "\r\n-------------- aOffset.y --------------\r\n");
//    PrintI16(TAG, aOffset.y);
//    PrintStringI(TAG, "\r\n-------------- aOffset.z --------------\r\n");
//    PrintI16(TAG, aOffset.z);
//    PrintStringI(TAG, "\r\n-------------- gOffset.x --------------\r\n");
//    PrintI16(TAG, gOffset.x);
//    PrintStringI(TAG, "\r\n-------------- gOffset.y --------------\r\n");
//    PrintI16(TAG, gOffset.y);
//    PrintStringI(TAG, "\r\n-------------- gOffset.z --------------\r\n");
//    PrintI16(TAG, gOffset.z);
//    PrintStringI(TAG, "\r\n-------------- end --------------\r\n");

    err = MPU6050_setDMPEnabled(true);
    if(err > 0)
    {
        PrintStringI(TAG, "Enable DMP ...fail");
        TIMEOUT_UserCallback();
        return LL_ERROR;
    }
    PrintStringI(TAG, "Enable DMP ...OK");

    if(SaveMemoryOffset(aOffset, gOffset))
    {
      TIMEOUT_UserCallback();
      return LL_ERROR;
    }
    return LL_OK;
  } // setup
/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/
