
/*
 * Primary Author:  Brian R Taylor
 *                  brian.taylor@bolderflight.com
 *
 * Modified by:     Gian Fajardo
 *                  gianfajardo.prim@gmail.com
 *                  gian.fajardo.81@my.csun.edu
 * 
 * Copyright (c) 2021 Bolder Flight Systems Inc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */


 /*
  * there is currently a problem with the sketch. for SPI0, 
 
 */


#include "mpu9250.h"
#include "ecf.h"


// ----------------------------------------------------------------
//                           DECLARATIONS                          
// ----------------------------------------------------------------


// SPI Channels --------------------------------------
#define NUM_IMUs  4

#define SPI0_CS0  10
#define SPI0_INT0 9

#define SPI0_CS1  37
#define SPI0_INT1 6

#define SPI1_CS0  0
#define SPI1_INT0 2

#define SPI1_CS1  38
#define SPI1_INT1 3


bfs::Mpu9250  imu_0(&SPI,   SPI0_CS0),
              imu_1(&SPI,   SPI0_CS1),
              imu_2(&SPI1,  SPI1_CS0),
              imu_3(&SPI1,  SPI1_CS1);

bfs::Mpu9250 *imu_list[NUM_IMUs]  = {&imu_0, &imu_1, &imu_2, &imu_3};


// Interrupt Service Routine (ISR) Definitions -------
void imu_0_read();
void imu_1_read();
void imu_2_read();
void imu_3_read();
void imu_print(bfs::Mpu9250*, int, int = 0);





// Extended Complementary Filter ---------------------
ExtendedComplementaryFilter ecf(
        0.5f,   // K_norm,
        3.f,    // t_norm,
        15.f,   // K_init,
        1/50.f  // dt
);

// ECF elapsedMillis class ---------------------------
#define ECF_CALC_RATE 200 // ms
elapsedMillis ecf_clock;


#define BUFFER_LEN    200 // entries

States  ECF_states_0[BUFFER_LEN],  *s0_ptr = &ECF_states_0[0],
        ECF_states_1[BUFFER_LEN],  *s1_ptr = &ECF_states_1[0],
        ECF_states_2[BUFFER_LEN],  *s2_ptr = &ECF_states_2[0],
        ECF_states_3[BUFFER_LEN],  *s3_ptr = &ECF_states_3[0];


volatile unsigned int c0 = 0, l0 = 0,
                      c1 = 0, l1 = 0,
                      c2 = 0, l2 = 0,
                      c3 = 0, l3 = 0;


// ----------------------------------------------------------------
//                            SETTING UP                           
// ----------------------------------------------------------------

void setup() {

  // counter to set up the IMUs
  unsigned int n;

  Serial.begin(115200);
  while (!Serial);

  SPI.begin();
  SPI1.begin();

  n = 0;
  while (n < NUM_IMUs) {
    bfs::Mpu9250* imu = *(&imu_list[0] + n);
    
    Serial.print(F("initializing IMU"));   Serial.println(n);

    if (!imu->Begin()) {
      Serial.println(F("Error initializing communication with IMU"));
      while (1);
    }

    if (!imu->ConfigSrd(19)) {
      Serial.println(F("Error configured SRD"));
      while (1);
    }

    if (!imu->EnableDrdyInt()) {
      Serial.println(F("Error enabling data ready interrupt"));
      while (1);
    }

    n++;
  }

  
  // SPI0 CS0 ---------------------------------
  pinMode(SPI0_CS0,   OUTPUT);
  pinMode(SPI0_INT0,  INPUT_PULLUP);
  attachInterrupt(SPI0_INT0, imu_0_read, RISING);

  // SPI0 CS1 ---------------------------------
  pinMode(SPI0_CS1,   OUTPUT);
  pinMode(SPI0_INT1,  INPUT_PULLUP);
  attachInterrupt(SPI0_INT1, imu_1_read, RISING);

  // SPI1 CS0 ---------------------------------
  pinMode(SPI1_CS0,   OUTPUT);
  pinMode(SPI1_INT0,  INPUT_PULLUP);
  attachInterrupt(SPI1_INT0, imu_2_read, RISING);

  // SPI1 CS1 ---------------------------------
  pinMode(SPI1_CS1,   OUTPUT);
  pinMode(SPI1_INT1,  INPUT_PULLUP);
  attachInterrupt(SPI1_INT1, imu_3_read, RISING);
  

  /* headers */
  // Serial.println("a.x\ta.y\ta.z\tg.x\tg.y\tg.z\tm.x\tm.y\tm.z");


  // // set up to record all of the data to the SD card
  // Serial.print("Initializing SD card...");

  // // see if the card is present and can be initialized:
  // if (!SD.begin(chipSelect)) {
  //   Serial.println("Card failed, or not present");

  //   // No SD card, so don't do anything more - stay stuck here
  //   while (1) {}
  // }
  // Serial.println("card initialized.");


  // while (1) {
  //   // Serial.print("elapsed-time = ");
  //   // Serial.println((lap_timer - time_ms));

  //   if ((lap_timer - time_ms) > 100) {
  //     // Serial.println("true");
  //     Serial.print(print_key);
  //     Serial.print(" to ");
  //     Serial.println(key);
      
  //     // update old time
  //     time_ms = lap_timer - 100;
  //     print_key = key;
      
  //     for(volatile int i = print_key; i > key; ++i) {
  //       Serial.print(ax[i]);  Serial.print("\t");
  //       Serial.print(ay[i]);  Serial.print("\t");
  //       Serial.println(az[i]);
  //     }
  //   }

  //   delayMicroseconds(1000000);
  // } // while(1) loop
}


// ----------------------------------------------------------------
//                            MAIN LOOP                           
// ----------------------------------------------------------------

void loop() {
  #define COUNT   c0
  #define LAST    l0
  #define POINTER s0_ptr

  // Serial.print(F("in loop()\t"));

  // TODO: fix this behavior
  States* current_state;

  if (ecf_clock >= ECF_CALC_RATE) {
    ecf_clock = ecf_clock - ECF_CALC_RATE;

    Serial.print(F("("));
    Serial.print(COUNT);
    Serial.println(F(")"));

    noInterrupts();
    // unsigned int current_count = COUNT;

    // while (current_count > (LAST+1)) {
    // while (c0 > (l0+1)) {
    while (COUNT > (LAST+1)) {

      interrupts();
      current_state = POINTER + ((LAST+1) % BUFFER_LEN);
      // interrupts();

      ecf.update(POINTER + (LAST+1) % BUFFER_LEN, POINTER + LAST % BUFFER_LEN);

      // printing -----------------------------------------------
      Serial.print(F("\t"));
      Serial.print(COUNT);
      // Serial.print(current_count);
      Serial.print(F(">"));
      Serial.print(LAST+1);

      // ... and this one does not.
      Serial.print(F(" -->\t"));
      // current_state->accel.print(2);  Serial.print(F(" | "));
      // current_state->gyro.print(2);   Serial.print(F(" | "));
      // current_state->mag.print(2);    Serial.print(F(" | "));
      current_state->q.print(2);      Serial.println();

      LAST++;

      noInterrupts();
    }
    
    interrupts();

  }
}


// ----------------------------------------------------------------
//                    INTERRUPT SERVICE ROUTINES                   
// ----------------------------------------------------------------

void imu_0_read() {

  States* current_state = s0_ptr + (c0 % BUFFER_LEN);

  // goes through if the IMU has read more data
  if (imu_list[0]->Read()) {

    current_state->sample = c0;
    current_state->accel  = VectorFloat(
        imu_list[0]->accel_x_mps2(),
        imu_list[0]->accel_y_mps2(),
        imu_list[0]->accel_z_mps2()
    );
    current_state->gyro   = VectorFloat(
        imu_list[0]->gyro_x_radps(),
        imu_list[0]->gyro_y_radps(),
        imu_list[0]->gyro_z_radps()
    );
    current_state->mag    = VectorFloat(
        imu_list[0]->mag_x_ut(),
        imu_list[0]->mag_y_ut(),
        imu_list[0]->mag_z_ut()
    );
            
    Serial.print(c0);                     Serial.print(F(" | ")); // c0 is not working. it may be because the interrupt pin is fried. easy swap. TODO
    // Serial.println();
    // // Serial.print(F("t="));  
    // // Serial.println(current_state->time);  Serial.print(F(" | "));
    // current_state->accel.print(2);        Serial.print(F(" | "));
    // current_state->gyro.print(2);         Serial.print(F(" | "));
    // current_state->mag.print(2);          Serial.print(F(" | "));
    // current_state->q.print(2);            Serial.println();

    c0++;
  }
}

void imu_1_read() {

  States* current_state = s1_ptr + (c1 % BUFFER_LEN);

  if (imu_list[1]->Read()) {
    current_state->sample  = c1;
    current_state->accel = VectorFloat(
        imu_list[1]->accel_x_mps2(),
        imu_list[1]->accel_y_mps2(),
        imu_list[1]->accel_z_mps2()
    );
    current_state->gyro = VectorFloat(
        imu_list[1]->gyro_x_radps(),
        imu_list[1]->gyro_y_radps(),
        imu_list[1]->gyro_z_radps()
    );
    current_state->mag = VectorFloat(
        imu_list[1]->mag_x_ut(),
        imu_list[1]->mag_y_ut(),
        imu_list[1]->mag_z_ut()
    );
            
    Serial.print(c1);                     Serial.print(F(" | "));

    c1++;
  }

}

void imu_2_read() {

  States* current_state = s2_ptr + (c2 % BUFFER_LEN);

  if (imu_list[2]->Read()) {
    current_state->sample  = c2;
    current_state->accel = VectorFloat(
        imu_list[2]->accel_x_mps2(),
        imu_list[2]->accel_y_mps2(),
        imu_list[2]->accel_z_mps2()
    );
    current_state->gyro = VectorFloat(
        imu_list[2]->gyro_x_radps(),
        imu_list[2]->gyro_y_radps(),
        imu_list[2]->gyro_z_radps()
    );
    current_state->mag = VectorFloat(
        imu_list[2]->mag_x_ut(),
        imu_list[2]->mag_y_ut(),
        imu_list[2]->mag_z_ut()
    );
            
    Serial.print(c2);                     Serial.print(F(" | "));

    c2++;
  }
  
}

void imu_3_read() {

  States* current_state = s3_ptr + (c3 % BUFFER_LEN);

  if (imu_list[3]->Read()) {
    current_state->sample  = c3;
    current_state->accel = VectorFloat(
        imu_list[3]->accel_x_mps2(),
        imu_list[3]->accel_y_mps2(),
        imu_list[3]->accel_z_mps2()
    );
    current_state->gyro = VectorFloat(
        imu_list[3]->gyro_x_radps(),
        imu_list[3]->gyro_y_radps(),
        imu_list[3]->gyro_z_radps()
    );
    current_state->mag = VectorFloat(
        imu_list[3]->mag_x_ut(),
        imu_list[3]->mag_y_ut(),
        imu_list[3]->mag_z_ut()
    );
            
    Serial.print(c3);                     Serial.print(F(" | "));

    c3++;
  }
  
}

// void imu_print(bfs::Mpu9250* imu, int label, int newline = 0) {
//   if (imu->Read()) {
//     Serial.print(F("| IMU")); Serial.print(label);  Serial.print(F("\t"));

//     Serial.print(imu->accel_x_mps2());     Serial.print(F("\t"));
//     Serial.print(imu->accel_y_mps2());     Serial.print(F("\t"));
//     Serial.print(imu->accel_z_mps2());     Serial.print(newline ? F("\n") : F(""));
//   }
// }

// void imu_store(bfs::Mpu9250* imu, int label, int newline = 0) {
//   float ax, ay, az;

//   if (imu->Read()) {

//     ax = imu->accel_x_mps2();
//     ay = imu->accel_y_mps2();
//     az = imu->accel_z_mps2();
//   }
// }



// ---------------------------------------------------------------
//                          PREVIOUS WORK                         
// ---------------------------------------------------------------

// void loop() {

//   if ((lap_timer - time_ms) > 100) {
//     noInterrupts(); // Disable interrupts before reading shared variables
//     int currentTail = tail;
//     interrupts();   // Re-enable interrupts immediately after reading
    
//     if (print_key != currentTail) {
//       Serial.print(print_key);
//       Serial.print(" to ");
//       Serial.println(currentTail);
//       time_ms = lap_timer - 100;
      
//       while (print_key != currentTail) {
//         Serial.print(buffer[print_key].ax); Serial.print("\t");
//         Serial.print(buffer[print_key].ay); Serial.print("\t");
//         Serial.println(buffer[print_key].az);
//         print_key = (print_key + 1) % BUFFER_LEN; // Wrap around
//       }
//     }
    
//   }
//   delayMicroseconds(1000000);
// }

// void imu_3_read() {

//   if (imu_ptr3->Read()) {
//     Serial.print(F("| IMU3\t"));

//     buffer[head].ax = imu_ptr3->accel_x_mps2();
//     buffer[head].ay = imu_ptr3->accel_y_mps2();
//     buffer[head].az = imu_ptr3->accel_z_mps2();

//     buffer[head].gx = imu_ptr3->gyro_x_radps();
//     buffer[head].gy = imu_ptr3->gyro_y_radps();
//     buffer[head].gz = imu_ptr3->gyro_z_radps();

//     buffer[head].mx = imu_ptr3->mag_x_ut();
//     buffer[head].my = imu_ptr3->mag_y_ut();
//     buffer[head].mz = imu_ptr3->mag_z_ut();

//     head = (head + 1) % BUFFER_LEN;

//     if (head == tail) {
      
//       // Buffer overflow! Handle the error (e.g., set an error flag)
//       Serial.println("Buffer Overflow!");

//       // Option 1: Drop the oldest data
//       tail = (tail + 1) % BUFFER_LEN;

//       // Option 2: Stop recording data
//       // head = (head - 1 + BUFFER_LEN) % BUFFER_LEN; //Decrement head, wraparound to keep last value.
//     }
//   }
// }
