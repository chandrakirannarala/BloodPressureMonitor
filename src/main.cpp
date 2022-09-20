/*
* Author(s): Chandra Kiran (cn2255) and Pooja Choudhary (pc3125)
* Project:    Embedded Challenge : Semi automated blood pressure/heart rate measuring device 
* File:       main.cpp
* Project code discription: This code implements a Blood Pressure and Pulse Monitoring system by implementing the Maximum Amplitude Algorithm (MAA)
* The project utilizes the STM32f429I discovery board interfaced to MPR sensor using SPI protocol. As per the MAA algorithm, we first plot the 
* Oscillometric Waveform Envelope (OMWE) graph by plotting the peak values in pressure oscillation. The maxima of the OMWE graph indicate a 
* pressure point called MAP (Mean Arterial Pressure). The Systolic and Diastolic pressure valus could be related to MAP (Mean Arterial Pressure) using two characteristic
* ratio "Rs" (Systolic ratio) and "Rd" (Diastolic ratio). The systolic pressure corresponds to the pressure value when the OMWE amplitude = Rs*MAP towards the left of 
* MAP pressure value. The diastolic pressure corresponds to the pressure value when the OMWE aplitude = Rd*MAP towards right of MAP pressure value.
* For this project, we took the Rs and Rd values and algorithm implementation strategies from a couple of sources like:
* https://www.nature.com/articles/s41371-019-0196-9
*/


#include <mbed.h>

// The given sensor follows transfer function B as per datasheet. MAX output for trans function B = 22.5% of max value possible for 24 bits = 3774873
// Min output value = 2.5% of max value possible for 24 bits = 419430
#define OUTPUT_MAX 3774873.0  // Maximum value of 24 bit output from sensor 
#define OUTPUT_MIN 419430     // Minimum value of 24 bit output from sensor
#define PRESSURE_MAX 300.0    // Maximum possible pressure that could be measured (300 mmHg)
#define PRESSURE_MIN 0.0      // Minimum possible pressure that could be measured (0 mmHg)
#define MIN_OMWE_THRESH 70.0   // Minimum OMWE graph filter for checking MAP values (helps to eradicate edge noices)
#define MAX_OMWE_THRESH 160.0  // Maximum OMWE graph filter for checking MAP values (helps to eradicate edge noices)
#define MAP_ERROR_THRESH 0.5   // Maximum supported error threshold while calculating the pressure position at Systolic and Diastolic pressure points in OMWE graph
#define SYSTOLIC_LOWER_CHAR_RATIO 0.45  // Lower bound of Rs
#define SYSTOLIC_UPPER_CHAR_RATIO 0.73  // Upper bound of Rs 
#define DIASTOLIC_LOWER_CHAR_RATIO 0.69 // Lower bound of Rd
#define DIASTOLIC_UPPER_CHAR_RATIO 0.83 // Upper bound of Rd
#define LOWER_PULSE_RANGE 35.0          // Minimum practical pulse (bpm)
#define UPPER_PULSE_RANGE 150.0         // Maximum practical pulse (bpm)

// Structure Containing parameters related to BP like Systolic and Diastolic BPs and parameters related MAA algorithm for BP estimation
struct BP_PARAMETER {
    double systolic_bloodpressure;
    double diastolic_bloodpressure;
    double systolic_char_ratio;
    double diastolic_char_ratio;
};

// Structure containing pulse value and the number of data points using which the pulse was evaluated
struct PULSE_READING {
    double pulse_value;
    long pulse_data_count;
};

Ticker pressure_gradient;
Timer pressure_display_timer;
Timer pulse_count_timer;
SPI spi_comm(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut cs(PB_6);
DigitalOut active_flag(LED2);       // LED indicator for active data plotting for OMWE
DigitalOut max_pressure(LED3);      // LED indicator to start releasing cuff pressure
DigitalOut flux_warning(LED4);      // LED indicator for high pressure release
DigitalIn dataread_push_button(USER_BUTTON); //Removed the push button
double current_pressure = 0;
double release_rate = 0;
double buffer_queue[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
double omwegraph_absicissa_buffer[1000];    // Oscillometeric Waveform Envelope (OMWE) graph x values.
double omwegraph_ordinate_buffer[1000];    // Oscillometric Waveform Envelope (OMWE) graph y values
double omwe_buffer_time[1000];       // Time buffer for storing time relative to first record when peak in OMWE was detected
double pressure_diff = 0.0;
double previous_pressure_diff = 0.0;
double peak_pressure_diff = 0.0;
double Mean_Arterial_Pressure;                    // Mean Arterial Pressure (MAP) value to be estimated for BP evaluation
BP_PARAMETER final_blood_pressure;       // Variable containing the final BP value
long caliberated_MIN_OUT = 0;
long iteration = 0;
long omwebuffer_pointer = 0;        // A pointer for storing the latest data location of x and y buffers of OMWE plot
long omwetime_buffer_pointer = 0;   // A pointer for storing the latest data location of the OMWE time buffer
bool caution_flag = false;           // Flag to indicate if pressure release is too fast!
bool change_warnflag;
bool active_recordflag = false;    // Flag to indicate if data measured is being recorded for OMWE plot.
bool end_record = false;
long measure_pressure();             // Function routine to measure pressure using MPR sensor
PULSE_READING measure_pulse();          // FUnction routine to evaluate pulse from the OMWE time buffer
void check_pressure_gradient_ISR();      // An Interrupt Service Routine attached to a Ticker to check if pressure release is too fast.
void auto_caliberate();              // This is an auto-caliberation routine that caliberates the sensor output at the start of the pressure measurement to be the 0 pressure point
void MAP_calculator();               // Routine to calculate MAP value from the OMWE graph
BP_PARAMETER Systolic_and_diastolic_bp_calculator();   // Routine to calculate the systolic and diastolic blood pressure
double calculate_normalized_pressure();   // To find peak values in OMWE, we compare the current pressure reading with a set of normalized pressure values over the previous readings. This routine calculates it

 int main() {
    BP_PARAMETER bp;
    PULSE_READING pulse;
  //////////////////////////// I use SPI protocol to interface with MPR Sensor. The following configures the SPI protocol using mbed API//////////////
    cs = 1;                            // Disabling slave select
    spi_comm.format(8, 1);             // SPI data transmission format
    spi_comm.frequency(100000);        // SPI communication frequency
    auto_caliberate();                 // Before starting the actual reading, Tare/Caliberate the base MPR sensor ouput to 0.
    change_warnflag = false;
    pressure_gradient.attach(&check_pressure_gradient_ISR, 1);  // Watchdog ticker to periodically check for pressure release rate 
    printf("\nNow measuring pressure!...");
    pressure_display_timer.start();       // Timer to time the pressure display on monitor
    pulse_count_timer.start();            // Starting the timer for OMWE time buffer
	while (!end_record) {          // Keep measuring pressure until end_record is active
		measure_pressure();    
		wait_us(200000);
	}
    pulse_count_timer.stop();
    printf("\n Calculating Systolic and Diastolic pressure values.....");
    bp = Systolic_and_diastolic_bp_calculator();
    if (bp.systolic_bloodpressure < 0 || bp.diastolic_bloodpressure < 0){     // If the bp measurement failed, the BP values will be set negative
        printf("\n Pressure measurement unsuccessful! Perform again...");
    }
    else {
        printf("\n Pressure measurement completed successfully!.");
    }
    printf("\n MAP value = %lf", Mean_Arterial_Pressure);
    printf("\n Characteristic systolic deviation in graph = %lf", bp.systolic_char_ratio);
    printf("\n Characteristic diastolic deviation in graph = %lf", bp.diastolic_char_ratio);
    printf("\n Systolic pressure = %lf", bp.systolic_bloodpressure);
    printf("\n Diastolic pressure = %lf", bp.diastolic_bloodpressure);
    printf("\n Calculating Pulse...");
    pulse = measure_pulse();
    printf("\n Pulse measurement completed!");
    if (pulse.pulse_data_count == 0){
        printf("\n No pulse Detected!. Perform again...");
    }
    else {
        printf("\n Your pulse = %lf. Number of reliable pulse values = %ld", pulse.pulse_value, pulse.pulse_data_count);
    }
   return 0;
}

/***Fuction to Measure Pulse using the OMWE graph time buffer*********
OMWE time buffer collects the time at which the OMWE peaks were detected.
This routine checks for the time between consecutive peaks and then checks if it comes under reliable pulse rate.
Multiple such reliable data points are found and the average is taken to be the pulse value.
The pulse_count gives the total number of pulse time data points using which the final pulse was evaluated.  */

PULSE_READING measure_pulse() {
    PULSE_READING pulse_data;
    double pulse_upper_value = (60.0/LOWER_PULSE_RANGE)*1000.0;        // Upper value of pulse in time difference between the consecutive pulse peaks
    double pulse_lower_value = (60.0/UPPER_PULSE_RANGE)*1000.0;        // Lower value of pulse in time difference between the consecutive pulse peaks
    double pulse = 0.0;
    double pulse_time_p2p;
    long pulse_count = 0;
    for (int i = 1; i < omwetime_buffer_pointer; i++){
        pulse_time_p2p = omwe_buffer_time[i] - omwe_buffer_time[i - 1];   // Time interval between adjactent peaks
        if (pulse_time_p2p > pulse_lower_value && pulse_time_p2p < pulse_upper_value){
            pulse += pulse_time_p2p;
            pulse_count++;
        }
    }
    if (pulse_count != 0){
        pulse /= (double)pulse_count;
        pulse = (1000/pulse) * (60.0);
    }
    else {
        pulse = -1;
    }
    pulse_data.pulse_value = pulse;
    pulse_data.pulse_data_count = pulse_count;   
    return pulse_data; 
}

/*****Fuction to Systolic and Diastolic Pressure using OMWE graph X and Y cordinates  
Useing MAA Algorithm to evaluate Systolic and Diastolic blood pressure values. 
As a part of this, during the measure_pressure routine,the MAP (Mean Arterial Pressure) value would be found and the OMWE graph would be plotted after the user manually gestures the controller to start plotting by pressing
the USER button. 
The MAA algorithm tells that :
Systolic Pressure = Pressure value (x cordinate) corresponding to (Rs*MAP) y cordinate in OMWE graph toward right of MAP peak. 
Diastolic value corresponds to the pressure value at (Rd*MAP). 
Here Rs and Rd are Systolic and Diastolic characteristic ratios. 
Assuming Rs = (0.45 + 0.73)/2 = 0.59 and Rd = (0.69 + 0.83)/2 = 0.76 BP estimation using MAA algorithm. ****/

BP_PARAMETER Systolic_and_diastolic_bp_calculator() {
    double lower_systolic, upper_systolic, lower_diastolic, upper_diastolic;
    double systolic_ordinate_value, diastolic_ordinate_value;
    int systolic_buffer = -1, diastolic_buffer = -1;
    double min_systolic_ordinate_error = MAP_ERROR_THRESH + 1;
    double min_diastolic_ordinate_error = MAP_ERROR_THRESH + 1;
    BP_PARAMETER bp_value;
    // Here peak delta pressure corresponds to the ordinate of OMWE graph(Y-axis) corresponding to x
    lower_systolic = SYSTOLIC_LOWER_CHAR_RATIO * peak_pressure_diff;
    upper_systolic = SYSTOLIC_UPPER_CHAR_RATIO * peak_pressure_diff;
    lower_diastolic = DIASTOLIC_LOWER_CHAR_RATIO * peak_pressure_diff;
    upper_diastolic = DIASTOLIC_UPPER_CHAR_RATIO * peak_pressure_diff;
    systolic_ordinate_value = (lower_systolic + upper_systolic)/2.0;           // Pressure peak value corresponding to Systolic pressure
    diastolic_ordinate_value = (lower_diastolic + upper_diastolic)/2.0;        // Pressure peak value corresponding to Diastolic pressure
    
    // min_systolic_ordinate_error/min_diastolic_ordinate_error are used to find the closest y value in OMWE graph that matches with the characteristic pressure peaks
    for (int i = 0; i < omwebuffer_pointer; i++){
        if (abs(omwegraph_ordinate_buffer[i] -  systolic_ordinate_value) < min_systolic_ordinate_error){
           if(omwegraph_absicissa_buffer[i] > 100 && omwegraph_absicissa_buffer[i] < 200) {          // Filter to check if pressure is reliable
              min_systolic_ordinate_error = abs(omwegraph_ordinate_buffer[i] -  systolic_ordinate_value);
              systolic_buffer = i;
           }   
        }
        if (abs(omwegraph_ordinate_buffer[i] -  diastolic_ordinate_value) < min_diastolic_ordinate_error){
           if(omwegraph_absicissa_buffer[i] > 50 && omwegraph_absicissa_buffer[i] < 90) { 
              min_diastolic_ordinate_error = abs(omwegraph_ordinate_buffer[i] -  diastolic_ordinate_value);
              diastolic_buffer = i;
           }   
        }        
    }

    // If pressure value found isn't reliable, set the pressure values to negative so that, we will be prompted to reconduct the test.
    if (systolic_buffer < 0 || diastolic_buffer < 0){
        bp_value.systolic_bloodpressure = -1;
        bp_value.diastolic_bloodpressure = -1;
        bp_value.diastolic_char_ratio = min_diastolic_ordinate_error;
        bp_value.systolic_char_ratio = min_systolic_ordinate_error;
    }
    // Else load the bp values with the respective pressure values and the deviation of the found y value from the extected characteristic y value
    else {
        bp_value.systolic_bloodpressure = omwegraph_absicissa_buffer[systolic_buffer];
        bp_value.diastolic_bloodpressure = omwegraph_absicissa_buffer[diastolic_buffer];
        bp_value.diastolic_char_ratio = min_diastolic_ordinate_error;
        bp_value.systolic_char_ratio = min_systolic_ordinate_error;        
    }
    return bp_value;
}


/* Function check for MAP values on the go as the data is being collected 
While the meaure_pressure funciton is running and the USER button (input from user) has been pressed the controller keeps checking for the event of a peak
pressure change. The normalized pressure value corresponding to the peak change is the MAP value. */

void MAP_calculator() {
    double normalized_pressure = calculate_normalized_pressure();
    if (pressure_diff > peak_pressure_diff && normalized_pressure > MIN_OMWE_THRESH && normalized_pressure < 110){        
        peak_pressure_diff = pressure_diff;     // The peak ordinate corresponding to MAP value in OMWE
        Mean_Arterial_Pressure = calculate_normalized_pressure();   // The MAP pressure value
    }
    
}

/**Function to auto-caliberate the base MPR pressure sensor output to 0 before starting to collect pressure values
The MPR sensor output value before pumping the cuff is taken as the 0 reference. For pressure caliberation sample is taken for
100 pressure readings from the MPR sensor and the MEAN VALUE is taken to be the base value reference for 0 mmHg */

void auto_caliberate() {
    unsigned long default_pressure = 0;
    printf("\nCaliberating the sensor now!..");  
    for(int i = 0; i < 100; i++ ){            // Sampling the 100 samples of initial pressure
       default_pressure += measure_pressure();
       wait_us(10000);
    }  
    default_pressure /= 100;
    caliberated_MIN_OUT = default_pressure;
    printf("\nCaliberation complete!");
}

/*****Fuction to Calculate normalized pressure at a point
As the sampling frequency of pressure data samples is large , an average of 5 immediate previous 
sensor readings are taken as the actual pressure at the point. This helps to reduce the imapact of noicy data readings. For this purpose, we use a normalize buffer that works like a circular queue of depth 5,
where the latest value replaces the oldest value and this replacement occurs in cycle.The normalized pressure is the mean of the pressure values in the queue. */

double calculate_normalized_pressure(){
  double total_count = 0.0;
  double total_value = 0.0;
  for (int i = 0; i < 5; i++){
      if (buffer_queue[i] != 0.0){   // normalize_buffer is the circular queue holding the latest 5 pressure readings
          total_value += buffer_queue[i];
          total_count += 1.0;
      } 
  }
  total_value = total_value / total_count;    // average of all the total values
  return total_value;
}

/***Interrupt Service Routine (ISR) for checking an increased pressure release rate*****
This ISR is attached to a Ticker, which is triggered every second to check for high release rate. i.e > 4 mmHg per sec. 
If the release rate is found high, a flux warning flag is set true, which lights up the BLUE LED6 */


void check_pressure_gradient_ISR() {
  if (iteration > 5){
      release_rate = calculate_normalized_pressure() - current_pressure;  // The difference between current pressure and the normalized pressure, depicts a change in release rate
      if (release_rate > 4.0){                                       
         flux_warning = true;     // For high release rate flux warning makes warning LED to ON
      }
      else {
          flux_warning = false;   //The LED will be OFF if the release rate is normal.
      }
  }
  return;
}

/***The main function that interfaces the sensor and calculates pressure readings*****
This is the main function that interfaces the sensor using SPI protocol and receives the pressure readings. An SPI communication
sends a set of 3 byte command sequence of 0xAA -> 0x00 -> 0x00 through MOSI at which point of time, the received data through MISO is don't care
and is dumped into a random dummy buffer. Following this, we issue a read command as a 4 byte sequence 0xF0 -> 0x00 -> 0x00 -> 0x00 at the MOSI,
where we get a status byte and a 3 byte output at MISO which is received in the data response buffer. Note that we use the SPI api method write() 
in mbed to transmit and receive data. The status value of 64 indicates a valid data reading. The 3 bytes of received data is concatenated and 
converted into actual pressure reading in mmHG using the conversion formula */

long measure_pressure () { 
    if (dataread_push_button){          // read data from the sensor is not recorded until the record_push_button is i pressed to neglet unwanted data
        active_recordflag = true;
    }
    active_flag = active_recordflag;
    long pressure_data = 0;
    double normalized_pressure = 0;
    char read_command_buffer[4] = {0xF0, 0x00, 0x00, 0x00};   // Buffer containing the read command bytes
    char write_command_buffer[3] = {0xAA, 0x00, 0x00};        // Buffer containing the write command bytes.
    char dummy_response_buffer[4] = {0, 0, 0, 0};             // Dummy response buffer to hold garbage values from MISO 
    char data_receive_buffer[4] = {0, 0, 0, 0};  
    char status;   

    double pressure_value;
    double scaler = (PRESSURE_MAX - PRESSURE_MIN) / (OUTPUT_MAX - OUTPUT_MIN); // Scaler value to convert 24 bit MPR data into actual pressure value 
     cs = 0;          // SS pin set to '0' to activate slave select before SPI communication starts 
     spi_comm.write(write_command_buffer, 3, dummy_response_buffer, 3);   // Initiate a command to read pressure
     cs = 1;          // Set the SS pin to end communication
     wait_us(10000);     // 10ms wait time for MPR sensor to sample and calculate pressure

     cs = 0;          // By reset SS pin we start the SPI communication again 
     spi_comm.write(read_command_buffer, 4, data_receive_buffer, 4);   // enable read command and receive data into data_receive_buffer
     pressure_data = pressure_data | (long)data_receive_buffer[3] | (long)data_receive_buffer[2] << 8 | (long)data_receive_buffer[1] << 16; // Concatenate the 3 data bytes
     status = data_receive_buffer[0];    // Status bit!
     pressure_value = scaler*(double)(pressure_data - caliberated_MIN_OUT);  // Conversion of 24-bit data to pressure reading in mmHg
     current_pressure = pressure_value;
    normalized_pressure = calculate_normalized_pressure();
    if (pressure_display_timer.read() > 1){              // to display the data on screen
        printf("\n Recorded pressure = %lf. Pressure release rate = %lf mmHg per second ",normalized_pressure, release_rate);
        pressure_display_timer.reset();
    }
     if (active_recordflag && abs(current_pressure - normalized_pressure) < 12.0) {   // If the button is pressed and data read is viable, record the readings
         pressure_diff = abs(current_pressure - normalized_pressure);   // Difference pressure is the change in the peak of current to normalised pressure
        if (normalized_pressure > MIN_OMWE_THRESH && normalized_pressure < MAX_OMWE_THRESH){
            if(pressure_diff < previous_pressure_diff){   // Condition to check if the graph passed a maxima that has to be stored
                if (omwetime_buffer_pointer > 0){
                  if (pulse_count_timer.read_ms() - omwe_buffer_time[omwetime_buffer_pointer - 1] > 500.0){  // Bandpass filter for pulse time minute measurement
                    omwe_buffer_time[omwetime_buffer_pointer++] = pulse_count_timer.read_ms();
                  }  
                }   
                else {
                   omwe_buffer_time[omwetime_buffer_pointer++] = pulse_count_timer.read_ms(); 
                }  
                omwegraph_ordinate_buffer[omwebuffer_pointer] = previous_pressure_diff;
                omwegraph_absicissa_buffer[omwebuffer_pointer++] = normalized_pressure;
            }
        } 
        previous_pressure_diff = pressure_diff;
     }      
    buffer_queue[iteration++ % 5] = pressure_value;  // Updating the buffer_queue (by circular queue manner)
    MAP_calculator();              // MAP calculater is called to check, if the passed maxima is the absolute maxima in the OMWE for which we have to store as MAP value
     if (normalized_pressure > 200.0){    // At the upper limit of 200.0 mmHg pressure, a motification is send to release the pressure in the pump and record data for OMWE
          max_pressure = 1;  
         }      // If red LED is ON, It is indicating Maximum pressure 
     cs = 1;  
     if (active_flag && normalized_pressure < 5.0){   // if the pressure is dropped less than 5 mmHg andIf the active flag used for rate measurement is active and , we can now stop pressure measurement
         end_record = true;
     } 
     return pressure_data;
}