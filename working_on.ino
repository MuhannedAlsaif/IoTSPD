#define BIT_A   ( 1 << 0 )
#define BIT_B   ( 1 << 1 )
#define BIT_C   ( 1 << 2 )
#define ANALOG_ADC1_0_VP 36
#define ANALOG_ADC2_8_25 25

// LED pins
#define ledA 2
#define ledB 0
#define ledC 4
#define LED_SAG 17
#define LED_REFERENCE_MODE 18

#define surgeNumSamples 3 // we need more samples with high sampling frequency to cover one cycle
#define brownoutNumSamples 350
#define normalNumSamples 350



#define surgeSamplesDelay 39 // in microseconds -- 39 us delay + 11 us of taking samples (processing time) = approximately 50 us
#define brownoutSamplesDelay 39 //
#define normalSamplesDelay 39 //

#define SurgeADCvalueThreshold 9999  // CORRECT VALUE IS 600 FOR TESTING PURPOSES simplify calculations, use adc value for now

 
// Sag functions
#define SAG_CUTOFF_PERCENTAGE 90 // there is voltage sag if ac signal 90% or less than ac reference


// FreeRTOS
TaskHandle_t Task1;
TaskHandle_t Task2;

EventGroupHandle_t BuffersEventGroup;  //flags
EventBits_t BufferFlags;



//Globals for surge sampling
int g_surgeCounter = 0;
int g_BufferA[surgeNumSamples]; // 
int g_BufferB[surgeNumSamples]; // 
int g_BufferC[surgeNumSamples]; // 
unsigned long timeVal[surgeNumSamples]; // WILL BE DELETED OR EXAPNDED FOR all buffers

//Globals for refernce
int g_reference_max;
int g_reference_min;
int g_reference_offset;



//-----------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200); 
  pinMode(ledA, OUTPUT);
  pinMode(ledB, OUTPUT);
  pinMode(ledC, OUTPUT);
  pinMode(LED_SAG, OUTPUT);
  pinMode(LED_REFERENCE_MODE, OUTPUT);
    
  BuffersEventGroup = xEventGroupCreate(); //test 

  Serial.print("void should be running on core ");
  Serial.println(xPortGetCoreID());
    
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

}

//-----------------------------------------------------------------------------------------
 // core 0 High Sampling to moniter surges
void Task1code( void * pvParameters ){
  int L_surgeCounter = 0;
  
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){ 
 
   if( analogRead(ANALOG_ADC1_0_VP) > SurgeADCvalueThreshold ){


       if (  !(BIT_A & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)))   ) { // Buffer A is empty and was processed 
          digitalWrite(ledA, HIGH); // for debugging - buffer A has value in it
          Serial.print("will right to A");
          measure_surges(g_BufferA);
          g_surgeCounter++; // increament global surge counter
          g_surgeCounter += L_surgeCounter;
          L_surgeCounter = 0;
          xEventGroupSetBits(BuffersEventGroup, BIT_A);
       } 
       
       else if (  !(BIT_B & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)))   ) { // Buffer B is empty and was processed 
          digitalWrite(ledB, HIGH); // for debugging - buffer A has value in it
          Serial.print("will right to B");
          measure_surges(g_BufferB); 
          L_surgeCounter++; // increament local copy cause other threads might try to access the global counter which is only protced with BIT_A flag
          xEventGroupSetBits(BuffersEventGroup, BIT_B);
       } 
       
       else  if (  !(BIT_C & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)))   ) { // Buffer C is empty and was processed 
          digitalWrite(ledC, HIGH); // for debugging - buffer A has value in it
          Serial.print("will right to C");
          measure_surges(g_BufferC);
          L_surgeCounter++;
          xEventGroupSetBits(BuffersEventGroup, BIT_C);
       } 
       
       else { // no buffer is available, 
        L_surgeCounter++; // increament counter
        delay(10); // delay so you don't increament ridiculously 
       }
       

    
    
   } // end of main if statement 



    delay(2500); // to make the watchdog timer happy
    
 //   Serial.print("task 1 loop...:");
 //   Serial.print(g_surgeCounter);
 //   Serial.print("  -  ");
 //   Serial.println(L_surgeCounter);
    
  } // End of Task 1 loop
}  // End of Task 1


//-----------------------------------------------------------------------------------------
// core 1
void loop() {


 // digitalWrite(LED_SAG, HIGH);

  
    // check for surges
    if (  (BIT_A & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)))  == (BIT_A)   ){ // if buffer has values
          process_surge(g_BufferA);
          digitalWrite(ledA, LOW); // done processing A
          xEventGroupClearBits(BuffersEventGroup, BIT_A); // signal core 0 that bufferA is available
    }
  

    if (  ( BIT_B & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)) )  == (BIT_B)   ){
      process_surge(g_BufferB);
      digitalWrite(ledB, LOW);
      xEventGroupClearBits(BuffersEventGroup, BIT_B ); 
    }

    if (  ( BIT_C & (BufferFlags = xEventGroupGetBits(BuffersEventGroup)) )  == (BIT_C)   ){
     process_surge(g_BufferC);
     digitalWrite(ledC, LOW);
     xEventGroupClearBits(BuffersEventGroup, BIT_C ); 
    }



    delay(5000);

 //   medium_adc();
   //  reference_mode(); add global for referencing once OR use GPIO to start this (simulate push butten)
    
    
  
}

//-----------------------------------------------------------------------------------------
bool  measure_ac(){ // NOT COMPLETE YET
  // for fast adc sampling
  int index = 0;
  int l_adc_max = 0;
  int l_adc_min = 4095;
  int l_adc_offset = 0;
  int tmp;
  unsigned long time1 = 0;
  unsigned long time2 = 0;
  unsigned long time3 = 0;


  // no LED
 // digitalWrite(LED_REFERENCE_MODE, HIGH); // show that we entered reference mode
 
   while (index<10000){ // measure adc for 1000 times then stop
      tmp = analogRead(ANALOG_ADC2_8_25);
      if(tmp>l_adc_max) l_adc_max = tmp;
      if(tmp<l_adc_min) l_adc_min = tmp;
      index++;
      delayMicroseconds(100); // with previouus procesing we get a delay of 100 us
    }

    l_adc_offset = (l_adc_max + l_adc_min)/2;

    // calculate period...use max or min +- error...once found record time and ignore the next few samples 
    // then look for max (or min) again +-error then record second time once foud
    while (index<750){ // measure adc for 1000 times then stop
      tmp = analogRead(ANALOG_ADC2_8_25);
      if(tmp>l_adc_max) l_adc_max = tmp;
      if(tmp<l_adc_min) l_adc_min = tmp;
      index++;
      delayMicroseconds(50); // with previouus procesing we get a delay of 100 us
    }
    


  // delay(1000); // Just so we can observe it
   
   Serial.print("Voltage peak is: ");
   Serial.println(l_adc_max);
   Serial.print("Voltage Offset: ");
   Serial.println(l_adc_offset);
   Serial.print("Voltage Period is: ");
   Serial.println(g_reference_offset);
   
   digitalWrite(LED_REFERENCE_MODE, LOW); // show that we exited reference mode
  
}

void  reference_mode(){ // NEED A WAY TO CALL THIS ONCE ONLY
  // for fast adc sampling
  int index = 0;
  int l_adc_ref_max = 0;
  int l_adc_ref_min = 4095;
  int tmp;


  digitalWrite(LED_REFERENCE_MODE, HIGH); // show that we entered reference mode
 
   while (index<10000){ // measure adc for 1000 times then stop
      tmp = analogRead(ANALOG_ADC2_8_25);
      if(tmp>l_adc_ref_max) l_adc_ref_max = tmp;
      if(tmp<l_adc_ref_min) l_adc_ref_min = tmp;
      index++;
      delayMicroseconds(100); // with previouus procesing we get a delay of 100 us
    }

   g_reference_max = l_adc_ref_max;
   g_reference_min = l_adc_ref_min;
   g_reference_offset = (l_adc_ref_max + l_adc_ref_min)/2;
   



  // delay(1000); // Just so we can observe it
   
   Serial.print("max: ");
   Serial.println(g_reference_max);
   Serial.print("min: ");
   Serial.println(g_reference_min);
   Serial.print("offset: ");
   Serial.println(g_reference_offset);
   
   digitalWrite(LED_REFERENCE_MODE, LOW); // show that we exited reference mode
  
}


void measure_surges(int * BufferX){
  int index1;
  while (index1<surgeNumSamples){ // measure adc1 for the surge and save it to empty buffer
      g_BufferA[index1] = analogRead(ANALOG_ADC1_0_VP);
      timeVal[index1] = micros(); // NOT PERMENANT will eventually removed cause not needed
      index1++;
      delayMicroseconds(surgeSamplesDelay);
  }
   
}
void  medium_adc(){ // WILL BE CHANGED TO BROWNOUT ADC + NEW LOGIC 
  // for fast adc sampling
  int l_timeIndex = 0;
  int l_printIndex = 0; //
  int l_printIndex2 = 0; // 
  unsigned long l_timeVal[320]; //
  double l_BufferA[320]; // 
  double tmp; //

   while (l_timeIndex<320){ // measure adc for 1000 times then stop
      l_BufferA[l_timeIndex] = 1.0000*analogRead(ANALOG_ADC2_8_25);
      l_timeVal[l_timeIndex] = micros();
      l_timeIndex++;
      delayMicroseconds(39);
    }

    while (l_printIndex<320){
        Serial.println(l_timeVal[l_printIndex]);
        l_printIndex++;
      }
    while (l_printIndex2<320){
        tmp = (3.3*l_BufferA[l_printIndex2])/4095;
        Serial.println(tmp,4);
        l_printIndex2++;
      }
  
}

void process_surge(int * BufferX){
  unsigned int index1 = 0;
  
 while (index1<surgeNumSamples){
    Serial.println(timeVal[index1]);
    index1++;
  }
  
 index1 = 0;
 
 while (index1<surgeNumSamples){
   // tmp = (3.3*BufferX[index1])/4095;
   // Serial.println(tmp,4);
   Serial.println(BufferX[index1]);
   index1++;
  }
 
 
}

void process_cloud(){
  delay(10000);
}
