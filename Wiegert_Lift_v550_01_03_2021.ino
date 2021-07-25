#include <Nextion.h>            // Library for the HMI, Nextion 7" touch screen display     
#include <AccelStepper.h>       // Library for Stepper Controller.   Used to talk to the stepper motor
#include "SdFat.h"
 
 /*************************************************************************************************************************************
                                    Wiegert Router Lift

                                    v.4.4.1_03_15_2020
                                    3_15_2020
                                    Cory D. Wiegert

                                    This is a router lift controller where an arduino mega is wired through a tb6600 connected to a nema23 stepper motor
                                    using a Nextion 7" intelligent HMI
                                    
                                    01/23/2020 -- CDW   important to note, the GetVal functions for all the Nextion objects will only work if utilizing
                                                        the new base class libraries  NexConfig.h, NexHardware.cpp and NexHardware.h downloaded from
                                                        https://forum.arduino.cc/index.php?topic=620821.0
                                                        
                                                        Through testing - any value over 18000 (at 8 mirosteps) starts overdriving the controller and setting the motors out of phase
                                                        speeds around 450-650 seem to phase switch the motors as well, and shuddering of the lead screw occurs.

                                    01/24/2020 -- CDW   IMPORTANT!!!!! Found out the hard way, if we remove buttons or slide things around in the Nextion IDE, make sure to check the object id's with the
                                                        NEX<object> definition section.   When the create/remove/move around happens in the IDE, it seems the object ID's are modified,
                                                        thereby causing the listening service on the arduino to mis-match the objects with the call backs.
                                                     
                                                        ALSO -- this may be a grounding problem.  If the Nextion is not grounded correctly, there are unexplainable behaviors
                                                        the same error is flagged when there is a grounding problem
                               
                                    02/12/2020 -- CDW   change the settings screen, putting the pin and variable configuration into the UI
                                              v2.7
                                    02/15/2020 -- CDW   Very good complete version finished.   Will call this version 3.0 and will deploy for burn-in test on teh router table.
                                             v 3.0      put to the router table on 2/17/2020, have the file for the Memory screen set to tokenize it based on the section
                                    02/21/2020 -- CDW   Error dialog implemented, fixed error logging to the HMI screen
                                             v 3.13     Deployed to the router lift on 02/22/2020 at 12/30 PM
                                    02/23/2020 -- CDW   added versioning, fixed a couple small bugs, redployed to router 
                                            v. 3.14_02_22_2020
                                    02/23/2020 -- CDW   modified how the pushcallbacks worked for the up and down buttons on settings and memory
                                            v. 3.15_02_23_2020
                                    02/23/2020 -- CDW   modified how all the saves, deletes and write to disk work on Memory screen.   Will add goto 0 button
                                                        on the memory screen as well.   button will lower router after completing mortise cuts
                                    02/24/2020 -- CDW   refactored some of the inefficient code, change some fo the callbacks to point to same functions
                                          v. 4.1_02_24_2020
                                    02/24/2020 -- CDW   refactored the write to file on the Memory screen.  Now, all lines are read to structure, and rewritten
                                                        no more losing content, or having strange characters bleed into the file.   No change to the UI in the Nextion
                                                        IDE
                                          v 4.1.1_02_24_2020
                                    02/26/2020 -- CDW   Refactored memory file config.   Allowed for an old file to be opened, and not over write the data in that file
                                          v. 4.2.1_02_26_2020
                                    02/29/2020  -- CDW  Refactored errors  to read the strings from a file.  Rewrote function ErrorDialog and all ErrorDialog commands
                                          v 4.3.1_02_29_2020
                                    03_07_2020 -- CDW   fixed error on memory screen.   change the check to -currentPosition() because we are storing positions above the tabel as 
                                                        positive #'s
                                          v.4.3.2_03_07_2020
                                    03_08_2020 -- CDW   modified the bits screen, have all the custom bit movement configured in HEIGHTS.cfg, and can configure the bits without
                                                        having to recompile.   Am using the LoadRouterfromMemory function to set the router off the bits screen
                                          v.4.4.0_03_08_2020
                                    03_15_2020 -- CDW   added a position field to the Memory screen so we are notified when the router has stopped moving.   Gives us a flag
                                                        that we have reached the height set in the depth field.
                                          v.4.4.1_03_15_2020
                                    09_19_2020  CDW -- line 1358 moved from 1351.   The top limit switch was not working when router was moving up.   Checking the buffer after 
                                                        a move insted of before the move.   Up function now exactly the same as the down function
                                          v. 5.5.0_0_03_2021
                                    01_03_2021  CDW --   starting the code on the power fence.   Adding ball screw and limit switches, controlling fence with HMI

*****************************************************************************************************************************************************************************/
      
        /****************************************************************************
                 HIGH -- arduino constant for 1.   Used for turning the digital pins on and off.  Motor state, HIGH = ON
                 LOW  -- arduino constant for 0    Used for turning the digital pins on and off   Motor state, LOW = OFF
                          no need to define them here, they are already defined in libraries
        *********************************************************************************/
        #define FILE_WRITE (O_WRITE | O_READ | O_CREAT)   // redefine the file write function based on this thread   https://forum.arduino.cc/index.php?topic=616594.0
        
        int         TOP_SWITCH;
        int         BOTTOM_SWITCH;
        int         FRONT_SWITCH;
        int         BACK_SWITCH;
        const int   SD_WRITE = 53;         //CS pin of the SD card
        int         REZERO;
        int         FENCE_ZERO;
        const byte  UP = 0;
        const byte  DOWN = 1;
        const byte  BACK = 1;             // for the fence
        const byte  FORWARD = 0;          // for the fence
                
        /*
                   Code for the stepper motor controller.   Need to set the appropriate pins for the digital write and signal
                    These wires go to the stepper controller, not the stepper directely 
                    directionPin and stepPin must be set prior to the AccelStepper object create.   Can not set them through a config file
        */
        byte   directionPin = 11;             // pin that will go to the D+ on the stepper controller  have to declare here, as the stepper is a global class
        byte   fenceDirPin = 28;              // pin that will go to the D+ on the fence stepper controller.   Have to declare here as the stepper is a global class
        byte   stepPin = 13;                  // pin that will wire to the P+ (pulse) on the stepper controlloer   have to declare here as the stepper is a global class
        byte   fenceStepPin = 30;             // pin that will wire to the P+ (pulse) on the Fence stepper controlloer   have to declare here as the stepper is a global class
        //byte   enablePin = 6;                 // pin that will wire to the E+ on the stepper controller.   Turns the controller on and off  LOW turns motor on, HIGH turns motor off
        byte   enablePin;                     //  pin that will wire to the E+ on the stepper controller.   Turns the controller on and off  LOW turns motor on, HIGH turns motor off
        byte   fenceEnablePin;                 //  pin that will wire to the E+ on the fence stepper controller.   Turns the controller on and off  LOW turns motor on, HIGH turns motor off
        float  initSpeed = 2000;              //  sets the initial speed of the motor.   Don't think we need acceleration in the router, but will find out
        long   maxMotorSpeed;                 //  as defined - maximum speed motor will run.   Sets the 100% on the speed slider
        float  maxAcceleration = 6000;        //  maximum number of steps for acceleration
        long   workingMotorSpeed;             //  active working speed, set by the slider, and will be somewhere between 0 and 100%
        int    stepsPerRevolution = 1600;     //  number of steps required for 1 revolution of leadscrew
        int    microPerStep =   8;            //  number of microSteps.   Set on the TB600 controller by dip switch   Off | On | Off
        float  distPerStep = 0.00024606;      //  inches per step with calculated off <microPerStep> = 8
        int    pulseWidthMicros = 20;         //  miliseconds  -- delay for the steps.   the higher the number the slower the lead screw will turn
        int    millisbetweenSteps = 20;       //  milliseconds - or try 1000 for slower steps   delay between pulses sent to the controller
        long   curPos;                        //  return value from CurrentPosition() call on the stepper
        byte   DIRECTION;                     //  tests the direction the motor should be turning
        byte   HOME_MOTOR = 1;                //  Used with the swFence switch to tell which motor to use when zeroing and resetting
        byte   SET_MOTOR = 1;                 //  Used on the Settings Screen to tell which motor to use when resetting calibrations
        int    stepSize = 10;                 //  used in the moving of the router to set precision on how large the steps should be for the runToPosition() function
        char   CR = '\n';                    //  Carraige return constant, used for the writeDebug function to add a new line at the end of the function
        String preSetTxt;
        int    preSetNum;
        long   lowLimit;                      //  step value when the lower limit switch is hit
        long   highLimit;                     //  step value when the upper limit switch is hit
        double  curPosInch;
        char    posInch[16] = {'\0'};
        SdFat   sdCard;                       //  pointer to the SD card reader.   Used in storing config and memory files
        int    bGo = DOWN;                    //  used as a flag to test if the stop button has been pressed.
        char    storeFile[25] = {'\0'};       //  name of file where the memory will be stored
        char    fileName[20] = "LiftPinsConfig.cfg";    // Configuration file for the pins, router speed, memory file and version #
        
        /****************************************************************************************
           Decare the AccelStepper object and use the FULL2WIRE as a type of stepper motor.
           connecting to the TB6600, it is a 2 wire control.   

           The pins must be defined and configured here, they can not be part of the Setup - because sRouter is set as a global variable
           it must have the buttons defined, prior to the setup
         *************************************************************************/
        AccelStepper  sRouter (AccelStepper::FULL2WIRE, stepPin, directionPin, HIGH);
        AccelStepper  sFence (AccelStepper:: FULL2WIRE, fenceStepPin, fenceDirPin, HIGH);
        
        // Declare your Nextion objects - Example (page id = 0, component id = 1, component name = "b0"    Fully qualifying with the screen name
        //  creates ability to use in Serial3.write commands, without having to go through the libraries)
        
        NexPage     Home =           NexPage(0, 0, "Home");
        NexPage     Settings =       NexPage (1,0,"Settings");
        NexButton   bMoveDown =      NexButton(0, 1, "bMoveDown");
        NexButton   bMoveUp =        NexButton(0, 2, "bMoveUp");
        NexButton   bForward =       NexButton (0, 36, "bForward");
        NexButton   bBack =          NexButton (0, 35, "bBack");
        NexButton   bChangeBit =     NexButton(0, 3, "bChangeBit");             //  as it says, the change bit button on the HOME page (not settings)
        NexVariable vaSpeed =        NexVariable( 0, 5, "vaSpeed");       
        NexButton   bReZero =        NexButton ( 0, 6, "bReZero");             //  sets the current position of the lift as the 0 position on the stepper controller
        NexSlider   hMoveSpeed =     NexSlider (0, 8, "Home.hMoveSpeed");      //  the slider for the speed of the motor.
        NexNumber   nSetSpeed =      NexNumber (0, 9, "Home.nSetSpeed");       //  display value on the bottom of the set speed slider
        NexNumber   nSpd0 =          NexNumber (0, 10, "nSpd0");                //  scale fields on the slider.   Becuase we can not calculate decimals in the UI
        NexNumber   nSpd4 =          NexNumber (0, 11, "nSpd4");                //  we have to handle the click here in the arduino
        NexNumber   nSpd2 =          NexNumber (0, 12, "nSpd2");                //  scale fields on the slider.   Becuase we can not calculate decimals in the UI
        NexNumber   nSpd6 =          NexNumber (0, 13, "nSpd6");                //  we are going to use the value of the number field as a % of the max speed of the slider
        NexNumber   nSpd8 =          NexNumber (0, 14, "nSpd8");                //  with that % calculation, we can set the motor speed in steps / second
        NexNumber   nSpd10 =         NexNumber (0, 15, "nSpd10");               //  scale fields on the slider.   Becuase we can not calculate decimals in the UI
        NexDSButton swDirection =    NexDSButton(0, 17, "swDirection");
        NexTouch    cbPreSets =      NexTouch (0, 18, "cbPreSets");             //  Drop down list of standard movements.  ONce selected, the "go" button will move this value
        NexDSButton swHow =          NexDSButton (0, 19, "swHow");              //  instruction set on how to handle the auto distance move
        NexButton   bAutoMove =      NexButton (0, 20, "bAutoMove");            //  the "go" button which will move the router according to the preset check
        NexText     tHolder =        NexText (0, 22, "tHolder");                //  Text box to hold temporary values.   The variable object is not stable in Nextion
        NexText     tHoldCombo =     NexText (0, 23, "tHoldCombo");             //  Tex box to hold temporary values.   The variable object is not stable in Nextion
        NexButton   bToBottom =      NexButton(0, 24, "bToBottom");             //  as it says, the change bit button on the HOME page (not settings)
        NexRadio    rMax =           NexRadio (0, 26, "rMax");                  //  radio button to move at max speed when auto moving to top or bottom limit
        NexButton   bgotoZero =      NexButton (0, 31, "bgotoZero");            //  moves the lift to the new 0 position
        NexDSButton btPower =        NexDSButton (0, 33, "Home.btPower");       //  flips the bit on enablePin to turn the motor controller ON and OFF
        NexText     tPowerStat =     NexText ( 0, 34, "Home.tPowerStat");       //  text field to show the current state of the motor controller
        NexDSButton swFence =        NexDSButton(0, 39, "swFence");             //  Switch to set global for setting which motor to run   router/Fence
        NexButton   bDown =          NexButton (1, 1, "Settings.bDown");        //  move down button on Settings Page
        NexButton   bUp =            NexButton (1, 2, "Settings.bUp");          //  move up on the Settings page
        NexButton   bSetForward =    NexButton (1, 48, "Settings.bSetForward");
        NexButton   bSetBack =       NexButton (1, 47, "Settings.bSetBack");
        NexButton   bLiftBit =       NexButton (1, 3, "bLiftBit");              //  the change bit button on the Settings page.
        NexButton   bZero =          NexButton (1, 5, "bZero");                 //  rezero button on the Settings Page
        NexButton   swDirect =       NexButton (1, 7, "swDirect");             //  Direction button on the Settings Screen
        NexButton   bBottomOut =     NexButton (1, 8, "bBottomOut");            //  move to bottom on the settings page
        NexText     tUpLimit =       NexText (1, 9, "Settings.tUpLimit");       //  Text box to hold the uppoer Limit of the lead screw
        NexText     tLowLimit =      NexText (1, 10, "Settings.tLowLimit");     //  Text Box to hold the Lower limit of the lead screw
        NexButton   bCalib =         NexButton (1, 11, "bCalib");               //  button to move to top and bottom limits.   Mark those limits for later calculations
        NexDSButton btSetOff =       NexDSButton (1, 14, "Settings.btSetOff");  //  flips the bit on enablePin to turn the motor controller ON and OFF on Settings page
        NexText     tMaxSpeed =      NexText (1, 16, "tMaxSpeed");              //  field to configure the max lift speed, will reset the top of the slider
        NexText     tWorkSpeed =     NexText (1, 19, "tWorkSpeed");             //  field to configure working speed, will set the slider
        NexText     tStepSize =      NexText (1, 21, "tStepSize");              //  field to configure stepSize - will control speed 
        NexDSButton swWhich =        NexDSButton(1, 51, "swWhich");             //  toggle switch on the Settings screen to direct which motor is to be used for functions
        NexText     tFileName =      NexText(1, 37, "Settings.tFileName");      //  field to hold the SD card file name to read and write to
        NexText     tVersion =       NexText(1, 42, "Settings.tVersion");       //  field to hold the version of the application and code
        NexButton   bSetPins =       NexButton (1, 38, "bSetPins");             //  Button to download the pins to the arduino and restart
        NexButton   bSetMotor =      NexButton (1, 40, "bSetMotor");            //  Button to download new settings for max, working speeds and step size3
        NexButton   bSaveFile =      NexButton (1, 41, "bSaveFile");            //  resets the configuration file, changes the Memory file name in configPins.cfg set up file
        NexDSButton swFenceDir =     NexDSButton (1, 52, "swFenceDir");          //  switch to set which way to zero the fence.  if Forward, come from the back, if Back, start in front
        NexButton   bSDCard   =      NexButton(2, 58, "bSDCard");               //  button to reset the SD card reader if the card has been pulled and reinserted
        NexButton   bMemZero =       NexButton (2, 62, "bMemZer0");             //  Drops the router bit back to the 0 position, at max speed
        /**************************************************************
           Text fields for the Memory page presets.   These fields will be written to the SD card or EPROMM.  
        ***************************************************************/

        NexVariable vaIndex =       NexVariable (2, 59, "vaIndex");
        NexVariable vaDelIndex =    NexVariable (2, 60, "Memory.vaDelIndex");
        NexVariable vaSDIndex =     NexVariable (2, 61, "vaSDIndex");
        NexVariable vaFence =       NexVariable (0, 40, "Home.vaFence");
        NexText    tMem1 =          NexText (2, 8, "tMem1");
        NexText    tMem2 =          NexText (2, 2, "tMem2");
        NexText    tMem3 =          NexText (2, 4, "tMem3");
        NexText    tMem4 =          NexText (2, 5, "tMem4");
        NexText    tMem5 =          NexText (2, 6, "tMem5");
        NexText    tMem6 =          NexText (2, 3, "tMem6");
        NexText    tMem7 =          NexText (2, 7, "tMem7");
        
        NexText    tZero1 =         NexText(2, 30, "tZero1");
        NexText    tZero2 =         NexText(2, 37, "tZero2");
        NexText    tZero3 =         NexText(2, 32, "tZero3");
        NexText    tZero4 =         NexText(2, 33, "tZero4");
        NexText    tZero5 =         NexText(2, 34, "tZero5");
        NexText    tZero6 =         NexText(2, 35, "tZero6");
        NexText    tZero7 =         NexText(2, 36, "tZero7");
        
        NexText    tMem1Steps =     NexText(2, 38, "tMem1Steps");
        NexText    tMem2Steps =     NexText(2, 39, "tMem2Steps");
        NexText    tMem3Steps =     NexText(2, 40, "tMem3Steps");
        NexText    tMem4Steps =     NexText(2, 41, "tMem4Steps");
        NexText    tMem5Steps =     NexText(2, 42, "tMem5Steps");
        NexText    tMem6Steps =     NexText(2, 43, "tMem6Steps");
        NexText    tMem7Steps =     NexText(2, 44, "tMem7Steps");
        
        NexButton   bSetUp =        NexButton(2, 53, "bSetUp");
        NexButton   bSetDown =      NexButton(2, 54, "bSetDown");
        NexButton   bMemForward =   NexButton (2, 70, "bMemForward");
        NexButton   bMemBack =      NexButton (2, 69, "bMemBack");
        NexButton  bSave1 =         NexButton (2, 9, "bSave1");
        NexButton  bSave2 =         NexButton (2, 11, "bSave2");
        NexButton  bSave3 =         NexButton (2, 13, "bSave3");
        NexButton  bSave4 =         NexButton (2, 15, "bSave4");
        NexButton  bSave5 =         NexButton (2, 17, "bSave5");
        NexButton  bSave6 =         NexButton (2, 19, "bSave6");
        NexButton  bSave7 =         NexButton (2, 21, "bSave7");
        
        NexButton  bDelete1 =       NexButton (2, 10, "bDelete1");
        NexButton  bDelete2 =       NexButton (2, 12, "bDelete2");
        NexButton  bDelete3 =       NexButton (2, 14, "bDelete3");
        NexButton  bDelete4 =       NexButton (2, 16, "bDelete4");
        NexButton  bDelete5 =       NexButton (2, 18, "bDelete5");
        NexButton  bDelete6 =       NexButton (2, 20, "bDelete6");
        NexButton  bDelete7 =       NexButton (2, 22, "bDelete7");
        
        NexButton  btoFile1 =       NexButton (2, 45, "btoFile1");
        NexButton  btoFile2 =       NexButton (2, 46, "btoFile2");
        NexButton  btoFile3 =       NexButton (2, 47, "btoFile3");
        NexButton  btoFile4 =       NexButton (2, 48, "btoFile4");
        NexButton  btoFile5 =       NexButton (2, 49, "btoFile5");
        NexButton  btoFile6 =       NexButton (2, 50, "btoFile6");
        NexButton  btoFile7 =       NexButton (2, 51, "btoFile7");
        
        NexButton  bLoad1 =         NexButton (2, 23, "bLoad1");
        NexButton  bLoad2 =         NexButton (2, 27, "bLoad2");
        NexButton  bLoad3 =         NexButton (2, 28, "bLoad3");
        NexButton  bLoad4 =         NexButton (2, 24, "bLoad4");
        NexButton  bLoad5 =         NexButton (2, 25, "bLoad5");
        NexButton  bLoad6 =         NexButton (2, 26, "bLoad6");
        NexButton  bLoad7 =         NexButton (2, 29, "bLoad7");
        
        NexButton   bLoadMem =       NexButton (2, 52, "bLoadMem");            //Loads data save on SD card into saved memory
        NexDSButton btMemOff =      NexDSButton (2, 56, "Memory.btMemOff");   //  flips the bit on enablePin to turn the motor controller ON and OFF
        NexHotspot  m0       =      NexHotspot ( 6, 2, "m0");                 // Hotspot for the custom bit settings
        NexHotspot  m2      =       NexHotspot (6, 11, "m2");
        NexHotspot  m3      =       NexHotspot (6, 12, "m3");
        NexHotspot  m4      =       NexHotspot (6, 15, "m4");
        NexHotspot  m5      =       NexHotspot (6, 16, "m5");
        NexText     tThickEntry =   NexText (6, 3, "tThickEntry");            // Enter the thickness of the stock to be worked by custom bits
        NexDSButton sw0     =       NexDSButton (6,4, "sw0");                 // decimal or fraction thickness
        NexNumber   nBit    =       NexNumber (6, 22, "nBit");                // field to hold the index of the button pressed
        NexButton  bLoad   =       NexButton (6, 20, "bLoad");               // calculates the total steps and moves the bit to correct height
        NexText    t3       =       NexText (6, 19, "Bits.t3");
        NexButton   bBitsZero =     NexButton( 6, 23, "bBitsZero");           // move to 0 position on the Bits screen
        
 
// Any object for which we will have an event handler will need to be listed in the nex_Listen_list array.   The send component ID must be set in the UI on the event
//  which will be handled

//  01/03/2021 --- add the buttons for moving the fence..   not going to add it today
        
        NexTouch *nex_listen_list[] = {
          &bMoveUp,
          &bMoveDown,
          &bBack,
          &bForward,
          &swHow,
          &bChangeBit,
          &bToBottom,
          &btPower,
          &bAutoMove,
          &swFence,
          &bReZero,
          &bLiftBit,
          &bCalib,
          &bZero,
          &bgotoZero,
          &bUp,
          &bDown,
          &bBottomOut,
          &hMoveSpeed,
          &btSetOff,
          &bSetPins,
          &bSetMotor,
          &bSetForward,
          &bSetBack,
          &swWhich, 
          &nSpd0,
          &nSpd2,
          &nSpd4,
          &nSpd6,
          &nSpd8,
          &nSpd10,
          &cbPreSets,
          &bSetUp,
          &bSetDown,
          &bSave1,
          &bSave2,
          &bSave3,
          &bSave4,
          &bSave5,
          &bSave6,
          &bSave7,
          &bDelete1,
          &bDelete2,
          &bDelete3,
          &bDelete4,
          &bDelete5,
          &bDelete6,
          &bDelete7,
          &btoFile1,
          &btoFile2,
          &btoFile3,
          &btoFile4,
          &btoFile5,
          &btoFile6,
          &btoFile7,
          &bLoad1,
          &bLoad2,
          &bLoad3,
          &bLoad4,
          &bLoad5,
          &bLoad6,
          &bLoad7,
          &bLoadMem,
          &btMemOff,
          &bSaveFile,
          &bSDCard,
          &bMemZero,
          &bMemForward,
          &bMemBack,
          &m0,
          &m2,
          &m3,
          &m4,
          &m5,
          &bLoad,
          &bBitsZero,
          NULL
        };
         
/***********************************************************************************
    Setting up an array of structures to hold the config to do a lookup for # of steps required to move a specific distance
    
    <lookup value>, <inch value>, <decimal equivalent>, <baseline steps>

    Baseline steps are based on a 200 steps per turn, on a 1.8 degree pitch, with a lead of 10mm Leed (10mm traveled per turn)
    and no microsteps
                      steps per turn * microsteps*4 (phases)                          inches
                      ---------------------------  = # of steps per mm to travel     --------  =  # of steps per inch travel
                          distance traveled                                         25.4 mm/inch

                          multiply the basics steps by 8, to get the 8 microsteps
************************************************************************************************/
    struct inchCacl {
          int    index;
          char   inches[6];
          float  decimal;
          float  steps;
          char   label[30];
        }  preSetLookup; 
        
        
/**********************
    My easy way to write to the Serial debugger screen
 **********************/
        
        void writeDebug (String toWrite, byte LF)
        {
          if (LF)
          {
            Serial.println(toWrite);
          }
          else
          {
            Serial.print(toWrite);
          }
        
        }
/*********************************
     int   ErrorDialog (char *outMessage, char *b1Text, char *b2Text, char *yesScreen , char *noScreen)
              this will pop up the error screen, set the message txt in multiline format, set the buttons
              to the text passed in, and if appropriate, navigate to a new place in the app
              The yesScreen and noScreen will navigate to the designated screen when the error dialog is closed.
              b1Text and b2Text will be displayed on the Yes and No buttons
 ***********************************************************************************************/
        
        int   ErrorDialog (int ErrorNum) {
         
          char  msgCommand[300] = {'\0'};
          char  errorString[200] = {'\0'};        // used for the fgets function to read a file line
          char  bYesLbl[24] = {'\0'};
          char  bNoLbl[24] = {'\0'};
          int   serRead[7];
          char  errorTitle[25] = {'\0'};
          char  errorText[200] = {'\0'};
          char  yesScreen[12] = {'\0'};
          char  noScreen[12] = {'\0'};
          char  *token, *elim;
          char  delim[] = {',','\0'};
          char  crCheck[2] = {'^', '\0'};
          char  checker;
          char  errText1[200];
          char  errText2[200];
          char  errText3[200];
          char  errText4[200];
          char  errText5[200];
          int   i, p;
          SdBaseFile   fError;
          if(!fError.open("Error.cfg"))
            {
              return 0;
            }
          fError.rewind();
          checker = '1';
          while (checker != '^')              // read the header
            {
           
              checker = fError.read();
            }

          checker = fError.fgets(errorTitle, sizeof(errorTitle));         //read the carraige return before the errors start
          while ( fError.available())
            {
              memset (errorString, '\0', sizeof(errorString));
              fError.fgets (errorString, sizeof(errorString));
              token =strtok(errorString, delim);

              if (atoi(token) == ErrorNum)
                {
                  for (i=2; i<= 7; i++)
                    {
                      token = strtok(NULL, delim);
                      switch (i)
                        {
                          case 2:
                            strcpy(errorTitle, token);
                            break;
                          case 3:
                            strcpy(errorText, token);
                            break;
                          case 4:
                            strcpy(bYesLbl, token);
                            break;
                          case 5:
                            strcpy(bNoLbl, token);
                            break;
                          case 6:
                            strcpy(yesScreen, token);
                            break;
                          case 7:
                            strcpy(noScreen, token);
                            fError.seekEnd();
                            break;
                        }
                    }
                }
            }
          fError.close();
          Serial3.write("page Error");
          FlushBuffer();   
          if (strlen(bYesLbl) > 0)
          {
            sprintf(msgCommand, "Error.bYes.txt=\"%s\"", bYesLbl);
            Serial3.write(msgCommand);
            FlushBuffer();
          }
        
          if (strlen(bNoLbl) > 0)
          {
            sprintf(msgCommand, "Error.bNo.txt=\"%s\"", bNoLbl);
        
            Serial3.write(msgCommand);
            FlushBuffer();
          }      
         
          sprintf(msgCommand,"Error.tErrorHead.txt=\"%s\"", errorTitle);
          Serial3.write(msgCommand);
          FlushBuffer();
          memset (msgCommand, '\0', sizeof(msgCommand));
          token = strtok(errorText, crCheck);
          p=0;
          for (i = 0; i<=4; i++)
            {
              if ( token !=NULL)
                {
                  switch (i)
                    {
                      case 0:
                        sprintf (errText1, "%s",token);
                        break;
                      case 1:
                        strcpy (errText2, token);
                        p++;
                        break;
                      case 2:
                        strcpy (errText3, token);
                        p++;
                        break;
                      case 3:
                        strcpy (errText4, token);
                        p++;
                        break;
                      case 4:
                        strcpy (errText5, token);
                        p++;
                        break;
                    }
                 token = strtok(NULL, crCheck);
                }
              else
                 i=6;
            }
          switch (p)
            {
              case 0:
                sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\"", errorText);
                break;
              case 1:
                sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\r\n%s\"", errText1, errText2);
                break;
              case 2:
                sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\r\n%s\r\n%s\"", errText1, errText2, errText3);
                break;
              case 3:
                sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\r\n%s\r\n%s\r\n\%s\"", errText1, errText2, errText3, errText4);
                break;
              case 4:
                sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\r\n%s\r\n%s\r\n\%s\r\n%s\"", errText1, errText2, errText3, errText4, errText5);
                break;
              
            } 
    //      sprintf(msgCommand, "Error.tErrorMsg.txt=\"%s\"", errorText);
          Serial3.write(msgCommand);
          FlushBuffer();
 
          if (Serial3.peek() != 101)
            Serial3.read();
          while (Serial3.peek() != 101 )
          {
            Serial3.read();
            delay(100);
          }
          memset(serRead, '\0', sizeof(serRead));
          for (int i = 0; i <= 6; i++)
            serRead[i] = Serial3.read();
          memset (msgCommand, '\0', sizeof(msgCommand));
          if (serRead[2] == 4)
            sprintf(msgCommand, "page %s", yesScreen);
          else if (serRead[2] == 5)
            sprintf(msgCommand, "page %s", noScreen);
          Serial3.write (msgCommand);
          FlushBuffer();
          return serRead[2];
        
        }
        
/********************************
  Single function call to turn off power to motor
*********************************/
        
        int turnMotorOff( int bON)
        {
          char  cmdOFF[35] = {'\0'};
          char  cmdON[34] = {'\0'};
          char  setOn[42] = {'\0'};
          char  memOn[39] = {'\0'};
          char  setOff[43] = {'\0'};
          char  memOff[40] = {'\0'};
          char  setButt[34] = {'\0'};
          char  memButt[33] = {'\0'};
          char  bitsOff[40] = {'\0'};
          char  bitsOn[40] = {'\0'};
          long  offColor = 63488;
          long  onColor = 34784;
        
        
          snprintf(cmdOFF, 34, "Home.tPowerStat.txt=\"Lift is OFF\"");
          snprintf(cmdON, 33, "Home.tPowerStat.txt=\"Lift is ON\"");
          snprintf(setOn, 41, "Settings.tSettPowerStat.txt=\"Lift is ON\"");
          snprintf(setOff, 42, "Settings.tSettPowerStat.txt=\"Lift is OFF\"");
          snprintf(memOn, 38, "Memory.tMemPowerStat.txt=\"Lift is ON\"");
          snprintf(memOff, 39, "Memory.tMemPowerStat.txt=\"Lift is OFF\"");
          snprintf(bitsOff, 34, "Bits.tBitsPower.txt=\"Lift is OFF\"");
          snprintf(bitsOn, 33, "Bits.tBitsPower.txt=\"Lift is ON\"");
          if (bON == UP)                  // Turn the motor controler off
          {
            digitalWrite ( enablePin, HIGH);
            digitalWrite (fenceEnablePin, HIGH);
            Serial3.print (cmdOFF);
            FlushBuffer();
            sprintf(cmdOFF, "Home.tPowerStat.pco=%ld\0", offColor);
            Serial3.write(cmdOFF);
            FlushBuffer();
            Serial3.print (memOff);
            FlushBuffer();
            sprintf(memOff, "Memory.tMemPowerStat.pco=%ld\0", offColor);
            Serial3.write(memOff);
            FlushBuffer();
            Serial3.print (setOff);
            FlushBuffer();
            sprintf(setOff, "Settings.tSettPowerStat.pco=%ld\0", offColor);
            Serial3.write(setOff);
            FlushBuffer();
            Serial3.print(bitsOff);
            FlushBuffer();
            sprintf(bitsOff, "Bits.tBitsPower.pco=%ld\0",offColor);
            Serial3.write(bitsOff);
            FlushBuffer();
            btPower.setValue(UP);
            btMemOff.setValue(UP);
            btSetOff.setValue(UP);
            digitalWrite (stepPin, LOW);
            digitalWrite (fenceStepPin, LOW);
            bON =  UP;
          }
          else                           // Turn the motor controller on
          {
            digitalWrite ( enablePin, LOW);
            digitalWrite (fenceEnablePin, LOW);
            Serial3.print (cmdON);
            FlushBuffer();
            sprintf(cmdON, "Home.tPowerStat.pco=%ld\0", onColor);
            Serial3.write(cmdON);
            FlushBuffer();
            Serial3.print (memOn);
            FlushBuffer();
            sprintf(memOn, "Memory.tMemPowerStat.pco=%ld\0", onColor);
            Serial3.write(memOn);
            FlushBuffer();
            Serial3.print (setOn);
            FlushBuffer();
            sprintf(setOn, "Settings.tSettPowerStat.pco=%ld\0", onColor);
            Serial3.write(setOn);
            FlushBuffer();
            Serial3.print(bitsOn);
            FlushBuffer();
            sprintf(bitsOn, "Bits.tBitsPower.pco=%ld\0", onColor);
            Serial3.print(bitsOn);
            FlushBuffer();
            btPower.setValue(DOWN);
            btMemOff.setValue(DOWN);
            btSetOff.setValue(DOWN);
            digitalWrite (stepPin, HIGH);
            digitalWrite (fenceStepPin, HIGH);
            bON = DOWN;
          }
          return bON;
        }
        
/******************************************************
  void  getSettingsScreen(char *verString)
      Function to load all the settings fro the pins and files.   be very careful with this section
      We will have a tokenized read from the config file, and the values should only  be changed in the
      these are the PINS on the arduio where all the digital communication happens

01/09/2021 ---->   need to put the settings from the fence ball screw in here.....
      
********************************************************/
        void   getSettingsScreen( char *verString)
        {
          char    tempWriter[60] = {'\0'};
          float   limTemp;
          int     i = 0;
        
          memset(tempWriter, '\0', sizeof(tempWriter));
          sprintf(tempWriter, "Settings.tUpVal.txt=\"%d\"\0", TOP_SWITCH);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tDownVal.txt=\"%d\"\0", BOTTOM_SWITCH);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tZeroPin.txt=\"%d\"\0", REZERO);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tStepPin.txt=\"%d\"\0", stepPin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tSDPin.txt=\"%d\"\0", SD_WRITE);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tEnablePin.txt=\"%d\"\0", enablePin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tDirectionPin.txt=\"%d\"\0", directionPin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tMaxSpeed.txt=\"%ld\"\0", maxMotorSpeed);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tWorkSpeed.txt=\"%ld\"\0", workingMotorSpeed);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tStepSize.txt=\"%d\"\0", stepSize);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tFStep.txt=\"%d\"\0", fenceStepPin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tFEnable.txt=\"%d\"\0", fenceEnablePin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tFDir.txt=\"%d\"\0", fenceDirPin);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tBackSW.txt=\"%d\"\0", BACK_SWITCH);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tFSW.txt=\"%d\"\0", FRONT_SWITCH);
          Serial3.write(tempWriter);
          FlushBuffer();
          sprintf(tempWriter, "Settings.tFZero.txt=\"%d\"\0", FENCE_ZERO);
          Serial3.write(tempWriter);
          FlushBuffer();

          sprintf(tempWriter, "Settings.tFileName.txt=\"%s\"\0", storeFile);
          Serial3.write(tempWriter);
          FlushBuffer();
          memset (tempWriter, '\0', sizeof(tempWriter));
          tLowLimit.getText(tempWriter, sizeof(tempWriter));
          if (strlen(tempWriter) > 0)
          {
            limTemp = atof(tempWriter) / (distPerStep * 4);
            lowLimit = -limTemp;
          }
          memset (tempWriter, '\0', sizeof(tempWriter));
          tUpLimit.getText(tempWriter, sizeof(tempWriter));
          if (strlen(tempWriter) > 0)
        
          {
            limTemp = atof(tempWriter) / (distPerStep * 4);
            highLimit = -limTemp;
          }
            tVersion.setText (verString);
            FlushBuffer();
        }
      
/****************************************************
  int stepsFromDistance( int move_go, int index)

     Returns the  index of the structure that matches with the value from the drop down
     This is typically used after the drop down selection for travel distance
     it will check the move/goto button to figure out if we are moving a set distance
     or setting the height from a fixed position.  If a fixed position, the 0 value of that
     position should be the "table" height
*********************************************/
        
        int stepsFromDistance(int move_go, int index) {
          /*
            struct inchCacl{
                int    index;
                char   inches[5];
                float  decimal;
                int    steps;
          */
          int     i = 0;
          SdFile  fSeek;
          char    fileLine[60] = {'\0'};
          char    divider[2] = {',', '\0'};
          char    *token;
          char    checker;
          int     limit;
          if (!fSeek.open("heights.txt"))
            {
              ErrorDialog( 108 );   
              return 0;
            }
          fSeek.rewind(); 
          checker = fSeek.read();
          if (move_go == HIGH)
            while (checker != '^')            // find the end of the header section, start populating the structure with preset configs
              checker = fSeek.read();
          else if (move_go == LOW)
            while (checker != char(160) )  // find the break in heights, then start reading special bit definitions 
               checker = fSeek.read(); 
          if (move_go == HIGH)
            limit = 4;
          else if ( move_go == LOW)
            limit = 5;
          checker = fSeek.read();
          while (fSeek.available())
            {
              fSeek.fgets(fileLine, sizeof(fileLine));
              token = strtok(fileLine, divider);        
              for (i=1; i<= limit; i++)
                {
                  token = strtok(NULL, divider);
                  switch (i)
                    {
                      case 1:
                        preSetLookup.index = atoi(token);
                        break;
                      case 2:
                        strcpy(preSetLookup.inches, token); 
                        break;
                      case 3:
                        preSetLookup.decimal = atof(token);
                        break;
                      case 4:
                        preSetLookup.steps = atoi(token);
                        break;
                      case 5:
                        strcpy (preSetLookup.label, token);
                        break;
                    }
                }
              if (move_go == HIGH)
                { 
                  if (preSetLookup.index == preSetNum)      // found the right line from the file, move to end of file
                    fSeek.seekEnd();
                }
              else if (move_go == LOW)
                {
                  if ( preSetLookup.index == index)         // found the right custom bit in the heights file, move to the end of the file
                  fSeek.seekEnd();
                }
                  
            }
          fSeek.close();
          return preSetLookup.steps;
        }
/****************************************************        
 *     long   calcSteps (float   inchValue)
 *         returns the number of steps to travel, given the value of inches passed to the function
 ***************************************************/
        long   calcSteps (float inchValue) {
            return inchValue / distPerStep * 4;  // 4 microsteps per step
          
        }

/*************************************************
   void   setPositionField()
            sets the tPosoition field on the home screen.   called after a router move
            03_15_2020 -- added the position field on the Memory screen
   01/03/2021 --- need a fence position field, need to set the position.   Need a variable in the function call to know which position field to change
 *************************************************/
        void setPositionField(bool bMotor) {
          char buffer[16];
          char  sCommand[40];
        
          
          if (bMotor == 1)
            curPosInch = sRouter.currentPosition() * distPerStep / 4;
          else
            curPosInch = sFence.currentPosition() * distPerStep / 4;
          if (curPosInch != 0) 
            {
              memset (buffer, '\0', sizeof(buffer));
              dtostrf(-curPosInch, 3, 4, buffer); 
              dtostrf(-curPosInch, 3, 4, posInch);
            }
          else
            strcpy(buffer, "0.00");
          if (bMotor == 1)
            {
              sprintf(sCommand, "Home.tPosition.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Settings.tSetPosit.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Bits.tBitsPos.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Memory.tMemPosit.txt=\"%s\"", buffer);
              Serial3.print(sCommand);
              FlushBuffer();
            }
          else
            {
              sprintf(sCommand, "Home.tFencePos.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Settings.tFencePos.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Memory.tFencePos.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
              sprintf(sCommand, "Bits.tFencePos.txt=\"%s\"", buffer);
              Serial3.write(sCommand);
              FlushBuffer();
            }      
        }
             
/****************************************************************
  void SetSpeedFromSlider (float iSpeed, byte button)

     setSpeedFromSlider will set the  new motor speed in steps per second
     and update the UI wth the new spead values
 *********************************************************/
        
        void setSpeedFromSlider (float iSpeed, byte button) {
          long perCalc;
          char   serCommand[55] = {'\0'};
           if (button == HIGH)
          {
            perCalc = (iSpeed / maxMotorSpeed) * maxMotorSpeed;
          }
          else
          {
            perCalc = (iSpeed / 100) * maxMotorSpeed;
          }
          hMoveSpeed.setValue(perCalc);
          nSetSpeed.setValue(perCalc);
          sprintf(serCommand, "Settings.tWorkSpeed.txt=\"%ld\"\0", perCalc);
        
          Serial3.write(serCommand);
          FlushBuffer();
          workingMotorSpeed = perCalc;
        
        }
/***************************************
    void Flushbuffer()
        when serial write to the Nextion, need to send final command
 ***********************************/
        void FlushBuffer() {
          Serial3.write(0xff);
          Serial3.write(0xff);
          Serial3.write(0xff);
        }
        
/*************************************************
    void  deleteFromMemory( int memrow)
            Deletes a row from the memory screen
 ******************************/
        void deleteFromMemory (int memrow) {
        
          char zero[21] = {'\0'};
          char steps[20] = {'\0'};
          char label[27] = {'\0'};
        
          snprintf (zero, 14, "tZero%d.txt=\"\"", memrow);
          snprintf(steps, 18, "tMem%dSteps.txt=\"\"", memrow);
          snprintf(label, 26, "tMem%d.txt=\"\"", memrow);
          Serial3.print(zero);
          FlushBuffer();
          Serial3.print(steps);
          FlushBuffer();
          Serial3.print(label);
          FlushBuffer();
        }
        
/*************************************************
    void  saveToMemory( int memrow, double posInch, char curPos)
            adds the current position of the router to 1 of the saved memory spots on Memory Page
 ******************************/
        void saveToMemory (int memrow) {
        
          char zero[30] = {'\0'};
          char steps[30] = {'\0'};
        
          snprintf(zero, 22, "tZero%d.txt=\"%s\"", memrow, posInch);
          snprintf(steps, 24, "tMem%dSteps.txt=\"%d\"", memrow, curPos);
          Serial3.print(zero);
          FlushBuffer();
          Serial3.print(steps);
          FlushBuffer();
        }
        
/*************************************************
   void  loadMemorytoRouter( int memrow)
           loads the values from memory and sets the router to that position.
******************************/
        void loadMemorytoRouter ( NexText &button, int bHow) {
          char buffer[31];
          float inches;
          long stepsToMove = 0;
          byte bGo = DOWN;
          int  buttonNum;
          byte direct;
          uint16_t  characters;

          while (Serial3.read() != -1);
          memset (buffer, '\0', sizeof(buffer));
          characters = button.getText(buffer, sizeof(buffer));
          inches = atof(buffer);  
          stepsToMove = calcSteps (inches); 
          while (Serial3.read() != -1);
          sRouter.move(stepsToMove);
          if (stepsToMove < -sRouter.currentPosition())  // modified 3/7/2020 - because of storing + positions when bit above table
           {
            sRouter.setSpeed(workingMotorSpeed);
            direct = UP;
           }
                
          else
          {
            sRouter.setSpeed (-workingMotorSpeed);
            direct = DOWN;
          }
/* Serial.println("about to call ErrorDialog(200)");
         ErrorDialog(200);
         if (lowLimit == 0)
            buttonNum = ErrorDialog( 200 );   
  */        while (-sRouter.currentPosition() != stepsToMove && bGo && digitalRead(TOP_SWITCH) == HIGH && digitalRead(BOTTOM_SWITCH) == HIGH)  // modified this, as the opposite of the standard values.   Saving above table as + numbers
            {          
              sRouter.runSpeed();
              if (bGo)
                bGo = checkStopButton();
            }
          if (digitalRead(TOP_SWITCH) != HIGH || digitalRead(BOTTOM_SWITCH) != HIGH)     // hit a limit switch in the previous loop
            {
              Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
              FlushBuffer();
              stepsToMove = sRouter.currentPosition() + 600; 
              sRouter.move(stepsToMove);                         // hit a limit switch and need to bounce the router
              if (direct == UP)
                sRouter.setSpeed (-workingMotorSpeed);
              else
                sRouter.setSpeed(workingMotorSpeed);
              while (sRouter.currentPosition() != stepsToMove)
                sRouter.runSpeed();
              bGo = UP;
            }
          Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
          FlushBuffer();
          setPositionField(1);    
        }
        
/********************************************************************
 *    void bCustBitPopCallback (void *ptr)
 *        used on the Bits screen to lift the bit to the right height 
 *        likely will call either LoadMemorytoRouter or bAutoCallBack (from the presets)
 *        Needs to calculate steps from thickness (1/2 thickness entered + bit center)
 ************************************************************************/
       void bCustBitPopCallback (void *ptr){
            
          char  thickness[16];
          int   fraction;
          float  inches;
          uint32_t   decimal;
          char   sCommand[80];
          char  newThick[16];
          char   buffer[31];

          memset (thickness, '\0', sizeof(thickness));
          tThickEntry.getText(thickness, sizeof(thickness));
          sw0.getValue(decimal);
          fraction = decimal;
          if (fraction == 0)
            {
              inches = (atof(thickness)/2) + preSetLookup.decimal;
              dtostrf(inches, 3, 4, newThick);
              sprintf(sCommand, "t3.txt=\"%s\"",newThick);
              Serial3.write(sCommand);
              FlushBuffer();
            memset (buffer, '\0', sizeof(buffer));
              fraction =t3.getText(buffer, sizeof(buffer));
            }
          
          loadMemorytoRouter (t3, LOW);
       }
      
/*************************************************
    void  memToFileSD(  NexText &Desc, NexText &Value, NexText &Steps)
            writes index, description, inch value and number of steps to the SD card
 ******************************/
        void memToFileSD ( int index, NexText &Desc, NexText &Value, NexText &Steps, char divider, char section) {
         //void memToFileSD ( int index, char *Desc, char *Value, char *Steps, char divider, char section) {
     
          SdBaseFile   fConfig;
          SdBaseFile   fReader;
          char  separate[2] = {divider, '\0'};
        
          struct preSets  {
            char    section[2];
            int     counter;
            char    Name[26];
            char   inches[12];
            char  steps[12];
          }  writeString[8];
        
          char  Name[40] = {'\0'};
          char  writeS[40] = {'\0'};
          char  *token;
          char  checker;
          byte  bFound = 0;
          byte  bGo = DOWN;
          unsigned long   fPos;
          int   element;
 
          
          
         
        
          //http://www.asciitable.com/
        
          if ( !fReader.open(storeFile))
            ErrorDialog (202); 
          if (!fConfig.open(storeFile, O_RDWR ))
            ErrorDialog (204); 
          fReader.seekSet(0);
          while (checker != '^')            // Read the header of the file, and bipass writing it
              checker = fReader.read();
          for (int counter = 1; counter <=7; counter++)
            memset (writeString, '\0', sizeof(writeString));
          fReader.fgets(Name, sizeof(Name));        // read the blank line prior to the memory starting
          fPos = fReader.curPosition();
          
          for (int counter = 1; counter <= 7; counter++)
            {
              memset(Name, '\0', sizeof(Name));
              fReader.fgets (Name, sizeof(Name));
              element = 1;
              if (counter != index)
                {
                   token = strtok(Name, separate);                      
                  strcpy(writeString[counter].section, token); 
                    while (element < 6)
                  {
                    token = strtok(NULL, separate);
                    switch (element)
                    {
                     case 1:
                        writeString[counter].counter = atoi(token);
                        element++;
                        break;
                      case 2:
                        if (token != NULL)
                          sprintf(writeString[counter].Name, "%s", token);
                          if (writeString[counter].Name[0] == '\n')
                            writeString[counter].Name[0] = '\0';
                        element++;
                        break;
                      case 3:
                        if (token != NULL)
                          sprintf(writeString[counter].inches, "%s", token);
                        element++;
                        break;
                      case 4:
                        if (token != NULL)
                          sprintf(writeString[counter].steps, "%s", token);
                        element++;
                        break;
                      default:
                        element++;
                        break;
                    }
                  }
                }
               else
                {
                   writeString[counter].section[0] = section;
                   writeString[counter].section[1] = '\0';
                   writeString[counter].counter = index;
                   Desc.getText(writeString[counter].Name, sizeof(writeString[counter].Name));
                   Value.getText(writeString[counter].inches, sizeof(writeString[counter].inches));
                   Steps.getText(writeString[counter].steps, sizeof(writeString[counter].steps));
                }
            }
          fReader.close();
          fConfig.seekSet (fPos);
          for (int counter=1; counter <=7; counter++)
            {
               sprintf(writeS, "%c%s%c%d%c%s%c%s%c%s%c\n", 
                  divider, writeString[counter].section,
                  divider, writeString[counter].counter,
                  divider, writeString[counter].Name,
                  divider, writeString[counter].inches,
                  divider, writeString[counter].steps,
                  divider);
               fConfig.write(writeS);
            }
          fConfig.sync();          
          fConfig.close();
        }
/*****************************************************
   int  LoadFromFile ( int StartRow, int NumRows)
          Loads data from config file for the start position up to te number of rows.
          input:
                  StartRow == which index row to start reading
                  MaxRow == which row to stop reading
                  fileName == which file to use for reading
                  PageNum == which section in the file (going to be used to store different type of settings)
          puts the data into the data fields on the screen
 *****************************************************/
        
        int  LoadFromFile ( int StartRow, int MaxRow, int tokenKey, int Section)  {
        
          SdBaseFile    fReader;
          int           element;
          int           counter;
          int           index;
          char          delimeter[2] = {char(tokenKey), '\0'};
          char          *token;
          char          checker;
          uint32_t      filePos;
          char          divider = '^';
        
          struct preSets  {
            char    delimeter[2];
            int     counter;
            char    Name[27];
            char   inches[12];
            char  steps[12];
          }  memory[8];
        
          char        Name[38] = {'\0'};
          char        inches[22] = {'\0'};
          char        steps[25] = {'\0'};
          char        writeS[75] = {'\0'};
        
          if ( !fReader.open(storeFile))
            ErrorDialog (206);   
          index = StartRow;
          fReader.rewind();
          while (writeS[3] != '0' + index && fReader.available()) // read through the header and excluded lines
            fReader.fgets(writeS, sizeof(writeS));
          while (index >= StartRow && index <= MaxRow && fReader.available())
          {
            if (index != StartRow)
            {
              memset (writeS, '\0', sizeof(writeS));
              fReader.fgets(writeS, sizeof(writeS));
            }
            if (writeS[3] != '\0')
              {
                token = strtok(writeS, delimeter);            // tokenize the line and split out the appropriate fields from the string
             
                strcpy(memory[index].delimeter, char(token));
                element = 1;
                while (token != NULL)
                {
                  token = strtok(NULL, delimeter);
                  switch (element)
                  {
                    case 0:
                      element++;
                      break;
                    case 1:
                      memory[index].counter = atoi(token);
                      element++;
                      break;
                    case 2:
                      sprintf(memory[index].Name, "%s", token);
                      if (memory[index].Name[0] == '\n')
                        memory[index].Name[0] = '\0';
                      element++;
                      break;
                    case 3:
                      sprintf(memory[index].inches, "%s", token);
                      element++;
                      break;
                    case 4:
                      sprintf(memory[index].steps, "%s", token);
                      element++;
                      break;
                    case 5:
                      break;
                  }
                }
                index++;
              }
          }
          MaxRow = index;
          for ( index = StartRow; index <= MaxRow; index++)                     //  write the lines to the UI, as many as were round in the file
          {
            snprintf(Name, 37, "tMem%d.txt=\"%s\"", index, memory[index].Name);
            snprintf(inches, 21, "tZero%d.txt=\"%s\"", index, memory[index].inches);
            snprintf(steps, 24, "tMem%dSteps.txt=\"%s\"", index, memory[index].steps);
            Serial3.write(Name);
            FlushBuffer();
            Serial3.write(inches);
            FlushBuffer();
            Serial3.write(steps);
            FlushBuffer();
          }
          fReader.close();
        }
        
/***************************************************
     int   checkStopButton ()
             checks to see if the Off button has been pressed during an automated move of the lift.
             usefull in chagnge bit, move router to limit or Auto button functions
 **********************************************************************/
        byte  checkStopButton ()
        {     
          int incomingByte;
          int index;
          int   StopButtons[3][2] = {
            {0, 33}, {1, 14}, {2, 56}     //   these are the pages/control #'s for the stop buttons.  if the GUI changes, check here
          };
          int  screen;
          int  button;
        
          bGo = DOWN;
          incomingByte = Serial3.read();

          while (incomingByte > -1 )
          {
            if (incomingByte == 101)
              {
                screen = Serial3.read();
                button = Serial3.read();
 
              }
            if ((StopButtons[0][0]== screen && StopButtons[0][1] == button) ||
                (StopButtons[1][0] == screen && StopButtons[1][1] == button) || 
                (StopButtons[2][0] == screen && StopButtons[2][1] == button) )
            {
              bGo = UP;
              turnMotorOff(UP);
              return bGo;
            }
            index++;
            incomingByte = Serial3.read();
          }
          return bGo;
        }
        
/*******************************************************
   int moveRouterToLimit(byte bDirection)

       Move to LIMIT is for moving the router until it hits a limit switch
       called by Change Bit  and the ToBottom
          01/24/2020   CDW -- Need to add the bounce function to the limit switch being triggered.    LIkely I will have a "bounce" function so I can call it from all
                              buttons that are free moving.   Whenever a limit switch is hit, we should set the speed to something slow, and bounce back until we enable the
                              limit switch again.   Using a "while LIMITSWITCH==0... move in the opposite direction....."
                              UP/DOWN is motor direction based on lead screw being inverted... UP is dow and DOWN is up
          01/31/2020   CDW -- change the function declaration to return a DOUBLE, it was an int.   Check where function is called to make sure the return is into a DOUBLE
          02/09/2020   CDW -- added the check of the OFF button being pushed.   Will use this functionality to read the reial buffer
                              if the off button is pushed, the motor shoudl stop moving.
  01/03/20201 --- need to rewrite logic to know which motor is moving.   bDirection could be up or down, front or back.   If front to back, need to use sFence instead of sRouter
 * ****************************************/
        
        double moveRouterToLimit (byte bDirection, double dSpeedFlag)
        {
          int whichSwitch;
          bGo = DOWN;
          if (bDirection == DOWN)
          {
            whichSwitch = TOP_SWITCH;
          }
          else
          {
            whichSwitch = BOTTOM_SWITCH;
          }
          while (digitalRead(whichSwitch) == HIGH && bGo)
          {
        
            if (bDirection == DOWN)
            {
              curPos = sRouter.currentPosition() - stepSize;
              sRouter.move(curPos);
              if ( dSpeedFlag == -1 )
              {
                sRouter.setSpeed(-workingMotorSpeed);
              }
              else
              {
                sRouter.setSpeed (-dSpeedFlag);
              }
            }
            else
            {
              curPos = sRouter.currentPosition() + stepSize;
              sRouter.move(curPos);
              if (dSpeedFlag == -1)
              {
                sRouter.setSpeed(workingMotorSpeed);
              }
              else
              {
                sRouter.setSpeed (dSpeedFlag);
              }
        
            }
            while (sRouter.currentPosition() != curPos )
            {
              sRouter.runSpeed();
            }
            bGo = checkStopButton();
          }
          Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
          FlushBuffer();
        
          while ( digitalRead(whichSwitch) == LOW  )   //  bounce the screw off the limiter switch
          {
            if (bDirection == DOWN)
            {
              curPos = sRouter.currentPosition() + stepSize;      //just the opposit as above
              sRouter.move(curPos);
              if ( dSpeedFlag == -1)
              {
                sRouter.setSpeed(workingMotorSpeed);
              }
              else
              {
                sRouter.setSpeed(dSpeedFlag);
              }
            }
            else
            {
             curPos = sRouter.currentPosition() - stepSize;      //just the opposite as above
              sRouter.move(curPos);
              if ( dSpeedFlag == -1)
              {
                sRouter.setSpeed(-workingMotorSpeed);
              }
              else
              {
                sRouter.setSpeed(-dSpeedFlag);
              }
            }
            while (sRouter.currentPosition() != curPos )
              sRouter.runSpeed();
          }
          Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
          FlushBuffer();
          return sRouter.currentPosition();
        }
        
/***********************************************
   void bMoveUpPushCallback(void *ptr)

         Button bMoveUP component push callback function.
         While the button is pressed, the router should move UP and interupt if the limit button is hit
   01/03/2021 --- copy this function for the Fence move back button
******************************************/
      
        void bMoveUpPushCallback(void *ptr) {
          int cBuff = -1;
        
          while (digitalRead(TOP_SWITCH) == HIGH && cBuff == -1 && bGo)
          {     
            curPos = sRouter.currentPosition() - stepSize;
            sRouter.move(curPos);
            sRouter.setSpeed(-workingMotorSpeed);
            while (sRouter.currentPosition() != curPos  && workingMotorSpeed > 0)
            {
              sRouter.runSpeed();
            }
            cBuff = Serial3.read ();
          }
        
/******************************************
  Becuase I intercepted the serial buffer to see if the button was lifted,
  I have to call the PopCallBack to emulate the button being lifted
  Ths test says, the button was lifted before the limit switch was hit
  because the event was not processed on the button itself, we have to tell the
  controller, the button was lifed up
******************************************************************/
          if (cBuff == 101 )
          {
            bMoveUpPopCallback(&bMoveUp);
          }
          else                                        // we have hit the bottom limit switch
          {
            Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
            while ( digitalRead(TOP_SWITCH) == LOW)   //  bounce the screw off the limiter switch
            {
              curPos = sRouter.currentPosition() + stepSize;      //just the opposite as above
              sRouter.move(curPos);
              sRouter.setSpeed(workingMotorSpeed);
        
              while (sRouter.currentPosition() != curPos  && workingMotorSpeed > 0)
              {
                sRouter.runSpeed();
              }
            }
            highLimit = -curPos;
            Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
          }
        
        }
        
/***************************************************
   void bMoveUpPopCallback(void *ptr)

         Button bMoveUp component pop callback function.
         When the ON button is released, the LED turns on and the state text changes.
************************************/
        void bMoveUpPopCallback(void *ptr) {
          setPositionField(1);     
        }
        
/********************************************************
    void bMoveDownPushCallback (void *ptr)

         Button bMoveUP component push callback function.
        While the button is pressed, the router should move DOWNP and interupt if the limit button is hit

 01/03/2021 -- need to copy this for the Fence move forward button, change sRouter to sFence
 *****************************************************/
        void bMoveDownPushCallback (void *ptr)  {
          int cBuff = -1;
          char buffer[16];     
          while (digitalRead(BOTTOM_SWITCH) == HIGH && cBuff == -1 && bGo)
          {
            curPos = sRouter.currentPosition() + stepSize;
            sRouter.move(curPos);
            sRouter.setSpeed(workingMotorSpeed);
        
            while (sRouter.currentPosition() != curPos  && workingMotorSpeed > 0)
            {
              sRouter.runSpeed();
            }
            cBuff = Serial3.read ();
          }
          /******************************************
            Becuase I intercepted the serial buffer to see if the button was lifted,
            I have to call the PopCallBack to emulate the button being lifted
            Ths test says, the button was lifted before the limit switch was hit
            because the event was not processed on the button itself, we have to tell the
            controller, the button was lifed up
          ******************************************************************/
          if (cBuff == 101 )
          {
            bMoveUpPopCallback(&bMoveDown);
          }
          else                                          // we have hit the bottom limit switch
          {
            Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
            while ( digitalRead(BOTTOM_SWITCH) == LOW)   //  bounce the screw off the limiter switch
            {
              curPos = sRouter.currentPosition() - stepSize;      //just the opposite as above
              sRouter.move(curPos);
              sRouter.setSpeed(-workingMotorSpeed);
        
              while (sRouter.currentPosition() != curPos  && workingMotorSpeed > 0)
              {
                sRouter.runSpeed();
              }
            }
            lowLimit = -curPos;
            Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
          }
        }
        
/****************************************************
   void bMoveDownPopCallback(void *ptr)

        Button bMoveDown component pop callback function.
        When the button is released, turn off the motor controller
*****************************************************/
        void bMoveDownPopCallback(void *ptr) {
        
          setPositionField(1);
        }
             

/***********************************************
   void bForwardPushCallback(void *ptr)

         Button bForward component push callback function.
         While the button is pressed, the router should move UP and interupt if the limit button is hit
   01/08/2021 --- Modify this for moving fence motor, this is only a prototype
******************************************/
      
        void bForwardPushCallback(void *ptr) {
          int cBuff = -1;
        
          while (digitalRead(FRONT_SWITCH) == HIGH && cBuff == -1 && bGo)
          {     
            curPos = sFence.currentPosition() - stepSize;
            sFence.move(curPos);
            sFence.setSpeed(-workingMotorSpeed);
            while (sFence.currentPosition() != curPos  && workingMotorSpeed > 0)
            {
              sFence.runSpeed();
            }
            cBuff = Serial3.read ();
          }
        
/******************************************
  Becuase I intercepted the serial buffer to see if the button was lifted,
  I have to call the PopCallBack to emulate the button being lifted
  Ths test says, the button was lifted before the limit switch was hit
  because the event was not processed on the button itself, we have to tell the
  controller, the button was lifed up
******************************************************************/
          if (cBuff == 101 )
          {
            bForwardPopCallback(&bMoveUp);
          }
          else                                        // we have hit the bottom limit switch
          {
            Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
            while ( digitalRead(FRONT_SWITCH) == LOW)   //  bounce the screw off the limiter switch
            {
              curPos = sFence.currentPosition() + stepSize;      //just the opposite as above
              sFence.move(curPos);
              sFence.setSpeed(workingMotorSpeed);
        
              while (sFence.currentPosition() != curPos  && workingMotorSpeed > 0)
              {
                sFence.runSpeed();
              }
            }
            highLimit = -curPos;
            Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
          }
        
        }
        
/***************************************************
   void bForwardPopCallback(void *ptr)

         Button bMoveUp component pop callback function.
         When the ON button is released, the LED turns on and the state text changes.
  01/08/2021 -- make sure to set the Fence position field on this - needs to be modified
************************************/
        void bForwardPopCallback(void *ptr) {
          setPositionField(0);     
        }
        
/********************************************************


/********************************************************
    void bBackPushCallback (void *ptr)

         Button bBack component push callback function.
        While the button is pressed, the fence will move away from the bit until the button is released 
        or you hit the limit switch

 *****************************************************/
        void bBackPushCallback (void *ptr)  {
          int cBuff = -1;
          char buffer[16];   
          while (digitalRead(BACK_SWITCH) == HIGH && cBuff == -1 && bGo)
          {
            curPos = sFence.currentPosition() + stepSize;
            sFence.move(curPos);
            sFence.setSpeed(workingMotorSpeed);
            while (sFence.currentPosition() != curPos  && workingMotorSpeed > 0)
            {
              sFence.runSpeed();
            }
            cBuff = Serial3.read ();
          }
          /******************************************
            Becuase I intercepted the serial buffer to see if the button was lifted,
            I have to call the PopCallBack to emulate the button being lifted
            Ths test says, the button was lifted before the limit switch was hit
            because the event was not processed on the button itself, we have to tell the
            controller, the button was lifed up
          ******************************************************************/
          if (cBuff == 101 )
          {
            bBackPopCallback(&bBack);
          }
          else                                          // we have hit the bottom limit switch
          {
            Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
 
            while ( digitalRead(BACK_SWITCH) == LOW)   //  bounce the screw off the limiter switch
            {
              curPos = sFence.currentPosition() - stepSize;      //just the opposite as above
              sFence.move(curPos);
              sFence.setSpeed(-workingMotorSpeed);
        
              while (sFence.currentPosition() != curPos  && workingMotorSpeed > 0)
              {
                sFence.runSpeed();
              }
            }
            lowLimit = -curPos;
            Serial3.print("vis pStop,0");              // bring up the Stop sign to help remind we have hit a limit switch
            FlushBuffer();
          }
        }
        
/****************************************************
   void bBackPopCallback(void *ptr)

        Button bBack component pop callback function.
        When the button is released, set the fence position field
*****************************************************/
        void bBackPopCallback(void *ptr) {
          setPositionField(0);
        }

/************************************************************
   void swHowPopCallback(void *ptr)

         Nothing implemented on this slider button yet
*************************************************************/
        void swHowPopCallback(void *ptr) {      
        }
                
/**************************************************
 * void  swFencePopCallback(void *ptr)
 *      switch swFence on the home screen, will set the HOM_MOTOR byte to know which motor is to be
 *      used by a number of buttons on the home screen   Implmeneted in v. 5.50
 **************************************************/
  void  swFencePopCallback(void *ptr)
    {
        uint32_t   uMotor;
 
        swFence.getValue(&uMotor);
        HOME_MOTOR = uMotor;
    }


/**************************************************
 * void  swWhichPopCallback(void *ptr)
 *      switch swFence on the home screen, will set the HOM_MOTOR byte to know which motor is to be
 *      used by a number of buttons on the home screen   Implmeneted in v. 5.50
 **************************************************/
  void  swWhichPopCallback(void *ptr)
    {
        uint32_t   uMotor;
 
        swWhich.getValue(&uMotor);
        SET_MOTOR = uMotor;
    }

/***********************************************

   void bReZeroPopCallback(void *ptr)

         Button bReZero component push callback function.
         When the button is released, we will want to set a new ZERO reference for the lift.

**************************/
        
        void bReZeroPopCallback(void *ptr) {
        
          if (HOME_MOTOR == 1)
            {
              sRouter.setCurrentPosition(0);
              setPositionField(1);
            }
          else
            {
              sFence.setCurrentPosition (0);
              setPositionField(0);
            }
          
        }
        
/***********************************************
   void bZeroPopCallback(void *ptr)

         Button bZero component push callback function.
         When the button is released, we will want to set a new ZERO reference for the lift.
         Make sure setspeed() is called after move()

 01/13/2021 -- tookk out while loop with run() function, changed to runSpeedToPosition() for router life section.   
**************************/
        
        void bZeroPopCallback(void *ptr) {
        
          char   buffer[20];
          uint32_t   uDirection;
          byte       bDir;
        
          if (SET_MOTOR == 1 )
            {
              while (digitalRead(REZERO) == HIGH && digitalRead(TOP_SWITCH) == HIGH && bGo)   // keep moving the router up until we hit the calibration bar or top limit
                {
                  curPos = sRouter.currentPosition() - stepSize;
                  sRouter.move(curPos);
                  sRouter.setSpeed(-workingMotorSpeed);
                  sRouter.runSpeedToPosition();
                  bGo = checkStopButton();
                }
              if (!bGo)
                bGo = turnMotorOff(UP);
            
              sRouter.setCurrentPosition(0);
              strcpy(buffer, "tUpLimit.txt=\"0.00\"");
              Serial3.write(buffer);
              FlushBuffer();
              setPositionField(1); 
            }
          else        // we have chosen to 0 the fence.   Need to check if zeroing from the front to back, or back to front
            {
              swFenceDir.getValue (&uDirection);
              while (digitalRead(FENCE_ZERO) == HIGH && digitalRead(BACK_SWITCH) == HIGH && bGo && digitalRead (FRONT_SWITCH) == HIGH)   // keep moving the router up until we hit the calibration bar or top limit
                {             
                  if (uDirection == 0 )
                    curPos = sFence.currentPosition() - stepSize;
                  else
                    curPos = sFence.currentPosition() + stepSize;
                  sFence.move(curPos);
                  if (uDirection == 0 )
                    sFence.setSpeed ( -workingMotorSpeed);
                  else
                    sFence.setSpeed (workingMotorSpeed);
                  sFence.runSpeedToPosition();
                  bGo = checkStopButton();
                }
              if (!bGo)
                bGo = turnMotorOff(UP);
            
              sFence.setCurrentPosition(0);
              setPositionField(0); 
            }
        }
        
/******************************************************************
   void bChangeBitPopCallback(void *ptr)

   Button bHomeChageBit component pop callback function.
   Moves the router up at max speed until the limit switch is hit

 01/03/2021 -- move the fence back to teh back limit switch.   Need to move the fence out of the way of the router hole insert  
******************************************************************/
        void bChangeBitPopCallback(void *ptr) {
        
          uint32_t  speedFlag;
          char   buffer[12];
        
          bGo = DOWN;
          rMax.getValue(&speedFlag);
          if (speedFlag == 1)
          {
            highLimit = -moveRouterToLimit (DOWN, maxMotorSpeed);
          }
          else
          {
            highLimit = -moveRouterToLimit ( DOWN, -1 );
          }        
          setPositionField(1);        
        }
                        
/*****************************************************
   bToBottomPopCallback(void *ptr)

         Button bToBottom component pop callback function.
         Moves the router up at max speed until the limit switch is hit (Bottom Limit Switch)
***************************************/
        void bToBottomPopCallback(void *ptr) {
        
          uint32_t  speedFlag;
        
          bGo = DOWN;
          rMax.getValue(&speedFlag);
          if (speedFlag == 1)
          {
            lowLimit = -moveRouterToLimit (UP, maxMotorSpeed);
          }
          else
          {
            lowLimit = -moveRouterToLimit ( UP, -1 );
          }
          setPositionField(1);
        }
               
/***********************************************
   void bCalibPopCallback(void *ptr)

         Button bCalib component push callback function.
         WMoves the router to the bottom limit switch, sets the bottom limit field, then moves to top limit switch and sets upper limit field value

  01/03/2021 --- implement the logic for limits for the fence as well.   There should be an upper, lower, front and back limit for the 2 motors  
**************************/
        
        void bCalibPopCallback(void *ptr) {
        
          char    buffer[16];
          float   temp;
          double  curPosInch;
        
          //  mover router to upper limit switch, set the uppor limit variable, then move to bottom Limit switch, store bottom limit
          bGo = DOWN;
          highLimit = -moveRouterToLimit( DOWN, maxMotorSpeed );
          if (bGo)
          {
            memset (buffer, '\0', sizeof(buffer));
            temp = sRouter.currentPosition() * distPerStep / 4;
            dtostrf(-temp, 3, 4, buffer);
            tUpLimit.setText(buffer);
        
            lowLimit = -moveRouterToLimit( UP, maxMotorSpeed );
            if (bGo)
            {
              memset (buffer, '\0', sizeof(buffer));
              curPosInch = sRouter.currentPosition() * distPerStep / 4; // converts the current position to inches from 0
              dtostrf(-curPosInch, 3, 4, buffer);
              tLowLimit.setText(buffer);
            }
          }
          setPositionField(1);
        }
        
/******************************************************
   void bAutoMovePopCallback(void *ptr)

         Button bAutoMove component pop callback function.
         need to check values of direction and move switch to ensure we are moving the right direction
         if switch is set to "goto" we can only go up... it doesn't make sense to goto a negative position
         as the bit will be below the table
         
*********************************************/
        void bAutoMovePopCallback(void *ptr) {
          //  value from swHow   1 is move ||  0 is goto top of table, set bit to height
          //  value from swDirection  0 moves DOWN  || 1 moves UP
        
          float       microSteps = 0;
          uint32_t    bDirection;
          uint32_t    bHow;
          byte        finished = LOW;
          int         StructIndex;
          byte        dir;
          byte        gomove;
          float       calcInch;
        
          swHow.getValue(&bHow);
          gomove = bHow;
          StructIndex = stepsFromDistance(HIGH,0);
          if (gomove == 0 && StructIndex >= 0)
          {
            bgotoZeroPopCallback(&bgotoZero);   // if we are setting an absolute value, go to 0 first, then set the bit height (testing for Fence handled inside)
          }
           
          if (StructIndex >= 0)
          {
            microSteps = (microPerStep * 4) * preSetLookup.steps;
 
            if (HOME_MOTOR == 1)
              curPos = sRouter.currentPosition();
            else
              curPos = sFence.currentPosition();
          }
        
          if (microSteps)       // only move if there are steps to move, All logic should be wrapped in a mocrosteps function
          {
            if (gomove == LOW)
            {
              swDirection.setValue(LOW);
            }
            swDirection.getValue(&bDirection);
            if (bDirection == UP)
            {
              if (HOME_MOTOR == HIGH )
                {
                  curPos = sRouter.currentPosition() - microSteps;
                  sRouter.move(curPos);
                  sRouter.setSpeed(-workingMotorSpeed);
                }
              else
                {
                  curPos = sFence.currentPosition() - microSteps;
                  sFence.move(curPos);
                  sFence.setSpeed(-workingMotorSpeed);
                }        
            }
            else
            {
              if (HOME_MOTOR == HIGH )
                {
                  curPos = sRouter.currentPosition() + microSteps;
                  sRouter.move(curPos);
                  sRouter.setSpeed(workingMotorSpeed);
                }
              else
                {
                  curPos = sFence.currentPosition() + microSteps;
                  sFence.move(curPos);
                  sFence.setSpeed(workingMotorSpeed);
                }
              
            }
            if (HOME_MOTOR == 1)
             {
                while (sRouter.currentPosition() != curPos  && bGo)
                  {
                    sRouter.runSpeed();
                    if (digitalRead(BOTTOM_SWITCH) == LOW )     // See if we have bottomed out the router
                    {
              
                      Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
                      FlushBuffer();
                      curPos = sRouter.currentPosition() - stepSize;
              
                      sRouter.move(curPos);
                      sRouter.setSpeed(-workingMotorSpeed);
                      while (digitalRead(BOTTOM_SWITCH) == LOW)  // if we have bottomed out - bounce the router off the limit switch
                      {
                        sRouter.runSpeed();
                      }
                      curPos = sRouter.currentPosition();
                      lowLimit = -curPos;
                    }
                    if (digitalRead(TOP_SWITCH) == LOW )          // see if we have hit the top switch
                    {
                      Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
                      FlushBuffer();
                      curPos = sRouter.currentPosition() + stepSize;
              
                      sRouter.move(curPos);
                      sRouter.setSpeed(workingMotorSpeed);
                      while (digitalRead(TOP_SWITCH ) == LOW)    //bounce the router after hitting the top switch
                      {
                        sRouter.runSpeed();
                      }
                      curPos = sRouter.currentPosition();
                      highLimit = -curPos;
                    }
                  }
              }
            else
              {
                 while (sFence.currentPosition() != curPos  && bGo)
                  {
                    sFence.runSpeed();
                    if (digitalRead(BACK_SWITCH) == LOW )     // See if we have bottomed out the router
                      {
              
                        Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
                        FlushBuffer();
                        curPos = sFence.currentPosition() - stepSize;
                
                        sFence.move(curPos);
                        sFence.setSpeed(-workingMotorSpeed);
                        while (digitalRead(BACK_SWITCH) == LOW)  // if we have bottomed out - bounce the router off the limit switch
                        {
                          sFence.runSpeed();
                        }
                        curPos = sFence.currentPosition();
                        lowLimit = -curPos;
                      }
                    if (digitalRead(FRONT_SWITCH) == LOW )          // see if we have hit the top switch
                      {
                        Serial3.print("vis pStop,1");              // bring up the Stop sign to help remind we have hit a limit switch
                        FlushBuffer();
                        curPos = sFence.currentPosition() + stepSize;
                
                        sFence.move(curPos);
                        sFence.setSpeed(workingMotorSpeed);
                        while (digitalRead(FRONT_SWITCH ) == LOW)    //bounce the router after hitting the top switch
                        {
                          sFence.runSpeed();
                        }
                        curPos = sFence.currentPosition();
                        highLimit = -curPos;
                      }
                  }
              }
            Serial3.print("vis pStop,0");              // Turn the stop sign off
            FlushBuffer();
        
          } // end of If microsteps > 0
          setPositionField(1);
          setPositionField(0);
        }
        
/***********************************************
   void bgotoZeroPopCallback(void *ptr)

         Button bgotoZero component push callback function.
         Moves the router to the bottom limit switch, sets the bottom limit field, then moves to top limit switch and sets upper limit field value


  01/03/2021 --- implement check for fence or router movement.   Same logic can be used, but need a flag for whether we are moving the fence       
**************************/
        
        void bgotoZeroPopCallback(void *ptr) {
          String  sHold;
          long    newPos;
          char    buffer[16];
          byte    bGo = DOWN;
          int     test;
          uint32_t  but;

          vaDelIndex.getValue(&but);
          test = but;
          vaFence.getValue (&but);   // need to test variable set by the screen navigation buttons.   When moving off th ehome screen, variable set to Router
          HOME_MOTOR = but;

          if (HOME_MOTOR == 1)
            {
               curPos = sRouter.currentPosition();
               sRouter.move(0);
            }
          else
            {
              curPos = sFence.currentPosition();
              sFence.move(0);
            }          
          if (curPos < 0)
            if (test != 100)
              if (HOME_MOTOR == 1)
                sRouter.setSpeed(workingMotorSpeed);
              else
                sFence.setSpeed (workingMotorSpeed);
            else
              if (HOME_MOTOR == 1)
                sRouter.setSpeed( maxMotorSpeed);
              else
                sFence.setSpeed (maxMotorSpeed);
          else
            if ( test != 100)
              if (HOME_MOTOR == 1)
                sRouter.setSpeed (-workingMotorSpeed);
              else
                sFence.setSpeed (-workingMotorSpeed);
            else
              if (HOME_MOTOR == 1)
                sRouter.setSpeed (-maxMotorSpeed);
              else
                sFence.setSpeed (-maxMotorSpeed);
          
          if (HOME_MOTOR == 1)
            {
              while (sRouter.currentPosition() != 0 && bGo )
                {
                  sRouter.runSpeed();
                  bGo = checkStopButton();
                }
              setPositionField(1);
            }
          else
            {
              while (sFence.currentPosition() != 0 && bGo )
                {
                  sFence.runSpeed();
                  bGo = checkStopButton();
                }
              setPositionField(0);
            }
          
        }
        
/***********************************************
   void bSetDownPushCallback(void *ptr)

         Button bSetDown component push callback function. (on Memory Screen)
         Moves the router down, using the same function as the button on the HOME screen
**************************/
        
        void bSetDownPushCallback(void *ptr) {
        
          bMoveDownPushCallback (&bSetDown);     //call the Callback function for the button from Home page
        }
          
/***********************************************
   void btPowerPopCallback(void *ptr)

         Button btPower component pop callback function.
         if the motor controller enablePin is off, sets it on, and vice versa.
**************************/
        
        void btPowerPopCallback(void *ptr) {
          uint32_t  butVal;
          int       temp;
      
          btPower.getValue(&butVal);
          temp = butVal;        
          if (temp == 0 )
            bGo = turnMotorOff (UP);
          else
            bGo = turnMotorOff( DOWN);
        }
        
/***********************************************
   void btSetOffPopCallback(void *ptr)

         Button btPower component push callback function.
         if the motor controller enablePin is off, sets it on, and vice versa.     
         
**************************/
        
        void btSetOffPopCallback(void *ptr) {
          uint32_t    butVal;
          int         temp;
          int         myVal;

         btSetOff.getValue(&butVal);
          temp = butVal;        
          if (temp == 0 )
          {
            bGo = turnMotorOff (UP);
          }
          else
          {
            bGo = turnMotorOff(DOWN);
          }    
        }
        
/***********************************************
   void btMemOffPopCallback(void *ptr)

         Button btPower component push callback function.
         if the motor controller enablePin is off, sets it on, and vice versa.
         
**************************/
        
        void btMemOffPopCallback(void *ptr) {
          uint32_t  butVal;
          int       temp;
        
          btMemOff.getValue(&butVal);
          temp = butVal;
        
          if (temp == 0)
          {
            bGo = turnMotorOff (UP);
          }
          else
          {
            bGo = turnMotorOff(DOWN);
          }
        }
        
/************************************************************
   void nSpdPopCallback(void *ptr)

         Number field nSpd<index> component pop callback function.
         When the field in the UI is clicked, get the field value and calculate the % of the MaxValue of the slider
         Set the value of the setSpeed field.  That value will be used later to control the speed at which the motor moves up and down
*************************************************************/
        void nSpdPopCallback(void *ptr) {
          uint32_t   scrVal;
          float     temp;
          int       spd;
        
          vaSpeed.getValue( &scrVal);

          temp = scrVal;
          if (temp == 0)
            spd = DOWN;
          else if (temp == 100)
            {
              spd = HIGH;
              temp = maxMotorSpeed;
            }
            
          else
            spd = UP;
          setSpeedFromSlider (temp, spd);
        
        }
        
/*******************************************************************
     void cbPreSetsPopCallBack(void *ptr(

           ComboBox cbPreSets component push callback function.
           A list of preset distances for the router to travel.   This will work with in conjuction with
           swHow and either move the selected amount of distance from a 0'ed spot, or set the bit to the selected
           amount from the surface of the table.   Obviously, we need to know where the surface of the table is

           Need to write serial commands directly for functions, as there are no libraries for this.   assemble the command, then FlushBuffer();
***************************************************************/
        
        void cbPreSetsPopCallBack(void *ptr) {
        
          char           buffer[15] = {'\0'};
        
          memset(buffer, '\0', sizeof(buffer));
          tHoldCombo.getText(buffer, sizeof(buffer));
          preSetTxt = String(buffer);
          memset(buffer, '\0', sizeof(buffer));
          tHolder.getText(buffer, sizeof(buffer));
          preSetNum = atoi(buffer);
          if (preSetNum > 0)
          {
            setPositionField(1);
          }
        }
        
/************************************************************
   void hMoveSpeedPopCallback(void *ptr)
        Slider hSlider component push callback function
        nSetSpeed is set through tht events in the Nextion IDE.  when the
        PopCallback is fired, we want to set the working motor speed, nothing else
**********************************************************/
        
        void hMoveSpeedPopCallback(void *ptr) {
          uint32_t scrVal;
          float perCalc;
          int slideVal;

          hMoveSpeed.getValue(&scrVal);
          slideVal = scrVal;    // This is the line the overwrites the memory buffer for workingMotorSpeed   conversion of a UINT32_t to a float, cannot be done this way
          setSpeedFromSlider(float(slideVal), DOWN);
        }
        
/***********************************************
    void  bSetPinsPopCallback (void *ptr)
        function to set the pin configuration off the settings screen in the app

  01/13/2021 -- need to modify this section to add the 6 pins fro the fence ball screw
**************************/
        
        void bSetPinsPopCallback (void *ptr) {
          SdBaseFile   fPins;
          char          fileName[20] = "LiftPinsConfig.cfg";
          char          pinsString[25] = {'\0'};
          char          motorString[25] = {'\0'};
          char          verString[25] = {'\0'};
          char          delim = char(222);
          char          section = char(174);
          char          verDelim = char(131);
          char          checker;
          char          verField[30] = {'\0'};
        
        
          fPins.open(fileName, O_RDWR);
          fPins.seekSet(0);
        
        
          checker = fPins.read();
          while (checker != delim && fPins.available())
            checker = fPins.read();
          // section, delim, UP, delim, down, delim, zero, delim, step, delim, SD, delim, enable, delim, direction, delim \n
          sprintf(pinsString, "%c%c%d%c%d%c%d%c%d%c%d%c%d%c%d%c\n", section, delim,
                  TOP_SWITCH, delim, BOTTOM_SWITCH, delim, REZERO, delim,
                  stepPin, delim, SD_WRITE, delim, enablePin, delim,
                  directionPin, delim );
          sprintf(motorString, "%c%c%ld%c%ld%c%d%c\n", section, delim,
                  maxMotorSpeed, delim, workingMotorSpeed, delim, stepSize, delim);
          memset (verField, '\0', sizeof(verField));
          tVersion.getText(verField, sizeof(verField));
          sprintf(verString, "%c%s%s", verDelim, verField, verDelim);
          fPins.write(pinsString);
          fPins.write(motorString);
          /*       checker=0;                 // I used this section when initially writing section to file
                 while (checker != -1)
                 {
                   checker = fPins.read();
            Serial.print(checker);
                 }
                 fPins.write(verString);*/
          fPins.sync();
          fPins.close();
        }
        
/***********************************************
    void  bSetMotorPopCallback (void *ptr)
        function to set the motor working speeds from  the settings screen in the app
        
**************************/
        
        void bSetMotorPopCallback (void *ptr) {
          char      motorMax[16] = {'\0'};
          char      workSpeed[16] = {'\0'};
          char      stepStore[8] = {'\0'};
          long      spdVal;
          long      maxMotorSpeed;
        
          memset (motorMax, '\0', sizeof(motorMax));
          memset (workSpeed, '\0', sizeof(workSpeed));
          memset (stepStore, '\0', sizeof(stepStore));
          tMaxSpeed.getText (motorMax, sizeof(motorMax));
          tWorkSpeed.getText (workSpeed, sizeof(workSpeed));
          tStepSize.getText (stepStore, sizeof(stepStore));
        
          maxMotorSpeed = atol(motorMax);
          hMoveSpeed.setMaxval(maxMotorSpeed);
          spdVal = atol(workSpeed);
          if (spdVal > maxMotorSpeed)
            spdVal = maxMotorSpeed;
          hMoveSpeed.setValue(spdVal);
          sRouter.setSpeed (spdVal);                     // set the initial router lift speed to the speed shown on the display
          sFence.setSpeed (spdVal);
          hMoveSpeedPopCallback(&hMoveSpeed);                       // if there is a value for the slider, set the variables in this program
          stepSize = atoi(stepStore);
        
        
        }
        
/***********************************************
    void  bSaveFilePopCallBack (void *ptr)
        function to set the motor working speeds from  the settings screen in the app
**************************/
        
        void bSaveFilePopCallback (void *ptr) {
        
          char  temp[300] = {'\0'};
          char  newFile[30] = {'\0'};
          SdBaseFile   fSave, fWrite;
          unsigned long   filePos;
          int   action;
          int   numChars = 1;
          char  oldFileName[30] = {'\0'};
          char  checker;
         
          memset (newFile, '\0', sizeof(newFile));
          tFileName.getText(newFile, sizeof(newFile));
          action = ErrorDialog (210);   
          strncpy (oldFileName, storeFile, sizeof(storeFile));
          strncpy(storeFile, newFile, sizeof(storeFile));
          temp[0] = '\0';
          if ( fSave.open (fileName) && fWrite.open (fileName, FILE_WRITE))
            {
              fSave.rewind();
              memset(temp, '\0', sizeof(temp));
              while (temp[0] != char(138)  )                        // read through the header of the file, find the file name section
              {
                filePos = fSave.curPosition();
                numChars = fSave.fgets(temp, sizeof(temp));
              }
              fSave.fgets(temp, sizeof(temp));                      // read the version configuration line
              fWrite.seekSet(filePos);
          
              sprintf(newFile, "%c%s%c\n", char(138), storeFile, char(138));
              fWrite.write(newFile);
              fWrite.write(temp);                                   // write version back into config file
              numChars = strlen(oldFileName) - strlen(storeFile);    // if the file name is shorter, clean out the remaining letters
              if ( numChars > 0)
              {
                for (int index = 1; index <= numChars; index++)
                {
                  fWrite.write('\0');
                }
              }
              fWrite.sync();
              fSave.close();
              fWrite.close();
            }
          else
            ErrorDialog (212 );  //212
          if (action == 5) 
            {
              //  reuse the file handles to open the old memory config file, read the comments from the top of the file
              //  write them into the new file as a header
              fSave.open(oldFileName);
              fWrite.open(storeFile, FILE_WRITE);
              checker = fSave.read();
              while (checker != '^' && fSave.available())
              {
                fWrite.write(checker);
                checker = fSave.read();
              }
              fWrite.write(checker);
              fWrite.write('\n');
              for (int index=1; index <= 7; index++)
                {
                  sprintf(temp, ",#,%d,,,,\n",index);
                  fWrite.write(temp);
                }
              fWrite.sync();
              fSave.close();
              fWrite.close();     
            }
          else if (action == 4)
            if (fSave.open (storeFile))
              fSave.close();
            else
              ErrorDialog (214);  
     }
        
/***********************************************
    void bSavePopCallback (void *ptr)
        when the button is pushed on the memory screen, the Nextion will set a screen variable to the button number
        all buttons will call the same PopCallback, and we will get the button number from the variable   
**************************/
        
        void bSavePopCallback (void *ptr) {

          int  index;
          uint32_t save;

          vaIndex.getValue(&save);
          index = save;
          saveToMemory(index);
        }
/***********************************************
    void bDeletePopCallback (void *ptr)
    when the button is pushed on the memory screen, the Nextion will set a screen variable to the button number
        all buttons will call the same PopCallback, and we will get the button number from the variable   
*********************************************/
        
        void bDeletePopCallback (void *ptr) {
          int  index;
          uint32_t save;

          vaDelIndex.getValue(&save);
          index = save;
          deleteFromMemory(index);
        }
/***********************************************
   void btoFilePopCallback (void *ptr)
      function to write values to the SD card.
      when the button is pressed, we set a variable in the Nextion.  That index will tell which button is pressed, 
      and we will construct the object names to pass to the memToFileSD function
**************************/
      
        void btoFilePopCallback (void *ptr) {
          int  index;
          uint32_t save;
          
          vaSDIndex.getValue(&save);
          index = save;

          switch (index)
            {
              case 1:
                memToFileSD ( index, tMem1, tZero1, tMem1Steps, ',', '#');
                break;
              case 2:
                memToFileSD ( index, tMem2, tZero2, tMem2Steps, ',', '#');
                break;
              case 3:
                memToFileSD ( index, tMem3, tZero3, tMem3Steps, ',', '#');
                break;
              case 4:
                memToFileSD ( index, tMem4, tZero4, tMem4Steps, ',', '#');
                break;
              case 5:
                memToFileSD ( index, tMem5, tZero5, tMem5Steps, ',', '#');
                break;
              case 6:
                memToFileSD ( index, tMem6, tZero6, tMem6Steps, ',', '#');
                break;
              case 7:
                memToFileSD ( index, tMem7, tZero7, tMem7Steps, ',', '#');
                break;
            }
        }

 /***********************************************
    void bLoadPopCallback (void *ptr)
        When the button is pressed, a variable is set in the NExtion.   This fuction will retrieve the variable
        and construct the object name to send to loadMemorytoRouter function
**************************/
              
        void bLoadPopCallback (void *ptr)  {
          int  index;
          uint32_t save;
          char  steps[17];

          vaIndex.getValue(&save);
          index = save;
          switch (index)
            {
              case 1:
                loadMemorytoRouter(tZero1, HIGH);
                break;
              case 2:
                loadMemorytoRouter( tZero2, HIGH);
                break;
              case 3:
                loadMemorytoRouter( tZero3, HIGH);
                break;
              case 4:
                loadMemorytoRouter( tZero4, HIGH);
                break;
              case 5:
                loadMemorytoRouter(tZero5, HIGH);
                break;
              case 6:
                loadMemorytoRouter( tZero6, HIGH);
                break;
              case 7:
                loadMemorytoRouter( tZero7,HIGH);
                break;
            }
          }

/*********************************************************
     void  bLoadMemPopCallback (void *ptr)
            Triggers the loading of data from a storage file to the memory screen fields
            will call a function called LoadFromFile()

            169 is the copyright symbol.  Is the token to separate the values in the stream
            216  is the section indicator to show the Memory screen
 *******************************************************/
        void  bLoadMemPopCallback (void *ptr)  {
        
          int  numWritten;
        
          //   numWritten = LoadFromFile(1, 7, 169, 216);
          numWritten = LoadFromFile(1, 7, ',', '#');
        }
/**************************************************
      void bSDCardPopCallback (void *ptr)
         function to restart the SD card controller.   Reissuing the SDCard Begin, resets the card reader after removing SDCard
 ***************************************************/

        void bSDCardPopCallback( void *ptr) {
        
          if (!sdCard.begin(SD_WRITE))
          {
            ErrorDialog (101 );
          }
        
        }

/************************************************
 * void mCustomBitPopCallback (void *ptr)
 *         function to set the heights for the custom bit configurations
 ************************************************/
        void mCustomBitPopCallback (void *ptr)   {

            int  index;
            uint32_t  save;
            char   sCommand[120];
            char   inches[9];

            nBit.getValue(&save);
            index = save;
            stepsFromDistance (LOW, index);

            sprintf( sCommand, "t2.txt=\"%s\"",preSetLookup.label);
            Serial3.write(sCommand);
            FlushBuffer(); 
            dtostrf(preSetLookup.decimal, 3, 4, inches);       
            t3.setText (inches);
            FlushBuffer();
        }
        
/******************************************************************************
   SETUP the main pieces of the program
        
        void setup(void) {
        
          /* You might need to change NexConfig.h file in your ITEADLIB_Arduino_Nextion folder
            Set the baudrate which is for debug and communicate with Nextion screen
            nexInit() = sets the Nextion to Seriel3, and sets baud rate to 9600 unless a value is passed to nexInit()
        
                 01/23/2020  CDW ==>    Modified NexConfig.h, and set the Serial3 to Seriel3.   When implemneting, don't know the exact wriring
                                        config I will use, but in the nexInit prototype , the value for Serial3 needs to be set to the appropriate
                                        port AND!!!!!! the ground from the connector to the nextion must be connected to the ground from the board.  If not,
                                        the listener does not hook up, the Nextion listener  does not function properly, and there is no event queue in
                                        the arduino.
*******************************************************************************/
       void setup (void) {
         
          float perCalc;
          float slideVal;
          uint32_t  scrVal;
          SdBaseFile   fPins;
          int   numChars;
        
          char          pinsString[40] = {'\0'};
          char          motorString[40] = {'\0'};
          char          memFile[30] = {'\0'};
          char          delim[2] = {char(222), '\0'};
          char          delimCheck = char(222);
          char          *token;
          int           index;
          char          checker;
        
          nexInit(115200);     //  with enahanced libraries, can pass a baud rate to the Nextion (115200 set on preinitialize of Home)
          //nexBAUD(250000);

          Serial.begin(250000);

        
        
          // Register the pop event callback function of the components
        
          btPower.attachPop(btPowerPopCallback, &btPower);                    //  turn the motor controller on and off by setting the enablePin
          btSetOff.attachPop(btSetOffPopCallback, &btSetOff);                  //  turn the motor controller on and off by setting the enablePin
          btMemOff.attachPop(btMemOffPopCallback, &btMemOff);                  //  turn the motor controller on and off by setting the enablePin     
          bCalib.attachPop(bCalibPopCallback, &bCalib);                       //  Button to move lift to top limit, set top, bottom limit set bottom       
 /*************  home screen 4 buttons Push*/         
          bMoveUp.attachPush(bMoveUpPushCallback, &bMoveUp);                  //  Move Up push - Home Screen
          bForward.attachPush(bForwardPushCallback, &bForward);               //  Move Fence to bit, push - home screen
          bBack.attachPush(bBackPushCallback,&bBack);                         //  Move Fence away from bit, push -- home screen
          bMoveDown.attachPush(bMoveDownPushCallback, &bMoveDown);            //  Move Down press - Home Screen

          /*********** home screen 4 buttons POP ****/
          bForward.attachPop(bForwardPopCallback, &bForward);                 //  Move fence to bit release - home screen
          bBack.attachPop(bBackPopCallback, &bBack);                          //  Move Fence away from bit, release -- home screen
          bMoveDown.attachPop(bMoveDownPopCallback, &bMoveDown);              //  Move Down release - Home Screen
          bMoveUp.attachPop(bMoveUpPopCallback, &bMoveUp);                    //  Move Up release - Home Screen      
 /*********** Settings screen 4 buttons PUSH****/         
          bUp.attachPush(bMoveUpPushCallback, &bUp);                          //  Move Up push - Settings Screen
          bDown.attachPush(bMoveDownPushCallback, &bDown);                    //  Move Down press - Settings Screen         
          bSetBack.attachPush(bBackPushCallback, &bSetBack);                    //  Move Back push - Settings Screen         
          bSetForward.attachPush(bForwardPushCallback, &bSetForward);          //  Move Forward press  (Settings)     
 /************ Settings Screen 4 buttons POP ******/        
          bSetForward.attachPop(bForwardPopCallback, &bSetForward);           //  Move Forward release - Settings Screen
          bSetBack.attachPop(bBackPopCallback, &bSetBack);                   //  Move Back release - Settings Screen
          bDown.attachPop (bMoveDownPopCallback, &bDown);
          bUp.attachPop (bMoveUpPopCallback, &bUp);
 /************Memory Screen   4 buttons Push ******/         
          bMemForward.attachPush(bForwardPushCallback, &bMemForward);         //  Move Forward press  (Memory Memory)                        
          bSetUp.attachPush(bMoveUpPushCallback, &bSetUp);                    //  Move Up push  (Memory Screen)
          bMemBack.attachPush(bBackPushCallback, &bMemBack);                  //  Move Up push  (Memory Screen)          
          bSetDown.attachPush(bMoveDownPushCallback, &bSetDown);              //  Move Down press  (Memory Screen)
/**********Memory Screen 4 buttons POP ******/
          bMemForward.attachPop(bForwardPopCallback, &bMemForward);         //  Move Forward release  (Memory Memory)                        
          bSetUp.attachPop(bMoveUpPopCallback, &bSetUp);                    //  Move Up release  (Memory Screen)
          bMemBack.attachPop(bBackPopCallback, &bMemBack);                  //  Move Up release  (Memory Screen)          
          bSetDown.attachPop(bMoveDownPopCallback, &bSetDown);              //  Move Down release  (Memory Screen)                                

          swHow.attachPop(swHowPopCallback, &swHow);                          //  handling the auto move   1 = MOVE selected amount, 0 = move to that absolute position (up only)
          bReZero.attachPop(bReZeroPopCallback, &bReZero);                    //  ReZero release
          bZero.attachPop(bZeroPopCallback, &bZero);                          //  Zero button on the Settings Page
          bChangeBit.attachPop(bChangeBitPopCallback, &bChangeBit);           //  Change bit on the home page
          bToBottom.attachPop(bToBottomPopCallback, &bToBottom);              //  Bottom out router - check where the bottom limit switch is
          bAutoMove.attachPop(bAutoMovePopCallback, &bAutoMove);              //  "GO!" button to move the distance off the preset drop down
          bgotoZero.attachPop(bgotoZeroPopCallback, &bgotoZero);              //  moves to the 0 setting, up if below, and down if above
          bBottomOut.attachPop(bToBottomPopCallback, &bBottomOut);           //  Bottom out router (Settings) - check where the bottom limit switch is
          bLiftBit.attachPop(bChangeBitPopCallback, &bLiftBit);                 //  Change bit on the Settings page
          nSpd0.attachPop(nSpdPopCallback, &nSpd0);                           //   0 on the scale next to the slider.   will set motor to 0% of MAXSPEED
          nSpd2.attachPop(nSpdPopCallback, &nSpd2);                           //  20 on the scale next to the slider.   will set motor to 20% of MAXSPEED
          nSpd4.attachPop(nSpdPopCallback, &nSpd4);                           //  40 on the scale next to the slider.   will set motor to 40% of MAXSPEED
          nSpd6.attachPop(nSpdPopCallback, &nSpd6);                           //  60 on the scale next to the slider.   will set motor to 60% of MAXSPEED
          nSpd8.attachPop(nSpdPopCallback, &nSpd8);                           //  80 on the scale next to the slider.   will set motor to 80% of MAXSPEED
          nSpd10.attachPop(nSpdPopCallback, &nSpd10);                         //  100 on the scale next to the slider.   will set motor to 100% of MAXSPEED
          bSetPins.attachPop(bSetPinsPopCallback, &bSetPins);                 //  set the new pins for the connections for overall digital configuration
          bSetMotor.attachPop(bSetMotorPopCallback, &bSetMotor);              //  Set the speeds and step variables for the motor
          bSaveFile.attachPop(bSaveFilePopCallback, &bSaveFile);              //  resets the momory save file name, and writes new file name to pinsconfig.cfg
          bMemZero.attachPop(bgotoZeroPopCallback, &bMemZero);                //  calls the home screen gotoZero event handler.  drops the router to the 0 position
          swFence.attachPop(swFencePopCallback, &swFence);                    //  Home screen switch for which motor to handle zero and resetting
          swWhich.attachPop(swWhichPopCallback, &swWhich);                    //  Settings screen switch for which motor is used to handle calibration
          cbPreSets.attachPop(cbPreSetsPopCallBack, &cbPreSets);
        
          hMoveSpeed.attachPop(hMoveSpeedPopCallback, &hMoveSpeed);
        
          /****************************************************
               setting up the memory screen.   not going to document every call back, as they all do the sam
           ***********************************/
        
          bLoad1.attachPop ( bLoadPopCallback, &bLoad1);
          bLoad2.attachPop ( bLoadPopCallback, &bLoad2);
          bLoad3.attachPop ( bLoadPopCallback, &bLoad3);
          bLoad4.attachPop ( bLoadPopCallback, &bLoad4);
          bLoad5.attachPop ( bLoadPopCallback, &bLoad5);
          bLoad6.attachPop ( bLoadPopCallback, &bLoad6);
          bLoad7.attachPop ( bLoadPopCallback, &bLoad7);
        
          bSave1.attachPop ( bSavePopCallback, &bSave1);
          bSave2.attachPop ( bSavePopCallback, &bSave2);
          bSave3.attachPop ( bSavePopCallback, &bSave3);
          bSave4.attachPop ( bSavePopCallback, &bSave4);
          bSave5.attachPop ( bSavePopCallback, &bSave5);
          bSave6.attachPop ( bSavePopCallback, &bSave6);
          bSave7.attachPop ( bSavePopCallback, &bSave7);
        
          btoFile1.attachPop ( btoFilePopCallback, &btoFile1);
          btoFile2.attachPop ( btoFilePopCallback, &btoFile2);
          btoFile3.attachPop ( btoFilePopCallback, &btoFile3);
          btoFile4.attachPop ( btoFilePopCallback, &btoFile4);
          btoFile5.attachPop ( btoFilePopCallback, &btoFile5);
          btoFile6.attachPop ( btoFilePopCallback, &btoFile6);
          btoFile7.attachPop ( btoFilePopCallback, &btoFile7);
        
          bDelete1.attachPop (bDeletePopCallback, &bDelete1);
          bDelete2.attachPop (bDeletePopCallback, &bDelete2);
          bDelete3.attachPop (bDeletePopCallback, &bDelete3);
          bDelete4.attachPop (bDeletePopCallback, &bDelete4);
          bDelete5.attachPop (bDeletePopCallback, &bDelete5);
          bDelete6.attachPop (bDeletePopCallback, &bDelete6);
          bDelete7.attachPop (bDeletePopCallback, &bDelete7);
          bLoadMem.attachPop (bLoadMemPopCallback, &bLoadMem);
          bSDCard.attachPop (bSDCardPopCallback, &bSDCard);
          m0.attachPop (mCustomBitPopCallback, &m0);
          m2.attachPop (mCustomBitPopCallback, &m2);
          m3.attachPop (mCustomBitPopCallback, &m3);
          m4.attachPop (mCustomBitPopCallback, &m4);
          m5.attachPop (mCustomBitPopCallback, &m5);
          bLoad.attachPop (bCustBitPopCallback, &bLoad);
          bBitsZero.attachPop (bgotoZeroPopCallback, &bBitsZero);
        
          pinMode(SD_WRITE, OUTPUT);
        
          if (!sdCard.begin(SD_WRITE))
            ErrorDialog (101);  
          else
          {
            if (fPins.open(fileName))
            {
              memset (pinsString, '\0', sizeof(pinsString));
              memset (motorString, '\0', sizeof(motorString));
              checker = fPins.read();
              while (checker != delimCheck)
              {
                checker = fPins.read();
              }
              checker = fPins.read();
              index = 0;
              while (checker != '\n' && fPins.available())
              {
                pinsString[index] = checker;
                index++;
                checker = fPins.read();
              }

              index = 0;
              checker = fPins.read();
              while (checker != '\n' && fPins.available())
              {
                motorString[index] = checker;
                index++;
                checker = fPins.read();
              }
              if (fPins.available())
                numChars = fPins.fgets(memFile, sizeof(memFile));       // read the filename to be used for storing custom router depths
              int roll = 1;
              memset(storeFile, '\0', sizeof(storeFile));
              while (roll < numChars - 2)
              {
                storeFile[roll - 1] = memFile[roll];
                roll++;
              }
        
              token = strtok(pinsString, delim);
        
              for (index = 0; index < 13; index++)
              {
                token = strtok(NULL, delim);
                switch (index)
                {
                  case 0:
                    TOP_SWITCH = atoi(token);
                    break;
                  case 1:
                    BOTTOM_SWITCH = atoi(token);
                    break;
                  case 2:
                    REZERO = atoi(token);
                    break;
                  case 3:
                    stepPin = atoi(token);
                    break;
                  case 4:
                    break;
                  case 5:
                    enablePin = atoi(token);
                    break;
                  case 6:
                    directionPin = atoi(token);
                    break;
                  case 7:
                    fenceStepPin = atoi(token);
                    break;
                  case 8:
                    fenceEnablePin = atoi(token);
                    break;
                  case 9:
                    fenceDirPin = atoi(token);
                    break;
                  case 10:
                    BACK_SWITCH = atoi(token);
                    break;
                  case 11:
                    FRONT_SWITCH = atoi(token);
                    break;
                  case 12:
                    FENCE_ZERO = atoi(token);
                    break;
                }
              }
              token = strtok(motorString, delim);
              for (index = 0; index < 3; index++)
              {
                token = strtok(NULL, delim);

                switch (index)
                {
                  case 0:
                    maxMotorSpeed = atol(token);
                    break;
                  case 1:
                    workingMotorSpeed = atol(token);
                    break;
                  case 2:
                    stepSize = atoi(token);
                    break;
                }
              }
              memset( pinsString, '\0', sizeof(pinsString));
              index=0;
              while (fPins.read() != 131);
              while (fPins.peek() != 131)
                {
                  pinsString[index] = fPins.read();
                  index++;
                }
              fPins.close();
            }
            else
              ErrorDialog ( 105 ); 

            pinMode(TOP_SWITCH, INPUT_PULLUP);          // Pin where the stop switch is wired PULLUP will define https://www.arduino.cc/en/Tutorial/InputPullupSerial
            pinMode(BOTTOM_SWITCH, INPUT_PULLUP);
            pinMode(REZERO, INPUT_PULLUP);
            pinMode(enablePin, OUTPUT);
            pinMode(stepPin, OUTPUT);
            pinMode(directionPin, OUTPUT);
            pinMode(fenceDirPin, OUTPUT);
            pinMode(fenceEnablePin, OUTPUT);
            pinMode(fenceStepPin, OUTPUT);
            pinMode(BACK_SWITCH, INPUT_PULLUP);
            pinMode(FRONT_SWITCH, INPUT_PULLUP);
            pinMode(FENCE_ZERO, INPUT_PULLUP);
            
          }
          //   set max speed and accelleration of the stepper motor
        
          sRouter.setMaxSpeed(maxMotorSpeed);
          sRouter.setAcceleration(maxAcceleration);
          sRouter.setMinPulseWidth(pulseWidthMicros);
          sFence.setMaxSpeed(maxMotorSpeed);
          sFence.setAcceleration(maxAcceleration);
          sFence.setMinPulseWidth(pulseWidthMicros);
 
          bGo = turnMotorOff( DOWN);                           // turns Router motor controller on, gives the torque necessary to hold the router in place
          getSettingsScreen(pinsString);
          scrVal = maxMotorSpeed;
          hMoveSpeed.setMaxval(scrVal);
          sRouter.setSpeed (workingMotorSpeed);                     // set the initial router lift speed to the speed shown on the display
          sFence.setSpeed (workingMotorSpeed);
          cbPreSetsPopCallBack(&cbPreSets);                         // if there is a value in the presets, we need to initialize the preSets variables
          hMoveSpeedPopCallback(&hMoveSpeed);                       // if there is a value for the slider, set the variables in this program    
          

          setPositionField(1);
          setPositionField(0);
          Home.show();
        }
/**********************************************        
 *     void  loop (void)
 *          main loop of the program
 ***********************************************/
        void loop(void) {
          /*
             When a pop or push event occured every time,
             the corresponding component[right page id and component id] in touch event list will be asked.
          */ 
          nexLoop(nex_listen_list);
        }
