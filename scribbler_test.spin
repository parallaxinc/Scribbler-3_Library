''*********************************************
''*  Scribbler Test Utility                   *
''*  Author: Ben Wirz, Element Products, Inc. *
''*  Copyright (c) 2010 Parallax, Inc.        *  
''*  See end of file for terms of use.        *
''********************************************* 
''

CON

  _clkmode      = xtal1 + pll16x
  _xinfreq      = 5_000_000

   VERSION       = 3                                'Release Version Number
   SUBVERSION    = 2574                             'Internal Version Number
              
OBJ

  sio       : "FullDuplexSerial" 
  scribbler : "scribbler"
  music     : "scribbler_music"
  pwm       : "PWMx8"  

VAR
  long last_call
  byte line_threshold,obs_threshold
   
PUB start(default_version,default_subversion,line_tst,obs_tst) | index
  ''Initialize the test utility
  ''Print the firmware version numbers.
  ''default_version - default program version number
  ''default_subversion - default program subversion number
  ''line_test - the trip value for the line sensor 
  ''obs_test - the obstacle sensor threshold
  
  'Save the line following and obstacle sensitivity
  line_threshold := (line_tst #> 0) <# $FF
  obs_threshold := (obs_tst #> 0) <# 100
  
  'Start hardware driver cogs and low level routlines  
  scribbler.start 
  scribbler.start_motors
  scribbler.start_tones
  
  'Start the Serial Communication Object
  sio.start(scribbler#RX, scribbler#TX, 0, 19200)
  sio.tx(CLS)
  sio.rxflush

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#BLUE) 
  scribbler.beep
  sio.tx(cls) 
  if scribbler.get_model_s2
    print_title(TEST_COL_START,TEST_COL_END,-1,string("Scribbler S2 Robot"))
  else
   print_title(TEST_COL_START,TEST_COL_END,-1,string("Scribbler S3 Robot"))
    
  print_str(TEST_COL_START,TEST_ROW_0,string("Press the Button for the Production Test"))   
  print_str(TEST_COL_START,TEST_ROW_1,string("Press the Space Bar for the Test Utility Menu"))
  print_str(TEST_COL_START,TEST_ROW_3,string("File Versions:"))
  print_str(constant(TEST_COL_START+1),TEST_ROW_4,string("scribbler"))
  cursor_col(TEST_COL_VER)
  sio.dec(scribbler#VERSION)
  sio.tx("(")   
  sio.dec(scribbler#SUBVERSION)
  sio.tx(")") 
  print_str(constant(TEST_COL_START+1),TEST_ROW_5,string("scribbler_default"))
  cursor_col(TEST_COL_VER)
  if default_version <> 0
    sio.dec(default_version)
  else
    sio.tx("?")
  sio.tx("(")
  if default_subversion <> 0
    sio.dec(default_subversion)
  else
    sio.tx("?")  
  sio.tx(")") 
  print_str(constant(TEST_COL_START+1),TEST_ROW_6,string("scribbler_test"))
  cursor_col(TEST_COL_VER)
  sio.dec(VERSION)
  sio.tx("(")
  sio.dec(SUBVERSION)
  sio.tx(")")  
  print_str(constant(TEST_COL_START+1),TEST_ROW_7,string("scribbler_music"))
  cursor_col(TEST_COL_VER)
  sio.dec(music#VERSION)
  sio.tx("(")
  sio.dec(music#SUBVERSION)
  sio.tx(")")     

  repeat until button_wait(1) == 1

  production_test
   
CON
  'Production Screen Layout

  TOP_MARGIN                = 4
  LEFT_MARGIN               = 3

  TEST_COL_START            = 1
  TEST_COL_END              = 52
  TEST_COL_MID              = (TEST_COL_END-TEST_COL_START)/2
  TEST_COL_VER              = TEST_COL_START + 20
  TEST_COL_STEP_NUM         = TEST_COL_START + 4
  TEST_COL_STEP_NAME        = TEST_COL_STEP_NUM + 5
  TEST_COL_PASS             = TEST_COL_STEP_NAME + 28
                 
  TEST_ROW_0                = TOP_MARGIN
  TEST_ROW_1                = TOP_MARGIN + 1
  TEST_ROW_2                = TOP_MARGIN + 2
  TEST_ROW_3                = TOP_MARGIN + 3
  TEST_ROW_4                = TOP_MARGIN + 4
  TEST_ROW_5                = TOP_MARGIN + 5
  TEST_ROW_6                = TOP_MARGIN + 6
  TEST_ROW_7                = TOP_MARGIN + 7
  TEST_ROW_8                = TOP_MARGIN + 8
          
  TEST_ROW_RESULTS          = TEST_ROW_2
  TEST_ROW_BUTTON           = TEST_ROW_RESULTS + TEST_LED + 2

  TEST_ROW_NUMBER           = TOP_MARGIN - 1
  TEST_COL_NUMBER           = TEST_COL_END - 8
                            
  TEST_PASS_PAUSE           = 20                                'Message display pause (10th secs)
  UNTESTED                  = 1

  'Production Test Steps
  #0,TEST_ELECTRICAL,TEST_LIGHT,TEST_IDLER,TEST_MIC,TEST_LINE,TEST_TRACK,TEST_SPEED,TEST_IR,TEST_SPEAKER,TEST_LED

  MOTOR_SAMPLE_LEN          = 1
  #0,MOT_LEFT,MOT_RIGHT

PUB production_test | current_test,step_pass[TEST_LED+1],index
  ''Perform a series of tests to verify the scribbler's operation

  current_test := TEST_ELECTRICAL
  
  longfill (@step_pass,UNTESTED,TEST_LED+1)
  repeat
    sio.tx(cls)
    print_str(TEST_COL_NUMBER,TEST_ROW_NUMBER,string("Step: "))
    sio.dec(current_test + 1)
    case current_test
      TEST_ELECTRICAL:
        step_pass[TEST_ELECTRICAL] := electrical_test         
      TEST_LIGHT:
        step_pass[TEST_LIGHT] := light_test
      TEST_IDLER:
        step_pass[TEST_IDLER] := idler_test(8)
      TEST_MIC:
        step_pass[TEST_MIC] := mic_test
      TEST_LINE:
        step_pass[TEST_LINE] := line_test
      TEST_TRACK:
        step_pass[TEST_TRACK] := track_test
      TEST_SPEED:
        step_pass[TEST_SPEED] := speed_test
      TEST_IR:
        step_pass[TEST_IR] := ir_test
      TEST_SPEAKER:
        step_pass[TEST_SPEAKER] := speaker_test
      TEST_LED:
        step_pass[TEST_LED] := led_test

    print_str(TEST_COL_NUMBER,TEST_ROW_NUMBER,string("Step: "))
    sio.dec(current_test + 1)          

    if step_pass[current_test]
      scribbler.set_leds(scribbler#GREEN,scribbler#GREEN,scribbler#GREEN,scribbler#OFF)
    else 
      scribbler.set_leds(scribbler#RED,scribbler#RED,scribbler#RED,scribbler#OFF)

    repeat index from TEST_ELECTRICAL to TEST_LED
      'Mark the current test step with an arrow
      cursor(constant(TEST_COL_START+1),TEST_ROW_RESULTS + index)
      if index == current_test
        sio.tx("-")
        sio.tx(">")
        
      'Print the test step number & name 
      cursor_col(TEST_COL_STEP_NUM)
      sio.tx("(")
      sio.dec(index+1)
      sio.tx(")")
      cursor_col(TEST_COL_STEP_NAME) 
      case index
        TEST_ELECTRICAL: sio.str(string("Electrical Test"))
        TEST_LIGHT: sio.str(string("Light Sensor Test"))        
        TEST_IDLER: sio.str(string("Idler Wheel Test"))
        TEST_MIC: sio.str(string("Microphone Test"))
        TEST_LINE: sio.str(string("Line Sensor Test"))
        TEST_TRACK: sio.str(string("Track Test"))
        TEST_SPEED: sio.str(string("Motor Speed Test"))             
        TEST_IR: sio.str(string("IR Obstacle Sensor Test"))                  
        TEST_SPEAKER: sio.str(string("Speaker Test"))
        TEST_LED: sio.str(string("Visible LED Test"))
      'Print the status of each step
      cursor_col(TEST_COL_PASS)
      sio.str(string(" - "))
      if step_pass[index] == UNTESTED
        sio.str(string("Untested"))
      elseif step_pass[index] == FALSE
        sio.str(string("FAIL"))
      else
        sio.str(string("Pass"))
          
    print_str(TEST_COL_START,TEST_ROW_BUTTON,string("Button Press:"))   
    if current_test < TEST_LED
      print_str(constant(TEST_COL_START+1),constant(TEST_ROW_BUTTON+1),string("Press Once - Next Step"))
    print_str(constant(TEST_COL_START+1),constant(TEST_ROW_BUTTON+2),string("Press Twice - Repeat Current Step"))
    if current_test > TEST_ELECTRICAL
      print_str(constant(TEST_COL_START+1),constant(TEST_ROW_BUTTON+3),string("Press Three Times - Previous Step"))
    case index := button_wait(3)
      1:                                                  'Move to the next test                                                           
        current_test := (current_test + 1) <# TEST_LED
        scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#GREEN,scribbler#OFF)
      2:                                                  'Repeat the current test
        scribbler.set_leds(scribbler#OFF,scribbler#GREEN,scribbler#OFF,scribbler#OFF)
      3:                                                  'Go back to the previous test
        current_test := (current_test - 1) #> TEST_ELECTRICAL
        scribbler.set_leds(scribbler#GREEN,scribbler#OFF,scribbler#OFF,scribbler#OFF)
      "1".."9":                                           'Number key press
        current_test := index - "1"
      "0":
        current_test := 9   
CON
  'Power Screen Layout
  PWR_COL_VLT               = LEFT_MARGIN + 9
  PWR_COL_FAIL              = PWR_COL_VLT + 12 
  PWR_ROW                   = TOP_MARGIN
  PWR_ROW_END               = PWR_ROW + 8
  ' Test Voltage Limits
  PWR_BATT_MIN_S2           = (600*255)/330/4 '1.5V V (S2)
  PWR_BATT_MIN_S3           = (300*255)/330/2 '1.5V V (S3)  
  PWR_3V3_MIN               = (310*255)/330   '3.10 V
  PWR_3V3_MAX               = (345*255)/330   '3.45 V
  PWR_5V0_MIN_DIV           = (475*255)/330   '4.75 V - divided 5V bus with 3.3V reference
  PWR_5V0_MAX_DIV           = (525*255)/330   '5.25 V
  PWR_5V0_MIN               = (475*255)/500   '4.75 V - non divided 5V bus with 5.0V reference
  PWR_5V0_MAX               = (525*255)/500   '5.25 V
  PWR_VMOT_MIN              = (850*255)/330   '8.50 V (S3)                          
  PWR_VMOT_MAX              = (950*255)/330   '9.50 V (S3)
  PWR_TRIP_MIN              = (18*255)/330    ' 18 mV (S2)                      
  PWR_TRIP_MAX              = (26*255)/330    ' 26 mV (S2)
  PWR_I_MAX                 = 150             '150 mA (S2)
  
PUB electrical_test | calc, cog_index, cog_list[8], pass
  ''Check Bus Voltages

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)   
  pass := TRUE
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Electrical Test"))

  'Start all unused cogs to load the 3V3 Bus  
  repeat cog_index from 0 to 7    
    cog_list[cog_index] := cognew(@busy, 0)
    if cog_list[cog_index] == -1
      quit

  print_str(LEFT_MARGIN,constant(PWR_ROW+1),string("Battery"))
  if scribbler.get_model_s2
    calc := scribbler.get_adc_results(scribbler#ADC_VBAT)
    print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+1), calc * 4)
    if calc < PWR_BATT_MIN_S2
      clear_row(constant(PWR_ROW+1))
      print_str(LEFT_MARGIN,constant(PWR_ROW+1),string("FAIL - Replace Batteries"))
      repeat
  else
    calc := scribbler.get_adc_results(scribbler#ADC_VBAT)
    print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+1), calc * 2)
    if calc < PWR_BATT_MIN_S3
      clear_row(constant(PWR_ROW+1))
      print_str(LEFT_MARGIN,constant(PWR_ROW+1),string("FAIL - Recharge Battery"))
    
      
  calc := scribbler.get_adc_results(scribbler#ADC_VDD) 
  print_str(LEFT_MARGIN,constant(PWR_ROW+2),string("3V3 Bus"))
  print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+2),calc)
  ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+2),calc,PWR_3V3_MIN,PWR_3V3_MAX)
    pass := FALSE

  if (scribbler.get_model_s2)    ' only the S2 has the 5V bus current monitor 
    calc := ((scribbler.get_adc_results(scribbler#ADC_5V) - scribbler.get_adc_results(scribbler#ADC_IDD)) * 50000) / 11985
    print_str(constant(PWR_COL_VLT+6),constant(PWR_ROW+3),string("mA")) 
    print_dec_word(constant(PWR_COL_VLT-1),constant(PWR_ROW+3),calc)
    ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+3),calc,20,PWR_I_MAX)
      pass := FALSE
    
    if calc > PWR_I_MAX
      print_str(PWR_COL_FAIL,constant(PWR_ROW+3),string("FAIL - Over Current"))
      pass := FALSE  

  ' Resistor divided 5V bus, 3.3V reference  
  calc := scribbler.get_adc_results(scribbler#ADC_5V_DIV) * 2       
  print_str(LEFT_MARGIN,constant(PWR_ROW+4),string("5V0 Div"))
  print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+4),calc)
  ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+4),calc,PWR_5V0_MIN_DIV,PWR_5V0_MAX_DIV)
    pass := FALSE

  '5V bus with no resistor divider, 5.0V reference
  calc := scribbler.get_adc_results(scribbler#ADC_5V) 
  print_str(LEFT_MARGIN,constant(PWR_ROW+5),string("5V0 Bus"))
  print_volts_5v0(PWR_COL_VLT,constant(PWR_ROW+5),calc)
  ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+5),calc,PWR_5V0_MIN,PWR_5V0_MAX)
    pass := FALSE                                                

    
  if (scribbler.get_model_s2)                        'S2 - Over Current Reference Voltage     
    calc := scribbler.get_adc_results(scribbler#ADC_VTRIP)
    print_str(LEFT_MARGIN,constant(PWR_ROW+6),string("Mot Trip")) 
    print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+6),calc) 
    ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+6),calc,PWR_TRIP_MIN,PWR_TRIP_MAX)
      pass := FALSE
  else                                                  'S3 - Motor Power Supply Voltage
    calc := (scribbler.get_adc_results(scribbler#ADC_VMOT) * (100+33)) / 33 
    print_str(LEFT_MARGIN,constant(PWR_ROW+6),string("Motor")) 
    print_volts_3v3(PWR_COL_VLT,constant(PWR_ROW+6),calc) 
    ifnot limit_test(PWR_COL_FAIL,constant(PWR_ROW+6),calc,PWR_VMOT_MIN,PWR_VMOT_MAX)
      pass := FALSE
     
  'Stop extra cogs
  repeat while cog_index > 0
    cog_index -= 1
    cogstop(cog_list[cog_index])

  if (scribbler.get_model_s2)  
    scribbler.stop_all
    'Read the undriven hacker port to check the pull-up resistors 
    dira[scribbler#P5..scribbler#P0]~           
    scribbler.delay_tenths(2)
    if INA[scribbler#P5..scribbler#P0] <> %111111
      print_str(LEFT_MARGIN,constant(PWR_ROW+7),string("Fail - Hacker Port Pull-Up: "))
      sio.bin(INA[scribbler#P5..scribbler#P0],6)    '
      pass := FALSE
    else                          'Drive the hacker port low and check the port 
      dira[scribbler#P5..scribbler#P0]~~  
      outa[scribbler#P5..scribbler#P0]~
      scribbler.delay_tenths(1)  
    if INA[scribbler#P5..scribbler#P0] <> %000000
      print_str(LEFT_MARGIN,constant(PWR_ROW+7),string("Fail - Hacker Port Stuck High: "))
      sio.bin(INA[scribbler#P5..scribbler#P0],6)    '
      pass := FALSE
    else
      print_str(LEFT_MARGIN,constant(PWR_ROW+7),string("Hacker Port Pass"))    
    scribbler.start 
    scribbler.start_motors
    scribbler.start_tones 

  'Print the test results
  scribbler.delay_tenths(TEST_PASS_PAUSE)  
  sio.tx(cls) 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Electrical Test"))
  print_str(TEST_COL_START,TEST_ROW_0,string("Electrical Test "))  
  if pass
    sio.str(string("Pass"))
  else
    sio.str(string("Fail"))
  return pass

PUB limit_test(col,row,vlt,vlt_min,vlt_max)
  ''Check a voltage and print Pass or Fail

  if vlt < vlt_min
    print_str(col,row,string("FAIL - Under"))
    return FALSE
  elseif vlt > vlt_max
    print_str(col,row,string("FAIL - Over"))
    return FALSE   
  else
    print_str(col,row,string("Pass"))
    return TRUE


CON
  LIGHT_LOOP_FREQ             = 10                    'Loop frequency (Hz)
  LIGHT_TEST_TIME             = 20 * LIGHT_LOOP_FREQ  'Test Time for each step
  LIGHT_ROW_TIME              = 7
  LIGHT_ROW_END               = LIGHT_ROW_TIME
  LIGHT_SHINE_TRIP            = 200                   'Min Flaslight Trip Level
  
  #0,COVER_LEFT,COVER_CENTER,COVER_RIGHT,SHINE_LEFT,SHINE_CENTER,SHINE_RIGHT
  
PUB light_test | sample_left,sample_center,sample_right,cover_trip_left,cover_trip_center,cover_trip_right,state,time
  ''Test the light sensor
  ''
  ''Display light sensor levels on the corresponding left, center and right LED's
  ''If a light sensors hole is covered - set the LED to off.  If the hole is uncovered,
  ''the LED will be green or yellow. 

  time := LIGHT_TEST_TIME    
  'Sample and scale light sensor levels with the fingers off
  cover_trip_left := (scribbler.light_sensor(scribbler#LEFT) * 8) / 10                  
  cover_trip_center := (scribbler.light_sensor(scribbler#CENTER) * 8) / 10   
  cover_trip_right := (scribbler.light_sensor(scribbler#RIGHT) * 8) / 10  

  print_title(TEST_COL_START,TEST_COL_END,-1,string("Light Sensor Test")) 
  music.play_song(music#LIGHT,2000)
  print_str(TEST_COL_START,TEST_ROW_0,string("Cover the Left Light Sensor"))
  print_str(TEST_COL_START,LIGHT_ROW_TIME,string("Time:  ")) 
  state := COVER_LEFT
    
  periodic_start
  repeat
    print_dec(constant(TEST_COL_START + 7),LIGHT_ROW_TIME,time / LIGHT_LOOP_FREQ) 
    sio.str(string(" secs "))
    
    sample_left := scribbler.light_sensor(scribbler#LEFT)
    sample_center := scribbler.light_sensor(scribbler#CENTER)
    sample_right := scribbler.light_sensor(scribbler#RIGHT)         
    scribbler.set_led(scribbler#LEFT,lookupz(scribbler.light_sensor(scribbler#LEFT) >> 6 :scribbler#OFF,scribbler#YELLOW,scribbler#DIM_GREEN,scribbler#GREEN))
    scribbler.set_led(scribbler#CENTER,lookupz(scribbler.light_sensor(scribbler#CENTER) >> 6 :scribbler#OFF,scribbler#YELLOW,scribbler#DIM_GREEN,scribbler#GREEN))
    scribbler.set_led(scribbler#RIGHT,lookupz(scribbler.light_sensor(scribbler#RIGHT) >> 6 :scribbler#OFF,scribbler#YELLOW,scribbler#DIM_GREEN,scribbler#GREEN)) 

    case state
      COVER_LEFT:
        if (sample_left =< cover_trip_left) and (sample_center > cover_trip_center) and (sample_right > cover_trip_right)    
          music.play_note(music#HALF,music#A4,0)
          clear_row(TEST_ROW_0) 
          print_str(TEST_COL_START,TEST_ROW_0,string("Cover the Center Light Sensor"))
          state := COVER_CENTER
          time := LIGHT_TEST_TIME  
          
      COVER_CENTER:
        if (sample_left > cover_trip_left) and (sample_center =< cover_trip_center) and (sample_right > cover_trip_right)  
          music.play_note(music#HALF,music#B4,0)
          clear_row(TEST_ROW_0)
          print_str(TEST_COL_START,TEST_ROW_0,string("Cover the Right Light Sensor"))
          state := COVER_RIGHT
          time := LIGHT_TEST_TIME  

      COVER_RIGHT:
        if (sample_left > cover_trip_left) and (sample_center > cover_trip_center) and (sample_right =< cover_trip_right)   
          music.play_note(music#HALF,music#C4,0)           
          clear_row(TEST_ROW_0) 
          print_str(TEST_COL_START,TEST_ROW_0,string("Shine a Light into the Left Light Sensor"))
          state := SHINE_LEFT
          time := LIGHT_TEST_TIME

      SHINE_LEFT:
        if sample_left > LIGHT_SHINE_TRIP   
          music.play_note(music#HALF,music#D4,0)
          clear_row(TEST_ROW_0) 
          print_str(TEST_COL_START,TEST_ROW_0,string("Shine a Light into the Center Light Sensor"))
          state := SHINE_CENTER
          time := LIGHT_TEST_TIME 

      SHINE_CENTER:
        if sample_center > LIGHT_SHINE_TRIP   
          music.play_note(music#HALF,music#E4,0)
          clear_row(TEST_ROW_0) 
          print_str(TEST_COL_START,TEST_ROW_0,string("Shine a Light into the Right Light Sensor"))
          state := SHINE_RIGHT
          time := LIGHT_TEST_TIME
          
      SHINE_RIGHT:
        if sample_right > LIGHT_SHINE_TRIP   
          music.play_note(music#HALF,music#F4,0)
          clear_row(TEST_ROW_0)
          clear_row(LIGHT_ROW_TIME)
          print_str(TEST_COL_START,TEST_ROW_0,string("Light Test Pass"))
          return TRUE
          
    time -= 1        
    if (time == 0)
      scribbler.delay_tenths(TEST_PASS_PAUSE)    
      clear_row(TEST_ROW_0)
      clear_row(LIGHT_ROW_TIME)
      print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Light Timer Expired."))                                        
      return FALSE  

    periodic(LIGHT_LOOP_FREQ)

CON
  'Idler Test Screen Layout
  IDLER_COL_A                   = LEFT_MARGIN + 16 
  IDLER_COL_B                   = LEFT_MARGIN + 28
  IDLER_ROW_PASS                 = 7
  IDLER_ROW_TIME                = 10

  IDLER_LOOP_FREQ               = 5                  'Loop frequency (Hz)
  IDLER_TEST_TIME               = 15                 'Max Test Time (Sec)
  
PUB idler_test(count_pass) | sample,cnt_last,cnt_init,time
  ''Print the Idler Wheel ADC return
  ''Print the Idler Wheel counter
  ''Beep on each Idler Wheel hole detect
  ''count_pass = number of counts to consider a test pass if positive
  ''repeat until key press if count_pass is negative

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF) 
  scribbler.set_voices(scribbler#SQU,scribbler#SQU)
  scribbler.set_volume(100) 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Idler Wheel Encoder Test"))
  if count_pass < 0
    exit_print(TEST_COL_MID,10)    

  print_center(TOP_MARGIN,string("Spin the Idler Wheel"))  
  print_str(LEFT_MARGIN,6,string("ADC Value:"))
  print_str(LEFT_MARGIN,8,string("Idler Counter:"))
  cnt_init := cnt_last := scribbler.get_results(scribbler#CNT_IDLER)
  if (count_pass => 0) 
    time := constant(IDLER_TEST_TIME * IDLER_LOOP_FREQ) 
    print_str(LEFT_MARGIN,IDLER_ROW_TIME,string("Test Time:  "))

  periodic_start 

  repeat
    'Print Idler ADC Voltage
    sample := scribbler.get_adc_results(scribbler#ADC_IDLER)
    print_volts_3v3(IDLER_COL_A,6,sample)
    print_bar(IDLER_COL_B,6,sample << 3)
    'Print Idler Wheel Counter
    sample := scribbler.get_results(scribbler#CNT_IDLER)
    print_dec_word(IDLER_COL_A,8,sample)
    'Beep if Idler Wheel Counter has Changed
    if sample > cnt_last
      scribbler.play_tone(100,622,0)
      cnt_last := sample 

    if (count_pass => 0)      
      print_dec_word(IDLER_COL_A,IDLER_ROW_TIME,time/IDLER_LOOP_FREQ) 
      sio.str(string(" secs "))
      if (sample => (cnt_init + count_pass))
        sio.tx(cls)
        print_title(TEST_COL_START,TEST_COL_END,-1,string("Idler Wheel Encoder Test"))  
        print_str(TEST_COL_START,TEST_ROW_0,string("Idler Wheel Test Pass"))
        return TRUE
      elseif (time =< 0)
        sio.tx(cls)
        print_title(TEST_COL_START,TEST_COL_END,-1,string("Idler Wheel Encoder Test"))  
        print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Idler Timer Expired"))
        return FALSE
      time -= 1 
    else   
      exit_check
    periodic(IDLER_LOOP_FREQ)

CON
  MIC_ROW_START             = TOP_MARGIN
  MIC_ROW_VALUE             = MIC_ROW_START + 2
  MIC_ROW_PASS              = MIC_ROW_VALUE + 2 
  MIC_ROW_TIME              = MIC_ROW_PASS + 3
  MIC_ROW_END               = 12  
    
  MIC_SAMPLES               = 4                   'Number of microphone samples
  MIC_LOOP_FREQ             = 10                  'Loop frequency (Hz)
  MIC_TEST_TIME             = 15                  'Max Test Time (Sec)
  
  MIC_LOWER_PASS            = 32                  'Lower value required to pass
  MIC_UPPER_PASS            = 100                 'Upper value required to pass

PUB mic_test | mic[MIC_SAMPLES+2],index,mic_max,pass_lower,pass_upper,time 
  ''Microphone Check
  ''Display the microphone level on the LED's

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF) 
  pass_lower := FALSE
  pass_upper := FALSE 
  scribbler.start_mic_env    
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Microphone Test")) 
  time := constant(MIC_TEST_TIME * MIC_LOOP_FREQ)
  print_str(TEST_COL_START,MIC_ROW_TIME,string("Time:  ")) 
  print_center(MIC_ROW_START,string("Speak into the Microphone"))

  periodic_start
  longfill(@mic,scribbler.get_mic_env >> 14,MIC_SAMPLES+2)
  repeat
    print_dec(constant(TEST_COL_START + 7),MIC_ROW_TIME,time / MIC_LOOP_FREQ) 
    sio.str(string(" secs "))
    mic[0] := scribbler.get_mic_env >> 14                                          'Sample microphone level
    if mic[0] =< MIC_LOWER_PASS
      pass_lower := TRUE
      print_str(TEST_COL_START,MIC_ROW_PASS,string("Lower Limit Pass"))      
    if mic[0] => MIC_UPPER_PASS
      pass_upper := TRUE
      print_str(TEST_COL_START,constant(MIC_ROW_PASS+1),string("Upper Limit Pass"))

    mic_max := 0
    repeat index from constant(MIC_SAMPLES+1) to 1                        'Search samples for max
      mic[index] := mic[index-1] 
      if mic[index] > mic_max
        mic_max := mic[index]

    case mic_max >> 5                                                     'Display mic level on LED's 
        0: scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#NO_CHANGE)
        1: scribbler.set_leds(scribbler#GREEN,scribbler#OFF,scribbler#OFF,scribbler#NO_CHANGE)
        2: scribbler.set_leds(scribbler#GREEN,scribbler#GREEN,scribbler#OFF,scribbler#NO_CHANGE)
        3: scribbler.set_leds(scribbler#GREEN,scribbler#GREEN,scribbler#RED,scribbler#NO_CHANGE)

    print_dec_byte(TEST_COL_START,MIC_ROW_VALUE,mic_max)
    print_bar(constant(TEST_COL_START + 8),MIC_ROW_VALUE,mic_max)   

    time -= 1

    if (pass_lower and pass_upper)
      scribbler.start 
      scribbler.start_motors
      scribbler.start_tones 
      scribbler.delay_tenths(TEST_PASS_PAUSE)
      sio.tx(cls)
      print_title(TEST_COL_START,TEST_COL_END,-1,string("Microphone Test"))  
      print_str(TEST_COL_START,TEST_ROW_0,string("Microphone Test Pass"))
      return TRUE
        
    if (time == 0)
      scribbler.start 
      scribbler.start_motors
      scribbler.start_tones     
      scribbler.delay_tenths(TEST_PASS_PAUSE)
      sio.tx(cls)
      print_title(TEST_COL_START,TEST_COL_END,-1,string("Microphone Test"))  
      print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Microphone Timer Expired"))
      return FALSE
            
    periodic(MIC_LOOP_FREQ)    

CON
  'Line Sensor Test Screen Layout Constants
  LINE_COL_LABEL            = LEFT_MARGIN 
  LINE_COL_MIN              = LINE_COL_LABEL + 8
  LINE_COL_MAX              = LINE_COL_MIN + 8 
  LINE_COL_VALUE            = LINE_COL_MAX + 10
  LINE_COL_FAIL             = LINE_COL_VALUE + 10   
  
  LINE_ROW_MESSAGE_A        = TOP_MARGIN
  LINE_ROW_MESSAGE_B        = LINE_ROW_MESSAGE_A + 1
  LINE_ROW_LABEL            = LINE_ROW_MESSAGE_B +2
  LINE_ROW_BLACK            = LINE_ROW_LABEL + 1
  LINE_ROW_GRAY             = LINE_ROW_BLACK + 4 
  LINE_ROW_WHITE            = LINE_ROW_GRAY + 4
  LINE_ROW_END              = LINE_ROW_WHITE + 4
  
  'Line Sensor Limits for each Color Reference Card
  LINE_BLACK_MIN            = (0*255)/330           '0.00 V
  LINE_BLACK_MAX            = (41*255)/330          '0.40 V
  LINE_GRAY_MIN             = (41*255)/330          '0.40 V
  LINE_GRAY_MAX             = (101*255)/330         '1.00 V
  LINE_WHITE_MIN            = (91*255)/330          '0.90 V
  LINE_WHITE_MAX            = (330*255)/330         '3.30 V     
  
PUB line_test | pass,sample_right,sample_left,index,row,bounce_cnt
  ''The utility tests the line sensor using a photography color reference card set.
  ''Opteka DGC-LARGE-8X10 / 8" X 10" Digital Color & White Balance Card Set
  ''
  ''Black Card RGB: 16,16,15
  ''Gray Card RGB:  162,162,160
  ''White Card RGB: 220,224,223
  ''
  ''Place the scribbler on each of the black, gray & white reference cards
  ''one at a time as directed by the program.
  ''The sensors are checked to be within the correct operating limits.

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF) 
  pass := TRUE 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Line Sensor Test")) 
  print_str(constant(LINE_COL_MIN+3),LINE_ROW_LABEL,string("Min"))
  print_str(constant(LINE_COL_MAX+3),LINE_ROW_LABEL,string("Max"))
  print_str(constant(LINE_COL_VALUE+1),LINE_ROW_LABEL,string("Sensor"))      
  repeat index from 0 to 2
    print_str(constant(TEST_COL_MID - 30/2),LINE_ROW_MESSAGE_A,string("Place the scribbler on the "))
    case index
      0:
        sio.str(string("Black Card"))
        row := LINE_ROW_BLACK
        print_str(LINE_COL_LABEL,row,string("Black")) 
      1:
        sio.str(string("Gray Card"))
        row := LINE_ROW_GRAY        
        print_str(LINE_COL_LABEL,row,string("Gray"))
      2:
        sio.str(string("White Card"))
        row := LINE_ROW_WHITE        
        print_str(LINE_COL_LABEL,row,string("White"))
        
    print_center(LINE_ROW_MESSAGE_B,string("and then Press the Button."))
    print_str(constant(TEST_COL_MID - 27/2),LINE_ROW_MESSAGE_B,string("and then Press the Button."))
    print_str(LINE_COL_LABEL,row + 1,string("Left"))
    print_str(LINE_COL_LABEL,row + 2,string("Right"))   
    print_volts_3v3(LINE_COL_MIN,row + 1,lookupz(index : LINE_BLACK_MIN,LINE_GRAY_MIN,LINE_WHITE_MIN))
    print_volts_3v3(LINE_COL_MAX,row + 1,lookupz(index : LINE_BLACK_MAX,LINE_GRAY_MAX,LINE_WHITE_MAX))

    'Debounce unpushed button state  
    bounce_cnt := 0
    repeat until bounce_cnt == 3 
      if scribbler.button_press
        bounce_cnt := 0
      else
        bounce_cnt += 1
      print_volts_3v3(LINE_COL_VALUE,row + 1,sample_left := scribbler.line_sensor(scribbler#LEFT,TRUE))       
      print_volts_3v3(LINE_COL_VALUE,row + 2,sample_right := scribbler.line_sensor(scribbler#RIGHT,TRUE))
      scribbler.delay_tenths(1)
       
    repeat until scribbler.button_press
      print_volts_3v3(LINE_COL_VALUE,row + 1,sample_left := scribbler.line_sensor(scribbler#LEFT,TRUE)) 
      print_volts_3v3(LINE_COL_VALUE,row + 2,sample_right := scribbler.line_sensor(scribbler#RIGHT,TRUE))
      scribbler.delay_tenths(1)

    ifnot limit_test(LINE_COL_FAIL,row + 1,sample_left,{
    }lookupz(index:LINE_BLACK_MIN,LINE_GRAY_MIN,LINE_WHITE_MIN),{
    }lookupz(index:LINE_BLACK_MAX,LINE_GRAY_MAX,LINE_WHITE_MAX))
      pass := FALSE
      scribbler.set_led(scribbler#LEFT,scribbler#BLINK_RED)                   
          
    ifnot limit_test(LINE_COL_FAIL,row + 2,sample_right,{
    }lookupz(index:LINE_BLACK_MIN,LINE_GRAY_MIN,LINE_WHITE_MIN),{
    }lookupz(index:LINE_BLACK_MAX,LINE_GRAY_MAX,LINE_WHITE_MAX))
      pass := FALSE             
      scribbler.set_led(scribbler#RIGHT,scribbler#BLINK_RED)

    clear_rows(TEST_ROW_0,TEST_ROW_1)
    scribbler.delay_tenths(5) 

  'Print the test results
  scribbler.delay_tenths(TEST_PASS_PAUSE)
  sio.tx(cls) 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Line Sensor Test"))
  if pass
    print_str(TEST_COL_START,TEST_ROW_0,string("Line Sensor Test Pass"))
  else
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Line Readings out of range."))     
  return pass  
  
PUB track_test | distance
  ''Test the scribbler's Motor and Encoders
  ''Print out the drive_track.pdf file
  ''Tape the print out on top of a blank piece of paper to
  ''a desk top or other hard surfaces. 
  ''Place the scribbler on the track and start the utility.
  ''The utility measure distance between the start and finish
  ''line as a check of the encoder.
  ''The line sensors must be operational for the
  ''test to work correctly.
  
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Motor & Wheel Encoder Track Test")) 
  print_center(TEST_ROW_0,string("Place the scribbler on the Printed Test Track"))
  print_center(TEST_ROW_1,string("Press the Button to Start"))
  scribbler.set_leds(scribbler#OFF,scribbler#ORANGE,scribbler#OFF,scribbler#OFF)
  
  button_wait(1)
  clear_rows(TEST_ROW_0,TEST_ROW_1)
    
  if not scribbler.line_sensor(scribbler#LEFT,line_threshold) or not scribbler.line_sensor(scribbler#RIGHT,line_threshold)
    clear_rows(TEST_ROW_0,TEST_ROW_1)
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Black Line Detect at Start"))
    return FALSE    

  'Move to front edge of start line
  clear_rows(TEST_ROW_0,TEST_ROW_1) 
  print_center(TEST_ROW_0,string("Moving to Start Line"))
  scribbler.set_speed(3)
  scribbler.go_forward (200)
  repeat while scribbler.moving
    if not scribbler.line_sensor(scribbler#LEFT,line_threshold) and not scribbler.line_sensor(scribbler#RIGHT,line_threshold)
      scribbler.stop_now 
      scribbler.set_leds(scribbler#YELLOW,scribbler#OFF,scribbler#YELLOW,scribbler#NO_CHANGE)
      scribbler.beep
      quit
      
  clear_row(TEST_ROW_0)
  print_center(TEST_ROW_0,string("Moving to Finish Line"))
  scribbler.set_speed(4) 
  repeat 4            'Move Forward 100mm - The Start to Finish Line Spacing 
    scribbler.go_forward (50)   
    scribbler.wait_stop
    if not scribbler.line_sensor(scribbler#LEFT,line_threshold) or not scribbler.line_sensor(scribbler#RIGHT,line_threshold)
      clear_rows(TEST_ROW_0,TEST_ROW_1) 
      print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Black Detect between Start & Finish"))
      return FALSE 

  repeat distance from 200 to 210
    if not scribbler.line_sensor(scribbler#LEFT,line_threshold) and not scribbler.line_sensor(scribbler#RIGHT,line_threshold)
      scribbler.stop_now 
      quit
    scribbler.go_forward (1) ' Move forward 0.5mm
    scribbler.wait_stop
  
  clear_rows(TEST_ROW_0,TEST_ROW_1) 

  if distance =< 210   
    print_str(TEST_COL_START,TEST_ROW_0,string("Pass - Finish Line at: "))
    sio.dec(distance/2)
    sio.str(string("mm"))
    return TRUE 
  else
    scribbler.beep
    scribbler.beep
    scribbler.beep        
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Finish Line Not Found in the 100-105mm Range")) 
    return FALSE     

CON
  'Speed Test Screen Layout
  SPD_COL_LABEL            = LEFT_MARGIN 
  SPD_COL_LIMIT            = SPD_COL_LABEL + 10 
  SPD_COL_VALUE            = SPD_COL_LIMIT + 12
  SPD_COL_FAIL             = SPD_COL_VALUE + 10  
  
  SPD_RPM_TOL             = 25      'Maximum Motor Speed Variation (+/- %)           
    
  SPD_3V0_RPM_MIN         = 2000 * (100 - SPD_RPM_TOL)/100 
  SPD_3V0_RPM_MAX         = 2000 * (100 + SPD_RPM_TOL)/100
  SPD_6V0_RPM_MIN         = 5000 * (100 - SPD_RPM_TOL)/100 
  SPD_6V0_RPM_MAX         = 5000 * (100 + SPD_RPM_TOL)/100
  SPD_3V0_I_MIN           = 0 
  SPD_3V0_I_MAX           = 90 
  SPD_6V0_I_MIN           = 0 
  SPD_6V0_I_MAX           = 135

  SPD_VLT_STEP            = 3000

PUB speed_test | vmot,volt,duty,countl,countr,side,dir,rpm,cur,min_rpm,max_rpm,min_cur,max_cur,pass   
  ''Test the Motor Speed
  
  pass := TRUE
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)              
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Motor Speed Test"))

  'Stops all scribbler Object Cogs so native PWM routines can be ran
  'Restart ADC cog for measuring the battery voltage
  scribbler.start  
  scribbler.button_mode(FALSE,FALSE)
  
  start_encoder_count             'Start encoder counter
  pwm.start(constant(scribbler#MOT_LEFT_PWM & $1c), constant(1 << (scribbler#MOT_LEFT_PWM & 7) | 1 << (scribbler#MOT_RIGHT_PWM & 7)), 20000)
  dira := constant(1 << scribbler#MOT_LEFT_DIR | 1 << scribbler#MOT_RIGHT_DIR)

  if scribbler.get_model_s2 
    vmot := (scribbler.get_adc_results(scribbler#ADC_VBAT) * constant(100 * 33 * 4)) / 255  'Convert Battery Voltage Units
  else
    vmot := (scribbler.get_adc_results(scribbler#ADC_VMOT) * constant(100*33*(100+33))) / constant(255*33)  'Convert Boost Power Supply Voltage Unit 

   ' calc := (scribbler.get_adc_results(scribbler#ADC_VMOT) * (100+33)) / 33
    
  print_str(SPD_COL_LABEL,constant(TOP_MARGIN-1),string("Motor Supply: "))
  print_10ths(vmot)
  sio.tx("V")
  'Motors forward
  OUTA[scribbler#MOT_LEFT_DIR]~~
  OUTA[scribbler#MOT_RIGHT_DIR]~~  

  repeat side from MOT_LEFT to MOT_RIGHT
    repeat volt from 3000 to 6000 step SPD_VLT_STEP 
      min_rpm := lookup(volt/SPD_VLT_STEP:SPD_3V0_RPM_MIN,SPD_6V0_RPM_MIN)
      max_rpm := lookup(volt/SPD_VLT_STEP:SPD_3V0_RPM_MAX,SPD_6V0_RPM_MAX)
      min_cur := lookup(volt/SPD_VLT_STEP:SPD_3V0_I_MIN,SPD_6V0_I_MIN)
      max_cur := lookup(volt/SPD_VLT_STEP:SPD_3V0_I_MAX,SPD_6V0_I_MAX)

      cursor(SPD_COL_LABEL,TEST_ROW_2)
      sio.str(string("Test Voltage:"))
      cursor(SPD_COL_LIMIT+6,TEST_ROW_2)
      print_100ths(volt)
      sio.str(string(" V"))
      
      'Print limits
      print_str(SPD_COL_LIMIT+3,TEST_ROW_5,string("min"))
      print_str(SPD_COL_LIMIT+9,TEST_ROW_5,string("max"))
      print_str(SPD_COL_LABEL,TEST_ROW_6,string("Speed"))
      print_dec_word(SPD_COL_LIMIT,TEST_ROW_6,min_rpm)
      sio.str(string(" - "))
      print_dec_word(SPD_COL_LIMIT+6,TEST_ROW_6,max_rpm)
      print_str(SPD_COL_LABEL,TEST_ROW_7,string("Current"))
      print_dec_word(SPD_COL_LIMIT,TEST_ROW_7,min_cur)
      sio.str(string(" - "))
      print_dec_word(SPD_COL_LIMIT+6,TEST_ROW_7,max_cur)
           
      'Calculate and print the PWM duty cycle
      duty := (volt * 100)/vmot 
      print_str(SPD_COL_LABEL,TEST_ROW_3,string("Duty Cycle:"))
      print_dec_word(SPD_COL_LIMIT+6,TEST_ROW_3,duty)                            
      sio.str(string("%"))
     
      'Print motor side
      if (side == MOT_LEFT)
        print_str(SPD_COL_LABEL,TEST_ROW_1,string("Left Motor"))
        pwm.duty(scribbler#MOT_LEFT_PWM, duty * 255 / 100)             
      else
        print_str(SPD_COL_LABEL,TEST_ROW_1,string("Right Motor"))
        pwm.duty(scribbler#MOT_RIGHT_PWM, duty * 255 / 100)
        
      waitcnt(cnt + clkfreq*2)          'Pause two seconds for motors to accelerate
      countl := phsa                  'Read initial encoder counts.
      countr := phsb
      waitcnt(cnt + clkfreq * MOTOR_SAMPLE_LEN)  
      countl := phsa - countl         'Read encoder counts after sample length
      countr := phsb - countr
      if side == MOT_LEFT
        rpm := countl*constant(60/MOTOR_SAMPLE_LEN)/4
      else
        rpm := countr*constant(60/MOTOR_SAMPLE_LEN)/4 
      cur := (scribbler.get_adc_results(scribbler#ADC_IMOT) * 33000) / 5865 

      print_dec_word(SPD_COL_VALUE,TEST_ROW_6,rpm)
      ifnot limit_test(SPD_COL_FAIL,TEST_ROW_6,rpm,min_rpm,max_rpm)
        pass := FALSE
      print_dec_word(SPD_COL_VALUE,TEST_ROW_7,cur) 
      ifnot limit_test(SPD_COL_FAIL,TEST_ROW_7,cur,min_cur,max_cur)
        pass := FALSE

      scribbler.delay_tenths(TEST_PASS_PAUSE)
      clear_rows(TEST_ROW_0,TEST_ROW_7)

    'Turn Motors Off 
    pwm.duty(scribbler#MOT_LEFT_PWM, 0)   
    pwm.duty(scribbler#MOT_RIGHT_PWM, 0)

  clear_row(constant(TOP_MARGIN-1))
  clear_row(TEST_ROW_0)
  if pass
    print_str(TEST_COL_START,TEST_ROW_0,string("Speed Test Pass."))
  else 
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Speed Test."))     
     
  scribbler.start 
  scribbler.button_mode(FALSE,FALSE)
  scribbler.start_motors
  scribbler.start_tones     
  scribbler.delay_tenths(5)
  return pass

CON
  IR_DETECT_NUM = 4             'Number of repeated IR detects to pass
    
PUB ir_test | detect_cnt,side,time,position
  ''Test the IR Obstacle Sensors
  
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)
  print_title(TEST_COL_START,TEST_COL_END,-1,string("IR Obstacle Sensor Test"))    
  print_str(TEST_COL_START,TEST_ROW_0,string("Place the scribbler on Carpet."))
  print_str(TEST_COL_START,TEST_ROW_2,string("Press the Button to Continue.")) 
  button_wait(1)
  clear_row(TEST_ROW_0)
  clear_row(TEST_ROW_2)
  print_str(TEST_COL_START,TEST_ROW_0,string("Checking for False Obstacle Detects."))  
  print_str(TEST_COL_START,TEST_ROW_1,string("Time:  ")) 

  repeat time from 50 to 0                          'Check for false obstacle detects 
    if scribbler.obstacle(scribbler#RIGHT,obs_threshold) or scribbler.obstacle(scribbler#LEFT,obs_threshold)
      clear_row(TEST_ROW_0)
      clear_row(TEST_ROW_1)
      print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - False Detect on Carpet"))
      return FALSE
    print_dec(constant(TEST_COL_START + 7),TEST_ROW_1, time/10)
    sio.tx(" ") 
    scribbler.delay_tenths(1)
  clear_row(TEST_ROW_0)
  clear_row(TEST_ROW_1)

  print_str(TEST_COL_START,TEST_ROW_0,string("Place the scribbler on Obstacle Range Printout"))

  repeat position from "A" to "E"
    scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)   
    detect_cnt := IR_DETECT_NUM
    clear_row(TEST_ROW_1)
    print_str(TEST_COL_START,TEST_ROW_1,string("Place Pipe at Postion: "))
    sio.tx(position)
    print_str(TEST_COL_START,TEST_ROW_3,string("Press the Button to Continue.")) 
    button_wait(1)
    clear_row(TEST_ROW_3)
    repeat until detect_cnt == 0
      if position == "A" or position == "B"
        if scribbler.obstacle(scribbler#LEFT,obs_threshold)
          detect_cnt -= 1
          scribbler.set_led(scribbler#LEFT,scribbler#GREEN) 
          music.play_note(music#EGTH,music#C4,0)
          scribbler.delay_tenths(3)
        else
          detect_cnt := IR_DETECT_NUM
          scribbler.set_led(scribbler#LEFT,scribbler#RED)
                      
      elseif position == "C"
        if scribbler.obstacle(scribbler#LEFT,obs_threshold)
          music.play_note(music#EGTH,music#C4,0)
          scribbler.set_led(scribbler#LEFT,scribbler#GREEN)
          scribbler.delay_tenths(3)

          if scribbler.obstacle(scribbler#RIGHT,obs_threshold)
            music.play_note(music#EGTH,music#E4,0)
            scribbler.set_led(scribbler#RIGHT,scribbler#GREEN)
            detect_cnt -= 1
            scribbler.delay_tenths(3) 
          else
            detect_cnt := IR_DETECT_NUM/2 
            scribbler.set_led(scribbler#RIGHT,scribbler#RED)            

        else
          detect_cnt := IR_DETECT_NUM/2 
          scribbler.set_led(scribbler#LEFT,scribbler#RED)
          if scribbler.obstacle(scribbler#RIGHT,obs_threshold)
            music.play_note(music#EGTH,music#E4,0)
            scribbler.set_led(scribbler#RIGHT,scribbler#GREEN)
            scribbler.delay_tenths(3)
          else
            scribbler.set_led(scribbler#RIGHT,scribbler#RED)
            
      else
        if scribbler.obstacle(scribbler#RIGHT,obs_threshold)
          scribbler.set_led(scribbler#RIGHT,scribbler#GREEN) 
          detect_cnt -= 1
          music.play_note(music#EGTH,music#E4,0)
          scribbler.delay_tenths(3)
        else
          detect_cnt := IR_DETECT_NUM
          scribbler.set_led(scribbler#RIGHT,scribbler#RED)
          
  clear_row(TEST_ROW_0)
  clear_row(TEST_ROW_1)
  clear_row(TEST_ROW_2)
  clear_row(TEST_ROW_3)
  print_str(TEST_COL_START,TEST_ROW_0,string("IR Test Pass")) 
  return TRUE
   
PUB speaker_test
  ''Test the Speaker by asking the tester about the sound quality
  ''via the serial terminal                                       
  
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF) 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("Speaker Test"))       
  print_str(TEST_COL_START,TEST_ROW_0,string("Check the speaker's sound quality"))
  print_str(TEST_COL_START,TEST_ROW_2,string("Button Press:"))   
  print_str(constant(TEST_COL_START+1),TEST_ROW_3,string("1 - Pass"))
  print_str(constant(TEST_COL_START+1),TEST_ROW_4,string("2 - Fail"))

  music.play_song(music#CHARGE,1500)  
  if button_wait(2) == 2
    clear_rows(TEST_ROW_0,TEST_ROW_4)     
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Speaker Test."))
    return FALSE
  else
    clear_rows(TEST_ROW_0,TEST_ROW_4)  
    print_str(TEST_COL_START,TEST_ROW_0,string("Speaker Test Pass.")) 
    return TRUE
      
PUB led_test
  ''Test the LED's by asking the tester a series of questions
  ''via the serial terminal
  
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF) 
  print_title(TEST_COL_START,TEST_COL_END,-1,string("LED Test"))
  print_str(TEST_COL_START,TEST_ROW_2,string("Button Press:"))   
  print_str(constant(TEST_COL_START+1),TEST_ROW_3,string("1 - Pass"))
  print_str(constant(TEST_COL_START+1),TEST_ROW_4,string("2 - Fail"))
  print_str(TEST_COL_START,TEST_ROW_0,string("Check that the 3 Left LED's Blink Red/Green."))
  scribbler.set_leds(scribbler#ALT_RED_GREEN,scribbler#ALT_RED_GREEN,scribbler#ALT_RED_GREEN,scribbler#OFF)
  if button_wait(2) == 2
    clear_rows(TEST_ROW_0,TEST_ROW_4)
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Red/Green Test."))
    return FALSE 
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)
  clear_row(TEST_ROW_0)
  print_str(TEST_COL_START,TEST_ROW_0,string("Check that the Blue LED Blinks On & Off."))
  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#BLINK_BLUE)
  if button_wait(2) == 2
    clear_rows(TEST_ROW_0,TEST_ROW_4)
    print_str(TEST_COL_START,TEST_ROW_0,string("** Fail - Blue LED Test."))
    return FALSE 

  scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#OFF)
  clear_rows(TEST_ROW_0,TEST_ROW_4)
  print_str(TEST_COL_START,TEST_ROW_0,string("LED Test Pass.")) 
  return TRUE

         
PUB menu_print | rx  
  ''Print the Test Utility Menu

  repeat 
    scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#NO_CHANGE)
    sio.tx(cls)
    if (scribbler.get_model_s2)  
      print_title(TEST_COL_START,TEST_COL_END,-1,string("S2 Robot Detected"))
    elseif (scribbler.get_model_s3)
      print_title(TEST_COL_START,TEST_COL_END,-1,string("S3 Robot Detected"))
    else
      print_title(TEST_COL_START,TEST_COL_END,-1,string("** Model Number Detect Fail **"))    
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 2),string("Tests:"))
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 4),string("(P)ower Test"))
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 5),string("(S)ensor Display"))    
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 6),string("(D)rive"))
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 7),string("(T)rack Test")) 
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 8),string("(M)usic Test"))
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 9),string("(V)isible Light Sensor Angle Test (45 seconds)")) 
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 10),string("(C)og Usage"))     
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 11),string("(I)dler Wheel Test"))
    print_str(LEFT_MARGIN,constant(TOP_MARGIN + 12),string("(L)ED Test"))
   
    repeat while (rx := sio.rxcheck) == -1             'Wait for a serial command
    case rx 
      "m","M":
        music_test
      "p","P":
        sio.tx(cls)
        electrical_test
      "s","S":
        sio.tx(cls)
        sensor_print   
      "c","C":
        sio.tx(cls)
        prop_print            
      "d","D":
        man_drive
      "v","V":
        light_graph_raw
      "l","L":
        led_press
      "t","T":
        sio.tx(cls)
        track_test
        scribbler.delay_tenths(40) 
      "i","I":
        sio.tx(cls)
        idler_test (-1)
      other : menu_print  
        
CON
  'Sensor Screen Layout
  SENS_COL_START              = 1
  SENS_COL_END                = SENS_COL_C + 12
  SENS_COL_MID                = (SENS_COL_END - SENS_COL_START)/2 
  SENS_COL_A                  = LEFT_MARGIN + 9 
  SENS_COL_B                  = SENS_COL_A + 10 
  SENS_COL_C                  = SENS_COL_B + 11
  
  LINE_ROW                    = TOP_MARGIN  
  LIGHT_ROW                   = LINE_ROW + 5 
  OBS_ROW                     = LIGHT_ROW + 7
  MIC_ROW                     = OBS_ROW + 3
  SENS_ROW_END                = MIC_ROW+3 

PUB sensor_print
  ''Print all of the sensor values to the serial terminal
  
  scribbler.start_mic_env
  print_title(SENS_COL_START,SENS_COL_END,SENS_ROW_END,string("Sensor Test"))  
  print_str(LEFT_MARGIN,LINE_ROW,string("Line Sensors:"))
  print_str(LEFT_MARGIN,constant(LINE_ROW+2),string("Left"))  
  print_str(LEFT_MARGIN,constant(LINE_ROW+3),string("Right"))
  print_str(LEFT_MARGIN,LIGHT_ROW,string("Light Sensors:"))
  print_str(constant(SENS_COL_A+3),constant(LIGHT_ROW+2),string("Raw")) 
  print_str(constant(SENS_COL_B+3),constant(LIGHT_ROW+2),string("Compressed")) 
  print_str(LEFT_MARGIN,constant(LIGHT_ROW+3),string("Left"))
  print_str(LEFT_MARGIN,constant(LIGHT_ROW+4),string("Middle"))   
  print_str(LEFT_MARGIN,constant(LIGHT_ROW+5),string("Right"))
  print_str(LEFT_MARGIN,OBS_ROW,string("Obstacle:"))
  print_str(LEFT_MARGIN,MIC_ROW,string("Microphone:"))

  periodic_start
  repeat
  ' 'Line sensor
    print_dec_word(SENS_COL_A,constant(LINE_ROW+2),scribbler.line_sensor(scribbler#LEFT,TRUE))
    print_volts_3v3(SENS_COL_B,constant(LINE_ROW+2),scribbler.line_sensor(scribbler#LEFT,TRUE)) 
    print_bar(SENS_COL_C,constant(LINE_ROW+2),scribbler.line_sensor(scribbler#LEFT,TRUE))
    
    print_dec_word(SENS_COL_A,constant(LINE_ROW+3),scribbler.line_sensor(scribbler#RIGHT,TRUE))  
    print_volts_3v3(SENS_COL_B,constant(LINE_ROW+3),scribbler.line_sensor(scribbler#RIGHT,TRUE)) 
    print_bar(SENS_COL_C,constant(LINE_ROW+3),scribbler.line_sensor(scribbler#RIGHT,TRUE))

  'Light Sensor
    print_dec_word(SENS_COL_A,constant(LIGHT_ROW+3),scribbler.light_sensor_raw(scribbler#LEFT))
    print_dec_word(SENS_COL_B,constant(LIGHT_ROW+3),scribbler.light_sensor(scribbler#LEFT))
    print_bar(SENS_COL_C,constant(LIGHT_ROW+3),scribbler.light_sensor(scribbler#LEFT))

    print_dec_word(SENS_COL_A,constant(LIGHT_ROW+4),scribbler.light_sensor_raw(scribbler#CENTER)) 
    print_dec_word(SENS_COL_B,constant(LIGHT_ROW+4),scribbler.light_sensor(scribbler#CENTER))
    print_bar(SENS_COL_C,constant(LIGHT_ROW+4),scribbler.light_sensor(scribbler#CENTER))

    print_dec_word(SENS_COL_A,constant(LIGHT_ROW+5),scribbler.light_sensor_raw(scribbler#RIGHT))
    print_dec_word(SENS_COL_B,constant(LIGHT_ROW+5),scribbler.light_sensor(scribbler#RIGHT)) 
    print_bar(SENS_COL_C,constant(LIGHT_ROW+5),scribbler.light_sensor(scribbler#RIGHT))  

  'Obstacle sensor
    if scribbler.obstacle(scribbler#LEFT,obs_threshold)
      if scribbler.obstacle(scribbler#RIGHT,obs_threshold)
        print_str(constant(LEFT_MARGIN+12),OBS_ROW,string("Left & Right"))
      else
        print_str(constant(LEFT_MARGIN+12),OBS_ROW,string("Left"))
        sio.tx(CEL)
    elseif scribbler.obstacle(scribbler#RIGHT,obs_threshold)
      cursor(constant(LEFT_MARGIN+12),OBS_ROW)
      sio.tx(CEL)
      print_str(constant(LEFT_MARGIN+19),OBS_ROW,string("Right"))
    else
      cursor(constant(LEFT_MARGIN+12),OBS_ROW)
      sio.tx(CEL)
      print_str(constant(LEFT_MARGIN+13),OBS_ROW,string("No Detect"))

    'Microphone 
    print_percentage(SENS_COL_B,MIC_ROW,scribbler.get_mic_env>>14)
    print_bar(SENS_COL_C,MIC_ROW,scribbler.get_mic_env>>14)

    exit_check
    periodic(5)

CON
  'Manual Drive Screen Layout
  MOT_ROW                   = TOP_MARGIN      
  MOT_MENU_ROW              = MOT_ROW + 6

  DRV_COL_START               = 1
  DRV_COL_END                 = 52
  DRV_COL_MID                 = (DRV_COL_END - DRV_COL_START)/2
  ENC_COL                   = LEFT_MARGIN + 9 
  SPD_COL                   = LEFT_MARGIN + 20
  DRV_COLA                  = LEFT_MARGIN
  DRV_COLB                  = DRV_COLA + 14
  DRV_COLC                  = DRV_COLB + 14

  'Drive Speeds
  VEL_ARC                   = 192
  VEL_ARC_SLOW              = 64
  VEL_STRAIGHT              = 150
  VEL_MAX                   = 255
  VEL_INC                   = 16
  DEG_ROTATE                = 45

  'Drive Commands
  DRV_ARC_LEFT              = "7"
  DRV_FORWARD               = "8"
  DRV_ARC_RIGHT             = "9"
  DRV_ROT_LEFT              = "4"
  DRV_STOP                  = "5"
  DRV_ROT_RIGHT             = "6"
  DRV_REV                   = "2"
  
PUB man_drive | calc,rx_byte,rx_last,left_vl, right_vl,volts, volts_last
  ''Manually drive the scribbler
  
  sio.tx(cls)
  print_title(DRV_COL_START,DRV_COL_END,-1,string("Manual Drive"))
  print_str(LEFT_MARGIN,MOT_ROW,string("Motor Supply"))
  print_str(constant(ENC_COL+7),(MOT_ROW +1),string("mA"))
  print_str(LEFT_MARGIN,constant(MOT_ROW+2),string("Left"))
  print_str(constant(ENC_COL+7),constant(MOT_ROW+2),string("RPM"))  
  print_str(LEFT_MARGIN,constant(MOT_ROW+3),string("Right"))
  print_str(constant(ENC_COL+7),constant(MOT_ROW+3),string("RPM"))
  print_str(LEFT_MARGIN,MOT_MENU_ROW,string("Drive Commands:"))
  print_str(DRV_COLA,constant(MOT_MENU_ROW+2),string("(7) Arc Left"))
  print_str(DRV_COLB,constant(MOT_MENU_ROW+2),string("(8) Forward"))
  print_str(DRV_COLC,constant(MOT_MENU_ROW+2),string("(9) Arc Right"))  
  print_str(DRV_COLA,constant(MOT_MENU_ROW+3),string("(4) Rot Left"))
  print_str(DRV_COLB,constant(MOT_MENU_ROW+3),string("(5) Stop"))
  print_str(DRV_COLC,constant(MOT_MENU_ROW+3),string("(6) Rot Right"))
  print_str(DRV_COLB,constant(MOT_MENU_ROW+4),string("(2) Reverse"))
  print_str(DRV_COLC,constant(MOT_MENU_ROW+5),string("(B) Beep"))
  print_str(DRV_COLC,constant(MOT_MENU_ROW+6),string("(X) Exit"))
  print_str(DRV_COLC,MOT_MENU_ROW,string("[Num Lock On]"))
  scribbler.set_speed (15)
  periodic_start
  repeat
    rx_byte := sio.rxcheck
    if rx_byte <> -1
      if rx_byte == "b" or rx_byte == "B"
        scribbler.beep
      elseif rx_byte == "x" or rx_byte == "X"
        menu_print        
      elseif rx_byte == DRV_ROT_LEFT
        scribbler.turn_deg_now (DEG_ROTATE)
      elseif rx_byte == DRV_ROT_RIGHT
        scribbler.turn_deg_now (constant(-DEG_ROTATE))
      else   
        case rx_byte
          DRV_ARC_LEFT:
            right_vl := VEL_ARC
            left_vl :=  VEL_ARC_SLOW
          DRV_FORWARD:
            if rx_last == DRV_FORWARD
              left_vl := VEL_MAX
            else 
              left_vl := VEL_STRAIGHT 
            right_vl := left_vl 
          DRV_ARC_RIGHT:
            left_vl := VEL_ARC
            right_vl :=  VEL_ARC_SLOW
          DRV_STOP:
            left_vl:=  0
            right_vl:= 0 
          DRV_REV:
            if rx_last == DRV_REV
              left_vl := constant(-VEL_MAX)
            else 
              left_vl := constant(-VEL_STRAIGHT)
            right_vl := left_vl
          OTHER:
            right_vl := left_vl := 0       
        scribbler.wheels_now(left_vl,right_vl,30_000)
        scribbler.delay_tenths(1)
        rx_last := rx_byte     



    'Print motor supply voltage
    if scribbler.get_model_s2
      volts := scribbler.get_adc_results(scribbler#ADC_VBAT) * 4 
    else
      volts := (scribbler.get_adc_results(scribbler#ADC_VMOT) * (100+33)) / 33 

    if volts <> volts_last
      print_volts_3v3(constant(ENC_COL+3),MOT_ROW,volts)
      volts_last := volts  


    if scribbler.moving
    'Print total motor supply current
      calc := (scribbler.get_adc_results(scribbler#ADC_IMOT) * 33000) / 5865
      if calc > 1200
        sio.tx(CLS)
        print_str(3,3,string("*** Motor Over Current ***"))
        print_str(7,4,string("*** Cycle Power ***"))
        repeat 
      else
        print_dec_word(ENC_COL,constant(MOT_ROW + 1),calc)
                          
      'Print Speed
      print_dec_word(ENC_COL,constant(MOT_ROW+2),((scribbler.motion ~> 24) * 30_000)/50_743)
      print_dec_word(ENC_COL,constant(MOT_ROW+3),(((scribbler.motion << 8) ~> 24)* 30_000)/50_743)

    else
      'Zero speeds and current if not moving
      print_dec_word(ENC_COL,constant(MOT_ROW+1),0)     
      print_dec_word(ENC_COL,constant(MOT_ROW+2),0)
      print_dec_word(ENC_COL,constant(MOT_ROW+3),0)
          
    
    periodic(4)

CON
  'Light Graph Screen Layout

  GRAPH_COL_START           = 0
  GRAPH_COL_END             = GRAPH_SAMPLES + GRAPH_COL_START + 13  
  GRAPH_COL_MID             = (GRAPH_COL_END - GRAPH_COL_START)/2 + 3
  
  GRAPH_ROW_LEFT            = 0
  GRAPH_ROW_CENTER          = GRAPH_ROW_LEFT + 12
  GRAPH_ROW_RIGHT           = GRAPH_ROW_CENTER + 12
  GRAPH_ROW_END             = GRAPH_ROW_RIGHT + 15
  GRAPH_DEG                 = 60                                            'Test Angle (deg)
  GRAPH_STEP                = 3                                             'Angular Resolution (deg)
  GRAPH_SAMPLES             = 2 * GRAPH_DEG / GRAPH_STEP + 1

PUB light_graph_raw | deg,index,left[GRAPH_SAMPLES],center[GRAPH_SAMPLES],right[GRAPH_SAMPLES]
  ''Measure the scribbler's light sensor radial sensitivity
  ''Rotate the robot degrees measuring the light sensor output at each angle.
  ''The results are printed to the serial terminal as a graph

  sio.tx(CLS)
  print_title(GRAPH_COL_START,GRAPH_COL_END,GRAPH_ROW_END,string("Scribbler Light Sensor Raw Values vs Angle"))
  print_str(GRAPH_COL_START,TOP_MARGIN,string("Test Angle: ")) 
  print_str(constant(GRAPH_COL_START+16),TOP_MARGIN,string("deg")) 
  print_str(GRAPH_COL_START,constant(TOP_MARGIN+1),string("Left: "))
  print_str(GRAPH_COL_START,constant(TOP_MARGIN+2),string("Center: "))  
  print_str(GRAPH_COL_START,constant(TOP_MARGIN+3),string("Right: "))
   scribbler.set_speed(2)
  scribbler.heading_is_deg(0)
    
  'Rotate through test angle and take a light reading at each step 
  index := 0
  repeat deg from GRAPH_DEG to -GRAPH_DEG step GRAPH_STEP
    scribbler.turn_to_deg(deg)     
    scribbler.delay_tenths(10)
    scribbler.wait_stop
    print_dec_byte(constant(GRAPH_COL_START+11),TOP_MARGIN,deg)   
    print_dec_word(constant(GRAPH_COL_START+9),constant(TOP_MARGIN+1),left[index] := scribbler.light_sensor_raw(scribbler#LEFT))     
    print_dec_word(constant(GRAPH_COL_START+9),constant(TOP_MARGIN+2),center[index] := scribbler.light_sensor_raw(scribbler#CENTER))
    print_dec_word(constant(GRAPH_COL_START+9),constant(TOP_MARGIN+3),right[index] := scribbler.light_sensor_raw(scribbler#RIGHT))           
    exit_check
    index += 1

  'Graph the results
  sio.tx(CLS)
  graph_init(GRAPH_COL_START,GRAPH_ROW_LEFT,GRAPH_DEG,4000,-GRAPH_DEG,0,GRAPH_SAMPLES,8,-1,string("Left Light Sensor"))
  repeat index from 0 to GRAPH_SAMPLES - 1
    graph_line(index,left[index],"*")
  graph_init(GRAPH_COL_START,GRAPH_ROW_CENTER,GRAPH_DEG,4000,-GRAPH_DEG,0,GRAPH_SAMPLES,8,-1,string("Center Light Sensor"))
  repeat index from 0 to GRAPH_SAMPLES - 1
    graph_line(index,center[index],"*")
  graph_init(GRAPH_COL_START,GRAPH_ROW_RIGHT,GRAPH_DEG,4000,-GRAPH_DEG,0,GRAPH_SAMPLES,8,string("deg"),string("Right Light Sensor"))
  repeat index from 0 to GRAPH_SAMPLES - 1
    graph_line(index,right[index],"*")
  print_str(constant(GRAPH_COL_START+8),constant(GRAPH_ROW_END-1),string("Counterclockwise"))
  print_str(constant(GRAPH_COL_END-11),constant(GRAPH_ROW_END-1),string("Clockwise"))
  exit_print(GRAPH_COL_MID,GRAPH_ROW_END)

  scribbler.turn_to_deg(0)
  scribbler.delay_tenths(5)  
  scribbler.wait_stop
  scribbler.beep
  
  repeat
    exit_check

PUB led_press | counter
  ''Sequence through every possible LED state with each button press.

  sio.tx(cls)
  print_title(TEST_COL_START,TEST_COL_END,constant(TOP_MARGIN+6),string("LED Test"))
  print_center(constant(TOP_MARGIN+1),string("Press the button to cycle"))
  print_center(constant(TOP_MARGIN+2),string("through each LED state."))
  
  scribbler.button_mode(FALSE,FALSE) 
  counter := 0
  scribbler.set_led(scribbler#POWER,scribbler#OFF)
  repeat
    scribbler.set_led(scribbler#RIGHT,lookupz(counter: scribbler#ALT_RED_GREEN,scribbler#BLINK_GREEN,scribbler#BLINK_RED,scribbler#DIM_GREEN,scribbler#DIM_RED,scribbler#GREEN,scribbler#CHARTREUSE,scribbler#YELLOW,scribbler#ORANGE,scribbler#RED)) 
    scribbler.set_led(scribbler#CENTER,lookupz(counter: scribbler#ALT_RED_GREEN,scribbler#BLINK_GREEN,scribbler#BLINK_RED,scribbler#DIM_GREEN,scribbler#DIM_RED,scribbler#GREEN,scribbler#CHARTREUSE,scribbler#YELLOW,scribbler#ORANGE,scribbler#RED)) 
    scribbler.set_led(scribbler#LEFT,lookupz(counter: scribbler#ALT_RED_GREEN,scribbler#BLINK_GREEN,scribbler#BLINK_RED,scribbler#DIM_GREEN,scribbler#DIM_RED,scribbler#GREEN,scribbler#CHARTREUSE,scribbler#YELLOW,scribbler#ORANGE,scribbler#RED)) 
    scribbler.delay_tenths(10)
    button_wait(1)
    counter += 1
    if counter => 10
      scribbler.set_leds(scribbler#OFF,scribbler#OFF,scribbler#OFF,scribbler#DIM_BLUE)
      button_wait(1)
      scribbler.set_led(scribbler#POWER,scribbler#BLINK_BLUE)
      button_wait(1)
      scribbler.set_led(scribbler#POWER,scribbler#BLUE)
      button_wait(1)
      scribbler.set_led(scribbler#POWER,scribbler#OFF)
      counter := 0

CON
  'Propeller Info Screen Layout  
  PROP_COL_START            = 1
  PROP_COL_END              = 52
  PROP_COL_MID              = (PROP_COL_END-PROP_COL_START)/2
  PROP_COLA                 = LEFT_MARGIN
  PROP_COLB                 = PROP_COLA + 26
  PROP_ROW                  = TOP_MARGIN
  PROP_ROW_END              = TOP_MARGIN + 6

PUB prop_print | calc,index
  ''Display the current Propeller Cog Usage
  sio.tx(cls)
  print_title(PROP_COL_START,PROP_COL_END,PROP_ROW_END,string("Propeller Cog Utilization")) 

  calc := cog_check
  print_str(PROP_COLA,constant(PROP_ROW+2),string("Cog #  "))
  repeat index from 0 to 7
    sio.dec(index)
    sio.tx(" ")
  cursor_col(PROP_COLB)
  sio.str(string("Freq   : "))
  sio.dec(CLKFREQ / 1_000_000)
  sio.str(string(" MHz"))    
  print_str(PROP_COLA,constant(PROP_ROW+4),string("Free   "))
  repeat index from 0 to 7
    if (calc & (1 << index))
      sio.tx("O")
    else
      sio.tx(" ")
    sio.tx(" ")
  print_str(PROP_COLA,constant(PROP_ROW+3),string("Busy   "))
  repeat index from 0 to 7
    if (calc & (1 << index))
      sio.tx(" ")
    else
      sio.tx("X")
    sio.tx(" ")

  repeat
    exit_check

PUB cog_check : cog_status | cog_index, cog_list [8]
  ''Check which Propeller Cogs are free
  ''Set a corresponding bit in cog_status return if the cog is free 

  repeat cog_index from 0 to 7
    cog_list[cog_index] := cognew(@busy, 0)
    if cog_list[cog_index] == -1
      quit
    cog_status := cog_status | (1 << cog_list[cog_index])

  repeat while cog_index > 0
    cog_index -= 1
    cogstop(cog_list[cog_index])


PUB start_encoder_count
  'Start counting positive edges from the encoder inputs.
  ctra := %01010 << 26 | scribbler#MOT_LEFT_ENC
  ctrb := %01010 << 26 | scribbler#MOT_RIGHT_ENC
  frqa := 1
  frqb := 1
   
CON
  'Speaker Utility Screen Layout
  notES_ROW                 = TOP_MARGIN
  notES_ROW_END             = notES_ROW + 18

  notES_COL_START           = 1
  notES_COL_END             = 40
  notES_COL_MID             = (notES_COL_END - notES_COL_START)/2 
  notES_COLA                = LEFT_MARGIN
  notES_COLB                = notES_COLA + 17 

  'Invididual Note Octave Limits
  BOT_OCTAVE                = 2    
  TOP_OCTAVE                = 6             
  'Sweep Octave Limits
  SWEEP_BOT                 = 2
  SWEEP_TOP                 = 6
  
PUB music_test | index,octave,length
  ''Speaker Test Utility
   
  sio.tx(cls)
  octave := 3
  length := 500
    
  'Setup sound output
  scribbler.set_voices(scribbler#SQU,scribbler#SQU)
  scribbler.set_volume(100)

  periodic_start
  repeat     'Print the Speaker Utility Menu Screen
    print_title(notES_COL_START,notES_COL_END,-1,string("Speaker Test")) 
    print_str(LEFT_MARGIN,notES_ROW,string("Play Notes:")) 

    cursor(notES_COLA, constant(notES_ROW+7)+ index/2)  
    print_str(notES_COLA,constant(notES_ROW+7),string("Note Length: "))
    sio.dec(length)
    sio.str (string("mS")) 
    print_str(notES_COLA,constant(notES_ROW+9),string("(S) Sweep All Notes"))
    print_str(notES_COLA,constant(notES_ROW+10),string("(M) Sweep All Freq - Step 10 Hz")) 
    print_str(notES_COLA,constant(notES_ROW+11),string("(>) Increase Scale"))
    print_str(notES_COLA,constant(notES_ROW+12),string("(<) Decrease Scale"))
    print_str(notES_COLA,constant(notES_ROW+13),string("(+) Increase Note Length"))
    print_str(notES_COLA,constant(notES_ROW+14),string("(-) Decrease Note Length"))
    print_str(notES_COLA,constant(notES_ROW+15),string("(X) Exit"))
        
    repeat index from 0 to 6
      if (index//2 == 0)
        cursor(notES_COLA, constant(notES_ROW+2)+ index/2)
      else
        cursor(notES_COLB, constant(notES_ROW+2)+ index/2) 
      sio.tx("(")
      sio.tx("A" + index)
      sio.tx(")")
      sio.str(string(" - "))
      sio.dec(music.freq(lookupz(index:music#A2,music#B2,music#C2,music#D2,music#E2,music#F2,music#G2) + (octave - BOT_OCTAVE) * 12))
      sio.str(string(" Hz   "))

    'Check for Rx 
    case sio.rxcheck
      "a", "A" : scribbler.play_tone(length,music.freq(music#A2 + (octave - BOT_OCTAVE) * 12),0) 
      "b", "B" : scribbler.play_tone(length,music.freq(music#B2 + (octave - BOT_OCTAVE) * 12),0)
      "c", "C" : scribbler.play_tone(length,music.freq(music#C2 + (octave - BOT_OCTAVE) * 12),0)
      "d", "D" : scribbler.play_tone(length,music.freq(music#D2 + (octave - BOT_OCTAVE) * 12),0)
      "e", "E" : scribbler.play_tone(length,music.freq(music#E2 + (octave - BOT_OCTAVE) * 12),0)
      "f", "F" : scribbler.play_tone(length,music.freq(music#F2 + (octave - BOT_OCTAVE) * 12),0)
      "g", "G" : scribbler.play_tone(length,music.freq(music#G2 + (octave - BOT_OCTAVE) * 12),0)
      "s", "S" : sweep
      "m","M"  : sweep_max_check
      ".",">"  : if (octave < TOP_OCTAVE)
                   octave += 1
      "<",","  : if (octave > BOT_OCTAVE)
                   octave -= 1
      "=","+"  : if (length < 5000)
                   length += 100
      "-","_"  : if (length > 200)
                   length -= 100
     "x","X"  : menu_print                             
    periodic(2)         


PUB sweep | octave,index,note_index
  ''Play every note one at a time
  
  repeat octave from constant((SWEEP_BOT - BOT_OCTAVE)) to constant(SWEEP_TOP- BOT_OCTAVE)
    repeat index from 0 to 6
      cursor(notES_COLA, constant(notES_ROW_END-1)) 
      sio.str(string("Sweep Freq: "))
      note_index := lookupz(index:music#A2,music#B2,music#C2,music#D2,music#E2,music#F2,music#G2)
      sio.tx("A" + index)
      sio.dec(octave + BOT_OCTAVE)
      sio.str(string(" - ")) 
      sio.dec(music.freq(octave * 12 + note_index))
      sio.str(string(" Hz"))
      scribbler.play_tone(2000,music.freq(octave * 12 + note_index),0)
      scribbler.delay_tenths(21)
      exit_check 
       
  print_str(notES_COLA,constant(notES_ROW_END-2),string("Sweep Complete      ")) 

PUB sweep_max_check | index
  ''Play every frequency one at a time
  
  repeat index from 100 to 2500 step 10
    cursor(notES_COLA, constant(notES_ROW_END-1)) 
    sio.str(string("Sweep Freq: "))
    sio.dec(index)
    sio.str(string(" Hz"))
    scribbler.play_tone(500,index,0)
    scribbler.delay_tenths(6)
    exit_check
    
  print_str(notES_COLA,constant(notES_ROW_END-2),string("Sweep Complete      "))  

PUB idler_beep | idler_cnt,idler_last,loop_cnt
  ''Idler Wheel Check
  ''
  ''Beep as the idler wheel is rotated.

  periodic_start
  idler_last :=  scribbler.get_results(scribbler#CNT_IDLER)
  loop_cnt := 0
  repeat
    idler_cnt := scribbler.get_results(scribbler#CNT_IDLER)         
    if idler_last <> idler_cnt                          'Beep if idler wheel encoder count has changed
      music.play_note(music#SXTH,music#G4,0)
      idler_last :=  idler_cnt
    periodic(IDLER_LOOP_FREQ)

 
PUB periodic_start
  ''Initialize the periodic function
  
  last_call := CNT

PUB periodic (freq) | calc
  ''Pause for a fixed amount since the function was last called.
  ''Sets up a periodic timer - freq = 1 to 30 Hz.

  waitcnt(last_call + clkfreq / freq)
  last_call := CNT  

PUB exit_check
  ''Exit to main menu if any key has been pressed

  if sio.rxcheck <> -1
    scribbler.start 
    scribbler.start_motors
    scribbler.start_tones 
    scribbler.delay_tenths(10)
    menu_print 

PUB exit_print (mid_col,row)
  ''Print the Press any Key to Exit Message
  ''mid_col is the middle column position for the message
  ''row is the row of the message
  
  print_str((mid_col-12)#>1,row,string("(Press Any Key to Exit)"))  
  
PUB print_box (col_start,row_start,col_end,row_end,char) | row
  ''Print a rectangular box on the debug terminal screen
  ''Can also be used to print horizontal or vertical lines
  ''char is the character to use for the box

  if col_start =< col_end
    cursor(col_start,row_start)
    repeat (col_end - col_start + 1)
      sio.tx(char)
    cursor(col_start,row_end)
    repeat (col_end - col_start + 1)
      sio.tx(char)
  if row_start < row_end 
    repeat row from (row_start + 1) to (row_end - 1)
      cursor(col_start,row)
      sio.tx(char) 
    repeat row from (row_start + 1) to (row_end - 1)
      cursor(col_end,row)
      sio.tx(char)



PUB print_center(row,str)
  'Print a text string in the in the middle of screen
  print_str(TEST_COL_MID - strsize(str)/2,row,str)  
  
       
PUB print_title (col_start,col_end,row_end,titlestr) : mid_col
  'Print the Title Block for the Test Screens
  'col_start - Starting Column
  'col_end - End Column
  'end_row - Last Row
  'titlestr - Title String
  'If end_row is > 0,print the Press Any Key to Exist message.

  mid_col := (col_end - col_start) / 2 + col_start 
  print_box(col_start,0,col_end,2,".")
  print_str(mid_col - strsize(titlestr)/2,1,titlestr)
  if row_end > 0
    print_str((mid_col-11)#>1,row_end,string("(Press Any Key to Exit)"))

VAR     'Graph Variables 
  byte graph_col,graph_row,graph_x_points,graph_y_points
  long graph_x_max,graph_x_min,graph_y_max,graph_y_min

PUB graph_init(col,row,x_max,y_max,x_min,y_min,x_pts,y_pts,x_label,y_label) |index,calc,step_size
  ''Setup an X,Y data graph to be printed to the serial terminal
  ''Data points are added with graph_point() for discrete points
  ''or graph_line() for linear data sequences.
  ''
  ''col,row - graph upper left position
  ''x_max,y_max,x_min,y_min - Value limits for each axis 
  ''x_pts,y_pts - qty of data points for each axis
  ''
  ''Example: graph_init(4,4,200,500,-200,-500,40,20,string("RPM"),string("Volt")) 

  graph_col := col + 9
  graph_row := row + 2
  graph_x_max := x_max
  graph_x_min := x_min
  graph_x_points := x_pts
  graph_y_max := y_max
  graph_y_min := y_min
  graph_y_points := y_pts

  'Clear the graph area
  repeat index from row to graph_row+y_pts+3
    clear_row(index)

  'Print the graph outline  
  print_box(graph_col-1,graph_row-1,graph_col+x_pts+1,graph_row+y_pts+1,".")
  
  'Print the y-axis numeric labels 
  'Set the number of rows between the y-axis labels based on the number of points
  if y_pts // 10 == 0 AND y_pts => 40
    step_size := 10
  elseif y_pts // 8 == 0 AND y_pts => 24
     step_size := 8
  elseif y_pts // 5 == 0
    step_size := 5
  elseif y_pts // 4 == 0 AND y_pts => 16 
    step_size := 4
  elseif y_pts // 3 == 0  
    step_size := 3
  else
    step_size := 2
  'Print y axis
  print_str(col+1,row,y_label)      
  calc := y_min
  repeat index from graph_row + y_pts to graph_row step step_size
    print_dec_word(col,index,calc)
    if y_min < 0  'Account for zero axis line 
      calc := calc + ((y_max-y_min) * step_size) / (y_pts -1)
    else
      calc := calc + ((y_max-y_min) * step_size) / y_pts  
  'Print the x-axis if a x-axis label exists
  if x_label <> -1  
    if ||x_min < 100 AND x_max < 100 AND x_pts =< 20
      step_size := 5
    else
      step_size := 10
    calc := x_min 
    repeat index from graph_col to (graph_col + x_pts) step step_size
      print_dec(index,graph_row+y_pts+3,calc)
      if x_min < 0   
        calc := calc + ((x_max-x_min) * step_size) / (x_pts - 1) 
      else
        calc := calc + ((x_max-x_min) * step_size) / x_pts
    sio.tx(" ")
    sio.str(x_label)

  'Print zero lines if required
  if (x_min < 0)
   calc := graph_col + (||graph_x_min*graph_x_points)/(graph_x_max-graph_x_min)
   print_box(calc,graph_row,calc,graph_row+graph_y_points,"|")     
  if (y_min < 0)
    calc := graph_row + (graph_y_max*graph_y_points)/(graph_y_max-graph_y_min)
    print_box(graph_col,calc,graph_col+graph_x_points,calc,"-") 

   
PUB graph_point(x,y,char)
  ''Add a data point to the previously defined graph at position x,y
  ''Example:  graph_point(-100,250,"*")

  x := (x <# graph_x_max) #> graph_x_min
  y := (y <# graph_y_max) #> graph_y_min

  sio.tx(CXY)
  sio.tx(graph_col + (||graph_x_min*graph_x_points)/(graph_x_max-graph_x_min) + (x*graph_x_points)/(graph_x_max-graph_x_min)) 
  sio.tx(graph_row + graph_y_points - (||graph_y_min*graph_y_points)/(graph_y_max-graph_y_min) - (y*graph_y_points)/(graph_y_max-graph_y_min)) 
  sio.tx(char)

PUB graph_line(point_num,value,char)
  ''Add a data point to the previously defined graph
  ''point_num is the sequence number for the point (1 to y_points)
  ''value is the magnitude of the point
  ''
  ''Example:  graph_line(1,100,"*")
  ''          graph_line(2,200,"*")  
  ''          graph_line(3,300,"*")

  value := (value <# graph_y_max) #> graph_y_min
  point_num := (point_num <# graph_x_points) #> 0
  
  sio.tx(CXY)
  sio.tx(graph_col+point_num)
  sio.tx(graph_row + graph_y_points - (||graph_y_min*graph_y_points)/(graph_y_max-graph_y_min) - (value*graph_y_points)/(graph_y_max-graph_y_min)) 
  sio.tx(char)
      
PUB print_volts_3v3(col,row,val)
  ''Print byte sized 3.3V referenced ADC value in volts at the col,row screen coordinates
  
  cursor(col,row) 
  print_100ths((val * 3300) / 255)
  sio.str(string(" V")) 

PUB print_mvolts_3v3(col,row,val)
  ''Print byte sized 3.3V referenced ADC value in mV at the col,row screen coordinates
  
  cursor(col,row) 
  val := val * 3300 / 255
  if val < 1000
    sio.tx(" ")
  if val < 100
    sio.tx(" ")
  if val < 10
    sio.tx(" ")
  sio.dec(val)
  sio.str(string(" mV"))

PUB print_volts_5v0(col,row,val)
  ''Print byte sized 5.0V referenced ADC value in volts
  ''at the col,row screen coordinates on the serial terminal.
    
  cursor(col,row) 
  print_100ths((val * 5000) / 255)
  sio.str(string(" V")) 

PUB print_percentage(col,row,value)
  ''Print signed byte value as right justified percentage
  ''at the col,row screen coordinates on the serial terminal. 
    
  cursor(col,row) 
  value := ((value * 100) / $FF) <# 100 
  if value => 0
    sio.tx(" ")
  if ||value < 100
    sio.tx(" ")
  if ||value < 10
    sio.tx(" ")      
  sio.dec(value) 
  sio.str(string("%"))

PUB print_bar(col,row,val) | bar_cnt
  ''Print byte sized value as a bar graph with 1 to 10 segments
  ''at the col,row screen coordinates on the serial terminal. 
    
  cursor(col,row) 
  val := ((val * 10) / $FF)  <# 9
  repeat bar_cnt from 0 to val
    sio.tx("#")
  repeat bar_cnt from val to 9
    sio.tx(" ")

PUB print_scaled_bar(col,row,val,top) | bar_cnt
  ''Print bar graph with 1 to 10 segments
  ''at the col,row screen coordinates on the serial terminal. 
  ''top is the maximum value to scale graph against
  
  cursor(col,row) 
  val := ((val * 10) / top)  <# 10
  repeat bar_cnt from 0 to val
    sio.tx("#")
  repeat bar_cnt from val to 10
    sio.tx(" ")

PUB print_str (col,row,stringptr)
  ''Print string at the col,row screen coordinates

  cursor(col,row)
  sio.str(stringptr)
    
PUB print_dec(col,row,value)
  ''Print signed value as a left justified decimal number at the col,row debug terminal coordinates  

  cursor(col,row)
  sio.dec(value)
  
PUB print_dec_word (col,row,value)
  ''Print signed word as a right justified decimal number at the col,row debug terminal coordinates
  
  cursor(col,row)
  if value => 0
    sio.tx(" ") 
  if ||value < 10_000
    sio.tx(" ")
  if ||value < 1_000
    sio.tx(" ")
  if ||value < 100
    sio.tx(" ")
  if ||value < 10
    sio.tx(" ")      
  sio.dec(value) 

PUB print_dec_byte (col,row,value) | calc
  ''Print signed byte as a right justified decimal number at the col,row debug terminal coordinates 
  
  cursor(col,row)
  if value => 0
    sio.tx(" ")  
  if ||value < 100
    sio.tx(" ")
  if ||value < 10
    sio.tx(" ")     
  sio.dec(value)

PUB print_hex (col,row,value,digits) 
  ''Print hexadecimal number at the col,row screen coordinates
  
  cursor(col,row) 
  sio.tx("$")
  sio.hex(value,digits)

PUB clear_row(row)
  ''Delete a single row on the serial terminal
  sio.tx(CXY)
  sio.tx(0)
  sio.tx(row #> 0)
  sio.tx(CEL)

PUB clear_rows(first_row,last_row) | row
  ''Clear all rows between first_row and last_row
  repeat row from (first_row #>0) to (last_row #>first_row)
    clear_row(row)
  
PUB cursor (col,row)
  ''Move the cursor to terminal position Row, Col
  sio.tx(CXY)
  sio.tx(col)
  sio.tx(row)

PUB cursor_row (row)
  ''Move the cursor to row
  sio.tx(CY)
  sio.tx(row)

PUB cursor_col (col)
  ''Move the cursor to column
  sio.tx(CX)
  sio.tx(col)

PUB print_10ths(value)

  sio.tx((value < 0) & "-" | (value => 0) & " ")
  ||value
  sio.dec(value / 1000)
  sio.tx(".")
  value //= 1000
  sio.dec(value / 100)

PUB print_100ths(value)

  sio.tx((value < 0) & "-" | (value => 0) & " ")
  ||value
  sio.dec(value / 1000)
  sio.tx(".")
  value //= 1000
  sio.dec(value / 100)
  sio.dec(value / 10 // 10)

PUB modulate_idler | frq0
  ''Modulate the Idler Wheel LED
  ''~2,000 Hz

  frqb := 110_000 
  dira[scribbler#IDLER_TX]~~ 
  ctrb := %00100 << 26 | scribbler#IDLER_TX 
  repeat

PUB button_wait(press) | time_out,debounce_cnt,rx
  ''Wait until at least the specified number of button presses are detected or the
  ''the timer expires before returning.
  ''A 1/2 second plus 1/2 second each button press timeout after the first press 
  ''Returns the number of presses detected before the time out 
  ''Branch to Serial Test Menu if the space bar is pressed
  ''Return the ASCI value of the number key 0 to 9 if pressed
  
  press #>= 1                                     'minimum number of presses must be >= 1
  debounce_cnt := 0
  repeat until debounce_cnt == 3                  'Wait util the button is not pressed
    if scribbler.button_press                            'for at least 300 mS
      debounce_cnt := 0
    else
      debounce_cnt += 1
    case (rx := sio.rxcheck) 
      " " : menu_print
      "0".."9":
        return rx

    scribbler.delay_tenths(1)
    
  scribbler.reset_button_count  
  repeat until scribbler.button_press                    'Check for initial press
    case (rx := sio.rxcheck) 
      " " : menu_print
      "0".."9":
        return rx
  if press == 1
    return 1
    
  time_out := cnt + (CLKFREQ*(press+1))/2    'Set the time out
     
  repeat until (cnt => time_out)                       
    case (rx := sio.rxcheck) 
      " " : menu_print
      "0".."9":
        return rx 
      
  return scribbler.button_count

DAT
''Do Nothing
              org       0
busy          jmp       #busy

CON
  'Serial Terminal Commands
  HOM           =  1  ' Move cursor Home
  CXY           =  2  ' Move cursor to X,Y        
  BEP           =  7  ' Beep the Speaker 
  LF            = 10  ' Line Feed
  CEL           = 11  ' Clear to End of Line 
  NL            = 13  ' New Line          
  CX            = 14  ' Move cursor to X         
  CY            = 15  ' Move cursor to Y
  CLS           = 16  ' Clear Screen
  SP            = 32  ' Space
  DWN           = 6   ' Move cursor Down             
  UP            = 5   ' Move cursor Up

{{
┌──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                                                   TERMS OF USE: MIT License                                                  │                                                            
├──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation    │ 
│files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,    │
│modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software│
│is furnished to do so, subject to the following conditions:                                                                   │
│                                                                                                                              │
│The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.│
│                                                                                                                              │
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT not LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}