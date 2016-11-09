'Modified from S2 include code version 2015.07.08

'---[Constants]----------------------------------------------------------------

CON

  BAR_THLD      = 32
  OBSTACLE_THLD = 20
  SPKR_VOL      = 35

  #0, NONE, LEFT, RIGHT, CENTER, POWER, DETECTED
  #0, IS, IS_NOT, WAS, WAS_NOT
  #0, BLACK, WHITE
  #0, STRAIGHT, SLIGHT_RIGHT, GENTLE_RIGHT, SHARP_RIGHT, SLIGHT_LEFT, GENTLE_LEFT, SHARP_LEFT
  #0, HIGH, LOW, INPUT, OUTPUT, TOGGLE_STATE, TOGGLE_DIRECTION

  COLOR_ff0000    = Scribbler#RED
  COLOR_ff7f00    = $5A
  COLOR_00ff00    = Scribbler#GREEN
  OFF             = $00
  COLOR_000000    = OFF

''    Control Character Constants
''─────────────────────────────────────
  CS = 16  ''CS: Clear Screen
  PC =  2  ''PC: Position Cursor in x,y
  PX = 14  ''PX: Position cursor in X
  PY = 15  ''PY: Position cursor in Y
  NL = 13  ''NL: New Line
  LF = 10  ''LF: Line Feed
  BS =  8  ''BS: BackSpace

'---[Global Variables]---------------------------------------------------------

VAR

  long  pLeftMotor, pRightMotor, pMoveTime, FMStack[50], stack[30], seed
  byte  RunningCog, obs[3], WasObs[3], WasLine[3], WasLight[4], WasRandom, WasStalled, WasButton, WasLeftVelocity, WasRightVelocity

'---[Object Declaration]-------------------------------------------------------

OBJ

  Scribbler         : "scribbler"
  Serial            : "FullDuplexSerial"
  ServoDriver       : "Servo32v7"

'---[Start of Program]---------------------------------------------------------

PUB start

  Scribbler.start
  Scribbler.start_motors
  Scribbler.start_tones
  Scribbler.button_mode(true, true)
  Scribbler.set_volume(SPKR_VOL)
  Scribbler.set_voices(Scribbler#SAW, Scribbler#SAW)
  RunningCog := cogid + 1
  cognew(FaultMonitor, @FMStack)
  cognew(Obstacler, @stack)
  waitcnt(cnt + 10_000_000) 


Pub SerialStart(BaudRate)

  Serial.Start(31, 30, 0, BaudRate)
  waitcnt(80_000_000 + cnt)


Pub ServoStart

  ServoDriver.Start


Pri Obstacler | side, ObstacleThld

  if (Scribbler.get_obstacle_threshold <> Scribbler#DEFAULT_OBSTACLE_THLD)
    ObstacleThld := Scribbler.get_obstacle_threshold
  else
    ObstacleThld := OBSTACLE_THLD 

  repeat
    repeat side from Scribbler#OBS_TX_LEFT to Scribbler#OBS_TX_RIGHT step constant(Scribbler#OBS_TX_RIGHT - Scribbler#OBS_TX_LEFT)
      frqa := 14000 * ObstacleThld + 20607 * (100 - ObstacleThld)
      ctra := %00100 << 26 | side
      dira[side]~~
      waitcnt(cnt + 24000)
      obs[-(side == Scribbler#OBS_TX_RIGHT) + 1] := ina[Scribbler#OBS_RX] == 0
      dira[side]~
      waitcnt(cnt + clkfreq / 8)


'---[Battery and Over-current Monitor Cog]-------------------------------------

Pri FaultMonitor : value

  value := $ffff
  waitcnt(cnt + 80_000_000)
  repeat
    value <#= Scribbler.get_adc_results(Scribbler#ADC_VBAT)
    if value > constant((700*2550)/(400*33))      '7.0V
      Scribbler.set_led(Scribbler#POWER,Scribbler#BLUE)
    elseif value > constant((600*2550)/(400*33))  '6.0V
      Scribbler.set_led(Scribbler#POWER,$20)
    else
      Scribbler.set_led(Scribbler#POWER,Scribbler#BLINK_BLUE)
    if Scribbler.get_adc_results(Scribbler#ADC_IMOT) > 210
      cogstop(RunningCog - 1)
      RunningCog~
      Scribbler.stop_now
      Scribbler.set_leds(Scribbler#BLINK_RED,Scribbler#BLINK_RED,Scribbler#BLINK_RED,Scribbler#OFF)
      repeat

'---[Set Motor Speeds]---------------------------------------------------------

Pub MotorSet(LeftVelocity, RightVelocity, move_time)

  LeftVelocity := -255 #> LeftVelocity * 255 / 100 <# 255
  RightVelocity := -255 #> RightVelocity * 255 / 100 <# 255
  if move_time > 0
    move_time <#= 65_535
  else
    move_time~

  if move_time or WasLeftVelocity <> LeftVelocity or WasRightVelocity <> RightVelocity
  
      Scribbler.wheels_now(LeftVelocity, RightVelocity, move_time)
      WasLeftVelocity := LeftVelocity
      WasRightVelocity := RightVelocity
    
      if move_time
        WasLeftVelocity~
        WasRightVelocity~
        Scribbler.wait_stop
    
      WasLeftVelocity := LeftVelocity
      WasRightVelocity := RightVelocity


Pub MotorSetDistance(left_distance, right_distance, max_speed)

  left_distance := -10_000 #> left_distance <# 10_000 
  right_distance := -10_000 #> right_distance <# 10_000 
  max_speed := 1 #> max_speed * 15 /100 <# 15 

  Scribbler.move_now(left_distance, right_distance, 0, max_speed, 0)
  Scribbler.wait_stop


Pub MotorSetRotate(degrees, radius, max_speed)

  if radius => 0
   -degrees

  degrees := -1_080 #> degrees <# 1_080 
  radius := -4_400 #> radius <# 4_400 
  max_speed := 1 #> max_speed * 15 /100 <# 15 

  Scribbler.set_speed(max_speed)
  Scribbler.arc_deg_now(degrees, -radius)
  Scribbler.wait_stop


Pub MotorsMoving

  return Scribbler.moving


Pub MoveXY(X, Y, MaxSpeed)

  Scribbler.set_speed(1 #> MaxSpeed * 15 /100 <# 15)
  Scribbler.Move_By(-$3F_FF_FF_FF #> X <# $3F_FF_FF_FF, -$3F_FF_FF_FF #> Y <# $3F_FF_FF_FF)
  Scribbler.wait_stop


Pub SimpleDrive(Direction, Speed) | LeftVelocity, RightVelocity

  case Direction
    STRAIGHT:
      LeftVelocity := RightVelocity := Speed
    SLIGHT_RIGHT:
      LeftVelocity := Speed
      RightVelocity := Speed * 3 / 4
    GENTLE_RIGHT:
      LeftVelocity := Speed
      RightVelocity := Speed / 2
    SHARP_RIGHT:
      LeftVelocity := Speed
      RightVelocity := Speed / 4
    SLIGHT_LEFT:
      LeftVelocity := Speed * 3 / 4
      RightVelocity := Speed
    GENTLE_LEFT:
      LeftVelocity := Speed / 2
      RightVelocity := Speed
    SHARP_LEFT:
      LeftVelocity := Speed / 4
      RightVelocity := Speed

  if WasLeftVelocity <> LeftVelocity or WasRightVelocity <> RightVelocity
  
      Scribbler.wheels_now(LeftVelocity, RightVelocity, 0)
      WasLeftVelocity := LeftVelocity
      WasRightVelocity := RightVelocity
    
      WasLeftVelocity := LeftVelocity
      WasRightVelocity := RightVelocity


Pub SimpleSpin(Angle, MaxSpeed, Resume)

  Scribbler.set_speed(1 #> MaxSpeed <# 15)
  Scribbler.arc_deg_now(-1_080 #> -Angle <# 1_080, 0)
  Scribbler.wait_stop

  if Resume
  Scribbler.wheels_now(WasLeftVelocity, WasRightVelocity, 0)


Pub SimpleStop

  Scribbler.wheels_now(0, 0, 0)
  WasLeftVelocity~
  WasRightVelocity~


Pub SetVolume(Volume) 

  Volume := 0 #> Volume <# 100 
  Scribbler.set_volume(volume)      


Pub SetVoices(voice1, voice2)

  Scribbler.set_voices(voice1, voice2)


'---[Play an Individual Note, Separated by a Short Delay.]---------------------

Pub PlayNote(freq1, freq2, duration)

  Scribbler.play_tone(duration - 1 #> 1, freq1, freq2)
  Scribbler.play_tone(1, 0, 0)
  Scribbler.wait_sync(0)


Pub SimplePlay(Frequency, Duration, Volume)

  Volume := 0 #> Volume <# 100 
  Scribbler.set_volume(Volume)      

  Scribbler.play_tone(Duration - 1 #> 1, Frequency, 0)
  Scribbler.play_tone(1, 0, 0)
  Scribbler.wait_sync(0)


'---[Play an Individual Pulse, Followed by a Delay.]---------------------------
{
Pub PlayPulse(on_duration, nil, off_duration) | on_freq
  on_duration := on_duration * 2 + 1380
  on_freq := 1_000_000 / on_duration
  on_duration := on_duration / 700 #> 1
  Scribbler.play_tone(on_duration, on_freq, 0)
  if (off_duration)
    Scribbler.play_tone(off_duration, 0, 0)
'---[Read Bar Codes]-----------------------------------------------------------
}
Pub ReadBars | w0, w1, barcount, midwidth, t

  if (Scribbler.line_sensor(Scribbler#LEFT, BAR_THLD) or Scribbler.line_sensor(Scribbler#RIGHT, BAR_THLD))
    return
  t := cnt
  repeat until Scribbler.line_sensor(Scribbler#RIGHT, BAR_THLD)
    if (cnt - t > 80_000_000)
        return
  ifnot (w1 := ReadBarWidth)
    return
  ifnot (w0 := ReadBarWidth)
    return
  midwidth := (w0 + w1) >> 1
  repeat barcount from 1 to 5
    if (barcount =< 4)
      result <<= 1
      if (w0 > midwidth)
        result |= 1
    ifnot (w0 := ReadBarWidth)
      result~
      return
  if (result & %1000)
    result := (result >< 3) | %1000

'---[Read the Width of One Dark Bar Code Bar]----------------------------------

PRI ReadBarWidth | t

  t := cnt
  repeat while Scribbler.line_sensor(Scribbler#RIGHT, BAR_THLD)
    if ((cnt - t) > 80_000_000)
      return 0
  t := cnt
  repeat until Scribbler.line_sensor(Scribbler#RIGHT, BAR_THLD) 
    if ((cnt - t) > 80_000_000)
      return 0
  return cnt - t

'---[Read Light Sensors]-------------------------------------------------------

Pub ReadObstacle(Side)

  if Side == Scribbler#LEFT or Side == Scribbler#Right 
    return obs[Side]


Pub SimpleObstacle(Condition, Location)

  if Condition == IS OR Condition == IS_NOT
      WasObs[LEFT] := Obs[LEFT]
      WasObs[RIGHT] := Obs[RIGHT]
  if Condition == IS OR Condition == WAS
    result := TRUE

  case Location
    CENTER:
      if WasObs[LEFT] AND WasObs[RIGHT]
        return
    LEFT:
      if WasObs[LEFT] AND not WasObs[RIGHT]
        return
    RIGHT:
      if not WasObs[LEFT] AND WasObs[RIGHT]
        return
    DETECTED:
      if WasObs[LEFT] or WasObs[RIGHT]
        return

  not result
  


Pub LineSensor(Side)

  if Side == LEFT or Side == RIGHT
    return Scribbler.line_sensor(Side, -1)


Pub SimpleLine(Condition, Location, Color) | Position

  ifnot RunningCog
    start

  Position~

  if Condition == IS OR Condition == IS_NOT
      WasLine[LEFT] := Scribbler.line_sensor(LEFT, -1)
      WasLine[RIGHT] := Scribbler.line_sensor(RIGHT, -1)
  if Condition == IS OR Condition == WAS
    result := TRUE

  if ||(WasLine[LEFT] - WasLine[RIGHT]) < 30          ' Low difference, not on an edge
    if WasLine[LEFT] + WasLine[RIGHT] < 60            ' Average reading is dark
      if Color == BLACK AND (Location == CENTER OR Location == DETECTED)
        return
    elseif Color == WHITE AND (Location == CENTER OR Location == DETECTED) ' Average reading is light
      return
  else                                                ' Over an edge
    if Location == DETECTED
      return
    elseif (WasLine[LEFT] > WasLine[RIGHT]) AND ((Location == LEFT AND Color == BLACK) OR (Location == RIGHT AND Color == WHITE)) ' Left is brighter
      return
    elseif (WasLine[RIGHT] > WasLine[LEFT]) AND ((Location == LEFT AND Color == WHITE) OR (Location == RIGHT AND Color == BLACK)) ' Right is brighter
      return

  not result


Pub LightSensor(Side)

  if Side == Scribbler#LEFT or Side == Scribbler#CENTER or Side == Scribbler#RIGHT 
    return Scribbler.light_sensor(Side)  * 100 / 256


Pub SimpleLight(Condition, Location)

  if Condition == IS OR Condition == IS_NOT
      WasLight[LEFT] := Scribbler.light_sensor(LEFT)
      WasLight[RIGHT] := Scribbler.light_sensor(RIGHT)
      WasLight[CENTER] := Scribbler.light_sensor(CENTER)
  if Condition == IS OR Condition == WAS
    result := TRUE

  if (WasLight[LEFT] > WasLight[RIGHT] + 50 AND WasLight[LEFT] > WasLight[CENTER] + 50) OR (WasLight[LEFT] > WasLight[RIGHT] * 3 / 2 AND WasLight[LEFT] > WasLight[CENTER] * 3 / 2)
    if Location == LEFT
      return
  elseif (WasLight[RIGHT] > WasLight[LEFT] + 50 AND WasLight[RIGHT] > WasLight[CENTER] + 50) OR (WasLight[RIGHT] > WasLight[LEFT] * 3 / 2 AND WasLight[RIGHT] > WasLight[CENTER] * 3 / 2)
    if Location == RIGHT
      return
  elseif (WasLight[CENTER] > WasLight[LEFT] + 50 AND WasLight[CENTER] > WasLight[RIGHT] + 50) OR (WasLight[CENTER] > WasLight[LEFT] * 3 / 2 AND WasLight[CENTER] > WasLight[RIGHT] * 3 / 2)
    if Location == CENTER
      return
  elseif Location == DETECTED AND WasLight[LEFT] + WasLight[CENTER] + WasLight[RIGHT] > 50 
      return

  not result


Pub SetLED(LED, Color)

  if LED == Scribbler#LEFT or LED == Scribbler#CENTER or LED == Scribbler#RIGHT 
    case Color
      COLOR_ff0000, COLOR_00ff00, COLOR_ff7f00, OFF:
        Scribbler.set_led(LED, Color)


Pub Stalled

  return Scribbler.stalled


Pub SimpleStalled(Condition)

  case Condition
    IS:
      WasStalled := Scribbler.stalled
      return WasStalled
    IS_NOT:
      WasStalled := Scribbler.stalled
      return not WasStalled
    WAS:
      return WasStalled
    WAS_NOT:
      return not WasStalled


Pub SimpleButton(Condition)

  case Condition
    IS:
      WasButton := Scribbler.button_press
      return WasButton
    IS_NOT:
      WasButton := Scribbler.button_press
      return not WasButton
    WAS:
      return WasButton
    WAS_NOT:
      return not WasButton


Pub SimpleRandom(Condition)

  case Condition
    IS:
      WasRandom := BooleanRandom
      return WasRandom
    IS_NOT:
      WasRandom := BooleanRandom
      return not WasRandom
    WAS:
      return WasRandom
    WAS_NOT:
      return not WasRandom


Pub BooleanRandom

  if Seed? & 1
    return TRUE


Pub RandomRange(A, B) | Higher, Lower, Range

  ' Ser High and Low to their repective numbers
  if A < B
    Lower := A
    Higher := B
  elseif A == B ' return if the range is zero
    return A
  else{if A > B}
    Lower := B
    Higher := A

  ' Calculate the range
  Range := Higher - Lower
  ' and return 0 if the range is too large to calculate
  if Range < 0
    return 0
  elseif range == posx
    return Lower + (Seed? & posx)
    
  return Lower + (Seed? & posx) / (posx / (Range + 1))


Pub ButtonCount

  return Scribbler.button_count


Pub ButtonPressed

  return Scribbler.button_press


Pub ResetButtonCount

  return Scribbler.reset_button_count


Pub RunWithoutResult(null)

  return


Pub SerialStr(StringPointer)

  Serial.Str(StringPointer)


Pub SerialDec(Number)

  Serial.Dec(Number)


Pub SerialChar(Character)

  Serial.Tx(Character)


Pub SerialPositionX(Position)

  Serial.Tx(PX)
  Serial.Tx(Position)


Pub SerialPositionY(Position)

  Serial.Tx(PY)
  Serial.Tx(Position)


Pub SerialCharIn

  return Serial.RxCheck


Pub Ping(Pin) | MaxLoops, StartCnt, EndCnt

  ifnot 0 =< Pin and Pin =< 5
    return 0
  outa[Pin]~~
  dira[Pin]~~
  waitcnt(clkfreq / 200_000 + cnt)
  outa[Pin]~
  dira[Pin]~

  MaxLoops := 80
  repeat while --MaxLoops and not ina[Pin]
  StartCnt := cnt
  ifnot MaxLoops
    return 0
  
  MaxLoops := 2_000
  repeat while --MaxLoops and ina[Pin]
  EndCnt := cnt
  ifnot MaxLoops
    return 0

  waitcnt(16000 + cnt)

  result := EndCnt - StartCnt
  
  if result < 9200 or result > 1480000
    return 0


Pub Servo(Pin, Angle)

  ifnot 0 =< Pin and Pin =< 5
    return 0

  Angle := (0 #> Angle <# 180) * 2_000 / 180 + 500
  ServoDriver.Set(Pin, Angle)


Pub ServoStop(Pin)

  ServoDriver.Set(Pin, 0)


Pub ADC(Pin)

  case Pin
    0:
      return Scribbler.get_results(Scribbler#ADC_P6)
    1:
      return Scribbler.get_results(Scribbler#ADC_P7)


Pub DigitalInput(Pin)

  ifnot 0 =< Pin and Pin =< 5
    return 0

  ServoDriver.Set(Pin, 0)
  dira[Pin]~
  return ina[Pin]


Pub DigitalOutput(Pin, Action)

  ifnot 0 =< Pin and Pin =< 5
    return

  ServoDriver.Set(Pin, 0)

  case Action
    HIGH:
      outa[Pin]~~
      dira[Pin]~~
    LOW:
      outa[Pin]~
      dira[Pin]~~
    INPUT:
      dira[Pin]~
    OUTPUT:
      dira[Pin]~~
    TOGGLE_STATE:
      !outa[Pin]
      dira[Pin]~~
    TOGGLE_DIRECTION:
      !dira[Pin]

