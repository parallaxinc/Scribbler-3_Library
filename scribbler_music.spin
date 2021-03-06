''*********************************************
''*  Scribbler Music & Songs                  *
''*  Author: Ben Wirz, Element Products, Inc. *
''*  Copyright (c) 2010 Parallax, Inc.        *  
''*  See end of file for terms of use.        *
''********************************************* 
  
OBJ

  scribbler   : "scribbler" 

 
CON

   VERSION       = 2                                'Release Version Number
   SUBVERSION    = 2528                             'Internal Version Number

  'Song Numbers

  #0,ENTERTAINER,LOST,OBSTACLE,LIGHT,MIC,LINESENS,REVEILLE,CHARGE,LAC,EXCLAIM,TAPS,AMBULANCE

PUB play_song (song_num,tempo)
  ''Play one of the predefined songs
  ''song_num - the song number 
  ''tempo - length of a Whole Note in mS
  ''Example: music.load_song(music.CHICKEN_DANCE,1600)

  case song_num
    ENTERTAINER:    load_song(@entertainer_song,tempo)
    LOST:           load_song(@lost_song,tempo)  
    OBSTACLE:       load_song(@obstacle_song,tempo)
    LIGHT:          load_song(@light_song,tempo)
    MIC:            load_song(@mic_song,tempo)
    LINESENS:       load_song(@line_song,tempo)
    REVEILLE:       load_song(@reveille_song,tempo)
    CHARGE:         load_song(@charge_song,tempo)
    LAC:            load_song(@lac_song,tempo) 
    EXCLAIM:        load_song(@exclaim_song,tempo)
    TAPS:           load_song(@taps_song,tempo)
    AMBULANCE:      load_song(@ambulance_song,tempo)
      
CON
  'Note Lengths - Fractions of a Whole Note
  WHOL                      = 1         'Whole Note
  HALF                      = 2         'Half Note
  QURT                      = 4         'Quarter Note
  EGTH                      = 8         'Eight Note
  SXTH                      = 16        'Sixteenth Note
  THSC                      = 32        'Thirty-secondth Note
  
  'Control Commands
  SONG_END                  = 255
  SET_VOL                   = 254
  SET_VOICES                = 253       
  SET_SYNC                  = 252
  SET_PAUSE                 = 251
  
  'Musical Note Index
  #1,A2,AS2,B2,C2,CS2,D2,DS2,E2,F2,FS2,G2,GS2,A3,AS3,B3,C3,CS3,D3,DS3,E3,F3,FS3,G3,GS3
  A4,AS4,B4,C4,CS4,D4,DS4,E4,F4,FS4,G4,GS4,A5,AS5,B5,C5,CS5,D5,DS5,E5,F5,FS5,G5,GS5
  A6,AS6,B6,C6,CS6,D6,DS6,E6,F6,FS6,G6,GS6

PUB play_note(len_index,note_a_index,note_b_index)
  ''Play a single note
  ''Whole Note = 2000 mS
  
  scribbler.set_voices(scribbler#SIN,scribbler#SIN)
  scribbler.set_volume(90)
  scribbler.play_tone(2000/len_index,WORD[@note_freq][note_a_index],WORD[@note_freq][note_b_index]) 

PUB freq(note_index)
  ''Return the frequency corresponding to a note
  
  return  WORD[@note_freq][note_index]
  
PRI load_song (song_addr, tempo) | index, note_len 
  ''Enque a song to be played
  ''song_addr - address of the song 
  ''tempo - length of a Whole Note in mS
  ''Example: load_song(@lost_song,1600)

  repeat index from 0 to scribbler#_TONE_Q_SIZE
    case BYTE[song_addr][3*index]  
      SET_VOL : scribbler.set_volume(BYTE[song_addr][3*index+1])
      SET_VOICES: scribbler.set_voices(BYTE[song_addr][3*index+1],BYTE[song_addr][3*index+2])
      SONG_END: QUIT
      SET_SYNC: scribbler.play_sync(BYTE[song_addr][3*index+1])
      SET_PAUSE: scribbler.play_pause(BYTE[song_addr][3*index+1]) 
      OTHER:                   
        note_len :=  tempo/BYTE[song_addr][3*index]                    'Double note check
        if index <> 0 AND (BYTE[song_addr][3*index+1]==BYTE[song_addr][3*index-2] OR BYTE[song_addr][3*index+2]==BYTE[song_addr][3*index-1])
          if BYTE[song_addr][3*index] == BYTE[song_addr][3*index-3]   'Previous note was the same length
            scribbler.play_tone(1,0,0)                                       'Insert a short pause to create a double notes
        scribbler.play_tone(note_len,WORD[@note_freq][BYTE[song_addr][3*index+1]],WORD[@note_freq][BYTE[song_addr][3*index+2]])

               
DAT
'Note Frequency Look Up Table
'A,A#,B,C,C#,D,D#,E,F,F#,G,G#

'Octaves 2-6 (A4 = 440 Hz)
note_freq     word      0
              word      110,117,123,131,139,147,156,165,175,185,196,208
              word      220,233,247,262,277,294,311,330,349,370,392,415
              word      440,466,494,523,554,587,622,659,698,740,784,831
              word      880,932,988,1047,1109,1175,1245,1319,1397,1480,1568,1661
              word      1760,1865,1976,2093,2217,2349,2489,2637,2794,2960,3136,3322
    
DAT
''Song Definitions
'' 
''Songs are stored as a series of 3 byte commands per note.
''Each note has the form: note length index, first note index, second note index
''Long notes
''Two notes in a row with the same length will be played as double note with
''with a small pause between them.
''QURT,G3,0,QURT,G3,0  will be played as a pair or 1/4 notes with a short pause between
''
''Two notes in a row with different lengths will be played as a single longer note.
''EGTH,G3,0,QURT,G3,0  will played as single 3/8 note  

''In addition to note definitions, a series of control commands are also
''stored in the song definition tables.
''
''Song End Command    : SONG_END
''Set Volume Command  : SET_VOL,(0 to 100),0  
''Set Voices Command  : SET_VOICES,voice index,voice index
''Set Sync Command    : SET_SYNC,#,0
''Set Pause Command   : PAUSE,#,0


'The Entertainer by Scott Joplin, 1902
'Translated from sheet music by Kristi Dulberger
'Tempo = 3000 
entertainer_song        byte    SET_VOL,70,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    SXTH,D4,0,SXTH,DS4,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0
                        byte    QURT,C5,0,EGTH,0,0,SXTH,C5,0,SXTH,D5,0,SXTH,DS5,0,SXTH,E5,0,SXTH,C5,0
                        byte    SXTH,D5,0,EGTH,E5,0,SXTH,B5,0,EGTH,D5,0,QURT,C5,0,EGTH,0,0,SXTH,D4,0
                        byte    SXTH,DS4,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0,QURT,C5,0
                        byte    EGTH,0,0,SXTH,0,0,SXTH,A5,0,SXTH,G4,0,SXTH,FS4,0,SXTH,A5,0,SXTH,C5,0
                        byte    EGTH,E5,0,SXTH,D5,0,SXTH,C5,0,SXTH,A5,0,QURT,D5,0,EGTH,0,0,SXTH,D4,0
                        byte    SXTH,DS4,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0,EGTH,C5,0,SXTH,E4,0,QURT,C5,0
                        byte    EGTH,0,0,SXTH,C5,0,SXTH,D5,0,SXTH,DS5,0,SXTH,E5,0,SXTH,C5,0,SXTH,D5,0
                        byte    EGTH,E5,0,SXTH,B5,0,EGTH,D5,0,QURT,C5,0,EGTH,0,0,SXTH,C5,0,SXTH,D5,0
                        byte    SXTH,E5,0,SXTH,C5,0,SXTH,D5,0,EGTH,E5,0,SXTH,C5,0,SXTH,D5,0,SXTH,C5,0
                        byte    SXTH,E5,0,SXTH,C5,0,SXTH,D5,0,EGTH,E5,0,SXTH,C5,0,SXTH,D5,0,SXTH,C5,0
                        byte    SXTH,E5,0,SXTH,C5,0,SXTH,D5,0,EGTH,E5,0,SXTH,B5,0,EGTH,D5,0,QURT,C5,0
                        byte    SXTH,0,0,SXTH,E4,0,SXTH,F4,0,SXTH,FS4,0,EGTH,G4,0,SXTH,A5,0,EGTH,G4,0
                        byte    SXTH,E4,0,SXTH,F4,0,SXTH,FS4,0,EGTH,G4,0,SXTH,A5,0,EGTH,G4,0,SXTH,E4,0
                        byte    SXTH,C4,0,SXTH,G3,0,SXTH,A4,0,SXTH,B4,0,SXTH,C4,0,SXTH,D4,0,SXTH,E4,0
                        byte    SXTH,D4,0,SXTH,C4,0,SXTH,D4,0,SXTH,G3,0,SXTH,E4,0,SXTH,F4,0,SXTH,G4,0
                        byte    SXTH,A5,0,SXTH,G4,0,SXTH,E4,0,SXTH,F4,0,EGTH,G4,0,SXTH,A5,0,EGTH,G4,0
                        byte    SXTH,E4,0,SXTH,F4,0,SXTH,FS4,0,EGTH,G4,0,SXTH,A5,0,EGTH,G4,0,SXTH,G4,0
                        byte    SXTH,A5,0,SXTH,AS5,0,SXTH,B5,0,EGTH,B5,0,EGTH,B5,0,SXTH,A5,0,SXTH,FS4,0
                        byte    SXTH,D4,0,QURT,G4,0,SXTH,0,0,SXTH,E4,0,SXTH,F4,0,SXTH,FS4,0,EGTH,G4,0
                        byte    SXTH,A5,0,EGTH,G4,0,SXTH,E4,0,SXTH,F4,0,SXTH,FS4,0,EGTH,G4,0,SXTH,A5,0
                        byte    EGTH,G4,0,SXTH,E4,0,SXTH,C4,0,SXTH,G3,0,SXTH,A4,0,SXTH,B4,0,SXTH,C4,0
                        byte    SXTH,D4,0,SXTH,E4,0,SXTH,D4,0,SXTH,C4,0,SXTH,D4,0,QURT,C4,0,SXTH,0,0
                        byte    SXTH,G4,0,SXTH,FS4,0,SXTH,G4,0,EGTH,C5,0,SXTH,A5,0,EGTH,C5,0,SXTH,A5,0
                        byte    SXTH,C5,0,SXTH,A5,0,SXTH,G4,0,SXTH,C5,0,SXTH,E5,0,EGTH,G5,0,SXTH,E5,0
                        byte    SXTH,C5,0,SXTH,G4,0,EGTH,A5,0,EGTH,C5,0,SXTH,E5,0,EGTH,D5,0,QURT,C5,0
                        byte    SONG_END

'Lost Light Song
'Tempo = 1000
lost_song               byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SAW,scribbler#SAW
                        byte    QURT,D5,0,HALF,E5,0,QURT,D5,0,HALF,C5,0
                        byte    SONG_END


'Obstacle Avoid Tune
'Tempo = 2000
obstacle_song           byte    SET_VOL,70,0
                        byte    SET_VOICES,scribbler#SQU,scribbler#SQU
                        byte    HALF,C3,0,QURT,D4,0,QURT,F3,0,QURT,DS4,0,QURT,F3,0,HALF,C3,0
                        byte    SONG_END   
                         

'Light Sensor Check Song
'Tempo = 2000
light_song              byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    QURT,A4,0,EGTH,F3,0,QURT,C4,0
                        byte    SONG_END

'Microphone & Idler Check Song
'Tempo = 1000
mic_song                byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    QURT,A4,0,QURT,B4,0,QURT,C4,0,QURT,D4,0,QURT,E4,0,QURT,F4,0
                        byte    QURT,E4,0,QURT,D4,0,QURT,C4,0,QURT,B4,0,QURT,A4,0
                        byte    SET_SYNC,MIC,0 
                        byte    SONG_END

'Line Sensor Check Song
'Tempo = 2000
line_song               byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    SXTH,A4,0,EGTH,G3,0,SXTH,A4,0,EGTH,G3,0
                        byte    SONG_END

'Reveille Song
'Tempo = 1000
reveille_song           byte    SET_VOL,90,0
                        byte    SET_VOICES,scribbler#SAW,scribbler#SAW
                        byte    QURT,G3,0,QURT,C4,0,EGTH,E4,0,EGTH,C4,0,QURT,G3,0,QURT,E4,0,QURT,C4,0
                        byte    EGTH,E4,0,EGTH,C4,0,QURT,G3,0,QURT,E4,0,QURT,C4,0,EGTH,E4,0,EGTH,C4,0
                        byte    QURT,G3,0,QURT,C4,0,HALF,E4,0,QURT,C4,0,QURT,G3,0,QURT,C4,0,EGTH,E4,0
                        byte    EGTH,C4,0,QURT,G3,0,QURT,E4,0,QURT,C4,0,EGTH,E4,0,EGTH,C4,0,QURT,G3,0
                        byte    QURT,E4,0,QURT,C4,0,EGTH,E4,0,EGTH,C4,0,QURT,G3,0,THSC, 0,0,QURT,G3,0
                        byte    WHOL,C4,0,HALF,C4,0
                        byte    SONG_END 

'Charge Song
'Tempo = 1500
charge_song             byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SAW,scribbler#SAW
                        byte    EGTH,G3,0,EGTH,C4,0,EGTH,E4,0,EGTH,G4,0,EGTH, 0,0,EGTH,E4,0,WHOL,G4,0
                        byte    EGTH,G4,0
                        byte    SONG_END 

'Exclaim Song
'Tempo = 1500
exclaim_song            byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    SXTH,G4,0,THSC,G4,0,THSC, 0,0,EGTH,G4,0,EGTH,E4,0,EGTH,A5,0,QURT,G4,0
                        byte    QURT,E4,0
                        byte    SONG_END 
'LAC Song                     
'Tempo = 1500
lac_song                byte    SET_VOL,100,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    SXTH,G3,0,THSC,G3,0,THSC, 0,0,SXTH,G3,0,THSC,G3,0,THSC, 0,0,EGTH,G3,0
                        byte    QURT,C4,0,EGTH,C4,0,QURT,E4,0,EGTH,G3,0,THSC, 0,0,EGTH,G3,0,THSC, 0,0
                        byte    EGTH,G3,0,QURT,C4,0,EGTH,C4,0,HALF,E4,0
                        byte    SONG_END 

'Ambulance Song
'Tempo = 2000
ambulance_song          byte    SET_VOL,80,0
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    QURT,G4,0,QURT,B5,0,QURT,G4,0,QURT,B5,0,QURT,G4,0,QURT,B5,0 
                        byte    SET_SYNC,128,0                                                    ' Mark song midway point
                        byte    QURT,G4,0,QURT,B5,0,QURT,G4,0,QURT,B5,0,QURT,G4,0,QURT,B5,0
                        byte    SONG_END 

'Taps Turn Off Song
'Tempo = 1000
taps_song               byte    SET_VOL,45,0 
                        byte    SET_VOICES,scribbler#SIN,scribbler#SIN
                        byte    HALF,D4,0,HALF,D4,0,WHOL,G4,0,HALF,G4,0,WHOL,G4,0,SET_VOL,0,0,HALF,G4,0
                        byte    SET_VOL,45,0,HALF,D4,0,HALF,G4,0,WHOL,B5,0,HALF,B5,0,WHOL,B5,0,SET_VOL,0,0,HALF,B5,0 
                        byte    SET_SYNC,1,0,SET_VOL,45,0,HALF,D4,0,HALF,G4,0,HALF,B5,0,SET_VOL,0,0,HALF,B5,0
                        byte    SET_VOL,45,0,HALF,D4,0,HALF,G4,0,HALF,B5,0,SET_VOL,0,0,HALF,B5,0
                        byte    SET_SYNC,2,0,SET_VOL,45,0,HALF,D4,0,HALF,G4,0,HALF,B5,0,WHOL,B5,0,HALF,B5,0,SET_VOL,0,0,WHOL,B5,0
                        byte    SET_VOL,45,0,HALF,G4,0,HALF,B5,0,SET_VOL,40,0,WHOL,D5,0,HALF,D5,0,WHOL,D5,0
                        byte    SET_VOL,0,0,HALF,D5,0,SET_VOL,40,0,HALF,B5,0,HALF,G4,0
                        byte    SET_SYNC,3,0, WHOL,D4,0,HALF,D4,0,WHOL,D4,0,SET_VOL,0,0,HALF,D4,0
                        byte    SET_VOL,35,0,HALF,D4,0,HALF,D4,0,WHOL,G4,0,HALF,G4,0,WHOL,G4,0,HALF,G4,0
                        byte    SET_SYNC,4,0,SONG_END
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
│THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE          │
│WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR         │
│COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,   │
│ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                         │
└──────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
}}               