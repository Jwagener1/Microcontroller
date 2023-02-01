;-------------------------------------------------------------------------------
    
;--- Device definition and header file ---
    list        p=PIC18F45K22
    #include    "p18f45K22.inc"
    
;--- Configuration bits ---
    CONFIG  FOSC = INTIO67        ; Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
    CONFIG  WDTEN = OFF           ; Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
    CONFIG  LVP = ON             ; Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)    
    
;--- Configuration bits ---
    cblock  0x00
	count
	count1
	DelayCount
	DelayCount1
	DelayCount2
	NumH
	NumL
	Loop
	Green
	Red
	Blue
	ADRR
	EDATA
	INPUT
	SWITCH
	Char1
	Char2
	Char3
	Char4
	Char5
	Char6
	Char7
	Char8
	STORE_OLD
	STORED
	G_CAL
	R_CAL
	B_CAL
	Control
	Strat
	Left
	Right
    endc

COM_COL	    EQU	    b'01001111'
COM_MES	    EQU	    b'01000101'
COM_RCE	    EQU	    b'01000011'
COM_CAL	    EQU	    b'01000001'
COM_T	    EQU	    b'01010100'
red	    EQU	    b'01010010'
blue	    EQU	    b'01000010'
green	    EQU	    b'01000111'
black	    EQU	    b'01101110'
maze	    EQU	    b'01001100'
STRAT	    EQU	    0x00
CAL_S1G	    EQU	    0x01
CAL_S1B	    EQU	    0x02
CAL_S1R	    EQU	    0x03
CAL_S1W	    EQU	    0x04
CAL_S1b	    EQU	    0x05
	    

;--- Vectors ---
    org	    00h
    goto    Start
    org	    08h
    goto    ISR
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
;---------- Configuration ------------------------------------------------------
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Start
    MOVLB   0xF
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;Initialize variables
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    CLRF    count
    CLRF    DelayCount
    CLRF    ADRR
    CLRF    Control
    MOVLW   .66
    MOVWF   Left
    MOVLW   .86
    MOVWF   Right
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ; Set up oscillator
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    BSF	OSCCON,IRCF0
    BCF	OSCCON,IRCF1
    BSF	OSCCON,IRCF2
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;PORT CONFIG
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;Port A Config
    BSF	    TRISA,0	;Set RA0 as input
    BSF	    ANSELA,0	;Set RA0 to analog
    BCF	    TRISA,4
    BCF	    ANSELA,4
    
    ;Port B Config
    CLRF    PORTB
    CLRF    LATB
    MOVLW   b'00000000'
    MOVWF   ANSELB
    MOVLW   b'00000000'
    MOVWF   TRISB
    
    ;PORT C Config
    BSF	    TRISC,RC6	;TX
    BCF	    ANSELC,RC6	;TX
    BSF	    TRISC,RC7	;RX
    BCF	    ANSELC,RC7	;RX
    BSF	    TRISC,5	;Switch 2_Input
    BCF	    ANSELC,5	;SWitch 2_Digital
    BCF	    TRISC,0
    BCF	    TRISC,1
    BCF	    TRISC,2
    BCF	    ANSELC,0
    BCF	    ANSELC,1
    BCF	    ANSELC,2
    ;Port D Config
    CLRF    TRISD
    BCF	    ANSELD,1
    CLRF    LATD
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;RGB LED PORT CONTROLL
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    BSF	    PORTC,2 ; Red LED (ACTIVE LOW)
    BSF	    PORTC,1 ; Green LED (ACTIVE LOW)
    BSF	    PORTC,0 ; Blue LED (ACTIVE LOW)
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;UART CONFIG
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;Baud rate setup (Datasheet RX#1)
    MOVLW   d'12'	  	; 19200 BAUD @ 4 MHz
    MOVWF   SPBRG1	  	; load baudrate register
    CLRF    SPBRGH1
    BSF     TXSTA1,BRGH   	; Enable high BAUDrate
    BCF	    BAUDCON1,BRG16	; Use 8 bit baud generator
    ; Enable asynchronous serial port
    BCF     TXSTA1,SYNC		; Enable asynchronous transmission
    BSF	    RCSTA1,SPEN		; Enable Serial Port (Datasheet RX#3)
    ; Transmit setup (TX)
    BCF	    BAUDCON1,CKTXP	; Inverted polarity
    BSF	    TXSTA1,TXEN		; Enable transmit
    ; Receive setup (RX)
    BCF	    BAUDCON1,DTRXP	; Inverted polarity (Datasheet RX#5)
    BSF	    RCSTA1,CREN		; Enable continuous reception (Datasheet RX#6)
    BSF	    INTCON,PEIE
    
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;A/D Conversion Setup
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;ADCON0
    MOVLW   b'00000101' ; AN0,ADC on
    MOVWF   ADCON0
    ;ADCON1
    MOVLW   b'00000000' ; ADC ref = Vdd,Vss
    MOVWF   ADCON1
    ;ADCON 2
    MOVLW   b'10101110'; right justify (LSB -> Result 0 of lower result register
    MOVWF   ADCON2     ; Frc , Acquisition Time
    BSF	    ADCON0,GO
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;PWM SETUP
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;PWM period setup
    MOVLW   b'11111111'	;Setting the period of PWM signal
    MOVWF   PR2
    ;Timer2 Setup
    MOVLW   b'01111010' ;l:16 Prescale ;1:16 Postscale turns the timer on
    MOVWF   T2CON
     ;Set Which timer to use
    MOVLW   b'00000000'
    MOVWF   CCPTMRS0
    ;Setup the compare module
    MOVLW   b'00101100'
    MOVWF   CCP4CON
    ;Setup Pulse width/Duty Cycle
    BSF	    CCP4CON,5   ; Sets Servero straight
    BCF	    CCP4CON,4
    MOVLW   .76
    MOVWF   CCPR4L
    ;Clear Interupt flag
    CLRF    PIR2
    BSF	    T2CON,2
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    ;OTHER
    ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    MOVLB   0x0
    goto    Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;---------- Subroutines --------------------------------------------------------

;--- Delay ---		
DELAY:			; Subroutine DELAY
    movlw	0x0F
    movwf	DelayCount2  
Loop1
    movlw	0xe9		;
    movwf	DelayCount1	;
Loop2			;
    movlw	0xff
    movwf	DelayCount
Loop3
    decfsz	DelayCount,f
    goto	Loop3
    decfsz	DelayCount1,f	;
    goto	Loop2		;
    decfsz	DelayCount2,f	;
    goto	Loop1		;
    RETURN
DELAY_14us:
    movlw	0x0A
    movwf	DelayCount
    movwf	DelayCount1
Loop4
    decfsz	DelayCount,f
    goto	Loop5
    RETURN
Loop5
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop4
CLEAR:
    CLRF    INPUT
    CLRF    RCREG1
    BCF	    PIR1,RCIF
    CLRF    count	;Reset Count
    RETURN
READ_IN:
    BTFSS   PIR1,RCIF
    GOTO    $-2
    MOVFF   RCREG1,INPUT
    RETURN
RX_DATA:
    CALL    READ_IN	;Call RX 
    CLRF    RCREG1	;Clear the RX_Register
    BCF	    PIR1,RCIF	;Clear the RX_Flag
    
    MOVFF   INPUT,WREG	;Move Value from RX to WREG
    xorlw   '\r'	;Test to see if enter key was pressed
    btfsc   STATUS,Z
    CALL    CLEAR
    btfsc   STATUS,Z
    RETURN		;Exit Instruction
    
    INCF   count	;Increment count
    MOVFF  count,WREG	;Move Value from count to WREG
    xorlw  .1
    btfsc  STATUS,Z	;If count is 1 then INPUT -> Char 1
    MOVFF  INPUT,Char8
    xorlw  .2^.1
    btfsc  STATUS,Z	;If count is 2 then INPUT -> Char 2
    MOVFF  INPUT,Char7
    xorlw  .3^.2
    btfsc  STATUS,Z	;If count is 3 then INPUT -> Char 3
    MOVFF  INPUT,Char6
    xorlw  .4^.3
    btfsc  STATUS,Z	;If count is 4 then INPUT -> Char 4
    MOVFF  INPUT,Char5
    xorlw  .5^.4
    btfsc  STATUS,Z	;If count is 5 then INPUT -> Char 5
    MOVFF  INPUT,Char4
    xorlw  .6^.5
    btfsc  STATUS,Z	;If count is 6 then INPUT -> Char 6
    MOVFF  INPUT,Char3
    xorlw  .7^.6
    btfsc  STATUS,Z	;If count is 7 then INPUT -> Char 7
    MOVFF  INPUT,Char2
    xorlw  .8^.7
    btfsc  STATUS,Z	;If count is 8 then INPUT -> Char 8
    MOVFF  INPUT,Char1
    CALL   RX_DATA
;--- Tx Byte (Byte must be pre-loaded in WREG) ---
BYTE_TX
    MOVWF   TXREG1
POLL_TX
    BTFSS   TXSTA1,TRMT
    GOTO    POLL_TX
    RETURN

REG_PRINT_NUMH:
    movlw   '1'
    BTFSS   NumH,1
    movlw   '0'
    CALL    BYTE_TX

    movlw   '1'
    BTFSS   NumH,0
    movlw   '0'
    CALL    BYTE_TX
    
    CALL    REG_PRINT_NUML
    RETURN
REG_PRINT_NUML:
    movlw   '1'
    BTFSS   NumL,7
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,6
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,5
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,4
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,3
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,2
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,1
    movlw   '0'
    CALL    BYTE_TX
    
    movlw   '1'
    BTFSS   NumL,0
    movlw   '0'
    CALL    BYTE_TX
    
    CALL    MES_RET
    RETURN
    
REG_PRINT:
    movlw   '1'
    BTFSS   STORED,7
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,6
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,5
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,4
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,3
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,2
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,1
    movlw   '0'
    CALL    BYTE_TX
    movlw   '1'
    BTFSS   STORED,0
    movlw   '0'
    CALL    BYTE_TX
    CALL    MES_RET
    Return
    
ADC_FUNC:
    clrf	count
    MOVLW	.32
    MOVWF	count
Sample_Buf
    ;Sample and return the sum 
    BSF		ADCON0,GO
    BTFSC	ADCON0,GO
    GOTO	$-2
    MOVFF	ADRESL,NumL
Average
    ;Moves value from STORED to STORE_OLD without effecting STORED
    MOVLW	0xFF
    ANDWF	STORED,0
    MOVWF	STORE_OLD
    BCF		STATUS,C ;Clear the carry flag
    ;Shift is the same divide
    RRCF	STORE_OLD,1  ; Divide by 2
    BCF		STATUS,C
    RRCF	STORE_OLD,0 ; Divide by 2 again
    SUBWF	STORED,1    ;STORE_OLD/4 - STORED
    ;
    BCF		STATUS,C ;Clear the carry flag
    RRCF	NumL,1
    BCF		STATUS,C
    RRCF	NumL,0
    ;
    ADDWF	STORED,1
    BCF		STATUS,C
    DECFSZ	count
    goto	Sample_Buf
    RETURN
ADC_LOOP:
    clrf    count1
    MOVLW   b'01000000'
    MOVWF   count1
ADC_SUB_LOOP
    CALL    ADC_FUNC
    DECFSZ  count1
    GOTO    ADC_SUB_LOOP
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;CALIBRATION FUNCTION
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Calibration:
    BSF	    PORTC,0
    BSF	    PORTC,1
    BSF	    PORTC,2
    CALL    CAL_MESSAGE_1
    CLRF    STORED
    CALL    GREEN_CAL
    CLRF    STORED
    BSF	    PORTC,0
    BSF	    PORTC,1
    BSF	    PORTC,2
    CALL    RED_CAL
    CLRF    STORED
    BSF	    PORTC,0
    BSF	    PORTC,1
    BSF	    PORTC,2
    CALL    BLUE_CAL
    CLRF    STORED
    BSF	    PORTC,0
    BSF	    PORTC,1
    BSF	    PORTC,2
    CALL    CAL_MESSAGE_5
    GOTO    RCE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
GREEN_CAL:
    CALL   CAL_MESSAGE_2  
    movlw   'G'
    CALL    BYTE_TX
    movlw   'R'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'N'
    CALL    BYTE_TX
    CALL    CAL_MESSAGE_3
    CALL    RX_DATA   
    BCF	    PORTC,2
    CALL    ADC_LOOP
    BSF	    PORTC,2
    MOVFF   STORED,G_CAL
    MOVFF   CAL_S1G,ADRR
    MOVFF   G_CAL,EDATA
    CALL    EEPROM_WRITE
    CALL    DELAY_14us
    call    CAL_MESSAGE_4
    RETURN
RED_CAL:
    CALL   CAL_MESSAGE_2   
    movlw   'R'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'D'
    CALL    BYTE_TX
    CALL   CAL_MESSAGE_3
    CALL    RX_DATA
    BCF	    PORTC,1
    CALL    ADC_LOOP
    BSF	    PORTC,1
    MOVFF   STORED,R_CAL
    MOVFF   CAL_S1R,ADRR
    MOVFF   R_CAL,EDATA
    CALL    EEPROM_WRITE
    CALL    DELAY_14us
    call    CAL_MESSAGE_4
    RETURN
BLUE_CAL:
    CALL   CAL_MESSAGE_2  
    movlw   'B'
    CALL BYTE_TX
    movlw   'L'
    CALL BYTE_TX
    movlw   'U'
    CALL BYTE_TX
    movlw   'E'
    CALL BYTE_TX
    movlw   ' '
    CALL BYTE_TX
    CALL   CAL_MESSAGE_3
    CALL    RX_DATA
    BCF	    PORTC,1
    CALL    ADC_LOOP
    BSF	    PORTC,1
    MOVFF   STORED,B_CAL
    MOVFF   CAL_S1B,ADRR
    MOVFF   B_CAL,EDATA
    CALL    EEPROM_WRITE
    CALL    DELAY_14us
    call    CAL_MESSAGE_4
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EEPROM CAL READ AND WRITE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
EEPROM_READ:
   MOVFF    ADRR,EEADR	    ;Memory Adress to read
   BCF	    EECON1, EEPGD   ;Point to data memory
   BCF	    EECON1, CFGS    ;Access EEPROM
   BSF	    EECON1, RD	    ;EEPROM Read
   MOVF	    EEDATA, W	    ;W = EEDATA
   MOVFF    W,PORTB
   RETURN  
EEPROM_WRITE:
   MOVFF    ADRR,EEADR
   MOVFF    EDATA,EEDATA
   BCF	    EECON1, EEPGD
   BCF	    EECON1, CFGS
   BSF	    EECON1, WREN
   BCF	    INTCON, GIE
   MOVLW    55h
   MOVWF    EECON2
   MOVLW    0AAh
   MOVWF    EECON2
   BSF	    EECON1, WR
   BCF	    INTCON, GIE 
   RETURN 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;UART MESSAGES
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
CAL_MESSAGE_1:
    
    movlw   'E'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   'g'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'c'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   'l'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'b'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   't'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'm'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   '.'
    CALL BYTE_TX
    
    movlw   '\r'
    CALL BYTE_TX
    movlw   '\n'
    CALL BYTE_TX
    
    
    
    RETURN
CAL_MESSAGE_2:
    
    
    movlw   'P'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'p'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'c'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'M'
    CALL BYTE_TX
    movlw   'A'
    CALL BYTE_TX
    movlw   'R'
    CALL BYTE_TX
    movlw   'v'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'o'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX  
    
    RETURN
CAL_MESSAGE_3:
    
    movlw   ' '
    CALL BYTE_TX

    movlw   't'
    CALL BYTE_TX
    movlw   'h'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'p'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'E'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    
    
    
    RETURN
    
CAL_MESSAGE_4:
    
    movlw   'C'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   'l'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'b'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   't'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX
    movlw   '.'
    CALL BYTE_TX
    
    movlw   '\r'
    CALL BYTE_TX
    movlw   '\n'
    CALL BYTE_TX
    
    
    RETURN
    
CAL_MESSAGE_5:
    
    movlw   'C'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   'l'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'b'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   't'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'c'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX 
    movlw   'm'
    CALL BYTE_TX
    movlw   'p'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX 
    movlw   'e'
    CALL BYTE_TX
    movlw   '.'
    CALL BYTE_TX
    
      
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    
    RETURN
    
RACE_MESSAGE:
    
    movlw   'R'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   'c'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   'g'
    CALL BYTE_TX
    movlw   ' '
    CALL BYTE_TX 
    
    movlw   'm'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'e'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX 
    movlw   'a'
    CALL BYTE_TX
    movlw   'b'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX 
    movlw   '.'
    CALL BYTE_TX
    
      
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    RETURN
RACE_MESSAGE1:
    
    movlw   'R'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX 
    movlw   'c'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   'g'
    CALL BYTE_TX
    
   
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    RETURN

RACE_STRAT_MESSAGE:
    movlw   'W'
    CALL BYTE_TX
    movlw   'h'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX
    movlw   ' '
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'h'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'u'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX 
    movlw   ' '
    CALL BYTE_TX
    movlw   'M'
    CALL BYTE_TX
    movlw   'A'
    CALL BYTE_TX
    movlw   'R'
    CALL BYTE_TX
    movlw   'v'
    CALL BYTE_TX 
    movlw   ' '
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'c'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   '?'
    CALL BYTE_TX
    
    movlw   '\r'
    CALL BYTE_TX
    movlw   '\n'
    CALL BYTE_TX
    
    RETURN

RACE_STRAT_MESSAGE1:
    movlw   'M'
    CALL BYTE_TX
    movlw   'A'
    CALL BYTE_TX
    movlw   'R'
    CALL BYTE_TX
    movlw   'v'
    CALL BYTE_TX
    movlw   '.'
    CALL BYTE_TX
    movlw   ' '
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'h'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX 
    movlw   ' '
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'c'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX 
    movlw   ':'
    CALL BYTE_TX 
    RETURN
NON_VALID:
    movlw   'P'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX 
    movlw   'e'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX 
    
    movlw   'e'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'a'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'v'
    CALL BYTE_TX 
    movlw   'a'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX
    movlw   'i'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'c'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX
    movlw   'm'
    CALL BYTE_TX 
    movlw   'm'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   'd'
    CALL BYTE_TX
      
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    
    RETURN

MES_MESSAGE1:
    movlw   'P'
    CALL BYTE_TX
    movlw   'l'
    CALL BYTE_TX 
    movlw   'e'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX 
    
    movlw   'e'
    CALL BYTE_TX
    movlw   'n'
    CALL BYTE_TX
    movlw   't'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'y'
    CALL BYTE_TX 
    movlw   'o'
    CALL BYTE_TX
    movlw   'u'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'm'
    CALL BYTE_TX 
    movlw   'e'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'g'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    
    RETURN
MES_MESSAGE2:
    movlw   'Y'
    CALL BYTE_TX
    movlw   'o'
    CALL BYTE_TX 
    movlw   'u'
    CALL BYTE_TX
    movlw   'r'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'm'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX 
    movlw   's'
    CALL BYTE_TX
    movlw   's'
    CALL BYTE_TX
    movlw   'a'
    CALL BYTE_TX
    movlw   'g'
    CALL BYTE_TX
    movlw   'e'
    CALL BYTE_TX
    
    movlw   ' '
    CALL BYTE_TX
    
    movlw   'i'
    CALL BYTE_TX 
    movlw   's'
    CALL BYTE_TX
    movlw   ':'
    CALL BYTE_TX
    
    RETURN
MES_RET:
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    RETURN
    
MES_GREEN:
    movlw   'G'
    CALL    BYTE_TX
    movlw   'R'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'N'
    CALL    BYTE_TX
    
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    
    movlw   .1
    movwf   Control
    movwf   PORTB
    
    RETURN
    
MES_RED:
    movlw   'R'
    CALL    BYTE_TX
    movlw   'E'
    CALL    BYTE_TX
    movlw   'D'
    CALL    BYTE_TX
    
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    
    movlw   .2
    movwf   Control
    movwf   PORTB
    RETURN
    
MES_BLUE:
    movlw   'B'
    CALL BYTE_TX
    movlw   'L'
    CALL BYTE_TX
    movlw   'U'
    CALL BYTE_TX
    movlw   'E'
    CALL BYTE_TX
    
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    movlw   .3
    movwf   Control
    RETURN
MES_BLACK:
    
    movlw   'B'
    CALL BYTE_TX
    movlw   'L'
    CALL BYTE_TX
    movlw   'A'
    CALL BYTE_TX
    movlw   'C'
    CALL BYTE_TX
    movlw   'K'
    CALL BYTE_TX
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    movlw   .4
    movwf   Control
    movwf   PORTB
    RETURN
MES_WHITE:
    movlw   'W'
    CALL BYTE_TX
    movlw   'H'
    CALL BYTE_TX
    movlw   'I'
    CALL BYTE_TX
    movlw   'T'
    CALL BYTE_TX
    movlw   'E'
    CALL BYTE_TX
    movlw   .0
    movwf   Control
    movwf   PORTB
    CALL    MES_RET
    RETURN
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;UART COMMAND
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
COMMAND_CHECK_RCE
    BSF	    PORTA,4
    MOVFF   Char7,WREG
    ADDWF   Char6
    ADDWF   Char8
    
    
    XORLW   COM_MES
    BTFSC   STATUS,Z
    CALL    MES
    
    XORLW   COM_RCE^COM_MES
    BTFSC   STATUS,Z
    CALL    RCE
    
    XORLW   COM_COL^COM_RCE
    BTFSC   STATUS,Z
    CALL    COL
    
    XORLW   COM_CAL^COM_COL
    BTFSC   STATUS,Z
    CALL    Calibration
    
    XORLW   COM_T^COM_CAL
    BTFSC   STATUS,Z
    CALL    BANG_BANG
    
    CALL    NON_VALID
    CALL    CLEAR
    CALL    RX_DATA
    CALL    COMMAND_CHECK_RCE
    
COMMAND_CHECK_MES:
    BSF	    PORTA,4
    MOVFF   Char7,WREG
    ADDWF   Char6
    ADDWF   Char8
    
    ;Check the Value against Constants
    XORLW   COM_MES
    BTFSC   STATUS,Z
    CALL    MES
    
    XORLW   COM_RCE^COM_MES
    BTFSC   STATUS,Z
    CALL    RCE
    
    CALL    NON_VALID
    CALL    CLEAR
    CALL    RX_DATA
    CALL    COMMAND_CHECK_MES
    
STRAT_CHECK:
    MOVFF   STRAT,ADRR
    CALL    EEPROM_READ
    
    XORLW   green
    BTFSC   STATUS,Z
    GOTO    GREEN
    
    XORLW   blue^green
    BTFSC   STATUS,Z
    GOTO    BLUE
    
    XORLW   red^blue
    BTFSC   STATUS,Z
    GOTO    RED
    
        
    XORLW   black^red
    BTFSC   STATUS,Z
    GOTO    BLACK
    
    XORLW   maze^black
    BTFSC   STATUS,Z
    GOTO    MAZE
    
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;COLOUR DETECTION
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
RGB_VALUE:  
GREEN_VALUE
    BCF	    PORTC,1
    CALL    DELAY_14us
    CLRF    STORED
    CALL    ADC_FUNC
    BSF	    PORTC,1
    CALL    DELAY_14us
    MOVFF   STORED,Green
    CLRF    STORED
RED_VALUE
    BCF     PORTC,2
    CALL    DELAY_14us
    CALL    ADC_FUNC
    BSF	    PORTC,2
    CALL    DELAY_14us
    MOVFF   STORED,Red
    CLRF    STORED
BLUE_VALUE
    BCF	    PORTC,0
    CALL    DELAY_14us
    CALL    ADC_FUNC
    BSF	    PORTC,0
    CALL    DELAY_14us
    MOVFF   STORED,Blue
    CLRF    STORED
    CALL    COLOUR_PROCESS
    RETURN
COLOUR_PROCESS:
GREEN_T
    MOVFF   Green,WREG
    CPFSLT  G_CAL
    GOTO    BLUE_T
    GOTO    WHITE_T
WHITE_T
    MOVFF   Blue,WREG
    CPFSLT  B_CAL
    CALL    MES_GREEN
    CPFSLT  B_CAL
    RETURN
    CALL    MES_WHITE
    RETURN
BLUE_T
    MOVFF   Blue,WREG
    CPFSLT  B_CAL
    GOTO    RED_T
    CPFSLT  B_CAL
    RETURN
    CALL    MES_BLUE
    RETURN
RED_T
    MOVFF   Red,WREG
    CPFSLT  R_CAL
    CALL    MES_BLACK
    CPFSLT  R_CAL
    RETURN
    CALL    MES_RED
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;MESSAGE PROGRAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  
MES:
    CALL    MES_MESSAGE1
    CALL    RX_DATA
    CALL    MES_MESSAGE2
    MOVFF   Char8,WREG
    CALL    BYTE_TX
    MOVFF   Char7,WREG
    CALL    BYTE_TX
    MOVFF   Char6,WREG
    CALL    BYTE_TX
    MOVFF   Char5,WREG
    CALL    BYTE_TX
    MOVFF   Char4,WREG
    CALL    BYTE_TX
    MOVFF   Char3,WREG
    CALL    BYTE_TX
    MOVFF   Char2,WREG
    CALL    BYTE_TX
    MOVFF   Char1,WREG
    CALL    BYTE_TX
    CALL    MES_RET
    CALL    CLEAR
    CALL    RX_DATA
    CALL    COMMAND_CHECK_MES 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;STRATEGY PROGRAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
COL:
    CALL    RACE_STRAT_MESSAGE
    CALL    CLEAR
    CALL    RX_DATA
    MOVFF   Char8,WREG
    MOVFF   Char8,WREG
    MOVWF   EDATA
    CALL    EEPROM_WRITE
    CALL    EEPROM_READ
    
    XORLW   green
    BTFSC   STATUS,Z
    GOTO    GREEN
    
    XORLW   blue^green
    BTFSC   STATUS,Z
    GOTO    BLUE
    
    XORLW   red^blue
    BTFSC   STATUS,Z
    GOTO    RED
    
        
    XORLW   black^red
    BTFSC   STATUS,Z
    GOTO    BLACK
    
    XORLW   maze^black
    BTFSC   STATUS,Z
    GOTO    MAZE   
 
    CALL    NON_VALID
    CALL    CLEAR
    CALL    COL

GREEN
    CALL    RACE_STRAT_MESSAGE1
    CALL    MES_GREEN
    MOVLW   .1
    MOVWF   Strat
    CALL    RCE
BLUE
    CALL    RACE_STRAT_MESSAGE1
    CALL    MES_BLUE
    MOVLW   b'11111111'
    MOVWF   Strat
    CALL    RCE
RED 
    CALL    RACE_STRAT_MESSAGE1
    CALL    MES_RED
    MOVLW   .2
    MOVWF   Strat
    CALL    RCE
BLACK
    CALL    RACE_STRAT_MESSAGE1
    CALL    MES_BLACK
    MOVLW   .4
    MOVWF   Strat
    CALL    RCE
MAZE
    CALL    RACE_STRAT_MESSAGE1
    movlw   'M'
    CALL    BYTE_TX
    movlw   'a'
    CALL    BYTE_TX
    movlw   'z'
    CALL    BYTE_TX
    movlw   'e'
    CALL    BYTE_TX
    movlw   '.'
    CALL    BYTE_TX
    CALL    MES_RET
    MOVLW   .66
    MOVWF   Strat
    CALL    RCE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;MAIN PROGRAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
RCE:    
    CALL    CLEAR
    CALL    RX_DATA
    CALL    CLEAR
    GOTO    COMMAND_CHECK_RCE
    CALL    RX_DATA
    CALL    BANG_BANG
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;MAIN PROGRAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
START:    
    CALL    RACE_MESSAGE
    ;CALL    RACE_MESSAGE1
    CALL    STRAT_CHECK
    MOVLW   .16
    MOVWF   CCPR4L 
    CALL    RCE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
;BANG BANG STEERING CONTROL
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 
BANG_BANG:   
LEFT
    CLRF    STATUS,Z
    MOVFF   Left,CCPR4L
    CALL    RGB_VALUE
    MOVFF   Control,WREG
    XORLW   Strat
    BTFSC   STATUS,Z
    GOTO    LEFT
RIGHT
    CLRF    STATUS,Z
    MOVFF   Right,CCPR4L
    CALL    RGB_VALUE
    MOVFF   Control,WREG
    XORLW   Strat
    BTFSC   STATUS,Z
    GOTO    RIGHT
    GOTO    LEFT
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
;PID STEERING CONTROL
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 

ISR
    goto    Main
Main
    MOVFF   CAL_S1G,ADRR
    CALL    EEPROM_READ
    MOVWF   WREG,G_CAL
    MOVFF   CAL_S1R,ADRR
    CALL    EEPROM_READ
    MOVWF   WREG,R_CAL
    MOVFF   CAL_S1B,ADRR
    CALL    EEPROM_READ
    MOVWF   WREG,B_CAL
    CALL    START
    end
    
