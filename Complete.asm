;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
;Device definition and header file
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
    list        p=PIC18F45K22
    #include    "p18f45K22.inc"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
;Configuration bits
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
    CONFIG  FOSC = INTIO67
    CONFIG  WDTEN = OFF
    CONFIG  LVP = ON
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
;Variables in RAM
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    NUME    EQU	    0x00 ;Numerator Ram location
    QU	    EQU	    0x20 ;Quotient Ram location
    COUNT   EQU	    0x10 ;Count Reg
    CNTVAL  EQU	    d'3' ;count value
    DEN	    EQU	    d'10';Divide by 10
    UNPBCDA EQU	    0x30
    RESULT  EQU	    0x40
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Variable Declaration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    cblock  0x00
	INPUT
	Char1
	Char2
	Char3
	Char4
	Char5
	Char6
	Char7
	Char8
	Count
	NumL
	NumH
	STORED
	STORE_OLD
	STORED_OLD
	ADRR
	EDATA
	Cap
	DelayCount
	DelayCount1
	BreN
	BreO
	BreI
	NIB1
	NIB2
	NIB3
	ADDRESS
	CHAR
	Timecnt
	TimeAvgOld
	TimeAvg
	MINUTE
	RMDH
	RMDM
	RMDL
	BRH
	BRM
	BRL
	Lcnt
	Temp
	Y1
	Delta
	Der
	DerR
	Timecnt1
	DIFFLOW
	DIFFHIGH
	I2C_DATA
	CHAR_H
	CHAR_E
	CHAR_L
	CHAR_O
	CHAR_SPACE
	CHAR_W
	CHAR_R
	CHAR_D
	CHAR_EX
	Calresult
	
    endc
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Vectors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    org	    0x0000
    goto    Start
    org	    08h
    BTFSS   INTCON,RBIF
    goto    TIME_TEST 
    goto    ISR_PORT
TIME_TEST   
    BTFSS   INTCON,TMR0IF
    RETFIE
    goto    ISR_TIME
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Setup
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Start
    MOVLB   0xF
   ;Variable Initialization
    CLRF    INPUT
    MOVLW   b'00110000'
    MOVWF   Cap
    MOVLW   0x02
    MOVWF   Timecnt
    CLRF    Timecnt1
    MOVLW   0xEF ;Hex for 239 
    MOVWF   MINUTE
    CLRF    TimeAvgOld
    CLRF    TimeAvg
    CLRF    RMDH
    CLRF    RMDM
    CLRF    RMDL
    MOVLW   .0
    MOVWF   TimeAvgOld
    MOVLW   .171
    MOVWF   Delta
    MOVLW   .9
    MOVWF   DIFFHIGH
    MOVLW   .19
    MOVWF   DIFFLOW
   ;Oscillator Setup (Page 30)
    BSF		OSCCON,IRCF0
    BCF		OSCCON,IRCF1
    BSF		OSCCON,IRCF2
IOSC	
    BTFSS	OSCCON,IOFS	; Is the Internal Oscillator Stable Yet?
    GOTO	IOSC
    
   ;I/O Port Setup
	;Tris controls weather the port is set as an input or output  0=output
	;ANSELA controls weather the port is set as digital or analog 0=digital
    ;Port A
    BSF		 TRISA,0	;Set RA0 as input
    BSF		 ANSELA,0	;Set RA0 to analog
    BCF		 TRISA,4	;Set RA4 as output (Curiosity led D2)
    BCF		 ANSELA,4	;Set RA4 as digital
    BCF		 TRISA,5	;Set RA4 as output (Curiosity led D3)
    BCF		 ANSELA,5	;Set RA4 as digital
    BCF		 TRISA,6	;Set RA4 as output (Curiosity led D4)
    BCF		 ANSELA,6	;Set RA4 as digital
    BCF		 TRISA,7	;Set RA4 as output (Curiosity led D5)
    BCF		 ANSELA,7	;Set RA4 as digital
    ;Port B
   CLRF		 TRISB
   BSF		 TRISB,5
   BCF		 ANSELB,5
   BSF		 TRISB,4
   BCF		 ANSELB,4
    ;Port C
	;TX pin for UART must be set as digital input
   BSF		TRISC,RC6	;TX
   BCF		ANSELC,RC6	;TX
	;RX pin for UART must be set as digital input
   BSF		TRISC,RC7	;RX
   BCF		ANSELC,RC7	;RX
	;SCL pin for I2C must be set as an output
    BSF	    TRISC,   RC3	    
    BSF	    TRISC,   RC4	    
    BCF	    ANSELC,  RC3	    
    BCF	    ANSELC,  RC4
    bsf	    PORTC,SDA
	;Port D
    CLRF    TRISD
    BCF	    ANSELD,1
    CLRF    LATD
   ;UART Setup
    ;Baud Rate Config
   MOVLW	d'12'	  	; 19200 BAUD @ 4 MHz
   MOVWF	SPBRG1	  	; load baudrate register
   CLRF		SPBRGH1
   BSF		TXSTA1,BRGH   	; Enable high BAUDrate
   BCF		BAUDCON1,BRG16	; Use 8 bit baud generator
    ;Enable asynchronous serial port
   BCF		TXSTA1,SYNC	; Enable asynchronous transmission
   BSF		RCSTA1,SPEN	; Enable Serial Port (Datasheet RX#3)
    ;Transmit setup (TX)
   BCF		BAUDCON1,CKTXP	; Inverted polarity
   BSF		TXSTA1,TXEN	; Enable transmit
    ;Receive setup (RX)
   BCF		BAUDCON1,DTRXP	; Inverted polarity (Datasheet RX#5)
   BSF		RCSTA1,CREN	; Enable continuous reception (Datasheet RX#6)
   BSF		INTCON,PEIE
   
   ;ADC Setup (Page 289)
    ;ADCON0
   MOVLW	b'00000101' ; AN1,ADC on
   MOVWF	ADCON0 
   ;MOVLW	b'00000001' ; AN1,ADC on
   ;MOVWF	ADCON0
    ;ADCON1
   MOVLW	b'00000000' ; ADC ref = Vdd,Vss
   MOVWF	ADCON1
    ;ADCON 2
   MOVLW	b'00101110'; left justify 
   MOVWF	ADCON2     ; Frc , Acquisition Time
   BSF		ADCON0,GO  ;	Starts the adc module
   
   ;PWM Setup (Page 180)
    ;PWM period setup (Formula Page 181)
   MOVLW	0xFF	
   MOVWF	PR2
    ;Timer2 Setup
   MOVLW	b'01111010' ;l:16 Prescale there is no Postscale
   MOVWF	T2CON	    ;with timer2 and PWM
    ;Set Which timer to use
   MOVLW	b'00000000'
   MOVWF	CCPTMRS0
    ;Setup the compare module
   MOVLW	b'00101100'
   MOVWF	CCP4CON
    ;Setup Pulse width/Duty Cycle (Note its a 10bit value)
   BCF		CCP4CON,5   ;Controls the PWM decimal 
   BCF		CCP4CON,4   ;Controls the PWM decimal
   MOVLW	.76
   MOVWF	CCPR4L
   ;Start the PWM on PIN D1
   BSF		T2CON,2
   ;Timer Setup	    (Page 155)
    ;Timer0
   MOVLW	b'00000001'
   MOVWF	T0CON
   ;Interupts
   BCF		INTCON,GIE	;Global Interupts Enable
   BCF		INTCON,RBIF	;Chnage to portB<4:7> intterupt flag clear
   BSF		INTCON,RBIE	;Chnage to portB<4:7> intterupt enable
   BCF		INTCON,TMR0IF	;Timer0 intterupt flag clear
   BSF		INTCON,TMR0IE	;Timer0 intterupt enable
   BCF		IOCB,5
   ;I2C
    ;Baudrate config (100KHz) Eq.15-1 Page 251
   MOVLW	0x09	    
   MOVWF	SSP1ADD
    ;Clear the START/STOP bits
   CLRF		SSP1STAT 
   BSF		SSP1STAT, SMP	    
   BSF		SSP1STAT, CKE
   ;I2C Master Mode
   BCF		SSP1CON1, SSPM0	    ; 1000
   BCF		SSP1CON1, SSPM1	    ; 
   BCF		SSP1CON1, SSPM2	    ; 
   BSF		SSP1CON1, SSPM3	    ; 
   BSF		SSP1CON1, SSPEN	    ;
   BCF		PIR1,SSP1IF
   MOVLB	0x0
   CALL		LCD_INT
   BCF		T0CON,TMR0ON    ;Turn on timer0
   BCF		T2CON,TMR2ON
   GOTO		Main
   
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Delay Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
DELAY_1:	;(172ms)
    movlw	0xE0
    movwf	DelayCount
    movwf	DelayCount1
Loop1
    decfsz	DelayCount,f
    goto	Loop2
    RETURN
Loop2
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop1
    
DELAY_2:	;(5ms)
    movlw	0x08
    movwf	DelayCount
    movlw	0x70
    movwf	DelayCount1
Loop3
    decfsz	DelayCount,f
    goto	Loop4
    RETURN
Loop4
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop3
    
DELAY_3:	;(1ms)
    movlw	0x03
    movwf	DelayCount
    movlw	0x60
    movwf	DelayCount1
Loop5
    decfsz	DelayCount,f
    goto	Loop6
    RETURN
Loop6
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop5
    
DELAY_4:	;(70us)
    movlw	0x02
    movwf	DelayCount
    movlw	0x12
    movwf	DelayCount1
Loop7
    decfsz	DelayCount,f
    goto	Loop8
    RETURN
Loop8
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop7
Derivative_DELAY:
    movlw	0xE0
    movwf	DelayCount
    movwf	DelayCount1
Loop9
    decfsz	DelayCount,f
    goto	Loop10
    RETURN
Loop10
    decfsz	DelayCount1,f
    goto	$-2
    goto	Loop9
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;UART Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Recive a byte of data 
CLEAR:
   CLRF		INPUT
   CLRF		RCREG1
   BCF		PIR1,RCIF
   CLRF		Count		;Reset Count
   RETURN
READ_IN:
   BTFSS	PIR1,RCIF
   GOTO		$-2
   MOVFF	RCREG1,INPUT
   RETURN
RX_DATA:
   CALL		READ_IN		;Call RX 
   CLRF		RCREG1		;Clear the RX_Register
   BCF		PIR1,RCIF	;Clear the RX_Flag
    
   MOVFF	INPUT,WREG	;Move Value from RX to WREG
   xorlw	'\r'		;Test to see if enter key was pressed
   btfsc	STATUS,Z
   RETURN			;Exit Instruction
    
   INCF		Count		;Increment count
   MOVFF	Count,WREG	;Move Value from count to WREG
   xorlw	.1
   btfsc	STATUS,Z	;If count is 1 then INPUT -> Char 1
   MOVFF	INPUT,Char8
   xorlw	.2^.1
   btfsc	STATUS,Z	;If count is 2 then INPUT -> Char 2
   MOVFF	INPUT,Char7
   xorlw	.3^.2
   btfsc	STATUS,Z	;If count is 3 then INPUT -> Char 3
   MOVFF	INPUT,Char6
   xorlw	.4^.3
   btfsc	STATUS,Z	;If count is 4 then INPUT -> Char 4
   MOVFF	INPUT,Char5
   xorlw	.5^.4
   btfsc	STATUS,Z	;If count is 5 then INPUT -> Char 5
   MOVFF	INPUT,Char4
   xorlw	.6^.5
   btfsc	STATUS,Z	;If count is 6 then INPUT -> Char 6
   MOVFF	INPUT,Char3
   xorlw	.7^.6
   btfsc	STATUS,Z	;If count is 7 then INPUT -> Char 7
   MOVFF	INPUT,Char2
   xorlw	.8^.7
   btfsc	STATUS,Z	;If count is 8 then INPUT -> Char 8
   MOVFF	INPUT,Char1
   CALL		RX_DATA
   
;Transmits 1 bit of data from WREG  
BYTE_TX
   MOVWF	TXREG1
POLL_TX
   BTFSS	TXSTA1,TRMT
   GOTO		POLL_TX
   RETURN
Menu:
    CALL	Line
    MOVLW	'M'
    CALL	BYTE_TX
    MOVLW	'e'
    CALL	BYTE_TX
    MOVLW	'n'
    CALL	BYTE_TX
    MOVLW	'u'
    CALL	BYTE_TX
    CALL	Line
    CALL	MES_RET
    MOVLW	'1'
    CALL	BYTE_TX
    MOVLW	')'
    CALL	BYTE_TX
    MOVLW	' '
    CALL	BYTE_TX
    movlw	'C'
    CALL	BYTE_TX
    movlw	'a'
    CALL	BYTE_TX 
    movlw	'l'
    CALL	BYTE_TX
    movlw	'i'
    CALL	BYTE_TX
    movlw	'b'
    CALL	BYTE_TX
    movlw	'r'
    CALL	BYTE_TX
    movlw	'a'
    CALL	BYTE_TX 
    movlw	't'
    CALL	BYTE_TX
    movlw	'i'
    CALL	BYTE_TX
    movlw	'o'
    CALL	BYTE_TX
    movlw	'n'
    CALL	BYTE_TX
    CALL	MES_RET
    MOVLW	'2'
    CALL	BYTE_TX
    MOVLW	')'
    CALL	BYTE_TX
    MOVLW	' '
    CALL	BYTE_TX
    movlw	'S'
    CALL	BYTE_TX
    movlw	't'
    CALL	BYTE_TX 
    movlw	'a'
    CALL	BYTE_TX
    movlw	'r'
    CALL	BYTE_TX
    movlw	't'
    CALL	BYTE_TX
    CALL	MES_RET
    MOVLW	'3'
    CALL	BYTE_TX
    MOVLW	')'
    CALL	BYTE_TX
    MOVLW	' '
    CALL	BYTE_TX
    movlw	'L'
    CALL	BYTE_TX
    movlw	'C'
    CALL	BYTE_TX 
    movlw	'D'
    CALL	BYTE_TX
    movlw	' '
    CALL	BYTE_TX
    movlw	'T'
    CALL	BYTE_TX
    movlw	'e'
    CALL	BYTE_TX
    movlw	's'
    CALL	BYTE_TX 
    movlw	't'
    CALL	BYTE_TX 
    CALL	MES_RET
    RETURN
Line:
    MOVLW	0x10
    MOVWF	Lcnt
Lcount
    DCFSNZ	Lcnt
    RETURN
    MOVLW	'-'
    CALL	BYTE_TX
    GOTO	Lcount
MES_RET:
    MOVLW	'\r'
    CALL	BYTE_TX
    MOVLW	'\n'
    CALL	BYTE_TX
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;ADC Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
ADC_FUNC:
   clrf		Count
   MOVLW	.64
   MOVWF	Count
Sample_Buf
    ;Sample and return the sum 
   BSF		ADCON0,GO
   BTFSC	ADCON0,GO
   GOTO		$-2
   MOVFF	ADRESL,NumL
   MOVFF	ADRESH,NumH
Average
    ;Moves value from STORED to STORE_OLD without effecting STORED
   MOVLW	0xFF
   ANDWF	STORED,0
   MOVWF	STORE_OLD
   BCF		STATUS,C	;Clear the carry flag
    ;Shift is the same divide
   RRCF		STORE_OLD,1	;Divide by 2
   BCF		STATUS,C
   RRCF		STORE_OLD,0	;Divide by 2 again
   SUBWF	STORED,1	;STORE_OLD/4 - STORED
    ;
   BCF		STATUS,C	;Clear the carry flag
   RRCF		NumH,1
   BCF		STATUS,C
   RRCF		NumH,0
    ;
   ADDWF	STORED,1
   BCF		STATUS,C
   DECFSZ	Count
   goto		Sample_Buf
   MOVFF	STORED,CCPR4L
   RETURN
   
   ADC_FUNC_1:
   clrf		Count
   MOVLW	.64
   MOVWF	Count
Sample_Buf_1
    ;Sample and return the sum 
   BSF		ADCON0,GO
   BTFSC	ADCON0,GO
   GOTO		$-2
   MOVFF	ADRESL,NumH
Average_1
    ;Moves value from STORED to STORE_OLD without effecting STORED
   MOVLW	0xFF
   ANDWF	STORED,0
   MOVWF	STORE_OLD
   BCF		STATUS,C	;Clear the carry flag
    ;Shift is the same divide
   RRCF		STORE_OLD,1	;Divide by 2
   BCF		STATUS,C
   RRCF		STORE_OLD,0	;Divide by 2 again
   SUBWF	STORED,1	;STORE_OLD/4 - STORED
    ;
   BCF		STATUS,C	;Clear the carry flag
   RRCF		NumH,1
   BCF		STATUS,C
   RRCF		NumH,0
    ;
   ADDWF	STORED,1
   BCF		STATUS,C
   DECFSZ	Count
   goto		Sample_Buf_1
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
    movlw   '\r'
    CALL    BYTE_TX
    movlw   '\n'
    CALL    BYTE_TX
    Return
DerivativeADC:		;(Y2-Y1)/Delay
    CALL	ADC_FUNC	;Call the ADC Module
    MOVFF	STORED,WREG
    CALL	BIN_TO_ASCII
    MOVFF	BRH,WREG
    CALL	BYTE_TX
    MOVFF	RMDM,WREG
    CALL	BYTE_TX
    MOVFF	RMDL,WREG
    CALL	BYTE_TX
    CALL	MES_RET
    MOVFF	STORED,WREG	;Move the value from STORED to WREG
    ADDLW	.2
    MOVWF	Y1
    CALL	Derivative_DELAY ;Call the delay
    CALL	ADC_FUNC	;CALL the ADC 
    MOVFF	STORED,WREG
    CALL	BIN_TO_ASCII
    MOVFF	BRH,WREG
    CALL	BYTE_TX
    MOVFF	RMDM,WREG
    CALL	BYTE_TX
    MOVFF	RMDL,WREG
    CALL	BYTE_TX
    CALL	MES_RET
    MOVFF	STORED,WREG	;Move that vlaue to WREG
    CPFSLT	Y1
    BCF		PORTA,4
    CPFSGT	Y1
    BSF		PORTA,4
    RETURN  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;I2C Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
EEPROM_WRITE_EX:
    BCF	    INTCON,GIE
    BSF	    SSP1CON2, SEN	     
    CALL    WAIT_FUNC		     
    MOVLW   0xA0		    
    MOVWF   SSP1BUF	

    CALL    WAIT_FUNC		         
    BTFSC   SSP1CON2, ACKSTAT	     
    GOTO    EEPROM_WRITE_EX	    
   
    MOVFF   ADDRESS,SSP1BUF	   
    CALL    WAIT_FUNC		       
    BTFSC   SSP1CON2, ACKSTAT	    
    GOTO    FAILED
   
    
    MOVFF   I2C_DATA,SSP1BUF	    
    CALL    WAIT_FUNC		       
    BTFSC   SSP1CON2, ACKSTAT	    
    GOTO    FAILED	    

    BSF	    SSP1CON2, PEN	    
    CALL    WAIT_FUNC
    BSF	    INTCON,GIE
    RETURN
    
EEPROM_READ_EX:
    BCF	    INTCON,GIE
    BSF	    SSP1CON2, SEN	     
    CALL    WAIT_FUNC		     
    MOVLW   0xA0		    
    MOVWF   SSP1BUF	

    CALL    WAIT_FUNC		         
    BTFSC   SSP1CON2, ACKSTAT	     
    GOTO    EEPROM_READ_EX	    
    
    
    
    MOVFF   ADDRESS,SSP1BUF	   
    CALL    WAIT_FUNC
    BTFSC   SSP1CON2, ACKSTAT	    
    GOTO    FAILED
    
    
    
    BSF	    SSP1CON2,RSEN	     
    CALL    WAIT_FUNC		     
    MOVLW   0xA1	    
    MOVWF   SSP1BUF	
    
    
    
    CALL    WAIT_FUNC		         
    BTFSC   SSP1CON2, ACKSTAT	     
    GOTO    EEPROM_READ_EX
    
    BSF	    SSP1CON2,RCEN
    CALL    WAIT_FUNC
    
    
    CLRF    PORTA
    MOVFF   SSP1BUF,I2C_DATA
    BSF	    SSP1CON2,ACKDT
    BSF	    SSP1CON2,ACKEN
    CALL    WAIT_FUNC
    BSF	    SSP1CON2, PEN
    CALL    WAIT_FUNC
    BSF	    INTCON,GIE
    RETURN
    
WAIT_FUNC:
   BTFSS	PIR1,SSP1IF
   BRA		$-2		    
   BCF		PIR1,SSP1IF
   RETURN
   
FAILED:
    BSF		SSP1CON2, PEN
    GOTO	Main			    
    RETURN      
    
    
LCD_Write:
   BCF		INTCON,GIE
   BSF		SSP1CON2, SEN ;Set the SEN bit of  SSP1CON2 to start transmission     
   CALL		WAIT_FUNC     ;This function checks ans clears the SSP1IF			     
   
   MOVLW	0x4E	      ;Address of IO module (Write)
   MOVWF	SSP1BUF	
   
   CALL		WAIT_FUNC     ;This function checks ans clears the SSP1IF	         
   BTFSC	SSP1CON2, ACKSTAT ;Testing for Ack from slave	     
   GOTO		LCD_Write     ;If no ACK then repeats above steps 	    

   MOVFF	NIB1,SSP1BUF	;Moves adress where to write to SSP1BUF  
   CALL		WAIT_FUNC    ;This function checks ans clears the SSP1IF	       
   BTFSC	SSP1CON2, ACKSTAT ;Testing for Ack from slave		    
   GOTO		FAILED

   BSF		SSP1CON2, PEN	    
   CALL		WAIT_FUNC
   BSF		INTCON,GIE	;Enables global interupts
   SETF		WREG		    
   DECFSZ	WREG		    
   BRA		$-2
   RETURN


 
LCD_PRINT:
    ;Only the upper nibble is sent
    MOVWF	NIB2	;NIB1 = Upper nibble
    MOVWF	NIB3
    ;In order to send data to the LCD only 4 bits may be sent at a time
    ;thus we need to send 0xXD where X is the nibble need to send
    ;
    ;Upper Nibble
    MOVLW	0xF0	    ; This function AND NIB2 with '11110000'
    ANDWF	NIB2,1,0    ; thus we obtain 'xxxx0000'
    MOVLW	0x0D	    ; This function or NIB2 with 0x0D
    IORWF	NIB2,1,0    ; thus we obation 0xXD
    ;Lower Nibble
    SWAPF	NIB3,1,0    ; This function swaps the nibbles of REG F 
    MOVLW	0xF0	    ; This function AND NIB2 with '11110000'
    ANDWF	NIB3,1,0    ; thus we obtain 'xxxx0000'
    MOVLW	0x0D	    ; This function or NIB2 with 0x0D
    IORWF	NIB3,1,0    ; thus we obation 0xXD
    
    CALL	DELAY_4
    MOVLW	0x49
    MOVWF	NIB1
    CALL	LCD_Write
    MOVFF	NIB2,NIB1
    CALL	LCD_Write
    MOVLW	0x49
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    MOVLW	0x89
    MOVWF	NIB1
    CALL	LCD_Write
    MOVFF	NIB3,NIB1
    CALL	LCD_Write
    MOVLW	0x89
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
        

    
LCD_INT:
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1 ; 3xDelay_1 = 500ms
    
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x34
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_2 ; Delay_2 = 5ms
    
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x34
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_2 ; Delay_2 = 5ms
    
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x34
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x30
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_3 ; Delay_3 = 1ms
    
    MOVLW	0x20
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x24
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x20
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0x20
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x24
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x20
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0x80
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x84
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x80
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0x00
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x04
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x00
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0xC0
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC4
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC0
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0x00
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x04
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x00
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    
    MOVLW	0x10
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x14
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x10
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4 ; Delay_4 = 70us
    CALL	LCD_CLEAR
    CALL	Cursor00
    MOVLW	'I'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'n'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'i'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	't'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'i'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'a'
    CALL	LCD_PRINT
    CALL	DELAY_1
     MOVLW	'l'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'i'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'z'
    CALL	LCD_PRINT
     CALL	DELAY_1
    MOVLW	'e'
    CALL	LCD_PRINT
    CALL	DELAY_1
    MOVLW	'd'
    CALL	LCD_PRINT
    CALL	DELAY_1
    CALL	LCD_CLEAR
    RETURN
LCD_CLEAR:
    CALL	DELAY_4
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x0C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x1C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
    
LCD_CURSOR:
    CALL	DELAY_4
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x0C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xEC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
LCD_NoCURSOR:
    CALL	DELAY_4
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x0C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xCC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
CURX0:
    CALL	DELAY_4
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x8C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    CALL	DELAY_4
    RETURN
CURX1:
    CALL	DELAY_4
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xCC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    CALL	DELAY_4
    RETURN
Cursor00:
    CALL	CURX0
    
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x0C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor10:
    CALL	CURX0
    
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x1C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor20:
    CALL	CURX0
    
    MOVLW	0x28
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x2C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x28
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor30:
    CALL	CURX0
    
    MOVLW	0x38
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x3C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x38
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor40:
    CALL	CURX0
    
    MOVLW	0x48
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x4C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x48
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor50:
    CALL	CURX0
    
    MOVLW	0x58
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x5C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x58
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
 Cursor60:
    CALL	CURX0
    
    MOVLW	0x68
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x6C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x68
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor70:
    CALL	CURX0
    
    MOVLW	0x78
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x7C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x78
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor80:
    CALL	CURX0
    
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x8C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor90:
    CALL	CURX0
    
    MOVLW	0x98
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x9C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x98
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor100:
    CALL	CURX0
    
    MOVLW	0xA8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xAC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xA8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor110:
    CALL	CURX0
    
    MOVLW	0xB8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xBC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xB8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor120:
    CALL	CURX0
    
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xCC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor130:
    CALL	CURX0
    
    MOVLW	0xD8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xDC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xD8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor140:
    CALL	CURX0
    
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xEC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor150:
    CALL	CURX0
    
    MOVLW	0xF8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xFC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xF8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor01:
    CALL	CURX1
    
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x0C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x08
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor11:
    CALL	CURX1
    
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x1C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x18
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor21:
    CALL	CURX1
    
    MOVLW	0x28
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x2C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x28
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor31:
    CALL	CURX1
    
    MOVLW	0x38
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x3C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x38
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor41:
    CALL	CURX1
    
    MOVLW	0x48
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x4C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x48
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor51:
    CALL	CURX1
    
    MOVLW	0x58
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x5C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x58
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
 Cursor61:
    CALL	CURX1
    
    MOVLW	0x68
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x6C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x68
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor71:
    CALL	CURX1
    
    MOVLW	0x78
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x7C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x78
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor81:
    CALL	CURX1
    
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x8C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x88
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor91:
    CALL	CURX1
    
    MOVLW	0x98
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x9C
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0x98
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor101:
    CALL	CURX1
    
    MOVLW	0xA8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xAC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xA8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor111:
    CALL	CURX1
    
    MOVLW	0xB8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xBC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xB8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor121:
    CALL	CURX1
    
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xCC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xC8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor131:
    CALL	CURX1
    
    MOVLW	0xD8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xDC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xD8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor141:
    CALL	CURX1
    
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xEC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xE8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
Cursor151:
    CALL	CURX1
    
    MOVLW	0xF8
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xFC
    MOVWF	NIB1
    CALL	LCD_Write
    MOVLW	0xF8
    MOVWF	NIB1
    CALL	LCD_Write
    
    CALL	DELAY_4
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Cailibration Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Calibration:
   MOVLW	b'00000101'
   MOVWF	ADCON0
   CALL		RX_DATA
   CALL		ADC_FUNC
   MOVFF	STORED,BreN
   CALL		REG_PRINT
   CLRF		STORED
Inhale_Message
   CALL		Cursor00
   MOVLW	'I'
   CALL		LCD_PRINT
   MOVLW	'n'
   CALL		LCD_PRINT
   CALL		RX_DATA
   CALL		ADC_FUNC
   MOVFF	STORED,BreI
   CALL		REG_PRINT
   CLRF		STORED
Exhale_Message
   CALL		Cursor00
   MOVLW	'O'
   CALL		LCD_PRINT
   MOVLW	'u'
   CALL		LCD_PRINT
   MOVLW	't'
   CALL		LCD_PRINT
   CALL		RX_DATA
   CALL		ADC_FUNC
   CALL		REG_PRINT
   MOVFF	STORED,BreO
   CLRF		STORED
   RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Breating Rate Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Check
   CALL		DerivativeADC
   CALL		Cursor130
   MOVFF	Timecnt1,Temp
   BCF		STATUS,C
   RRCF		Temp,1
   BCF		STATUS,C
   RRCF		Temp,0
   CALL		BIN_TO_ASCII
   MOVFF	RMDM,WREG
   CALL		LCD_PRINT
   MOVFF	RMDL,WREG
   CALL		LCD_PRINT
   MOVLW	'S'
   CALL		LCD_PRINT
   BTFSS	PORTA,4 ;Check for postive for negative change
   GOTO		OUT
IN CALL		DELAY_1	    ;Call a 171ms delay
   CALL		Cursor00    ;Sets cursor on LCD to (0,0)
   MOVLW	'I'	    ;Move 'I' to WREG
   CALL		LCD_PRINT   ;Print WREG to LCD
   MOVLW	'n'	    ;Move 'n' to WREG
   CALL		LCD_PRINT   ;Print WREG to LCD
   MOVLW	'.'	    ;Move '.' to WREG
   CALL		LCD_PRINT   ;Print WREG to LCD
   MOVLW	' '	    ;Move ' ' to WREG	    
   CALL		LCD_PRINT   ;Print WREG to LCD
   CALL		Moving_Average
IN1
   CALL		Cursor130
   MOVFF	Timecnt1,Temp
   BCF		STATUS,C
   RRCF		Temp,1
   BCF		STATUS,C
   RRCF		Temp,0
   CALL		BIN_TO_ASCII
   MOVFF	RMDM,WREG
   CALL		LCD_PRINT
   MOVFF	RMDL,WREG
   CALL		LCD_PRINT
   MOVLW	'S'
   CALL		LCD_PRINT
   CALL		DerivativeADC	    ;CAll DerivativeADC
   BTFSS	PORTA,4	     ;Check for postive for negative change
   GOTO		OUT
   GOTO		IN1
OUT
    CALL	DELAY_1	    ;Call a 171ms delay
    CALL	Cursor00    ;Sets cursor on LCD to (0,0)
    MOVLW	'O'	    ;Move 'O' to WREG
    CALL	LCD_PRINT   ;Print WREG to LCD
    MOVLW	'u'	    ;Move 'O' to WREG
    CALL	LCD_PRINT   ;Print WREG to LCD
    MOVLW	't'	    ;Move 'O' to WREG
    CALL	LCD_PRINT   ;Print WREG to LCD
    MOVLW	'.'	    ;Move 'O' to WREG	    
    CALL	LCD_PRINT   ;Print WREG to LCD
    MOVFF	Timecnt,WREG;Move value from Timecnt to WREG
    CPFSEQ	MINUTE	    ;Compare to MINUTE value
    GOTO	Check	    ;Loop back to the top
    GOTO	Dead_Alarm
 

Moving_Average:			;Compute running average and print to LCD
   BCF		INTCON,GIE	;Global Interupts Disabled    
   MOVLW	0xFF
   ANDWF	TimeAvg,0
   MOVWF	TimeAvgOld
   BCF		STATUS,C	;Clear the carry flag
    ;Shift is the same divide
   RRCF		TimeAvgOld,1	;Divide by 2
   BCF		STATUS,C
   RRCF		TimeAvgOld,0	;Divide by 2 again
   SUBWF	TimeAvg,1	;STORE_OLD/4 - STORED
    ;
   BCF		STATUS,C	;Clear the carry flag
   RRCF		Timecnt,1
   BCF		STATUS,C
   RRCF		Timecnt,0
    ;
   ADDWF	TimeAvg,1
   BCF		STATUS,C
   ADDWF	TimeAvg,0
   BCF		STATUS,C
     
   BSF		INTCON,GIE	;Global Interupts Enable
   MOVLW	0x00
   MOVWF	Timecnt
   ;MOVFF	TimeAvg,CCPR4L	;Use the BPM to change the PWM
   CALL		DELAY_2
   CALL		Normal
   MOVFF	TimeAvg,WREG
   CPFSLT	DIFFHIGH	;Comapre greater than
   CALL		DifficultyF
   CPFSGT	DIFFLOW
   CALL		DifficultyL
Breathrate:
    MOVFF	MINUTE,NUME
    MOVFF	TimeAvg,WREG	
    CLRF	QU
B_1
    INCF	QU,F
    SUBWF	NUME,F
    BC		B_1
    DECF	QU,F
    ADDWF	NUME,F
    MOVFF	NUME,BRL
    MOVFF	QU,BRM
    CLRF	QU
    MOVFF	BRM,WREG
    CALL	BIN_TO_ASCII
    CALL	DELAY_1
    CALL	Cursor41
    CALL	DELAY_1
    MOVFF	RMDM,WREG
    CALL	LCD_PRINT
    MOVFF	RMDL,WREG
    CALL	LCD_PRINT
    MOVFF	BRL,WREG
    CALL	BIN_TO_ASCII
    Call	Cursor61
    MOVLW	','
    CALL	LCD_PRINT
    MOVFF	RMDM,WREG
    CALL	LCD_PRINT
    MOVFF	RMDL,WREG
    CALL	LCD_PRINT
    CALL	DELAY_1
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Touch Sensor Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Capsensor
    CALL	DELAY_1
    BCF		PORTA,4
    MOVLW	b'10101110'; right justify 
    MOVWF	ADCON2     ; Frc , Acquisition Time
    BSF		PORTA,4
    MOVLW	b'00010001' ; Point ADC to pin B5
    MOVWF	ADCON0 
    CALL	DELAY_1
    BCF		TRISA,2	    ;Set RA2 as output (Curiosity led D2)
    BCF		ANSELA,2    ;Set RA2 as digital
    BCF		PORTB,5
    BSF		TRISB,5	    ;Set RA2 as input
    MOVLW	b'00110101' ; Point ADC to pin B5
    MOVWF	ADCON0
    CALL	ADC_FUNC_1
    MOVFF	STORED,WREG
    CPFSLT	Cap
    GOTO	Capsensor
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Binary to ASCII
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
BIN_TO_DEC:
    MOVWF	NUME
    MOVLW	DEN
    CLRF	QU
D_1
    INCF	QU
    SUBWF	NUME
    BC		D_1
    ADDWF	NUME
    DECF	QU
    MOVFF	NUME,RMDL
    MOVFF	QU,NUME
    CLRF	QU
D_2
    INCF	QU
    SUBWF	NUME
    BC		D_2
    ADDWF	NUME
    DECF	QU
    MOVFF	NUME,RMDM
    MOVFF	QU,BRH
    CLRF	QU
    RETURN
DEC_TO_ASCII:
    ;Decimal to Ascii is done by adding 30h to the decimal value
   MOVLW	0x30
   ADDWF	BRH,1
   MOVLW	0x30
   ADDWF	RMDM,1
   MOVLW	0x30
   ADDWF	RMDL,1
   RETURN
BIN_TO_ASCII:
    CALL    BIN_TO_DEC
    CALL    DEC_TO_ASCII
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Interupt Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
ISR_PORT
    BCF		INTCON,GIE	;Disable interupts at the end of the function
    BCF		INTCON,RBIF	;Clears the change to port B<4:7> intterupt
    BCF		IOCB,5
    CALL	Capsensor
    
    CALL	REG_PRINT
    BSF		TRISB,5
    BCF		ANSELB,5
    MOVLW	b'00000101' ; AN1,ADC on
    MOVWF	ADCON0 
    ;ADCON1
    MOVLW	b'00000000' ; ADC ref = Vdd,Vss
    MOVWF	ADCON1
    ;ADCON 2
    MOVLW	b'00101110'; left justify 
    MOVWF	ADCON2     ; Frc , Acquisition Time
    BSF		ADCON0,GO  ;Starts the adc module
    BSF		INTCON,GIE ;Enable interupts at the end of the function
    BSF		PORTA,7
    BCF		PORTB,6
    RETFIE			;Return from interupt  
    
ISR_TIME
    BCF		INTCON,GIE	;Disables global interupts
    INCF	Timecnt		;Time inbetween preiods
    INCF	Timecnt1
    BCF		INTCON,TMR0IF	;Clear the timer flag
    BSF		INTCON,GIE	;Enables global interupts
    MOVFF	Timecnt1,WREG
    CPFSEQ	MINUTE		;If Timecnt = 60 skip
    RETFIE
    MOVLW	0x02
    MOVWF	Timecnt
    CLRF	Timecnt1
    RETFIE
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Alarm Programs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
Dead_Alarm	;Alarm that triggers when there is no breathing for 60 seconds
    BCF		INTCON,TMR0IE
    BSF		INTCON,GIE	;Global Interupts Enable
    BSF		IOCB,5
    BSF		PORTB,6
    BSF		PORTA,4
    CALL	DELAY_1
    BSF		PORTA,5
    CALL	DELAY_1
    BSF		PORTA,6
    CALL	DELAY_1
    BSF		PORTA,7
    CALL	DELAY_1
    CALL	Cursor00
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'E'
    CALL	LCD_PRINT
    MOVLW	'A'
    CALL	LCD_PRINT
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'!'
    CALL	LCD_PRINT
    CALL	Cursor41
    CALL	DELAY_1
    MOVLW	'0'
    CALL	LCD_PRINT
    MOVLW	'0'
    CALL	LCD_PRINT
    MOVLW	'0'
    CALL	BIN_TO_ASCII
    Call	Cursor61
    MOVLW	','
    CALL	LCD_PRINT
    MOVLW	'0'
    CALL	LCD_PRINT
    MOVLW	'0'
    CALL	LCD_PRINT
    BTFSS	PORTB,6
    GOTO	Start
    GOTO	Dead_Alarm
DifficultyF:
    BCF		PORTA,6
    BCF		PORTA,7
    CALL	Cursor131
    MOVLW	'B'
    CALL	LCD_PRINT
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'!'
    CALL	LCD_PRINT
    BTG		PORTA,5
    BSF		PORTA,6
    RETURN
DifficultyL:
    BCF		PORTA,6
    BCF		PORTA,7
    CALL	Cursor131
    MOVLW	'B'
    CALL	LCD_PRINT
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'!'
    CALL	LCD_PRINT
    BTG		PORTA,5
    BSF		PORTA,7
    RETURN
Normal:
    CALL	Cursor131
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    BCF		PORTA,5
    BCF		PORTA,6
    BCF		PORTA,7	
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EEPROM READ/Write Sub Program
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
EEPROM_WRITE_HELLO:
    CALL	Cursor70
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000000'
    MOVWF	ADDRESS
    MOVLW	'H'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor80
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000001'
    MOVWF	ADDRESS
    MOVLW	'E'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    
    CALL	Cursor90
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000010'
    MOVWF	ADDRESS
    MOVLW	'L'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor100
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000011'
    MOVWF	ADDRESS
    MOVLW	'O'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor110
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000100'
    MOVWF	ADDRESS
    MOVLW	' '
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor120
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000101'
    MOVWF	ADDRESS
    MOVLW	'W'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor130
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000110'
    MOVWF	ADDRESS
    MOVLW	'R'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor140
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00000111'
    MOVWF	ADDRESS
    MOVLW	'D'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	Cursor150
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	b'00001000'
    MOVWF	ADDRESS
    MOVLW	'!'
    MOVWF	I2C_DATA
    CALL	EEPROM_WRITE_EX
    CALL	DELAY_1
    CALL	DELAY_1
    
    CALL	EEPROM_MESSAGE_R
    
    CALL	Cursor71
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'0'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_H
    
    CALL	Cursor81
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'1'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_E
    
    CALL	Cursor91
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'2'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_L
    
    CALL	Cursor101
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'3'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_O
    
    CALL	Cursor111
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'4'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_SPACE
    
    CALL	Cursor121
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'5'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_W
    
    CALL	Cursor131
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'6'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_R
    
    CALL	Cursor141
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'7'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_D
    
    CALL	Cursor151
    MOVLW	'.'
    CALL	LCD_PRINT
    MOVLW	D'8'
    MOVWF	ADDRESS
    CALL	EEPROM_READ_EX
    MOVFF	I2C_DATA,CHAR_EX
    
    
    RETURN
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;EEPROM PRINT HELLO WORLD 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;   
HELLO_WORLD:
    CALL	LCD_CLEAR
    CALL	Cursor00
    MOVFF	CHAR_H,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_H,WREG
    CALL	LCD_PRINT
    
    MOVFF	CHAR_E,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_E,WREG
    CALL	LCD_PRINT
    
    MOVFF	CHAR_L,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_L,WREG
    CALL	LCD_PRINT
    
    
    MOVFF	CHAR_L,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_L,WREG
    CALL	LCD_PRINT
    
    MOVFF	CHAR_O,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_O,WREG
    CALL	LCD_PRINT
   
    
    MOVFF	CHAR_SPACE,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_SPACE,WREG
    CALL	LCD_PRINT
  
    
    MOVFF	CHAR_W,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_W,WREG
    CALL	LCD_PRINT
   
    
    MOVFF	CHAR_O,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_O,WREG
    CALL	LCD_PRINT
    
    
    MOVFF	CHAR_R,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_R,WREG
    CALL	LCD_PRINT
   
    
    MOVFF	CHAR_L,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_L,WREG
    CALL	LCD_PRINT
   
    
    MOVFF	CHAR_D,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_D,WREG
    CALL	LCD_PRINT
   
    
    MOVFF	CHAR_EX,WREG
    CALL	BYTE_TX
    MOVFF	CHAR_EX,WREG
    CALL	LCD_PRINT
   
    CALL	MES_RET
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    RETURN
BPM_MESSAGE_1
    CALL	LCD_CLEAR
    CALL	Cursor01
    MOVLW	'B'
    CALL	LCD_PRINT
    MOVLW	'P'
    CALL	LCD_PRINT
    MOVLW	'M'
    CALL	LCD_PRINT
    MOVLW	':'
    CALL	LCD_PRINT
    RETURN
    
BPM_MESSAGE_2
   ;Wrtite BPM to screen
    CALL	Cursor130
    MOVFF	Timecnt1,Temp
    BCF		STATUS,C
    RRCF	Temp,1
    BCF		STATUS,C
    RRCF	Temp,0
    CALL	BIN_TO_ASCII
    MOVFF	RMDM,WREG
    CALL	LCD_PRINT
    MOVFF	RMDL,WREG
    CALL	LCD_PRINT
    MOVLW	'S'
    CALL	LCD_PRINT
    RETURN
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Instruction Messages Subroutines
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;    
Startup_Mes
    CALL	LCD_CLEAR
    CALL	Cursor00
    MOVLW	'P'
    CALL	LCD_PRINT
    MOVLW	'l'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	'a'
    CALL	LCD_PRINT
    MOVLW	's'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'P'
    CALL	LCD_PRINT
    MOVLW	'r'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	's'
    CALL	LCD_PRINT
    MOVLW	's'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'T'
    CALL	LCD_PRINT
    MOVLW	'h'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    CALL	Cursor01
    MOVLW	'S'
    CALL	LCD_PRINT
    MOVLW	'w'
    CALL	LCD_PRINT
    MOVLW	'i'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'c'
    CALL	LCD_PRINT
    MOVLW	'h'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'T'
    CALL	LCD_PRINT
    MOVLW	'o'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'S'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'a'
    CALL	LCD_PRINT
    MOVLW	'r'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'.'
    CALL	LCD_PRINT
    RETURN
    
EEPROM_MESSAGE
    CALL	LCD_CLEAR
    CALL	Cursor00
    MOVLW	'W'
    CALL	LCD_PRINT
    MOVLW	'R'
    CALL	LCD_PRINT
    MOVLW	'I'
    CALL	LCD_PRINT
    MOVLW	'T'
    CALL	LCD_PRINT
    MOVLW	'I'
    CALL	LCD_PRINT
    MOVLW	'N'
    CALL	LCD_PRINT
    MOVLW	'G'
    CALL	LCD_PRINT 
    RETURN
    
EEPROM_MESSAGE_R
    CALL	Cursor01
    MOVLW	'R'
    CALL	LCD_PRINT
    MOVLW	'E'
    CALL	LCD_PRINT
    MOVLW	'A'
    CALL	LCD_PRINT
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'I'
    CALL	LCD_PRINT
    MOVLW	'N'
    CALL	LCD_PRINT
    MOVLW	'G'
    CALL	LCD_PRINT
    RETURN
EEPROM_DISPLAY_MESSAGE
    CALL	LCD_CLEAR
    CALL	Cursor00
    MOVLW	'N'
    CALL	LCD_PRINT
    MOVLW	'o'
    CALL	LCD_PRINT
    MOVLW	'w'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'p'
    CALL	LCD_PRINT
    MOVLW	'r'
    CALL	LCD_PRINT
    MOVLW	'i'
    CALL	LCD_PRINT
    MOVLW	'n'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'n'
    CALL	LCD_PRINT
    MOVLW	'g'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'd'
    CALL	LCD_PRINT
    MOVLW	'a'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'a'
    CALL	LCD_PRINT
    CALL	Cursor01
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'o'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	'L'
    CALL	LCD_PRINT
    MOVLW	'C'
    CALL	LCD_PRINT
    MOVLW	'D'
    CALL	LCD_PRINT
    MOVLW	'&'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	'r'
    CALL	LCD_PRINT
    MOVLW	'm'
    CALL	LCD_PRINT
    MOVLW	'i'
    CALL	LCD_PRINT
    MOVLW	'n'
    CALL	LCD_PRINT
    MOVLW	'a'
    CALL	LCD_PRINT
    MOVLW	'l'
    CALL	LCD_PRINT
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    CALL	DELAY_1
    RETURN

    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Calibration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;     
Cal 
    CALL	Cursor00
    MOVLW	'S'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	's'
    CALL	LCD_PRINT
    MOVLW	'e'
    CALL	LCD_PRINT
    MOVLW	'n'
    CALL	LCD_PRINT
    MOVLW	's'
    CALL	LCD_PRINT
    MOVLW	'o'
    CALL	LCD_PRINT
    MOVLW	'r'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	't'
    CALL	LCD_PRINT
    MOVLW	'o'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    CALL	DELAY_3
    CALL	Cursor01
    MOVLW	'1'
    CALL	LCD_PRINT
    MOVLW	'2'
    CALL	LCD_PRINT
    MOVLW	'7'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
    MOVLW	':'
    CALL	LCD_PRINT
    MOVLW	' '
    CALL	LCD_PRINT
Pressure:
    CALL	DELAY_1
    CALL	ADC_FUNC
    CALL	DELAY_1
    CALL	REG_PRINT
    MOVFF	STORED,WREG
    MOVWF	NUME
    MOVLW	DEN
    CLRF	QU
E_1
    INCF	QU
    SUBWF	NUME
    BC		E_1
    ADDWF	NUME
    DECF	QU
    MOVFF	NUME,NumL
    MOVFF	QU,NUME
    CLRF	QU
E_2
    INCF	QU
    SUBWF	NUME
    BC		E_2
    ADDWF	NUME
    DECF	QU
    MOVFF	NUME,NumH
    MOVFF	QU,WREG
    ADDLW	0x30
    CALL	LCD_PRINT
    CLRF	QU
    CALL	DELAY_3
    MOVFF	NumH,WREG
    ADDLW	0x30
    CALL	LCD_PRINT
    CALL	DELAY_3
    MOVFF	NumL,WREG
    ADDLW	0x30
    CALL	LCD_PRINT
    CALL	DELAY_1
    BTFSS	PORTB,4
    GOTO	Main1
    GOTO	Cal
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;Main Program
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  
Main 
    CALL	LCD_CLEAR
    CALL	Cal
Main1
    CALL	EEPROM_MESSAGE
    CALL	EEPROM_WRITE_HELLO
    CALL	EEPROM_DISPLAY_MESSAGE
    CALL	HELLO_WORLD
    
    CALL	Startup_Mes
    BSF		INTCON,GIE	;Global Interupts Enable
    BSF		IOCB,5
    BTFSS	PORTA,7
    GOTO	$-2
       
    CALL	BPM_MESSAGE_1
    BSF		T0CON,TMR0ON    ;Turn on timer0
    BSF		T2CON,TMR2ON	;Turn on timer2
    CALL	BPM_MESSAGE_2
    
    GOTO	Check  
    end