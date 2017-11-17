;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     WakeUp_IRQHandler         ; 16+0:  Wakeup 0 Handler
                DCD     PWM_Fault_IRQHandler      ; 16+1:  PWM Fault interrupt Handler
                DCD     I2C_IRQHandler            ; 16+2:  TWS interrupt Handler
                DCD     TIM0_IRQHandler           ; 16+3,  16-bit Timer0 Interrupt Handler 
                DCD     TIM1_IRQHandler           ; 16+4,  16-bit Timer1 Interrupt Handler
                DCD     TIM2_IRQHandler           ; 16+5,  16-bit Timer2 Interrupt Handler
                DCD     TIM3_IRQHandler           ; 16+6,  16-bit Timer3 Interrupt Handler
                DCD     UART0_IRQHandler          ; 16+7,  UART0 Interrupt Handler   
                DCD     UART1_IRQHandler          ; 16+8,  UART1 Interrupt Handler    
                DCD     ADC_IRQHandler            ; 16+9,  A/D Converter Interrupt Handler
                DCD     WDT_IRQHandler            ; 16+10, Watchdog timer Interrupt Handler   
                DCD     BOD_IRQHandler            ; 16+11, Brown Out Detect(BOD) Interrupt Handler
                DCD     GPIOA_IRQHandler          ; 16+12, External Interrupt 0 Interrupt Handler
                DCD     GPIOB_IRQHandler          ; 16+13, External Interrupt 1 Interrupt Handler
                DCD     GPIOC_IRQHandler          ; 16+14, External Interrupt 2 Interrupt Handler
                DCD     RTC_IRQHandler            ; 16+15, RTC Interrupt Handler           
                DCD     SPI_IRQHandler            ; 16+16, SSP Interrupt Handler           
                DCD     PWM_Reload_IRQHandler     ; 16+17, PWM Timer Interrupt Handler     
                DCD     0           ; 16+18: Reserved                
                DCD     0           ; 16+19: Reserved
                DCD     0           ; 16+20: Reserved                
                DCD     0           ; 16+21: Reserved
                DCD     0           ; 16+22: Reserved                
                DCD     0           ; 16+23: Reserved
                DCD     0           ; 16+24: Reserved
                DCD     0           ; 16+25: Reserved
                DCD     0           ; 16+26: Reserved
                DCD     0           ; 16+27: Reserved
                DCD     0           ; 16+28: Reserved
                DCD     0           ; 16+29: Reserved
                DCD     0           ; 16+30: Reserved
                DCD     0           ; 16+31: Reserved   

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
                EXPORT  NMI_Handler               [WEAK]
                EXPORT  WakeUp_IRQHandler         [WEAK]
                EXPORT  PWM_Fault_IRQHandler      [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  TIM0_IRQHandler           [WEAK]
                EXPORT  TIM1_IRQHandler           [WEAK]
                EXPORT  TIM2_IRQHandler           [WEAK]
                EXPORT  TIM3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  GPIOA_IRQHandler          [WEAK]
                EXPORT  GPIOB_IRQHandler          [WEAK]
                EXPORT  GPIOC_IRQHandler          [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  SPI_IRQHandler            [WEAK]
                EXPORT  PWM_Reload_IRQHandler     [WEAK]


WakeUp_IRQHandler
PWM_Fault_IRQHandler 
I2C_IRQHandler
TIM0_IRQHandler
TIM1_IRQHandler
TIM2_IRQHandler
TIM3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
ADC_IRQHandler
WDT_IRQHandler
BOD_IRQHandler
GPIOA_IRQHandler  
GPIOB_IRQHandler 
GPIOC_IRQHandler
RTC_IRQHandler
SPI_IRQHandler
PWM_Reload_IRQHandler

                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap
__user_initial_stackheap

                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR

                ALIGN

                ENDIF


                END

