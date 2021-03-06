/* 
 * File:   newmain.c
 * Author: Celil Can ANIK, Erdem Onat ARDALI, Oguzhan AKBAS
 * Created on 09 Kasim 2018 �arsamba, 20:28
 * 
 * First scene shows "Press OK to insert parameter" on LCD
 * Output scene shows the entered parameters on LCD
 * Position scene shows input mode for position on LCD
 * Velocity scene shows input mode for velocity on LCD
 * Proportional gain scene shows input mode for proportional gain on LCD
 * Integral gain scene shows input mode for integral gain on LCD
 * Derivative gain scene shows input mode for derivative gain on LCD
 * Input mode is the mode for entering parameters
 */

#include <stdio.h>
#include <stdlib.h>
#include <pic18f46k22.h>
#include <htc.h>
#include "LCD.H"
#include <math.h>
#define _XTAL_FREQ 8000000
#pragma config FOSC = HSHP
#pragma config WDTEN = OFF

#define RC0 PORTCbits.RC0
#define RC1 PORTCbits.RC1
#define RC2 PORTCbits.RC2
#define RE0 PORTEbits.RE0
#define RE1 PORTEbits.RE1
#define RE2 PORTEbits.RE2
#define MotorA PORTCbits.RC3
#define MotorB PORTDbits.RD0

#define A PORTBbits.RB0
#define B PORTBbits.RB1
#define A_not PORTBbits.RB2
#define B_not PORTBbits.RB3

#define p_controller_MAX 49
#define p_controller_MIN -49
#define v_controller_MAX 1000
#define v_controller_MIN -1000
#define p_integral_MAX   2000
#define p_integral_MIN   -2000
#define v_integral_MAX   2000
#define v_integral_MIN   -2000

// dummy variables
int c_o=0;
char run = 0;
// dummy variables end

unsigned char str[13] = "            ";
unsigned char str2[4] = "   ";
int cnt = 0; //Indicator for which digit cursor is on
int first_dig = 0; //First digit
int second_dig = 0; //Second digit
int third_dig = 0; //Third digit
int fourth_dig = 0; //Fourth digit
unsigned int scene_n = 0; //Scene number
double v_Kp = 0.0; //Proportional gain
double v_Ki = 0.0; //Integral gain
double v_Kd = 0.0; //Derivative gain
double p_Kp = 0.0; //Proportional gain
double p_Ki = 0.0; //Integral gain
double p_Kd = 0.0; //Derivative gain
char input_mode; //Input mode flag
bit isFirstScene; //First Scene flag
bit isEsc; //Escape button flag

//Old and new bits for catching rising trigger of buttons
bit RC0_old = 0;
bit RC1_old = 0;
bit RC2_old = 0;
bit RE0_old = 0;
bit RE1_old = 0;
bit RE2_old = 0;
bit RC0_new = 0;
bit RC1_new = 0;
bit RC2_new = 0;
bit RE0_new = 0;
bit RE1_new = 0;
bit RE2_new = 0;

long long int v_error = 0;
long long int v_old_error = 0;
int v_set_point = 0;
long long int v_integral = 0;
int v_derivative = 0;
int v_controller_out = 0;
long long double p_error = 0;
long long double p_old_error = 0;
long long double p_set_point = 0;
long long double p_integral = 0;
int p_derivative = 0;
int p_controller_out = 0;

unsigned int velocity = 0; // ANGULAR VELOCITY ( PULSE / dt )
long int position = 0;     // POSITION in PULSES
long int old_position = 0;

void first_scene(void); //Shows the first scene on LCD
void posVel_output_scene(void); //Shows the output scene on LCD for position and velocity
/*//  Comment next two functions to display both position and velocity gains
void K1_output_scene(void); //Shows the first output scene on LCD for gains 
void K2_output_scene(void); //Shows the second output scene on LCD for gains*/
//  Uncomment to display both position and velocity gains
void p_K1_output_scene(void); //Shows the first output scene on LCD for gains for position
void p_K2_output_scene(void); //Shows the second output scene on LCD for gains for position
void v_K1_output_scene(void); //Shows the first output scene on LCD for gains for velocity
void v_K2_output_scene(void); //Shows the second output scene on LCD for gains for velocity 
//
void pos_scene(void); //Shows the position scene on LCD
//  Uncomment to display input mode for velocity
void vel_scene(void); //Shows the velocity scene on LCD */
//
/*  Comment next three functions to display both position and velocity gains
void kp_scene(void); //Shows the proportional gain scene on LCD
void ki_scene(void); //Shows the integral gain scene on LCD
void kd_scene(void); //Shows the derivative gain scene on LCD*/
//  Uncomment to display both position and velocity gains
void p_kp_scene(void); //Shows the proportional gain scene on LCD for position
void p_ki_scene(void); //Shows the integral gain scene on LCD for position
void p_kd_scene(void); //Shows the derivative gain scene on LCD for position
void v_kp_scene(void); //Shows the proportional gain scene on LCD for velocity
void v_ki_scene(void); //Shows the proportional gain scene on LCD for velocity
void v_kd_scene(void); //Shows the proportional gain scene on LCD for velocity */
//
void delay(int ms); // delay
int velocity_control(int set_p); // velocity control loop
int position_control(int set_p); // position control loop

int main(int argc, char** argv) {
    ANSELA = 0;
    TRISA = 0xC0;
    PORTA = 0;

    ANSELB = 0x00; // ENCODER INPUTS
    TRISB = 0x6F;

    ANSELC = 0;
    TRISC = 0xF7;
    PORTC = 0;

    ANSELD = 0; // PWM OUTPUT
    TRISD = 0x00;
    PORTD = 0;

    ANSELE = 0;
    TRISE = 0x07;
    PORTE = 0;

    // ENCODER INPUT CONFIG  -- CONTROL LOOP TIMER CONFIG 
    INTCON2bits.RBPU = 0;
    WPUB = 0x00;
    INTCON = 0;
    INTCONbits.TMR0IE = 1;        // bit5 TMR0 Overflow Interrupt Enable bit...1 = Enables the TMR0 interrupt
    INTCONbits.TMR0IF = 0;        // Timer0 flag
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits. INT0IE = 1;
    INTCON2bits.INTEDG0 = 1;
    INTCON2bits.INTEDG1 = 1;
    INTCON3bits.INT1IE = 1;
    INTCONbits.INT0IF = 0;
    INTCON3bits.INT1IF = 0;
    
    TMR0 = 6;       // Timer0 (Control loop interupt initial value) - 125 Hz , 8 ms

    CCPTMRS0 = 0x00;
    CCPTMRS1 = 0x00; //  ' select TM1/TM2 for CCP4 use ( TIMER2 used for PWM)                      
    CCP4CON = 0x3C; // ' Setup CCP4 single output for PWM , bits 3,2 , bits 5,4 = 9th and 10 bit of duty cycle ,
    CCPR4L = 0x00; // ' PWM Value 

    PR2 = 0x31; //' set the PWM period = [(PR2+1)x 4xTosc / TM2 prescale value  
    T2CON = 0x04; //' Timer 2 ON - POSTSCALLER SETTING NOT USED BY PWM , OLY PRESCALE SETTING - PRESCALE: 00 =1 ,01 = 4,1x = 16
    // ' ( bit 7 = x, bit6-3 postscale, bit2=0 timer off ,bit1-0 = prescale )
    TMR2 = 49; //TMR2-1 must be maximum CCPR4L value 

    lcd_init();
    first_scene();

    while (1) {
        RC0_new = RC0;
        RC1_new = RC1;
        RC2_new = RC2;
        RE0_new = RE0;
        RE1_new = RE1;
        RE2_new = RE2;
                
        if (scene_n == 1) p_set_point = first_dig * 1000 + second_dig * 100 + third_dig * 10 + fourth_dig;
        // Uncomment to display velocity
        if (scene_n == 2) v_set_point = first_dig * 1000 + second_dig * 100 + third_dig * 10 + fourth_dig;
        //
        // Increment scene_n by 1 under this comment if velocity is displayed / Incremented
        if (scene_n == 3) p_Kp = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        if (scene_n == 4) p_Ki = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        if (scene_n == 5) p_Kd = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        //  Uncomment to display both position and velocity gains
        if (scene_n == 6) v_Kp = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        if (scene_n == 7) v_Ki = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        if (scene_n == 8) v_Kd = first_dig + second_dig / 10.0 + third_dig / 100.0 + fourth_dig / 1000.0;
        //

        //Up button
        if (RC0_old == 0 && RC0_new == 1 && input_mode) {
            // Increment scene_n by 1 if velocity is displayed / Incremented
            if (cnt == 0) {
                first_dig++;
                if (first_dig > 9) first_dig = 0;
                sprintf(str2, "%d", first_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 1) {
                second_dig++;
                if (second_dig > 9) second_dig = 0;
                sprintf(str2, "%d", second_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 2) {
                third_dig++;
                if (third_dig > 9) third_dig = 0;
                sprintf(str2, "%d", third_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 3) {
                fourth_dig++;
                if (fourth_dig > 9) fourth_dig = 0;
                sprintf(str2, "%d", fourth_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
        }

        //Down button
        if (RC1_old == 0 && RC1_new == 1 && input_mode) {
            // Increment scene_n by 1 if velocity is displayed / Incremented
            if (cnt == 0) {
                first_dig--;
                if (first_dig < 0) first_dig = 9;
                sprintf(str2, "%d", first_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 1) {
                second_dig--;
                if (second_dig < 0) second_dig = 9;
                sprintf(str2, "%d", second_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 2) {
                third_dig--;
                if (third_dig < 0) third_dig = 9;
                sprintf(str2, "%d", third_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
            if (cnt == 3) {
                fourth_dig--;
                if (fourth_dig < 0) fourth_dig = 9;
                sprintf(str2, "%d", fourth_dig);
                lcd_puts(str2);
                if (scene_n > 2 && cnt == 0) lcd_goto(64);
                else if (scene_n > 2 && cnt == 1) lcd_goto(66);
                else if (scene_n > 2 && cnt == 2) lcd_goto(67);
                else if (scene_n > 2 && cnt == 3) lcd_goto(68);
                else lcd_goto(64 + cnt);
            }
        }

        //Right button
        if (RC2_old == 0 && RC2_new == 1 && input_mode) {
            // Increment scene_n by 1 if velocity is displayed / Incremented
            cnt++;
            if (cnt == 4) cnt = 0;
            if (scene_n > 3 && cnt == 0) lcd_goto(64);
            else if (scene_n > 2 && cnt == 1) lcd_goto(66);
            else if (scene_n > 2 && cnt == 2) lcd_goto(67);
            else if (scene_n > 2 && cnt == 3) lcd_goto(68);
            else lcd_goto(64 + cnt);
        }

        //Left button
        if (RE0_old == 0 && RE0_new == 1 && input_mode) {
            // Increment scene_n by 1 if velocity is displayed / Increment
            cnt--;
            if (cnt < 0) cnt = 3;
            if (scene_n > 3 && cnt == 0) lcd_goto(64);
            else if (scene_n > 2 && cnt == 1) lcd_goto(66);
            else if (scene_n > 2 && cnt == 2) lcd_goto(67);
            else if (scene_n > 2 && cnt == 3) lcd_goto(68);
            else lcd_goto(64 + cnt);
        }

        //OK button
        if (RE1_old == 0 && RE1_new == 1 || isEsc) {
            input_mode = 1;
            isFirstScene = 0;
            /*
            // If velocity is displayed, comment to if(scene_n < 6) 
            if(scene_n < 4) {
                first_dig = 0;
                second_dig = 0;
                third_dig = 0;
                fourth_dig = 0;
            }
            
            lcd_clear();

            //If velocity is displayed, increment scene_n by 1
            if (scene_n > 3) input_mode = 0;

            if (scene_n == 0) pos_scene();
            //if (scene_n == 1) vel_scene(); Uncomment to display input mode for velocity
            // If velocity is displayed, increment scene_n by 1 for every scene under this comment
            if (scene_n == 1) kp_scene();  
            if (scene_n == 2) ki_scene();
            if (scene_n == 3) kd_scene();
            if (scene_n == 5) K1_output_scene();

            if (scene_n == 6) {
                K2_output_scene();
                scene_n = 3;
            }
            if (scene_n < 6) {
                scene_n++;
                cnt = 0;
                lcd_write(0xE);
            }*/
            
            //  Uncomment to display input mode for both position and
             //  velocity gains. Do not forget to uncomment the functions.
             //
            if(scene_n < 8) {
                first_dig = 0;
                second_dig = 0;
                third_dig = 0;
                fourth_dig = 0;
            }
             
             lcd_clear();
            
            if (scene_n > 7) input_mode = 0;

            if (scene_n == 0) pos_scene();
            if (scene_n == 1) vel_scene();
            if (scene_n == 2) p_kp_scene();
            if (scene_n == 3) p_ki_scene();
            if (scene_n == 4) p_kd_scene();
            if (scene_n == 5) v_kp_scene();
            if (scene_n == 6) v_ki_scene();
            if (scene_n == 7) v_kd_scene();
            if (scene_n == 9) p_K1_output_scene();
            if (scene_n == 10) p_K2_output_scene();
            if (scene_n == 11) v_K1_output_scene();

            if (scene_n == 12) {
                v_K2_output_scene();
                scene_n = 7;
            }
            if (scene_n < 12) {
                scene_n++;
                cnt = 0;
                lcd_write(0xE);
            }
            if (isEsc) isEsc = 0;
        }

        //Escape button
        if (RE2_old == 0 && RE2_new == 1) {
            if (!input_mode && !isFirstScene) {
                lcd_clear();
                scene_n = 0;
                isEsc = 1;
                first_dig = 0;
                second_dig = 0;
                third_dig = 0;
                fourth_dig = 0;
            }
        }
        RC0_old = RC0_new;
        RC1_old = RC1_new;
        RC2_old = RC2_new;
        RE0_old = RE0_new;
        RE1_old = RE1_new;
        RE2_old = RE2_new;
        
        //  For both position and velocity gain display
        if (scene_n == 9) posVel_output_scene();
        
        /*//If velocity is displayed, increment scene_n by 1
        if (scene_n == 5) posVel_output_scene(); */

        // ANY OTHER SLOW CODES
        

        
        
    }
    return (EXIT_SUCCESS);
}

void first_scene(void) {
    lcd_goto(0);
    lcd_puts("Press OK to");
    lcd_goto(64);
    lcd_puts("insert parameter");
    isFirstScene = 1;
}

void posVel_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    if(position < 0) sprintf(str, "Position: %06d", position);
    if(position >= 0) sprintf(str, "Position:  %05d", position);
    lcd_puts(str);
    // Uncomment to display velocity
    lcd_goto(64);
    sprintf(str, "Velocity:  %05d", velocity);
    lcd_puts(str);
    //
}

void K1_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Kp: %0.3f", p_Kp);
    lcd_puts(str);
    lcd_goto(64);
    sprintf(str, "Ki: %0.3f", p_Ki);
    lcd_puts(str);
}

void K2_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Kd: %0.3f", p_Kd);
    lcd_puts(str);
} 

//  Uncomment to display input mode for both position and velocity gains.
void p_K1_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Pos_Kp: %0.3f", p_Kp);
    lcd_puts(str);
    lcd_goto(64);
    sprintf(str, "Pos_Ki: %0.3f", p_Ki);
    lcd_puts(str);
}

void p_K2_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Pos_Kd: %0.3f", p_Kd);
    lcd_puts(str);
}
void v_K1_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Vel_Kp: %0.3f", v_Kp);
    lcd_puts(str);
    lcd_goto(64);
    sprintf(str, "Vel_Ki: %0.3f", v_Ki);
    lcd_puts(str);
}

void v_K2_output_scene(void) {
    lcd_write(0xC); //Closing cursor
    lcd_goto(0);
    sprintf(str, "Vel_Kd: %0.3f", v_Kd);
    lcd_puts(str);
}
//

void pos_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter position:");
    lcd_goto(64);
    sprintf(str2, "%d%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

//  Uncomment to display input mode for velocity
void vel_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter velocity:");
    lcd_goto(64);
    sprintf(str2, "%d%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}
//

void kp_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Kp:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void ki_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Ki:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void kd_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Kd:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

//  Uncomment to display both position and velocity gains
void p_kp_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Pos_Kp:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void p_ki_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Pos_Ki:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void p_kd_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Pos_Kd:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void v_kp_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Vel_Kp:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void v_ki_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Vel_Ki:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}

void v_kd_scene(void) {
    lcd_goto(0);
    lcd_puts("Enter Vel_Kd:");
    lcd_goto(64);
    sprintf(str2, "%d.%d%d%d", first_dig, second_dig, third_dig, fourth_dig);
    lcd_puts(str2);
    lcd_goto(64);
}
//

void interrupt high_priorty() {
    if (INT0IF) {
        if (!A_not && B) {
            position--;
        } else if (!A_not && !B) {
            position++;
        }
        INT0IF = 0;
    }
    if (INT1IF) {
        if (!B_not && A) {
            position++;
        } else if (!B_not && !A) {
            position--;
        }
        INT1IF = 0;
    }
    if (TMR0IF && run) {
        TMR0 = 6;
        c_o = position_control(p_set_point);
        if (c_o > 0) {
            MotorA = 1;
            MotorB = 0;
        }
        else if (c_o < 0){
            MotorA = 0;
            MotorB = 1;
            c_o = -c_o;
        }
        else {
            MotorA = 0;
            MotorB = 0;
            c_o = 0;
        }
        CCPR4L = c_o;
        TMR0IF = 0;   
    }
}

void delay(int ms) {
    for (int i = 1; i <= ms; i = i + 90) {
        __delay_us(90);
    }
}

int velocity_control(int v_set_point) {
    v_error = v_set_point - velocity;
    v_integral = floor(v_integral + v_error);
    if (v_integral > v_integral_MAX) {
        v_integral = v_integral_MAX;
    } else if (v_integral < v_integral_MIN) {
        v_integral = v_integral_MIN;
    }
    v_derivative = v_error - v_old_error;
    v_controller_out = floor(v_Kp * v_error + v_Ki * v_integral + v_Kd * v_derivative);
    v_error = v_old_error;

    if (v_controller_out > 1000) {
        v_controller_out = 1000;
    } else if (v_controller_out < -1000) {
        v_controller_out = -1000;
    }
    return v_controller_out;
}

int position_control(int p_set_point) {     // PI CONTROL
    p_error = p_set_point - position;
    if (p_error > -30 && p_error < 30) {
        return 0;
    }
    p_integral = floor(p_integral + p_error);
    /* if (p_integral > p_integral_MAX) {
        p_integral = p_integral_MAX;
    } else if (p_integral < p_integral_MIN) {
        p_integral = p_integral_MIN;
    } */
    p_derivative = p_error - p_old_error;
    p_controller_out = floor(p_Kp * p_error + p_Ki * p_integral + p_Kd * p_derivative);

    if (p_controller_out > p_controller_MAX) {
        p_controller_out = p_controller_MAX;
    } else if (p_controller_out < p_controller_MIN) {
        p_controller_out = p_controller_MIN;
    }
    
    return p_controller_out;
}