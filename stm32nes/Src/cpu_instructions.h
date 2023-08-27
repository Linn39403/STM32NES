#include "cpu_zpinstr.h"
//#define TODO_ENABLE
/* Set the Status register accroding to the value input */
#define SetFlag(f, v) \
    do { \
        if (v) { \
            P |= (f); \
        } else { \
            P &= ~(f); \
        } \
    } while (0)

/* Get the specific bit of Status register */
#define GetFlag(f) (((P) & (f)) > 0 ? 1 : 0)

/******************************************************************************************
**                                Load/Store Operations                                  **
******************************************************************************************/
/* LDA - Load Accumulator */
static inline void cpu_lda(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	A = cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80); /* 0x80 is the MSB bit to indicate the value is negative */
}   

/* LDX - Load X Register */
static inline void cpu_ldx(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	X = cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, X == 0x00);
	SetFlag(N_FLAG, X & 0x80); /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* LDY - Load Y Register */
static inline void cpu_ldy(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	Y = cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, Y == 0x00);
	SetFlag(N_FLAG, Y & 0x80); /* 0x80 is the MSB bit to indicate the value is negative */  
}   

/* STA - Store Accumulator */
static inline void cpu_sta(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	cpu_putmemory(savepc,A);   
}   

/* STX - Store X Register */
static inline void cpu_stx(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	cpu_putmemory(savepc,X);   
}   

/* STY - Store Y Register */
static inline void cpu_sty(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	cpu_putmemory(savepc,Y);   
}  


/******************************************************************************************
**                                   Register Transfers                                  **
******************************************************************************************/

/* TAX - Transfer Accumulator to X */
static inline void cpu_tax(CpuAdrMode adrmode)   
{   
	X=A;   
	SetFlag(Z_FLAG, X == 0x00);
	SetFlag(N_FLAG, X & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* TAY - Transfer Accumulator to Y */
static inline void cpu_tay(CpuAdrMode adrmode)   
{   
	Y=A;   
	SetFlag(Z_FLAG, Y == 0x00);
	SetFlag(N_FLAG, Y & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* TXA - Transfer X to Accumulator */
static inline void cpu_txa(CpuAdrMode adrmode)   
{   
	A=X;   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* TYA - Transfer Y to Accumulator */
static inline void cpu_tya(CpuAdrMode adrmode)   
{   
	A=Y;   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/******************************************************************************************
**                                   Stack Operations                                    **
******************************************************************************************/

/* TSX - Transfer Stack Pointer to X */
static inline void cpu_tsx(CpuAdrMode adrmode)   
{   
	X=S;   
	SetFlag(Z_FLAG, X == 0x00);
	SetFlag(N_FLAG, X & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* TXS - Transfer X to Stack Pointer */
static inline void cpu_txs(CpuAdrMode adrmode)   
{   
	S=X;   
}   

/* PHA - Push Accumulator */
static inline void cpu_pha(CpuAdrMode adrmode)   
{   
	cpu_pushstack(A);   
}   

/* PHP - Push Processor Status */
static inline void cpu_php(CpuAdrMode adrmode)   
{   
	cpu_pushstack(P);   
}   

/* PLA - Pull Accumulator */
static inline void cpu_pla(CpuAdrMode adrmode)   
{   
	A = cpu_popstack();   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* PLP - Pull Processor Status */
static inline void cpu_plp(CpuAdrMode adrmode)   
{    
	P = cpu_popstack();
	SetFlag(R_FLAG, 1);   
}

/******************************************************************************************
**                                     LOGICAL                                           **
******************************************************************************************/
/* AND - Locial AND */
static inline void cpu_and(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	A &= cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* EOR - Exclusive OR */
static inline void cpu_eor(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	A ^= cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */  
}   

/* ORA - Logical Inclusive OR */
static inline void cpu_ora(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	A |= cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, A == 0x00);
	SetFlag(N_FLAG, A & 0x80);  /* 0x80 is the MSB bit to indicate the value is negative */ 
}   

/* BIT - Bit Test */
static inline void cpu_bit(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint8_t value = cpu_getmemory(savepc);   
	SetFlag(Z_FLAG, (value & 0xFF) == 0x00);
	SetFlag(N_FLAG, value & N_FLAG);
	SetFlag(V_FLAG, value & V_FLAG);   
}   

/******************************************************************************************
**                                  ARITHMETIC                                           **
******************************************************************************************/
/* SBC - Subtract with Carry */
static inline void cpu_adc(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t value = cpu_getmemory(savepc);   
	uint16_t temp = (uint16_t)A + (uint16_t)value + (uint16_t)GetFlag(C_FLAG);

	/* The carry flag out exists in the high byte bit 0 */
	SetFlag(C_FLAG, temp > 255);

	/* The Zero flag is set if the result is 0 */
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0);

	/* The signed Overflow flag is set based on all that up there */
	SetFlag(V_FLAG, (~((uint16_t)A ^ (uint16_t)value) & ((uint16_t)A ^ (uint16_t)temp)) & 0x0080);

	/* The negative flag is set to the most significant bit of the result */
	SetFlag(N_FLAG, temp & 0x0080);

	/* Load the result into the accumulator */
	A = temp & 0x00FF;

#ifdef TODO_ENABLE
	if( (P & 0x08) == 0)
	{
		cpu_clockticks++;
	}
#endif
}  

/* SBC - Subtract with Carry */
static inline void cpu_sbc(CpuAdrMode adrmode)   
{   
	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	uint16_t value = cpu_getmemory(savepc) ^ 0xFF;   
    uint16_t temp = (uint16_t)A + (uint16_t)value + (uint16_t)GetFlag(C_FLAG);

	/* If there is a bit in the upper 8bit, set C_FLAG */
	SetFlag(C_FLAG, temp & 0xFF00);

	/* The Zero flag is set if the result is 0 */
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0);

	/* The signed Overflow flag is set based on all that up there */
	SetFlag(V_FLAG, (temp ^ (uint16_t)A) & (temp ^ value) & 0x0080);

	/* The negative flag is set to the most significant bit of the result */
	SetFlag(N_FLAG, temp & 0x0080);

	/* Load the result into the accumulator */
	A = temp & 0x00FF;

#ifdef TODO_ENABLE
	if( (P & 0x08) == 0)
	{
		cpu_clockticks++;
	}
#endif
}   

/* CMP - Compare */
static inline void cpu_cmp(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint8_t value = cpu_getmemory(savepc);   
	uint16_t temp = (uint16_t)A - (uint16_t)value;
	SetFlag(C_FLAG, A >= value);
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0x0000 );
	SetFlag(N_FLAG, temp & 0x0080);
}  

/* CPX - Compare X Register */
static inline void cpu_cpx(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint8_t value = cpu_getmemory(savepc);   
	uint16_t temp = (uint16_t)X - (uint16_t)value;
	SetFlag(C_FLAG, X >= value);
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, temp & 0x0080);
}   

/* CPY - Compare Y Register*/
static inline void cpu_cpy(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint8_t value = cpu_getmemory(savepc);   
	uint16_t temp = (uint16_t)Y - (uint16_t)value;
	SetFlag(C_FLAG, Y >= value);
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, temp & 0x0080); 
}  

/******************************************************************************************
**                               Increments & Decrements                                 **
******************************************************************************************/

/* INC - Increment Memory */
static inline void cpu_inc(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t temp = cpu_getmemory(savepc);   
	temp++;   
	cpu_putmemory(savepc,temp & 0x00FF);   
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, temp & 0x0080);
}   

/* INX - Increment X Register*/
static inline void cpu_inx(CpuAdrMode adrmode)   
{   
	X++;   
	SetFlag(Z_FLAG, X == 0x00);
	SetFlag(N_FLAG, X & 0x80);
}   

/* INX - Increment Y Register */
static inline void cpu_iny(CpuAdrMode adrmode)   
{   
	Y++;   
	SetFlag(Z_FLAG, Y == 0x00);
	SetFlag(N_FLAG, Y & 0x80);  
}  

/* DEC - Decrement Memory */
static inline void cpu_dec(CpuAdrMode adrmode)   
{   
	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	uint16_t temp = cpu_getmemory(savepc);   
	temp--;   
	cpu_putmemory(savepc, temp);   
	SetFlag(Z_FLAG, (temp & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, temp & 0x0080);
}   

/* DEX - Decrement X Register */
static inline void cpu_dex(CpuAdrMode adrmode)   
{   
	X--;   
	SetFlag(Z_FLAG, X == 0x00);
	SetFlag(N_FLAG, X & 0x80);
}   

/* DEY - Decrement Y Register*/
static inline void cpu_dey(CpuAdrMode adrmode)   
{   
	Y--;   
	SetFlag(Z_FLAG, Y == 0x00);
	SetFlag(N_FLAG, Y & 0x80);    
}   

/******************************************************************************************
**                                        SHIFTS                                         **
******************************************************************************************/
/* ASL - Arithmetic Shift Left */
static inline void cpu_asl(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t value = cpu_getmemory(savepc);   
	value = value << 1;
	cpu_putmemory(savepc,value & 0x00FF);  
	SetFlag(C_FLAG, (value & 0xFF00) > 0);
	SetFlag(Z_FLAG, (value & 0x00FF) == 0x00);
	SetFlag(N_FLAG, value & 0x80);
}   

/* LSR - Logical Shift Right */
static inline void cpu_lsr(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t value = cpu_getmemory(savepc);   
	value = value >> 1;
	cpu_putmemory(savepc,value & 0x00FF);   
    SetFlag(C_FLAG, value & 0x0001);
	SetFlag(Z_FLAG, (value & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, value & 0x0080);  
}   

/* ROL - Rotate Left */
static inline void cpu_rol(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t value = cpu_getmemory(savepc);   
	value = (uint16_t)(value << 1) | GetFlag(C_FLAG);
	cpu_putmemory(savepc,value & 0x00FF);   
	SetFlag(C_FLAG, value & 0xFF00);
	SetFlag(Z_FLAG, (value & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, value & 0x0080);  
}   

/* ROR - Rotate Right */
static inline void cpu_ror(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	uint16_t value = cpu_getmemory(savepc);   
	value = (uint16_t)GetFlag(C_FLAG) << 7 | (value >> 1);
	cpu_putmemory(savepc,value & 0x00FF);   
	SetFlag(C_FLAG, value & 0x01);
	SetFlag(Z_FLAG, (value & 0x00FF) == 0x0000);
	SetFlag(N_FLAG, value & 0x0080);  
}   

/******************************************************************************************
**                                      BRANCHES                                         **
******************************************************************************************/
/* BCC - Branch if Carry Clear */
static inline void cpu_bcc(CpuAdrMode adrmode)   
{   
	if(GetFlag(C_FLAG)==0) 
	{   
		cpu_clockticks++;   
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{   
		PC++;   
	}   
}   

/* BCS - Branch if Carry Set */
static inline void cpu_bcs(CpuAdrMode adrmode)   
{   
	if(GetFlag(C_FLAG)) 
	{  
		cpu_clockticks++;   
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{    
		PC++;   
	}   
}   

/* BEQ - Branch if Equal  */
static inline void cpu_beq(CpuAdrMode adrmode)   
{   
	if (GetFlag(Z_FLAG))
	{
		cpu_clockticks++;   
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{   
		PC++;   
	}   
}   

/* BMI - Branch if Minus */
static inline void cpu_bmi(CpuAdrMode adrmode)   
{   
	if (GetFlag(N_FLAG))
	{   
		cpu_clockticks++;   
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{    
		PC++;   
	}   
}   

/* BNE - Branch if Not Equal */
static inline void cpu_bne(CpuAdrMode adrmode)   
{   
	if (GetFlag(Z_FLAG) == 0)
	{ 
		cpu_clockticks++; 
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{   
		PC++;   
	}   
}   

/* BPL - Branch if Positive */
static inline void cpu_bpl(CpuAdrMode adrmode)   
{   
	if (GetFlag(N_FLAG) == 0)
	{   
		cpu_clockticks++;   
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{     
		PC++;   
	}   
}   

/* BVC - Branch if Overflow Clear */
static inline void cpu_bvc(CpuAdrMode adrmode)   
{   
	if (GetFlag(V_FLAG) == 0)
	{   
		cpu_clockticks++;   		
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{   
		PC++;   
	}   
}   

/* BVS - Branch if Overflow Set */
static inline void cpu_bvs(CpuAdrMode adrmode)   
{   
	if (GetFlag(V_FLAG))
	{   
		cpu_clockticks++;   		
		uint16_t savepc = adrmode();   
		PC += savepc;   
		cpu_clockticks++;   
	}else{   
		PC++;   
	}   
}   

/******************************************************************************************
**                                Status Flag Changes                                    **
******************************************************************************************/
/* Clear Carry Flag */
static inline void cpu_clc(CpuAdrMode adrmode)   
{   
	SetFlag(C_FLAG, 0);
}   

/* Clear Decimal Mode Flag */
static inline void cpu_cld(CpuAdrMode adrmode)   
{   
	SetFlag(D_FLAG, 0);
}   

/* Clear interrupt disable flag */
static inline void cpu_cli(CpuAdrMode adrmode)   
{   
	SetFlag(I_FLAG, 0); 
}   

/* Clear overflow flag */
static inline void cpu_clv(CpuAdrMode adrmode)   
{   
	SetFlag(V_FLAG, 0);   
}  

/* Set carry flag */
static inline void cpu_sec(CpuAdrMode adrmode)   
{   
	SetFlag(C_FLAG, 1);
}   

/* Set decimal mode flag */
static inline void cpu_sed(CpuAdrMode adrmode)   
{   
	SetFlag(D_FLAG, 1); 
}   

/* Set interrupt disable flag */
static inline void cpu_sei(CpuAdrMode adrmode)   
{   
	SetFlag(I_FLAG, 1); 
} 

/******************************************************************************************
**                                System Functions                                       **
******************************************************************************************/
/* Force an interrupt */
static inline void cpu_brk(CpuAdrMode adrmode)   
{   
	PC++; 

	SetFlag(I_FLAG, 1);
	cpu_push16stack(PC);

	SetFlag(B_FLAG,1);
	cpu_pushstack(P);
	SetFlag(B_FLAG,0);
   
	PC = IRQ_VECTOR;  
}   

/* No Operation */
static inline void cpu_nop(CpuAdrMode adrmode)   
{   
}   

/* Return from interrupt */
static inline void cpu_rti(CpuAdrMode adrmode)   
{   
	P   = cpu_popstack();   
	P  &= ~B_FLAG;
	P  &= ~R_FLAG;
	PC = cpu_pop16stack();
}   
/******************************************************************************************
**                                        XXXXXX                                         **
******************************************************************************************/
static inline void cpu_asla(CpuAdrMode adrmode)   
{   
	P= (P & 0xfe) | ((A >>7) & 0x01);   
	A = A << 1;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   






 

static inline void cpu_jmp(CpuAdrMode adrmode)   
{   
	uint16_t savepc = adrmode();   
	PC=savepc;   
}   

static inline void cpu_jsr(CpuAdrMode adrmode)   
{   
	PC++;   
	// cpu_putmemory(0x0100+S--,(uint8_t)(PC >> 8));   \
	cpu_putmemory(0x0100+S--,(uint8_t)(PC & 0xff));   
	cpu_push16stack(PC);
	PC--;   
	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	PC=savepc;   
}   


/******************************************************************************************
**                               XXXXXXXXXXXXXXXXXXXXXX                                  **
******************************************************************************************/



static inline void cpu_lsra(CpuAdrMode adrmode)   
{   
	P= (P & 0xfe) | (A & 0x01);   
	A = A >>1;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   




static inline void cpu_rola(CpuAdrMode adrmode)   
{   
	int saveflags=(P & 0x01);   
	P= (P & 0xfe) | ((A >>7) & 0x01);   
	A = A << 1;   
	A |= saveflags;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   



static inline void cpu_rora(CpuAdrMode adrmode)   
{   
	int saveflags=(P & 0x01);   
	P= (P & 0xfe) | (A & 0x01);   
	A = A >>1;   
	if (saveflags) A |= 0x80;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   



static inline void cpu_rts(CpuAdrMode adrmode)   
{   
//      PC++;   
	// PC  = cpu_popstack();   
	// PC |= (cpu_popstack() << 8); 
	PC = cpu_pop16stack();  
	PC++;   
}   



/******************************************************************************************
**                                  XXXXXXXXXXXXXXXXX                                    **
******************************************************************************************/

static inline void cpu_bra(CpuAdrMode adrmode)   
{   
	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	PC += savepc;   
	cpu_clockticks++;   
}   

static inline void cpu_dea(CpuAdrMode adrmode)   
{   
	A--;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   

static inline void cpu_ina(CpuAdrMode adrmode)   
{   
	A++;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   
}   

static inline void cpu_phx(CpuAdrMode adrmode)   
{   
	cpu_pushstack(X);   
}   

static inline void cpu_plx(CpuAdrMode adrmode)   
{   
	X = cpu_popstack();   

	if (X) P &= 0xfd; else P |= 0x02;   
	if (X & 0x80) P |= 0x80; else P &= 0x7f;   
}   

static inline void cpu_phy(CpuAdrMode adrmode)   
{   
	cpu_pushstack(Y);   
}   

static inline void cpu_ply(CpuAdrMode adrmode)   
{   
	Y = cpu_popstack();   

	if (Y) P &= 0xfd; else P |= 0x02;   
	if (Y & 0x80) P |= 0x80; else P &= 0x7f;   
}   

static inline void cpu_stz(CpuAdrMode adrmode)   
{   
	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	cpu_putmemory(savepc,0);   
}   

static inline void cpu_tsb(CpuAdrMode adrmode)   
{   
	uint8_t temp;    

	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	temp  = cpu_getmemory(savepc);   
	temp |= A;   
	cpu_putmemory(savepc, temp);   

	if(cpu_getmemory(savepc))P &= 0xfd; else P |= 0x02;   
}   

static inline void cpu_trb(CpuAdrMode adrmode)   
{   
	uint8_t temp;       

	uint16_t savepc = /*opcodetable[opcode].*/adrmode();   
	temp  = cpu_getmemory(savepc);   
	temp &= (A ^ 0xFF);   
	cpu_putmemory(savepc, temp);   

	if(cpu_getmemory(savepc))P &= 0xfd; else P |= 0x02;   
}

//Zero page indexing instructions
static inline void cpu_tsbzp(CpuAdrMode x) {
	uint8_t temp;    

	uint16_t savepc = cpu_zp();  
	temp  = cpu_getzeropage(savepc);   
	temp |= A;   
	cpu_putzeropage(savepc, temp);   

	if(cpu_getzeropage(savepc))P &= 0xfd; else P |= 0x02; 

}
static inline void cpu_orazp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	A |= cpu_getzeropage(savepc);    

	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;  
}
static inline void cpu_aslzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   
	P= (P & 0xfe) | ((value >>7) & 0x01);   
	value = value << 1;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_trbzp(CpuAdrMode x) {
	uint8_t temp;       

	uint16_t savepc = cpu_zp();   
	temp  = cpu_getzeropage(savepc);   
	temp &= (A ^ 0xFF);   
	cpu_putzeropage(savepc, temp);   

	if(cpu_getzeropage(savepc))P &= 0xfd; else P |= 0x02;

}
static inline void cpu_orazpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	A |= cpu_getzeropage(savepc);    

	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   

}
static inline void cpu_aslzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   
	P= (P & 0xfe) | ((value >>7) & 0x01);   
	value = value << 1;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_bitzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   

/* non-destrucive logically And between value and the accumulator  
* and set zero flag */   
	if (value & A) P &= 0xfd; else P |= 0x02;   

/* set negative and overflow flags from value */   
	P = (P & 0x3f) | (value & 0xc0);   
}
static inline void cpu_andzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   

	A &= value;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 

}
static inline void cpu_rolzp(CpuAdrMode x) {
	int saveflags=(P & 0x01);   
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   
	P= (P & 0xfe) | ((value >>7) & 0x01);   
	value = value << 1;   
	value |= saveflags;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 

}
static inline void cpu_bitzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   

/* non-destrucive logically And between value and the accumulator  
* and set zero flag */   
	if (value & A) P &= 0xfd; else P |= 0x02;   

/* set negative and overflow flags from value */   
	P = (P & 0x3f) | (value & 0xc0);   
}
static inline void cpu_andzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   

	A &= value;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;   

}
static inline void cpu_rolzpx(CpuAdrMode x) {
	int saveflags=(P & 0x01);   
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   
	P= (P & 0xfe) | ((value >>7) & 0x01);   
	value = value << 1;   
	value |= saveflags;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_eorzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	A ^= cpu_getzeropage(savepc);   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_lsrzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   

/* set carry flag if shifting right causes a bit to be lost */   
	P= (P & 0xfe) | (value & 0x01);   

	value = value >>1;   
	cpu_putzeropage(savepc,value);   

/* set zero flag if value is zero */   
	if (value != 0) P &= 0xfd; else P |= 0x02;   

/* set negative flag if bit 8 set??? can this happen on an LSR? */   
	if ((value & 0x80) == 0x80)   
		P |= 0x80;   
	else   
		P &= 0x7f; 
}
static inline void cpu_eorzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	A ^= cpu_getzeropage(savepc);   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_lsrzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   

/* set carry flag if shifting right causes a bit to be lost */   
	P= (P & 0xfe) | (value & 0x01);   

	value = value >>1;   
	cpu_putzeropage(savepc,value);   

/* set zero flag if value is zero */   
	if (value != 0) P &= 0xfd; else P |= 0x02;   

/* set negative flag if bit 8 set??? can this happen on an LSR? */   
	if ((value & 0x80) == 0x80)   
		P |= 0x80;   
	else   
		P &= 0x7f; 
}
static inline void cpu_stzzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();    
	cpu_putzeropage(savepc,0); 
}
static inline void cpu_adczp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();    
	uint8_t value = cpu_getzeropage(savepc);   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + saveflags;   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_rorzp(CpuAdrMode x) {
	int saveflags=(P & 0x01);   
	uint16_t savepc = cpu_zp();    
	uint8_t value = cpu_getzeropage(savepc);   

	P= (P & 0xfe) | (value & 0x01);   
	value = value >>1;   
	if (saveflags) value |= 0x80;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_stzzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	cpu_putzeropage(savepc,0); 
}
static inline void cpu_adczpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + saveflags;   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_rorzpx(CpuAdrMode x) {
	int saveflags=(P & 0x01);   
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   

	P= (P & 0xfe) | (value & 0x01);   
	value = value >>1;   
	if (saveflags) value |= 0x80;   
	cpu_putzeropage(savepc,value);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_styzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();   
	cpu_putzeropage(savepc,Y);  
}
static inline void cpu_stazp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();   
	cpu_putzeropage(savepc,A);
}
static inline void cpu_stxzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();   
	cpu_putzeropage(savepc,X); 
}
static inline void cpu_styzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	cpu_putzeropage(savepc,Y);  
}
static inline void cpu_stazpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	cpu_putzeropage(savepc,A);
}
static inline void cpu_stxzpy(CpuAdrMode x) {
	uint16_t savepc = cpu_zpy();
	cpu_putzeropage(savepc,X); 
}
static inline void cpu_ldyzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	Y = cpu_getzeropage(savepc);   
	if (Y) P &= 0xfd; else P |= 0x02;   
	if (Y & 0x80) P |= 0x80; else P &= 0x7f;  
}
static inline void cpu_ldazp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	A = cpu_getzeropage(savepc);   
// set the zero flag   
	if (A) P &= 0xfd; else P |= 0x02;   
// set the negative flag   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_ldxzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	X = cpu_getzeropage(savepc);   
	if (X) P &= 0xfd; else P |= 0x02;   
	if (X & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_ldyzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	Y = cpu_getzeropage(savepc);   
	if (Y) P &= 0xfd; else P |= 0x02;   
	if (Y & 0x80) P |= 0x80; else P &= 0x7f;  
}
static inline void cpu_ldazpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	A = cpu_getzeropage(savepc);   
// set the zero flag   
	if (A) P &= 0xfd; else P |= 0x02;   
// set the negative flag   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_ldxzpy(CpuAdrMode x) {
	uint16_t savepc = cpu_zpy();
	X = cpu_getzeropage(savepc);   
	if (X) P &= 0xfd; else P |= 0x02;   
	if (X & 0x80) P |= 0x80; else P &= 0x7f; 
}
static inline void cpu_cpyzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   
	if (Y+0x100-value>0xff) P |= 0x01; else P &= 0xfe;   
	value=Y+0x100-value;   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_cmpzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   
	if (A+0x100-value>0xff) P |= 0x01; else P &= 0xfe;   
	value=A+0x100-value;   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_deczp(CpuAdrMode x) {
	uint8_t temp;    

	uint16_t savepc = cpu_zp();  
	temp = cpu_getzeropage(savepc);   
	temp--;   
	cpu_putzeropage(savepc, temp);   

	uint8_t value = cpu_getzeropage(savepc);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;  
}
static inline void cpu_cmpzpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc);   
	if (A+0x100-value>0xff) P |= 0x01; else P &= 0xfe;   
	value=A+0x100-value;   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_deczpx(CpuAdrMode x) {
	uint8_t temp;    

	uint16_t savepc = cpu_zpx();
	temp = cpu_getzeropage(savepc);   
	temp--;   
	cpu_putzeropage(savepc, temp);   

	uint8_t value = cpu_getzeropage(savepc);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;  
}
static inline void cpu_cpxzp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc);   
	if (X+0x100-value>0xff) P |= 0x01; else P &= 0xfe;   
	value=X+0x100-value;   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f; 

}
static inline void cpu_sbczp(CpuAdrMode x) {
	uint16_t savepc = cpu_zp();  
	uint8_t value = cpu_getzeropage(savepc) ^ 0xFF;   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + (saveflags << 4);   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		A -= 0x66;     
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_inczp(CpuAdrMode x) {
	uint8_t temp;     

	uint16_t savepc = cpu_zp();  
	temp = cpu_getzeropage(savepc);   
	temp++;   
	cpu_putzeropage(savepc,temp);   

	uint8_t value = cpu_getzeropage(savepc);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_sbczpx(CpuAdrMode x) {
	uint16_t savepc = cpu_zpx();
	uint8_t value = cpu_getzeropage(savepc) ^ 0xFF;   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + (saveflags << 4);   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		A -= 0x66;     
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;
}
static inline void cpu_inczpx(CpuAdrMode x) {
	uint8_t temp;     

	uint16_t savepc = cpu_zpx();
	temp = cpu_getzeropage(savepc);   
	temp++;   
	cpu_putzeropage(savepc,temp);   

	uint8_t value = cpu_getzeropage(savepc);   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}

static inline void cpu_oraindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();     
	A |= cpu_getmemory(savepc);    

	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}

static inline void cpu_andindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();    
	uint8_t value = cpu_getmemory(savepc);   

	A &= value;   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}

static inline void cpu_eorindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();    
	A ^= cpu_getmemory(savepc);   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}

static inline void cpu_adcindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();      
	uint8_t value = cpu_getmemory(savepc);   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + saveflags;   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}

static inline void cpu_staindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();     
	cpu_putmemory(savepc,A);   
}

static inline void cpu_ldaindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();    
	A = cpu_getmemory(savepc);   
// set the zero flag   
	if (A) P &= 0xfd; else P |= 0x02;   
// set the negative flag   
	if (A & 0x80) P |= 0x80; else P &= 0x7f; 
}

static inline void cpu_cmpindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();    
	uint8_t value = cpu_getmemory(savepc);   
	if (A+0x100-value>0xff) P |= 0x01; else P &= 0xfe;   
	value=A+0x100-value;   
	if (value) P &= 0xfd; else P |= 0x02;   
	if (value & 0x80) P |= 0x80; else P &= 0x7f;
}

static inline void cpu_sbcindzp(CpuAdrMode x) {
	uint16_t savepc = cpu_indzp();    
	uint8_t value = cpu_getmemory(savepc) ^ 0xFF;   

	int saveflags=(P & 0x01);   
	int sum= ((char) A) + ((char) value) + (saveflags << 4);   
	if ((sum>0x7f) || (sum<-0x80)) P |= 0x40; else P &= 0xbf;   
	sum= A + value + saveflags;   
	if (sum>0xff) P |= 0x01; else P &= 0xfe;   
	A=sum;   
	if (P & 0x08)   
	{   
		A -= 0x66;     
		P &= 0xfe;   
		if ((A & 0x0f)>0x09)   
			A += 0x06;   
		if ((A & 0xf0)>0x90)   
		{   
			A += 0x60;   
			P |= 0x01;   
		}   
	}   
	else   
	{   
		cpu_clockticks++;   
	}   
	if (A) P &= 0xfd; else P |= 0x02;   
	if (A & 0x80) P |= 0x80; else P &= 0x7f;
}
