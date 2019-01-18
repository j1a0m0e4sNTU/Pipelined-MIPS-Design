// Top module of your design, you cannot modify this module!!
module CHIP (	clk,
				rst_n,
//----------for slow_memD------------
				mem_read_D,
				mem_write_D,
				mem_addr_D,
				mem_wdata_D,
				mem_rdata_D,
				mem_ready_D,
//----------for slow_memI------------
				mem_read_I,
				mem_write_I,
				mem_addr_I,
				mem_wdata_I,
				mem_rdata_I,
				mem_ready_I,
//----------for TestBed--------------				
				DCACHE_addr, 
				DCACHE_wdata,
				DCACHE_wen   
			);
input			clk, rst_n;
//--------------------------

output			mem_read_D;
output			mem_write_D;
output	[31:4]	mem_addr_D;
output	[127:0]	mem_wdata_D;
input	[127:0]	mem_rdata_D;
input			mem_ready_D;
//--------------------------
output			mem_read_I;
output			mem_write_I;
output	[31:4]	mem_addr_I;
output	[127:0]	mem_wdata_I;
input	[127:0]	mem_rdata_I;
input			mem_ready_I;
//----------for TestBed--------------
output	[29:0]	DCACHE_addr;
output	[31:0]	DCACHE_wdata;
output			DCACHE_wen;
//--------------------------

// wire declaration
wire        ICACHE_ren;
wire        ICACHE_wen;
wire [29:0] ICACHE_addr;
wire [31:0] ICACHE_wdata;
wire        ICACHE_stall;
wire [31:0] ICACHE_rdata;

wire        DCACHE_ren;
wire        DCACHE_wen;
wire [29:0] DCACHE_addr;
wire [31:0] DCACHE_wdata;
wire        DCACHE_stall;
wire [31:0] DCACHE_rdata;

//=========================================
	// Note that the overall design of your MIPS includes:
	// 1. pipelined MIPS processor
	// 2. data cache
	// 3. instruction cache


	MIPS_Pipeline i_MIPS(
		// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata
	);
	
	cache D_cache(
		clk,
		~rst_n,
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata,
		mem_read_D,
		mem_write_D,
		mem_addr_D,
		mem_rdata_D,
		mem_wdata_D,
		mem_ready_D
	);

	cache I_cache(
		clk,
		~rst_n,
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
		mem_read_I,
		mem_write_I,
		mem_addr_I,
		mem_rdata_I,
		mem_wdata_I,
		mem_ready_I
	);
endmodule



module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_wdata,
    proc_stall,
    proc_rdata,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input          proc_reset;
    input          proc_read, proc_write;
    input   [29:0] proc_addr;
    input   [31:0] proc_wdata;
    output   reg   proc_stall;
    output  [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output   reg   mem_read, mem_write;
    output  reg [27:0] mem_addr;
    output [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    //proc_tag = proc_addr[29:5],  proc_index = proc_addr[4:2], offset = proc_addr[1:0]
    //write_back = block_match[154] , valid= block_match[153], cache_tag = block_match[152:128],
    parameter IDLE          = 2'b00;
    parameter WRITE_BACK    = 2'b01;
    parameter READ_FROM_MEM = 2'b10;

    reg [1:0] state, state_next;
    reg [153:0] blocks[7:0];
    reg [153:0] blocks_next[7:0];
    wire [24:0] proc_tag;
    wire [2:0]  proc_index;
    wire [1:0]  offset;
    wire [153:0] block_match;
    wire valid,hit;
    wire [24:0] cache_tag;
    integer i;
    
//==== combinational circuit ==============================
    assign proc_tag = proc_addr[29:5];
    assign proc_index = proc_addr[4:2];
    assign offset = proc_addr[1:0];
    assign block_match = blocks[proc_index];
    assign proc_rdata = (offset == 2'b00)? block_match[31:0]:
                        (offset == 2'b01)? block_match[63:32]:
                        (offset == 2'b10)? block_match[95:64]: block_match[127:96];
    
    assign valid  = block_match[153];
    assign hit = (proc_tag == cache_tag) ? 1'b1: 1'b0;
    assign cache_tag = block_match[152:128];
    assign mem_wdata = block_match[127:0];

    always@(*)begin   // set memory-related signal
        case(state)
            IDLE:
            begin
                mem_read = 1'b0;
                mem_write= 1'b0;
                mem_addr = 28'b0;
            end
            WRITE_BACK:
            begin
                mem_read = 1'b0;
                mem_write= 1'b1;
                mem_addr = {cache_tag,proc_index};
            end
            READ_FROM_MEM:
            begin
                mem_read = 1'b1;
                mem_write= 1'b0;
                mem_addr = {proc_tag,proc_index};
            end
            default:
            begin
                mem_read = 1'b0;
                mem_write= 1'b0;
                mem_addr = 28'b0;
            end
        endcase
    end

    always@(*)begin
        for(i = 0;i<8; i=i+1)begin
            blocks_next[i] = blocks[i];
        end
        proc_stall = 1'b0;
        case(state)
            IDLE:
            begin
                if(proc_read && ~proc_write)begin               // read
                    if(hit) begin
                        proc_stall = 1'b0;
                        state_next = IDLE;
                    end else begin
                        proc_stall = 1'b1;
                        if(valid) state_next = READ_FROM_MEM;
                        else      state_next = WRITE_BACK;
                    end
                end else if(~proc_read && proc_write)begin      // write
                    if(hit)begin
                        proc_stall = 1'b0;
                        state_next = IDLE;
                        blocks_next[proc_index][153] = 1'b0;    // set valid to faulse
                        if     (offset == 2'b00)blocks_next[proc_index][31:0] = proc_wdata;
                        else if(offset == 2'b01)blocks_next[proc_index][63:32] = proc_wdata;
                        else if(offset == 2'b10)blocks_next[proc_index][95:64] = proc_wdata;
                        else                    blocks_next[proc_index][127:96] = proc_wdata;
                    end else begin
                        proc_stall = 1'b1;
                        if(valid) state_next = READ_FROM_MEM;
                        else      state_next = WRITE_BACK;
                    end
                end else state_next = IDLE;
            end
            WRITE_BACK:
            begin
                proc_stall = 1'b1;
                if(~mem_ready) state_next = WRITE_BACK;
                else           state_next = READ_FROM_MEM;
            end
            READ_FROM_MEM:
            begin
                proc_stall = 1'b1;
                if(~mem_ready) state_next = READ_FROM_MEM;
                else begin
                    blocks_next[proc_index] = {1'b1,proc_tag,mem_rdata};
                    state_next = IDLE;
                end
            end
            default:
            begin
                state_next = IDLE;
            end
        endcase
    end
    
//==== sequential circuit =================================
always@( posedge clk or posedge proc_reset ) begin
    if( proc_reset ) begin
        state <= 2'b00;
        for(i = 0;i<8;i=i+1)begin
            blocks[i] <= {1'b1,{153{1'b1}}};
        end 
    end
    else begin
        state <= state_next;
        for (i = 0; i<8; i=i+1)begin
            blocks[i] <= blocks_next[i];
        end
    end
end

endmodule

module MIPS_Pipeline(
	// control interface
		clk, 
		rst_n,
//----------I cache interface-------		
		ICACHE_ren,
		ICACHE_wen,
		ICACHE_addr,
		ICACHE_wdata,
		ICACHE_stall,
		ICACHE_rdata,
//----------D cache interface-------
		DCACHE_ren,
		DCACHE_wen,
		DCACHE_addr,
		DCACHE_wdata,
		DCACHE_stall,
		DCACHE_rdata
	);
	
	input clk, rst_n;
	output ICACHE_ren, ICACHE_wen;
	output [29:0] ICACHE_addr;
	output [31:0] ICACHE_wdata;	//*******************??????????????????
	input  ICACHE_stall;
	input  [31:0] ICACHE_rdata;
	
	output DCACHE_ren, DCACHE_wen;
	output [29:0] DCACHE_addr;
	output [31:0] DCACHE_wdata;
	input  DCACHE_stall;
	input  [31:0] DCACHE_rdata;
	
	
	
	
	//Overall
	//(1) control unit
	wire jump_out;			//id		1:jal or j															  0:else
	wire jr_pcaddr_out;		//id		1:jr or jalr, PC should jump to addr stored in register 0:else
	wire branch_out;			//id		1:branch taken 													  0:else
	wire flush_out; 		//id		1:flush, branch or any kind of  J							  0:else
	wire readhi_out;			//id		1:tell reg to read $HI											  0:else
	wire readlo_out;			//id		1:tell reg to read $LO											  0:else
	
	wire [1:0] aluop_out;			//ex     00:Itype 01:beq	10:Rtype	11:Jtype or	default
	wire alusrc_out;			//ex		1:alu take instruction as operand 							  0:alu take data from register as operand
	wire regwrite_ex_out;	//ex		1:regwrite instruction is in EX, tell forwarding unit   0:else
	wire stall_for_lw_out;	/*ex		1:lw is in EX, PC & IFID should stay, 
										  set all three id_ex_xx_w to 0,								  0:else */
	wire jal_addr_ex_out;
	wire jal_data_ex_out;
	wire multordiv_ex_out; //ex
	
	wire memread_out;		//mem		1:lw is in MEM, tell forwarding unit		 				  0:else
	wire memwrite_out;		//mem		1:write mem									 						  0:else
	wire regwrite_mem_out;	//mem		1:regwrite instruction is in MEM, tell forwarding unit  0:else
	wire jal_addr_mem_out;
	wire jal_data_mem_out;
	wire multordiv_mem_out;//mem
	
	wire multordiv_out;		//wb		1:tell reg to store data in HI and LO						  0:else
	wire memtoreg_out;		//wb		1:reg_write_data=mem												  0:reg_write_data=alu
	wire regwrite_wb_out;	//wb		1:regwrite instruction is in WB, tell forwarding unit   0:reg_not_write
	wire jal_addr_out;		//wb		1:jal, store in $31												  0: if jalr: regdst_out=1 due to Rtype else: regdst decides...
	wire jal_data_out;		//wb		1:jal or jalr, write the current PC into register		  0:else
	wire regdst_out;  		//wb		1:reg_write_addr=instruction[15:11] 						  0:reg_write_addr=instruction[20:16]
	
	//(2)PC
	wire [31:0] PC4;
	//(3)Register
	
	
	//(4)ALU
	wire stall_multordiv;
	wire [31:0] hi_data_alu;
	wire [31:0] lo_data_alu;
	
	//Forward C&D
	wire [2:0] ForwardC;
	wire [2:0] ForwardD;

	
	//IFID layer
	
	reg  [31:0] IFID_PC4_w;
	reg  [31:0] IFID_instruction_w;
	reg  [31:0] IFID_PC4_r;
	reg  [31:0] IFID_instruction_r;
	
	//IDEX layer
	wire [31:0] IDEX_PC4_w;
	wire [5:0]  IDEX_inst31_26_w;
	wire [31:0] IDEX_Rs_data_w;
	wire [31:0] IDEX_Rt_data_w;
	wire [15:0] IDEX_inst15_0_w;
	wire [4:0]  IDEX_Rs_addr_w;
	wire [4:0]  IDEX_Rt_addr_w;
	wire [4:0]  IDEX_Rd_addr_w;
	
	reg [31:0] IDEX_PC4_r;
	reg [5:0]  IDEX_inst31_26_r;
	reg [31:0] IDEX_Rs_data_r;
	reg [31:0] IDEX_Rt_data_r;
	reg [15:0] IDEX_inst15_0_r;
	reg [4:0]  IDEX_Rs_addr_r;
	reg [4:0]  IDEX_Rt_addr_r;
	reg [4:0]  IDEX_Rd_addr_r;
	
	wire [31:0] beq_addr;
	wire [31:0] jump_addr;
	wire [31:0] jr_addr;
	
		//DUMMY
		wire [5:0] IFID_inst_31_26;
		wire [31:0] IFID_Rs_data;
		wire [31:0] IFID_Rt_data;
		wire [15:0] IFID_inst_15_0;
		wire [4:0] IFID_Rs_addr;
		wire [4:0] IFID_Rt_addr;
		wire [4:0] IFID_Rd_addr;
		
	assign jr_addr = IFID_Rs_data;
	assign IDEX_PC4_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_PC4_r : IFID_PC4_r;
	assign IDEX_inst31_26_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_inst31_26_r : IFID_inst_31_26;
	assign IDEX_Rs_data_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_Rs_data_r : IFID_Rs_data;
	assign IDEX_Rt_data_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_Rt_data_r : IFID_Rt_data;
	assign IDEX_inst15_0_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_inst15_0_r : IFID_inst_15_0;
	assign IDEX_Rs_addr_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_Rs_addr_r : IFID_Rs_addr;
	assign IDEX_Rt_addr_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_Rt_addr_r : IFID_Rt_addr;
	assign IDEX_Rd_addr_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? IDEX_Rd_addr_r : IFID_Rd_addr;
	
	
	//EXMEM layer
	wire [31:0] EXMEM_PC4_w;
	wire [31:0] EXMEM_aluout_w;
	wire [31:0] EXMEM_Rt_data_w;
	wire [4:0] EXMEM_R_w;
	wire [31:0] hi_data_mem_w;
	wire [31:0] lo_data_mem_w;	
	
	reg [31:0] EXMEM_PC4_r;
	reg  [31:0] EXMEM_aluout_r;
	reg  [31:0] EXMEM_Rt_data_r;
	reg  [4:0] EXMEM_R_r;
	reg [31:0] hi_data_mem;
	reg [31:0] lo_data_mem;	
		//dummy
		wire [31:0] ALUout;
		wire [4:0] IDEX_R;
	
	assign EXMEM_PC4_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? EXMEM_PC4_r : IDEX_PC4_r;
	assign EXMEM_aluout_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? EXMEM_aluout_r : ALUout;
	assign EXMEM_Rt_data_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? EXMEM_Rt_data_r : IDEX_Rt_data_r;
	assign EXMEM_R_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? EXMEM_R_r : IDEX_R;
	assign hi_data_mem_w=(ICACHE_stall | DCACHE_stall | stall_multordiv) ? hi_data_mem : hi_data_alu;
	assign lo_data_mem_w=(ICACHE_stall | DCACHE_stall | stall_multordiv) ? lo_data_mem : lo_data_alu;
	//MEMWB layer
	wire [31:0] MEMWB_PC4_w;
	wire [31:0] MEMWB_memout_w;
	wire [31:0] MEMWB_aluout_w;
	wire [4:0]  MEMWB_R_w; 
	wire [31:0] hi_data_wb_w;
	wire [31:0] lo_data_wb_w;
	
	reg  [31:0] MEMWB_PC4_r;
	reg  [31:0] MEMWB_memout_r;
	reg  [31:0] MEMWB_aluout_r;
	reg  [4:0]  MEMWB_R_r;
	reg  [31:0] hi_data_wb;		//this is _r  
	reg  [31:0] lo_data_wb; 	//this is _r  
		//dummy
		wire [31:0] EXMEM_memout;
		assign EXMEM_memout=DCACHE_rdata;
	assign MEMWB_PC4_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? MEMWB_PC4_r : EXMEM_PC4_r;
	assign MEMWB_memout_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? MEMWB_memout_r : EXMEM_memout;
	assign MEMWB_aluout_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? MEMWB_aluout_r : EXMEM_aluout_r;
	assign MEMWB_R_w = (ICACHE_stall | DCACHE_stall | stall_multordiv) ? MEMWB_R_r : EXMEM_R_r;
	assign hi_data_wb_w= (ICACHE_stall | DCACHE_stall | stall_multordiv) ? hi_data_wb:hi_data_mem;
	assign lo_data_wb_w= (ICACHE_stall | DCACHE_stall | stall_multordiv) ? lo_data_wb:lo_data_mem;
	//MEMWB MUX
	wire MEMWB_memtoreg;
	wire [31:0] MEMWB_data;
	assign MEMWB_memtoreg=memtoreg_out;
	assign MEMWB_data = (MEMWB_memtoreg) ? MEMWB_memout_r : MEMWB_aluout_r;
		
	
	
	wire mem_stall;
	assign mem_stall = ICACHE_stall | DCACHE_stall | stall_multordiv;
	
	// cache
	assign ICACHE_ren=1'b1;
	assign ICACHE_wen=1'b0;
	assign ICACHE_wdata=0;
	
	assign DCACHE_ren=memread_out;
	assign DCACHE_wen=memwrite_out;
	assign DCACHE_addr=EXMEM_aluout_r[31:2];
	assign DCACHE_wdata=EXMEM_Rt_data_r;
	
	control control0(
		rst_n, 
		clk,
		IFID_instruction_r[31:26],
		IFID_instruction_r[25:21],
		IFID_instruction_r[20:16],
		IDEX_Rt_addr_r,
		Zero,
		mem_stall,
		IFID_instruction_r[5:0],
		jump_out,				
		jr_pcaddr_out,			
		branch_out,				
		flush_out,	
		readhi_out,			//id		1:tell reg to read $HI											  0:else
		readlo_out,			//id		1:tell reg to read $LO											  0:else
		aluop_out,				
		alusrc_out,						
		regwrite_ex_out,	
		stall_for_lw_out,
		jal_addr_ex_out,
		jal_data_ex_out,
		multordiv_ex_out, //ex
		memread_out,		
		memwrite_out,		
		regwrite_mem_out,
		jal_addr_mem_out,
		jal_data_mem_out,
		multordiv_mem_out,//mem
		multordiv_out,		//wb		1:tell reg to store data in HI and LO						  0:else
		memtoreg_out,		
		regwrite_wb_out,	
		jal_addr_out,		
		jal_data_out,		
		regdst_out 		
	);
	
	PC PC0(
		rst_n,
		clk,
		ICACHE_stall,
		DCACHE_stall,
		stall_multordiv,
		stall_for_lw_out,
		jump_out,
		jr_pcaddr_out,
		branch_out,
		jump_addr,
		jr_addr,
		beq_addr,
		PC4,
		ICACHE_addr
	);
	
	Register Register0(
		clk,
		rst_n,
		IFID_PC4_r,
		IFID_instruction_r,
		jal_data_ex_out,
		jal_data_mem_out,
		ALUout,
		IDEX_PC4_r,
		EXMEM_aluout_r,
		EXMEM_memout,
		EXMEM_PC4_r,
		MEMWB_data,
		ForwardC,
		ForwardD,
		regwrite_wb_out,
		jal_addr_out,
		jal_data_out,
		MEMWB_R_r,
		MEMWB_PC4_r,
		//mult div
		multordiv_out,
		hi_data_alu,
		lo_data_alu,
		hi_data_mem,
		lo_data_mem,
		hi_data_wb,
		lo_data_wb,
		readhi_out,
		readlo_out,
		
		Zero,
		IFID_inst_31_26,
		IFID_inst_15_0,
		IFID_Rs_data,
		IFID_Rt_data,
		IFID_Rs_addr,
		IFID_Rt_addr,
		IFID_Rd_addr,
		beq_addr,
		jump_addr
	);
	
	REG_forward REG_forward0(
		IFID_Rs_addr,
		IFID_Rt_addr,
		IDEX_R,
		EXMEM_R_r,
		MEMWB_R_r,
		regwrite_ex_out,
		regwrite_mem_out,
		regwrite_wb_out,
		memread_out,
		jal_addr_ex_out,
		jal_addr_mem_out,
		jal_addr_out,
		readhi_out,
		readlo_out,
		multordiv_ex_out,
		multordiv_mem_out,
		multordiv_out,
		ForwardC,
		ForwardD
	);
	
	ALU ALU0(
		rst_n,
		clk,
		IDEX_inst31_26_r,
		IDEX_Rs_data_r,
		IDEX_Rt_data_r,
		IDEX_inst15_0_r,
		regdst_out,
		aluop_out,
		IDEX_Rt_addr_r,
		IDEX_Rd_addr_r,
		jal_data_mem_out,
		jal_data_out,
		EXMEM_PC4_r,
		EXMEM_aluout_r,
		MEMWB_PC4_r,
		MEMWB_data,
		ALUout,
		IDEX_R,
		IFID_Rs_data,
		IFID_Rt_data,
		stall_multordiv,
		hi_data_alu,
		lo_data_alu
	);	
	
	always@(*) begin
		//IFID layer
		if(ICACHE_stall | DCACHE_stall | stall_for_lw_out | stall_multordiv) begin
			IFID_instruction_w = IFID_instruction_r;
			IFID_PC4_w = IFID_PC4_r;
		end
		else if(flush_out) begin
			IFID_instruction_w = 0;
			IFID_PC4_w = 0;
		end
		else begin
			IFID_instruction_w = ICACHE_rdata;
			IFID_PC4_w = PC4;
		end
	end
	
	
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			//IFID 
			IFID_PC4_r <= 0;
			IFID_instruction_r <= 0;
			
			//IDEX
			IDEX_PC4_r <= 0;
			IDEX_inst31_26_r <= 0;
			IDEX_Rs_data_r <= 0;
			IDEX_Rt_data_r <= 0;
			IDEX_inst15_0_r <= 0;
			IDEX_Rs_addr_r <= 0;
			IDEX_Rt_addr_r <= 0;
			IDEX_Rd_addr_r <= 0;
			
			//EXMEM
			EXMEM_PC4_r <= 0;
			EXMEM_aluout_r <= 0;
			EXMEM_Rt_data_r <= 0;
			EXMEM_R_r <= 0;
			hi_data_mem<=0;
			lo_data_mem<=0;
			
			//MEMWB
			MEMWB_PC4_r <= 0;
			MEMWB_memout_r <= 0;
			MEMWB_aluout_r <= 0;
			MEMWB_R_r <= 0;
			hi_data_wb<=0;
			lo_data_wb<=0;
		end
		else begin
			//IFID 
			IFID_PC4_r <= IFID_PC4_w;
			IFID_instruction_r <= IFID_instruction_w;
			
			//IDEX
			IDEX_PC4_r <= IDEX_PC4_w;
			IDEX_inst31_26_r <= IDEX_inst31_26_w;
			IDEX_Rs_data_r <= IDEX_Rs_data_w;
			IDEX_Rt_data_r <= IDEX_Rt_data_w;
			IDEX_inst15_0_r <= IDEX_inst15_0_w;
			IDEX_Rs_addr_r <= IDEX_Rs_addr_w;
			IDEX_Rt_addr_r <= IDEX_Rt_addr_w;
			IDEX_Rd_addr_r <= IDEX_Rd_addr_w;
			
			//EXMEM
			EXMEM_PC4_r <= EXMEM_PC4_w;
			EXMEM_aluout_r <= EXMEM_aluout_w;
			EXMEM_Rt_data_r <= EXMEM_Rt_data_w;
			EXMEM_R_r <= EXMEM_R_w;
			hi_data_mem<=hi_data_mem_w;
			lo_data_mem<=lo_data_mem_w;
			
			//MEMWB
			MEMWB_PC4_r <= MEMWB_PC4_w;
			MEMWB_memout_r <= MEMWB_memout_w;
			MEMWB_aluout_r <= MEMWB_aluout_w;
			MEMWB_R_r <= MEMWB_R_w;
			hi_data_wb<=hi_data_wb_w;
			lo_data_wb<=lo_data_wb_w;
		end
	end
endmodule
	

module control(
	//input
	rst_n,				//
	clk,					//
	instruction,		//op, instruction[31:26]
	if_id_rs_r,			//register read_addr1 instruction[25:21]	
	if_id_rt_r,			//register read_addr2 instruction[20:16]	
	id_ex_rt_r,			//alu operand 2, when lw is in EX, write_addr
	branch_trigger,	//the "==" on the output of register
	mem_stall,			//mem_stall	
	func,					//func, instruction[5:0]
	
	//output
	jump_out,			//id		1:jal or j															  0:else
	jr_pcaddr_out,		//id		1:jr or jalr, PC should jump to addr stored in register 0:else
	branch_out,			//id		1:branch taken 													  0:else
	flush_out,			//id		1:flush, branch or any kind of  J							  0:else
	readhi_out,			//id		1:tell reg to read $HI											  0:else
	readlo_out,			//id		1:tell reg to read $LO											  0:else
	
	aluop_out,			//ex     00:Itype 01:beq	10:Rtype	11:Jtype or	default
	alusrc_out,			//ex		1:alu take instruction as operand 							  0:alu take data from register as operand
	regwrite_ex_out,	//ex		1:regwrite instruction is in EX, tell forwarding unit   0:else
	stall_for_lw_out,	/*ex		1:lw is in EX, PC & IFID should stay, 
										  set all three id_ex_xx_w to 0,								  0:else */
	jal_addr_ex_out,  //ex		1:the data is tobe stored in $31								  0:else
	jal_data_ex_out,
	multordiv_ex_out, //ex
	
	memread_out,		//mem		1:lw is in MEM, tell forwarding unit		 				  0:else
	memwrite_out,		//mem		1:write mem									 						  0:else
	regwrite_mem_out,	//mem		1:regwrite instruction is in MEM, tell forwarding unit  0:else
	jal_addr_mem_out, //mem		1:the data is tobe stored in $31								  0:else
	jal_data_mem_out,	//mem
	multordiv_mem_out,//mem
	
	multordiv_out,		//wb		1:tell reg to store data in HI and LO						  0:else
	memtoreg_out,		//wb		1:reg_write_data=mem												  0:reg_write_data=alu
	regwrite_wb_out,	//wb		1:regwrite instruction is in WB, tell forwarding unit   0:reg_not_write
	jal_addr_out,		//wb		1:jal, store in $31												  0: if jalr: regdst_out=1 due to Rtype else: regdst decides...
	jal_data_out,		//wb		1:jal or jalr, write the current PC into register		  0:else
	regdst_out  		//wb		1:reg_write_addr=instruction[15:11] 						  0:reg_write_addr=instruction[20:16]
	
);
//input 
input	rst_n;
input	clk;
input	[5:0] instruction;
input [4:0] if_id_rs_r,if_id_rt_r,id_ex_rt_r;
input	branch_trigger;
input mem_stall;
input [5:0]func;


//output
output	jump_out;			//id		1:jal or j															  0:else
output	jr_pcaddr_out;		//id		1:jr or jalr, PC should jump to addr stored in register 0:else
output	branch_out;			//id		1:branch taken 													  0:else
output	flush_out;			//id		1:flush, branch or any kind of  J							  0:else
output	readhi_out;			//id		1:tell reg to read $HI											  0:else
output	readlo_out;			//id		1:tell reg to read $LO											  0:else

output[1:0]aluop_out;		//ex     00:Itype 01:beq	10:Rtype	11:Jtype or	default
output	alusrc_out;			//ex		1:alu take instruction as operand 							  0:alu take data from register as operand
output	regwrite_ex_out;	//ex		1:regwrite instruction is in EX, tell forwarding unit   0:else
output	stall_for_lw_out;	/*ex		1:lw is in EX, PC & IFID should stay, 
												  set all three id_ex_xx_w to 0,								  0:else */
output	jal_addr_ex_out;  //ex		1:the data is tobe stored in $31								  0:else
output	jal_data_ex_out;	//ex
output	multordiv_ex_out;
												  
output	memread_out;		//mem		1:lw is in MEM, tell forwarding unit		 				  0:else
output	memwrite_out;		//mem		1:write mem									 						  0:else
output	regwrite_mem_out;	//mem		1:regwrite instruction is in MEM, tell forwarding unit  0:else
output	jal_addr_mem_out; //mem		1:the data is tobe stored in $31								  0:else
output	jal_data_mem_out; //mem
output	multordiv_mem_out; //mem

output	multordiv_out;		//wb		1:tell reg to store data in HI and LO						  0:else
output	memtoreg_out;		//wb		1:reg_write_data=mem												  0:reg_write_data=alu
output	regwrite_wb_out;	//wb		1:regwrite instruction is in WB, tell forwarding unit   0:reg_not_write
output	jal_addr_out;		//wb		1:jal, store in $31												  0: if jalr: regdst_out=1 due to Rtype else: regdst decides...
output	jal_data_out;		//wb		1:jal or jalr, write the current PC into register		  0:else
output	regdst_out;  		//wb		1:reg_write_addr=instruction[15:11] 						  0:reg_write_addr=instruction[20:16]


//reg declare
reg [2:0] id_ex_ex_r,id_ex_ex_w;		//id_ex
reg [1:0]id_ex_m_r,id_ex_m_w;
reg [5:0]id_ex_wb_r,id_ex_wb_w;

reg [1:0]ex_mem_m_r,ex_mem_m_w;		//ex_mem
reg [5:0]ex_mem_wb_r,ex_mem_wb_w;

reg [5:0]mem_wb_wb_r,mem_wb_wb_w;	//mem_wb

//comb circuit
assign	jump_out=(instruction==6'b000010 || instruction==6'b000011);							//id
assign	jr_pcaddr_out=(instruction==6'b000000)&&(func==6'b001000 || func==6'b001001);		//id
assign	branch_out=(instruction==6'b000100)&&(branch_trigger);									//id
assign	flush_out=(instruction==6'b000010 || instruction==6'b000011)||
						 ((instruction==6'b000000)&&(func==6'b001000 || func==6'b001001))||
						 ((instruction==6'b000100)&&(branch_trigger));									//id
assign	readhi_out=((instruction==6'b000000)&&(func==6'b010000));								//id
assign	readlo_out=((instruction==6'b000000)&&(func==6'b010010));								//id

assign aluop_out=id_ex_ex_r[2:1];																								//ex 
assign alusrc_out=id_ex_ex_r[0];																									//ex
assign regwrite_ex_out=id_ex_wb_r[3];																							//ex
assign stall_for_lw_out=id_ex_m_r[1] &&(if_id_rs_r==id_ex_rt_r || if_id_rt_r==id_ex_rt_r)&& (id_ex_rt_r!=5'b00000);//ex 
assign jal_addr_ex_out=id_ex_wb_r[2];																							//ex
assign jal_data_ex_out=id_ex_wb_r[1];
assign multordiv_ex_out=id_ex_wb_r[5];
								
assign memread_out=ex_mem_m_r[1];			//mem
assign memwrite_out=	ex_mem_m_r[0];			//mem
assign regwrite_mem_out=ex_mem_wb_r[3]; 	//mem
assign jal_addr_mem_out=ex_mem_wb_r[2];	//mem
assign jal_data_mem_out=ex_mem_wb_r[1];	//mem
assign multordiv_mem_out=ex_mem_wb_r[5];  //mem

assign multordiv_out=mem_wb_wb_r[5];		//wb
assign memtoreg_out=mem_wb_wb_r[4];			//wb
assign regwrite_wb_out=mem_wb_wb_r[3];		//wb
assign jal_addr_out=mem_wb_wb_r[2];			//wb
assign jal_data_out=mem_wb_wb_r[1];			//wb
assign regdst_out=id_ex_wb_r[0];  			//wb




always@(*)begin
	if (mem_stall)begin																				//mem_stall
		id_ex_ex_w=id_ex_ex_r;
		id_ex_m_w=id_ex_m_r;
		id_ex_wb_w=id_ex_wb_r;
		ex_mem_m_w=ex_mem_m_r;
		ex_mem_wb_w=ex_mem_wb_r;
		mem_wb_wb_w=mem_wb_wb_r;

	end
	else if (id_ex_m_r[1] &&(if_id_rs_r==id_ex_rt_r || if_id_rt_r==id_ex_rt_r)&& (id_ex_rt_r!=5'b00000))begin 		//lw_stall
		id_ex_ex_w=3'b110;
		id_ex_m_w=2'b00;
		id_ex_wb_w=6'b000000;
		ex_mem_m_w=id_ex_m_r;
		ex_mem_wb_w=id_ex_wb_r;
		mem_wb_wb_w=ex_mem_wb_r;

	end
	else begin
		if(instruction==6'b000000)begin						//Rtype
			id_ex_ex_w=3'b100;
			id_ex_m_w=2'b00;
			id_ex_wb_w=(func==6'b001001)?6'b001011:						//when jalr, jal_data_out set to 1
						  (func==6'b011000||func==6'b011010)?6'b101001:	//when div or mult, set multordiv_out to 1
							6'b001001;		

		end
		else if(instruction==6'b001000 || 					//addi
				  instruction==6'b001100 || 					//andi
				  instruction==6'b001101 || 					//ori
				  instruction==6'b001110 ||					//xori
				  instruction==6'b001010)begin				//slti
			id_ex_ex_w=3'b001;
			id_ex_m_w=2'b00;
			id_ex_wb_w=6'b001000;

		end
		else if(instruction==6'b100011)begin				//LW
			id_ex_ex_w=3'b001;
			id_ex_m_w=2'b10;
			id_ex_wb_w=6'b011000;

		end
		else if(instruction==6'b101011)begin				//SW
			id_ex_ex_w=3'b001;
			id_ex_m_w=2'b01;
			id_ex_wb_w=6'b000000;

		end
		else if(instruction==6'b000100)begin				//beq
			id_ex_ex_w=3'b010;
			id_ex_m_w=2'b00;
			id_ex_wb_w=6'b000000;

		end
		else if(instruction==6'b000010)begin				//J
			id_ex_ex_w=3'b110;
			id_ex_m_w=2'b00;
			id_ex_wb_w=6'b000000;

		end
		else if(instruction==6'b000011)begin				//JAL
			id_ex_ex_w=3'b110;
			id_ex_m_w=2'b00;
			id_ex_wb_w=6'b001110;

		end
		else begin			//break down
			id_ex_ex_w=3'b110;
			id_ex_m_w=2'b00;
			id_ex_wb_w=6'b000000;

		end
		ex_mem_m_w=id_ex_m_r;
		ex_mem_wb_w=id_ex_wb_r;
		mem_wb_wb_w=ex_mem_wb_r;
	end
end

//sequential circuit
always@(posedge clk or negedge rst_n)begin
	if(!rst_n)begin
		
		id_ex_ex_r<=3'b110;
		id_ex_m_r<=2'b00;
		id_ex_wb_r<=6'b000000;
		ex_mem_m_r<=2'b00;
		ex_mem_wb_r<=6'b000000;
		mem_wb_wb_r<=6'b000000;
	end
	else begin
		id_ex_ex_r<=id_ex_ex_w;
		id_ex_m_r<=id_ex_m_w;
		id_ex_wb_r<=id_ex_wb_w;
		ex_mem_m_r<=ex_mem_m_w;
		ex_mem_wb_r<=ex_mem_wb_w;
		mem_wb_wb_r<=mem_wb_wb_w;
	end
end

endmodule

module REG_forward(
	//input
	if_id_rs_r,
	if_id_rt_r,
	wb_addr_alu,
	wb_addr_mem,
	wb_addr_wb,
	regwrite_id_ex_r,
	regwrite_ex_mem_r,
	regwrite_mem_wb_r,
	mem_read_ex_mem_r,
	jal_addr_ex_out,
	jal_addr_mem_out,
	jal_addr_out,
	//div mult
	readhi_out,
	readlo_out,
	multordiv_ex_out,
	multordiv_mem_out,
	multordiv_out,
	//output
	forward_c,
	forward_d
);
	//input
input[4:0]	if_id_rs_r;
input[4:0]	if_id_rt_r;
input[4:0]	wb_addr_alu;
input[4:0]	wb_addr_mem;
input[4:0]	wb_addr_wb;
input	regwrite_id_ex_r;
input	regwrite_ex_mem_r;
input	regwrite_mem_wb_r;
input mem_read_ex_mem_r;
input jal_addr_ex_out;
input jal_addr_mem_out;
input jal_addr_out;
input readhi_out;
input readlo_out;
input multordiv_ex_out;
input multordiv_mem_out;
input multordiv_out;
//output
output [2:0] forward_c,forward_d;

//reg
reg	[2:0] forward_c,forward_d;
wire [4:0] wb_addr_alu_true;
wire [4:0] wb_addr_mem_true;
wire [4:0] wb_addr_wb_true;

assign wb_addr_alu_true=jal_addr_ex_out?5'b11111:wb_addr_alu;
assign wb_addr_mem_true=jal_addr_mem_out?5'b11111:wb_addr_mem;
assign wb_addr_wb_true=jal_addr_out?5'b11111:wb_addr_wb;

always@(*)begin
	if ((readhi_out||readlo_out) && multordiv_ex_out)begin
		forward_c=3'b101;
	end
	else if ((readhi_out||readlo_out) && multordiv_mem_out)begin
		forward_c=3'b110;
	end
	else if ((readhi_out||readlo_out) && multordiv_out)begin
		forward_c=3'b111;
	end
	else if(wb_addr_alu_true==if_id_rs_r && regwrite_id_ex_r && wb_addr_alu_true!=5'b00000)begin
		forward_c=3'b001;
	end
	else if(wb_addr_mem_true==if_id_rs_r && regwrite_ex_mem_r && wb_addr_mem_true!=5'b00000)begin
		if(mem_read_ex_mem_r)begin
			forward_c=3'b011;
		end
		else begin
			forward_c=3'b010;
		end
	end
	else if(wb_addr_wb_true==if_id_rs_r && regwrite_mem_wb_r && wb_addr_wb_true!=5'b00000)begin
		forward_c=3'b100;
	end
	else begin
		forward_c=3'b000;
	end
end
always@(*)begin
	if(wb_addr_alu_true==if_id_rt_r && regwrite_id_ex_r && wb_addr_alu_true!=5'b00000)begin
		forward_d=3'b001;
	end
	else if(wb_addr_mem_true==if_id_rt_r && regwrite_ex_mem_r && wb_addr_mem_true!=5'b00000)begin
		if(mem_read_ex_mem_r)begin
			forward_d=3'b011;
		end
		else begin
			forward_d=3'b010;
		end
	end
	else if(wb_addr_wb_true==if_id_rt_r && regwrite_mem_wb_r && wb_addr_wb_true!=5'b00000)begin
		forward_d=3'b100;
	end
	else begin
		forward_d=3'b000;
	end
end

endmodule

module Register(
	clk,
	rst_n,
	IFID_PC4,
	IFID_instruction,
	//for forwarding
	jal_data_ex_out,
	jal_data_mem_out,
	ALUout,	//IDEX_ALUout	
	IDEX_PC4_r,
	EXMEM_aluout,
	EXMEM_memout,
	EXMEM_PC4_r,
	MEMWB_data,
	ForwardC,
	ForwardD,
	//for register write
	MEMWB_RegWrite,
	MEMWB_jal_addr,
	MEMWB_jal_data,
	MEMWB_R,
	MEMWB_PC4,
	//mult div
	multordiv_out,
	hi_data_alu,
	lo_data_alu,
	hi_data_mem,
	lo_data_mem,
	hi_data_wb,
	lo_data_wb,
	readhi_out,
	readlo_out,
	//for output
	Zero,
	inst31_26,
	inst15_0,
	IFID_Rs_data,		//equal to jr_addr
	IFID_Rt_data,
	IFID_Rs_addr,
	IFID_Rt_addr,
	IFID_Rd_addr,
	beq_addr,
	jump_addr
);
	input clk;
	input rst_n;
	input [31:0] IFID_PC4;
	input [31:0] IFID_instruction;
	input jal_data_ex_out, jal_data_mem_out;
	input [31:0] ALUout, EXMEM_aluout, EXMEM_memout, MEMWB_data;
	input [31:0] IDEX_PC4_r, EXMEM_PC4_r;
	input [2:0]  ForwardC, ForwardD;
	input 		 MEMWB_RegWrite;
	input        MEMWB_jal_addr, MEMWB_jal_data;
	input [4:0]  MEMWB_R;
	input [31:0] MEMWB_PC4;
	input multordiv_out;
	input [31:0]hi_data_alu;
	input [31:0]lo_data_alu;
	input [31:0]hi_data_mem;
	input [31:0]lo_data_mem;
	input [31:0]hi_data_wb;
	input [31:0]lo_data_wb;
	input readhi_out;
	input readlo_out;
	
	output Zero;
	output [5:0]  inst31_26;
	output [15:0] inst15_0;
	output [31:0] IFID_Rs_data, IFID_Rt_data;
	reg    [31:0] IFID_Rs_data, IFID_Rt_data;
	output [4:0]  IFID_Rs_addr, IFID_Rt_addr, IFID_Rd_addr;
	output [31:0] beq_addr;
	output [31:0] jump_addr;
	
	assign inst31_26 = IFID_instruction[31:26];
	assign inst15_0 = IFID_instruction[15:0];
	//wire reg declaration
	reg [31:0] register_w[31:0];
	reg [31:0] register_r[31:0];
	reg [31:0] hi_r,hi_w;
	reg [31:0] lo_r,lo_w;
	reg [31:0] ReadData_Rs, ReadData_Rt;
	integer i ;
	wire [4:0] Reg_Write_addr;
	wire [31:0] Reg_Write_data;
	wire [31:0] inst_sign_extend;
	
	assign Reg_Write_addr = (MEMWB_jal_addr) ? 5'd31 : MEMWB_R;
	assign Reg_Write_data = (MEMWB_jal_data) ? MEMWB_PC4 : MEMWB_data;
	
	assign Zero = (IFID_Rs_data==IFID_Rt_data) ? 1 : 0;
	assign IFID_Rs_addr = IFID_instruction[25:21];
	assign IFID_Rt_addr = IFID_instruction[20:16];
	assign IFID_Rd_addr = IFID_instruction[15:11];
	assign inst_sign_extend = {{16{IFID_instruction[15]}}, inst15_0};
	assign beq_addr = (inst_sign_extend << 2) + IFID_PC4;
	assign jump_addr = {IFID_PC4[31:28], IFID_instruction[25:0],2'b0 };
	
	
	
	always @(*) begin
		register_w[0]=0;
		hi_w=hi_r;
		lo_w=lo_r;
		for (i=1; i<32; i=i+1) begin
			register_w[i] = register_r[i];
		end
		
		
		if(multordiv_out)begin
			hi_w=hi_data_wb;
			lo_w=lo_data_wb;
		end
		else if(MEMWB_RegWrite && Reg_Write_addr!=5'b00000) begin
			register_w[Reg_Write_addr] = Reg_Write_data;
		end
		else ;
		ReadData_Rs =readhi_out?hi_r:
						 readlo_out?lo_r:
						 register_r[IFID_instruction[25:21]];
		ReadData_Rt = register_r[IFID_instruction[20:16]];
		
		//may need to modify
		if(ForwardC==3'b001) begin
			if(jal_data_ex_out==1)
				IFID_Rs_data = IDEX_PC4_r;
			else
				IFID_Rs_data = ALUout;
			end
		else if(ForwardC==3'b010) begin
			if(jal_data_mem_out==1)
				IFID_Rs_data = EXMEM_PC4_r;
			else
				IFID_Rs_data = EXMEM_aluout;
			end
		else if(ForwardC==3'b011) begin
			if(jal_data_mem_out==1)
				IFID_Rs_data = EXMEM_PC4_r;
			else
				IFID_Rs_data = EXMEM_memout;
			end
		else if(ForwardC==3'b100)
			IFID_Rs_data = Reg_Write_data;
		else if(ForwardC==3'b101)begin
			IFID_Rs_data = readhi_out?hi_data_alu:lo_data_alu;
		end
		else if(ForwardC==3'b110)begin
			IFID_Rs_data = readhi_out?hi_data_mem:lo_data_mem;
		end
		else if(ForwardC==3'b111)begin
			IFID_Rs_data = readhi_out?hi_data_wb:lo_data_wb;
		end
		else 
			IFID_Rs_data = ReadData_Rs;
			
		if(ForwardD==3'b001) begin
			if(jal_data_ex_out==1)
				IFID_Rt_data = IDEX_PC4_r;
			else
				IFID_Rt_data = ALUout;
			end
		else if(ForwardD==3'b010) begin
			if(jal_data_mem_out==1)
				IFID_Rt_data = EXMEM_PC4_r;
			else
				IFID_Rt_data = EXMEM_aluout;			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			end
		else if(ForwardD==3'b011) begin
			if(jal_data_mem_out==1)
				IFID_Rt_data = EXMEM_PC4_r;
			else
				IFID_Rt_data = EXMEM_memout;			//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
			end
		else if(ForwardD==3'b100)
			IFID_Rt_data = Reg_Write_data;
		else 
			IFID_Rt_data = ReadData_Rt;
	end
	
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			for (i=0; i<32; i=i+1) begin   
			register_r[i]<=0;
			end
			hi_r<=0;
			lo_r<=0;
		end
		else begin
			for (i=0; i<32; i=i+1) begin   
				register_r[i]<=register_w[i];
			end
			hi_r<=hi_w;
			lo_r<=lo_w;
		end
	end
	
endmodule
	
module PC(
	rst_n,
	clk,
	ICACHE_stall,
	DCACHE_stall,
	stall_multordiv,
	stall_for_lw,
	jump,
	jr_jalr,
	beq,
	jump_addr,
	jr_addr,
	beq_addr,
	PC4,
	ICACHE_proc_addr
);
	input rst_n, clk;
	input ICACHE_stall, DCACHE_stall, stall_multordiv;
	input stall_for_lw;  
	input jump, jr_jalr, beq;  //control unit input
	input [31:0] jump_addr, jr_addr, beq_addr;
	output [31:0] PC4;
	reg    [31:0] PC4;
	output [29:0] ICACHE_proc_addr;
	
	reg [31:0] PC_w;
	reg [31:0] PC_r;
	
	assign ICACHE_proc_addr = PC_r[31:2];
	
	always @(*) begin
		PC4 = PC_r+4;
		if(ICACHE_stall | DCACHE_stall | stall_for_lw | stall_multordiv) begin
			PC_w = PC_r;
		end
		else begin
			if(jump==1 && jr_jalr==0 && beq==0) 
				PC_w = jump_addr;
			else if(jump==0 && jr_jalr==1 && beq==0) 
				PC_w = jr_addr;
			else if(jump==0 && jr_jalr==0 && beq==1)
				PC_w = beq_addr;
			else 
				PC_w = PC4;
		end
	end
	always @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			PC_r <=0;
		end
		else begin
			PC_r <= PC_w;
		end
	end
endmodule
	
		
module ALU(
	rst_n,
	clk,
	operation,
	data_in1,
	data_in2,
	addr_in,
	RegDST,
	aluop,
	IDEX_Rt,
	IDEX_Rd,
	jal_data_mem_out,
	jal_data_out,
	EXMEM_PC4_r,
	EXMEM_aluout,
	MEMWB_PC4_r,
	MEMWB_data,
	ALUout,
	IDEX_R,
	//mult div
	first_num,
	second_num,
	stall_multordiv,
	hi_data_alu,
	lo_data_alu
);
	input rst_n;
	input clk;
	input [5:0] operation;
	input [31:0] data_in1, data_in2;
	input [15:0] addr_in;
	input		RegDST;
	input [1:0] aluop;
	input [4:0] IDEX_Rt, IDEX_Rd;
	input jal_data_mem_out, jal_data_out;
	input [31:0] EXMEM_PC4_r;
	input [31:0] EXMEM_aluout;
	input [31:0] MEMWB_PC4_r;
	input [31:0] MEMWB_data;
		
	output [31:0] ALUout;
	reg    [31:0] ALUout;
	output [4:0] IDEX_R;
		
	wire  [31:0] ALUin1;
	reg 	[31:0] ALUin2;
	wire  [31:0] ALUadd;
	wire  [31:0] ALUsub;
	wire	 [31:0] ALUand;
	wire  [31:0] ALUor;
	wire  [31:0] ALUxor;
	
	wire	 [31:0] data2_forward;
	
	wire [31:0] addr_in_zero;
	wire [31:0] addr_in_sign;
	assign addr_in_zero = {16'b0, addr_in};
	assign addr_in_sign = {{16{addr_in[15]}}, addr_in};
	
	wire [4:0] shamt;
	wire [5:0] func;
	assign shamt = addr_in[10:6];
	assign func = addr_in[5:0];
	
	
	assign IDEX_R = (RegDST) ?  IDEX_Rd:IDEX_Rt ;

	assign ALUin1=data_in1;
	assign data2_forward=data_in2;
	
	/*-----------------mult div---------------------*/
	input [31:0] first_num;
	input [31:0] second_num;
	output stall_multordiv;
	output [31:0] hi_data_alu;
	output [31:0] lo_data_alu;
	
	//------------------------------------------------
	reg [31:0]first_num_mult_w,first_num_mult_r;				//first_num
	reg [31:0]first_num_div_w,first_num_div_r;				//positive first_num
	reg [63:0]second_num_mult_w,second_num_mult_r;			//sign extension of second_num
	wire[63:0]second_num_mult_complement;						//2's complement of sign extension of second_num
	reg [63:0]second_num_div_w,second_num_div_r;				//positive second_num
	reg quotient_neg_w,quotient_neg_r;							//record qoutient's negativity
	reg remainder_neg_w,remainder_neg_r;						//record remainder's negativity
	wire[31:0]quotient_tonegative;								//negative quotient
	wire[31:0]remainder_tonegative;								//negative remainder
	//------------------------------------------------
	reg [4:0] countermult_w,countermult_r;
	reg [4:0] counterdiv_w,counterdiv_r;
	wire[63:0]temp2;
	wire[63:0]temp3;
	reg [63:0] result_w,result_r;
	
	assign stall_multordiv=(operation==6'b000000 && func==6'b011000 && countermult_r!=5'b11111)||	//mult
								  (operation==6'b000000 && func==6'b011010 && counterdiv_r!=5'b00000);//div
	assign quotient_tonegative=~result_w[31:0]+1;
	assign remainder_tonegative=~result_w[63:32]+1;
	assign hi_data_alu=(operation==6'b000000 && func==6'b011010 && remainder_neg_r)?remainder_tonegative:
							  result_w[63:32];
	assign lo_data_alu=(operation==6'b000000 && func==6'b011010 && quotient_neg_r)?quotient_tonegative:
							  result_w[31:0];
	assign temp2=second_num_mult_r<<countermult_r;
	assign temp3=second_num_div_r<<counterdiv_r;
	assign second_num_mult_complement=~second_num_mult_r+1;
	always@(*)begin
		if(operation==6'b000000 && func==6'b011000)begin //mult
			counterdiv_w=counterdiv_r;
			countermult_w=countermult_r+5'b00001;
			first_num_mult_w=first_num_mult_r;
			second_num_mult_w=second_num_mult_r;
			first_num_div_w=first_num_div_r;
			second_num_div_w=second_num_div_r;
			quotient_neg_w=quotient_neg_r;
			remainder_neg_w=remainder_neg_r;
			if(countermult_r==5'b00000)begin					//to reset result_r
				result_w=(first_num_mult_r[0])?second_num_mult_r:0;
			end
			else begin
				result_w=(first_num_mult_r[countermult_r])?(result_r+temp2):result_r;
				if(countermult_r==5'b11111)begin				//to read in correct two nums
					first_num_mult_w=first_num;
					second_num_mult_w={{32{second_num[31]}},second_num};
					first_num_div_w=first_num[31]?(~first_num+1):first_num;
					second_num_div_w=second_num[31]?({{32{1'b0}},{~second_num}}+1):{{32{1'b0}},second_num};
					quotient_neg_w=first_num[31]^second_num[31];
					remainder_neg_w=first_num[31];
					
					result_w=(first_num_mult_r[31])?(result_r+(second_num_mult_complement<<31)):result_r;
				end
			end
		end
		else if(operation==6'b000000 && func==6'b011010)begin//div
			countermult_w=countermult_r;
			counterdiv_w=counterdiv_r-5'b00001;
			if(counterdiv_r==5'b00000)begin					////to read in correct two nums
				result_w[31:1]=result_r[31:1];
				result_w[0]=(first_num_div_r>=second_num_div_r);
				result_w[63:32]=(first_num_div_r>=second_num_div_r)?first_num_div_r-second_num_div_r:first_num_div_r;
				first_num_mult_w=first_num;
				second_num_mult_w={{32{second_num[31]}},second_num};
				first_num_div_w=first_num[31]?(~first_num+1):first_num;
				second_num_div_w=second_num[31]?({{32{1'b0}},{~second_num}}+1):{{32{1'b0}},second_num};
				quotient_neg_w=first_num[31]^second_num[31];
				remainder_neg_w=first_num[31];
			end
			else begin
				first_num_mult_w=first_num_mult_r;
				second_num_mult_w=second_num_mult_r;
				first_num_div_w=(first_num_div_r>=temp3)?first_num_div_r-temp3:first_num_div_r;
				second_num_div_w=second_num_div_r;
				quotient_neg_w=quotient_neg_r;
				remainder_neg_w=remainder_neg_r;
				result_w=result_r;
				result_w[counterdiv_r]=(first_num_div_r>=temp3);
			end
		end
		else begin
			first_num_mult_w=first_num;
			second_num_mult_w={{32{second_num[31]}},second_num};
			first_num_div_w=first_num[31]?(~first_num+1):first_num;
			second_num_div_w=second_num[31]?({{32{1'b0}},{~second_num}}+1):{{32{1'b0}},second_num};
			quotient_neg_w=first_num[31]^second_num[31];
			remainder_neg_w=first_num[31];
			result_w=0;
			countermult_w=0;
			counterdiv_w=5'b11111;
		end
	end
	/*-----------------------------------------------*/
	
	always@(*)begin
		case(aluop)
		2'b00: begin	//I-type
			if((operation==6'b001100)||(operation==6'b001101)||(operation==6'b001110)) begin	//addi
				ALUin2 = addr_in_zero;
			end
			else begin	//slti
				ALUin2 = addr_in_sign;
			end
		end
		2'b10: begin	//R-type
			if((func==6'b100000)||(func==6'b100010)||(func==6'b100100)||(func==6'b100101)||(func==6'b100110)||(func==6'b101010))	begin	//add
				ALUin2 = data2_forward;
			end
			else begin	//sub
				ALUin2 = data2_forward;
			end
		end
		default : begin 
				ALUin2 = 0;	
		end
		endcase	
	end
	assign	ALUadd = ALUin1 + ALUin2;
	assign	ALUsub = ALUin1 - ALUin2;
	assign	ALUand = ALUin1 & ALUin2;
	assign	ALUor  = ALUin1 | ALUin2;
	assign	ALUxor = ALUin1 ^ ALUin2;
	always@(*) begin
		case(aluop)
		2'b00: begin	//I-type
			if(operation==6'b001000) begin	//addi
				ALUout = ALUadd;
				end
			else if(operation==6'b001100) begin	//andi
				ALUout = ALUand;
				end
			else if(operation==6'b001101) begin	//ori
				ALUout = ALUor;
				end
			else if(operation==6'b001110) begin	//xori
				ALUout = ALUxor;
				end
			else if(operation==6'b001010) begin	//slti
				ALUout = ALUsub[31] ? 1 : 0;
				end
			else if(operation==6'b100011) begin	//lw
				ALUout = ALUadd;
				end
			else if(operation==6'b101011) begin	//sw
				ALUout = ALUadd;
				end
			else begin
				ALUout = 0;	//do nothing
				end
			end
		2'b10: begin	//R-type
			if(func==6'b100000 || func==6'b010000 ||func==6'b010010)	begin	//add mfhi mfho
				ALUout = ALUadd;
				end
			else if(func==6'b100010) begin	//sub
				ALUout = ALUsub;
				end
			else if(func==6'b100100) begin	//and
				ALUout = ALUand;
				end
			else if(func==6'b100101) begin	//or
				ALUout = ALUor;
				end
			else if(func==6'b100110) begin	//xor
				ALUout = ALUxor;
				end
			else if(func==6'b100111) begin	//nor
				ALUout = ~ALUor;
				end
			else if(func==6'b000000) begin	//sll
				ALUout = ALUin1 << shamt;		//no influence to NOP
				end
			else if(func==6'b000011) begin	//sra
				ALUout = ALUin1 >>> shamt;
				end
			else if(func==6'b00010)	begin	//srl
				ALUout = ALUin1 >> shamt;
				end
			else if(func==6'b101010) begin	//slt
				ALUout = ALUsub[31] ? 1 : 0;
				end
			else if(func==6'b001000) begin	//jr
				ALUout = 0;
				end
			else if(func==6'b001001) begin	//jalr
				ALUout = 0;
				end
			else begin
				ALUout = 0;
				end
			end
		default : begin 
				ALUout = 0; 	
			end
		endcase	
	end
	always@(posedge clk or negedge rst_n)begin
		if(!rst_n)begin
			first_num_mult_r<=0;
			first_num_div_r<=0;
			second_num_mult_r<=0;
			second_num_div_r<=0;
			quotient_neg_r<=0;
			remainder_neg_r<=0;
			countermult_r<=0;
			counterdiv_r<=0;
			result_r<=0;
		end
		else begin
			first_num_mult_r<=first_num_mult_w;
			second_num_mult_r<=second_num_mult_w;
			first_num_div_r<=first_num_div_w;
			second_num_div_r<=second_num_div_w;
			quotient_neg_r<=quotient_neg_w;
			remainder_neg_r<=remainder_neg_w;
			countermult_r<=countermult_w;
			counterdiv_r<=counterdiv_w;
			result_r<=result_w;
		end
	end
endmodule
