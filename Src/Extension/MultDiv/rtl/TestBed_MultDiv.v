`timescale 1 ns/10 ps
`define	TestPort	30'h40      // r30
`define	BeginSymbol	32'h00000932
`define	EndSymbol	32'h00000D5D
`define	CheckNum	6'd13       // modify1 for number*2+1(CheckNum)

module	TestBed(
	clk,
	rst,
	addr,
	data,
	wen,
	error_num,
	duration,
	finish
);
	input			clk, rst;
	input	[29:0]	addr;
	input	[31:0]	data;
	input			wen;

	output	[7:0]	error_num;
	output	[15:0]	duration;
	output			finish;
	reg		[7:0]	error_num;
	reg		[15:0]	duration;
	reg				finish;
	
	reg		[31:0]	answer;

	reg		[1:0]	curstate;
	reg		[1:0]	nxtstate;
	reg		[5:0]	curaddr;
	reg		[5:0]	nxtaddr;
	reg		[7:0]	nxt_error_num;
	reg		[15:0]	nxtduration;
	
	reg				state,state_next;
		
	parameter	state_idle 	= 2'b00;
	parameter	state_check = 2'b01;
	parameter	state_report= 2'b10;	
		
	always@( posedge clk or negedge rst )						// State-DFF
	begin
		if( ~rst )
		begin
			curstate <= state_idle;
			curaddr  <= 0;
			duration <= 0;
			error_num <= 8'd255;
			
			state <= 0;
		end
		else
		begin
			curstate <= nxtstate;
			curaddr  <= nxtaddr;
			duration <= nxtduration;
			error_num <= nxt_error_num;
			
			state <= state_next;
		end
	end
			
	always@( curstate or curaddr or addr or data or wen or duration or error_num  )	// FSM for test
	begin
		finish = 1'b0;
		case( curstate )
		state_idle: 	begin
							nxtaddr = 0;
							nxtduration = 0;
							nxt_error_num = 0;
							nxtstate = state_check; 	
						end
		state_check:	begin
							nxtduration = duration + 1;
							nxtaddr = curaddr;						
							nxt_error_num = error_num;	
							if( addr==curaddr && wen )
							begin
								nxtaddr = curaddr + 1;
								if( data != answer )
									nxt_error_num = error_num + 8'd1;
							end
							nxtstate = curstate;
							if( curaddr==`CheckNum )	
								nxtstate = state_report;
						end
		state_report:	begin
							finish = 1'b1;
							nxtaddr = curaddr;
							nxtstate = curstate;		
							nxtduration = duration;
							nxt_error_num = error_num;	
						end				
		endcase	
	end

	always@( negedge clk )						
	begin
		if(curstate == state_report) begin
			$display("--------------------------- Simulation FINISH !!---------------------------");
			if (error_num) begin 
				$display("============================================================================");
				$display("\n (T_T) FAIL!! The simulation result is FAIL!!! there were %d errors at all.\n", error_num);
				$display("============================================================================");
			end
			 else begin 
				$display("============================================================================");
				$display("\n \\(^o^)/ CONGRATULATIONS!!  The simulation result is PASS!!!\n");
				$display("============================================================================");
			end
		end
	end
	
	always@(*)begin//sub-FSM (avoid the Dcache stall condition)
		case(state)
			1'b0:begin
				if(wen)
					state_next=1;
				else
					state_next=state;				
			end
			1'b1:begin
				if(!wen)
					state_next=0;
				else
					state_next=state;	
			end
		endcase
	end
	
	
	always@( curstate  or curaddr)	// ROM for correct result
	begin
		answer = 0;
		case( curaddr )
		// modify2 for 0-xxx, xxx-0, `EndSymbol
			 29'd0:		answer = 32'h0000_0002;  //     2
			 29'd1:		answer = 32'h0000_0006;  //     6
			 29'd2:		answer = 32'h0000_0018;  //    24 
			 29'd3:		answer = 32'h0000_0078;  //   120 
			 29'd4:		answer = 32'h0000_02D0;  //   720
			 29'd5:		answer = 32'h0000_13B0;  //  5040 
			 29'd6:		answer = 32'h0000_9D80;  // 40320
			 29'd7:		answer = 32'h0000_4EC0;  // 20160
			 29'd8: 	answer = 32'h0000_0000;  //     0
			 29'd9: 	answer = 32'h0000_1F80;  //  8064
			 29'd10:	answer = 32'h0000_0000;  //     0
			 29'd11:	answer = 32'h0000_0E51;  //  3665
			 29'd12:	answer = 32'h0000_0005;  //     5
		endcase			
	end

endmodule
