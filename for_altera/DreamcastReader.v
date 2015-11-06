module DreamcastReader
(
	input controller_pin1,
	input controller_pin5,
	input reset,
	output reg clock,
	output reg [7:0] data
);
	reg [1:0] bit_counter1;
	reg [1:0] bit_counter5;
	
	reg [2:0] init_counter1;
	reg [2:0] init_counter5;

	reg data0;
	reg data1;
	reg data2;
	reg data3;
	reg data4;
	reg data5;
	reg data6;
	reg data7;

	always @ (negedge reset, negedge controller_pin1)
	begin
		if (!reset)
		begin
			bit_counter1 = 0;
			init_counter1 = 0;
		end else
		begin
			if (init_counter1 < 1)
			begin
				init_counter1 = 1;
			end else if (init_counter1 == 1 && init_counter5 == 5)
			begin
				case (bit_counter1)
					0: data0 = controller_pin5;
					1: data2 = controller_pin5;
					2: data4 = controller_pin5;
					3: data6 = controller_pin5;
				endcase
				bit_counter1 = bit_counter1+1;
			end
		end
	end	

	always @ (negedge reset, negedge controller_pin5)
	begin
		if (!reset)
		begin
			bit_counter5 = 0;
			init_counter5 = 0;
			clock = 1;
		end else
		begin
			if (init_counter5 < 5)
			begin
				init_counter5 = init_counter5 + 1;
			end else if (init_counter1 == 1 && init_counter5 == 5)
			begin
				case (bit_counter5)
					0: data1 = controller_pin1;
					1: data3 = controller_pin1;
					2: data5 = controller_pin1;
					3: data7 = controller_pin1;
				endcase
				bit_counter5 = bit_counter5+1;
				if (bit_counter1 == 0 && bit_counter5 == 0)
				begin
					data = {data0, data1, data2, data3, data4, data5, data6, data7};
					clock = 0;
				end 
				else if (bit_counter5 == 2)
					clock = 1;
			end
		end
	end	
endmodule
