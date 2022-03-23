module ic74ls139 (
	input e, 
	input a0, 
	input a1, 
	
	output o0, 
	output o1, 
	output o2, 
	output o3
);

	assign o0 = e | a0   |  a1;
	assign o1 = e | ~a0  |  a1;
	assign o2 = e | a0   | ~a1;
	assign o3 = e | ~a0  |  ~a1;

endmodule
