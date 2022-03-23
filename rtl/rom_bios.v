
module rom_bios (
    input clk,              // clock signal
    input [10:0] address,   // address bus
    output reg [7:0] dout   // 8-bit data bus (output)
);

    reg [7:0] rom_data[0:2047];

    initial
        $readmemh("roms/bioscv.hex", rom_data, 0, 2047);

    always @(posedge clk)
        dout <= rom_data[address];

endmodule
    
