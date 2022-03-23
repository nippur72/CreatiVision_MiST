/***************************************************
  CreatiVision for the MiST FPGA
  https://github.com/nippur72/CreatiVision_MiST

  Written by Antonino Porcino, March 2022
  nino.porcino@gmail.com

  Uses third party modules that were available on the public domain
  see the copyright notices on the imported files
  
*/

/*

Some notes on the code: everything is placed on this file at the top level,
first are the MiST-related modules, and on the bottom the CreatiVision specific modules

Modules:

- creativision_mist: the top level MiST FPGA, with Audio, Video, SDRAM, clock, SPI interface, LED
- user_io: standard MiST module that interfaces with the MiST firmware
- mist_video: standard MiST module that provides everything about video and on-screen-menu (OSD)
- reset: provides the reset signal that goes in all the various chips
- pll: from the MiST 27MHz clock derives: sys_clock, vdp_clock and SDRAM_CLK
- downloader: safely injects files selected from the OSD into RAM
- audio: mixes the audio from cassette and SN76489 into final audio output stage
- ram: the CV bios rom and low RAM
- sdram: makes use of MiST 32MB SDRAM to populate the CreatiVision address space gaps
- cassette: CV cassette (not implemented yet)
- led: drives the debug led on the MiST front panel
- clock: derives the clken for cpu and vdp, also derives cpu_clock fake clock
- bus: the CreatiVision bus logic
- cpu: the 6502 CPU
- pia: the 6821 PIA chip
- vdp: the TMS998 video chip
- SN76489: the sound chip

*/


// TODO make roms loadable
// TODO use a CPU that allows illegal instructions
// TODO power on-off key ? init ram with values
// TODO ram powerup initial values
// TODO check diff with updated data_io.v and other modules
// TODO keyboard: check ps2 clock
// TODO display: check NTSC AD724 hsync problem (yellow menu doesn't work)
// TODO tms9918: fix video sync on composite and mist_video

module creativision_mist(
   input         CLOCK_27,
	
   // SPI interface to arm io controller 	
	input         SPI_SCK,
	output        SPI_DO,
	input         SPI_DI,
   input         SPI_SS2,
	input         SPI_SS3,
   input 		  SPI_SS4,
	input         CONF_DATA0,
	
	// SDRAM interface
	inout [15:0]  	SDRAM_DQ, 		// SDRAM Data bus 16 Bits
	output [12:0] 	SDRAM_A, 		// SDRAM Address bus 13 Bits
	output        	SDRAM_DQML, 	// SDRAM Low-byte Data Mask
	output        	SDRAM_DQMH, 	// SDRAM High-byte Data Mask
	output        	SDRAM_nWE, 		// SDRAM Write Enable
	output       	SDRAM_nCAS, 	// SDRAM Column Address Strobe
	output        	SDRAM_nRAS, 	// SDRAM Row Address Strobe
	output        	SDRAM_nCS, 		// SDRAM Chip Select
	output [1:0]  	SDRAM_BA, 		// SDRAM Bank Address
	output 			SDRAM_CLK, 		// SDRAM Clock
	output        	SDRAM_CKE, 		// SDRAM Clock Enable
	
	// VGA interface
	output  [5:0] VGA_R,
	output  [5:0] VGA_G,
	output  [5:0] VGA_B,
	output        VGA_HS,
	output        VGA_VS,

	// other
	output        LED,	
	input         UART_RX,
   output        AUDIO_L,
   output        AUDIO_R		
);

`include "rtl\build_id.v" 

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @user_io ***************************************/
/******************************************************************************************/
/******************************************************************************************/

localparam CONF_STR = {
	"CREATIVISION;;",        // 0 download index for "creativision.rom"
   "F,PRG,Load program;",   // 1 download index for ".prg" files
	"O3,Audio monitor,tape in,tape out;",
	"T6,Reset;",
	"V,",`BUILD_DATE
};

wire firmware_reset       = status[0];    // reset sent by the firmware
wire menu_reset           = status[6];    // reset from OSD menu
wire reset_switch         = buttons[1];   // reset switch on the front panel of the MiST

wire st_audio_mon_tape_in = ~status[3];

localparam conf_str_len = $size(CONF_STR)>>3;

wire [31:0] status;
wire  [1:0] buttons;
wire  [1:0] switches;

wire scandoubler_disable;
wire ypbpr;
wire no_csync;
wire ps2_kbd_clk;
wire ps2_kbd_data;

user_io
#(
	.STRLEN(conf_str_len)
	//.PS2DIV(14)              // ps2 clock divider: CLOCK / 14 must be approx = 15 Khz
)
user_io (
   .conf_str       (CONF_STR       ),

	.clk_sys        (sys_clock      ),

	.SPI_CLK        (SPI_SCK        ),
	.SPI_SS_IO      (CONF_DATA0     ),
	.SPI_MISO       (SPI_DO         ),
	.SPI_MOSI       (SPI_DI         ),

	.status         (status         ),
	.buttons        (buttons        ),
	.switches   	 (switches       ),

	.scandoubler_disable ( scandoubler_disable ),   // get this option from mist.ini
	.ypbpr               ( ypbpr               ),   // get this option from mist.ini
	.no_csync            ( no_csync            ),   // get this option from mist.ini

	.ps2_kbd_clk    (ps2_kbd_clk    ),              // ps2 keyboard from mist firmware
	.ps2_kbd_data   (ps2_kbd_data   )               // ps2 keyboard from mist firmware
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @mist_video ************************************/
/******************************************************************************************/
/******************************************************************************************/

// mix video
assign VGA_R   = tms_R;
assign VGA_G   = tms_G;
assign VGA_B   = tms_B;
assign VGA_HS  = tms_HS & tms_VS;
assign VGA_VS  = tms_VS;

wire  [5:0] tms_out_R;
wire  [5:0] tms_out_G;
wire  [5:0] tms_out_B;
wire        tms_out_HS;
wire        tms_out_VS;

mist_video 
#(
	.COLOR_DEPTH(6),    // 1 bit color depth
	.OSD_AUTO_CE(1),    // OSD autodetects clock enable
	.OSD_COLOR(3'b110), // yellow menu color
	.SYNC_AND(1),
	.SD_HCNT_WIDTH(11)	
)
tms_mist_video
(
	//.clk_sys(vdp_clock),    // OSD needs 2x the VDP clock for the scandoubler
	.clk_sys(osd_clock),    
	
	// OSD SPI interface
	.SPI_DI(SPI_DI),
	.SPI_SCK(SPI_SCK),
	.SPI_SS3(SPI_SS3),
		
	.scanlines(2'b00),                           // scanline emulation disabled for now
	.ce_divider(1),                              // non-scandoubled pixel clock divider 0 - clk_sys/4, 1 - clk_sys/2

	.scandoubler_disable(scandoubler_disable),   // disable scandoubler option from mist.ini	
	.no_csync(no_csync),                         // csync option from mist.ini
	.ypbpr(ypbpr),                               // YPbPr option from mist.ini

	.rotate(2'b00),                              // no ODS rotation
	.blend(0),                                   // composite-like blending
	
	// video input	signals to mist_video
	.R    (tms_R ),
	.G    (tms_G ),
	.B    (tms_B ),
	.HSync(tms_HS),
	.VSync(tms_vs),
	
	// video output signals that go into MiST hardware
	.VGA_R(tms_out_R),
	.VGA_G(tms_out_G),
	.VGA_B(tms_out_B),
	.VGA_VS(tms_out_VS),
	.VGA_HS(tms_out_HS)	
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @reset *****************************************/
/******************************************************************************************/
/******************************************************************************************/

wire reset = firmware_reset | menu_reset | reset_switch | reset_key_edge |!pll_locked;

// detects the rising edge of the keyboard reset key
// otherwise keyboard stays in reset mode

wire reset_key;

wire reset_key_edge = reset_key_old == 0 && reset_key == 1; 
reg reset_key_old = 0;
always @(posedge sys_clock) begin
	reset_key_old <= reset_key;
end

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @pll *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire pll_locked;         // indicates that pll is locked and working 

wire sys_clock;          // cpu x 8 system clock (sdram.v)
wire vdp_clock;          // tms9918 x 2 for osd menu

pll pll 
(
	.inclk0(CLOCK_27),      // MiST 27 MHz onboard clock
	.locked(pll_locked),    // indicates that pll is locked and working
	
	.c0( vdp_clock      ),  // tms9918 x 2 for osd menu (10.738635 x 2 = 21.47727 MHz)
   .c1( sys_clock      ),  // cpu x 8 system clock     ( 2.000000 x 2 = 16.00000 MHz)
	.c2( SDRAM_CLK      ),  // cpu x 8 phase shifted -2.5 ns
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @downloader ************************************/
/******************************************************************************************/
/******************************************************************************************/

wire        is_downloading;      // indicates that downloader is working
wire [24:0] download_addr;       // download address
wire [7:0]  download_data;       // download data
wire        download_wr;         // download write enable
wire        ROM_loaded;          // 1 when boot rom has been downloaded

// ROM download helper
downloader 
#(
   .BOOT_INDEX (0),    // menu index 0 is for automatic download of "creativision.rom" at FPGA boot
	.PRG_INDEX  (1),    // menu index for load .prg
	.ROM_INDEX  (2),    // menu index for load .prg	
	.ROM_START_ADDR(0)  // start of ROM (bank 0 of SDRAM)
)
downloader 
(	
	// new SPI interface
   .SPI_DO ( SPI_DO  ),
	.SPI_DI ( SPI_DI  ),
   .SPI_SCK( SPI_SCK ),
   .SPI_SS2( SPI_SS2 ),
   .SPI_SS3( SPI_SS3 ),
   .SPI_SS4( SPI_SS4 ),
	
	// signal indicating an active rom download
	.downloading ( is_downloading  ),
   .ROM_done    ( ROM_loaded      ),	
	         
   // external ram interface
   .clk     ( cpu_clock      ), // does not work with sys_clock+cpu_clken_noRF and SDRAM
	.clk_ena ( 1              ), // most likely because ioctl_wr isn't 1 for all the 8 sdram cycles
   .wr      ( download_wr    ),
   .addr    ( download_addr  ),
   .data    ( download_data  )	
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @audio *****************************************/
/******************************************************************************************/
/******************************************************************************************/

wire audio_out_1bit;
wire signed [18:0] sid_audio_out_combined = sid_dout;
wire signed [15:0] sid_audio_out_combined1 = sid_audio_out_combined[18:3];
wire        [15:0] sid_audio_out_combined2 = sid_audio_out_combined1 + 32767;

dac #(.C_bits(16)) dac_SID
(
	.clk_i(sys_clock),
   .res_n_i(pll_locked),	
	.dac_i(sid_audio_out_combined2),
	.dac_o(audio_out_1bit)
);

always @(posedge sys_clock) begin
	AUDIO_L <= audio_out_1bit; //audio_tape_monitor_1bit;
	AUDIO_R <= audio_out_1bit;
end

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @ram *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire [7:0] ram_dout;

// low system RAM
ram #(.SIZE(16384)) ram(
  .clk    (sys_clock ),
  .address(bus_addr[15:0]),
  .w_en   (bus_wr & ram_cs),
  .din    (bus_din ),
  .dout   (ram_dout  )  
);

// WozMon ROM
wire [7:0] rom_dout;
rom_bios rom_bios(
  .clk(sys_clock),
  .address(cpu_addr[7:0]),
  .dout(rom_dout)
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @sdram *****************************************/
/******************************************************************************************/
/******************************************************************************************/

// SDRAM_CLK is connected directly to PLL
			
assign SDRAM_CKE = 1'b1;

wire [7:0]  sdram_dout;

sdram sdram (
	// interface to the MT48LC16M16 chip
   .sd_data        ( SDRAM_DQ                  ),
   .sd_addr        ( SDRAM_A                   ),
   .sd_dqm         ( {SDRAM_DQMH, SDRAM_DQML}  ),
   .sd_cs          ( SDRAM_nCS                 ),
   .sd_ba          ( SDRAM_BA                  ),
   .sd_we          ( SDRAM_nWE                 ),
   .sd_ras         ( SDRAM_nRAS                ),
   .sd_cas         ( SDRAM_nCAS                ),

   // system interface
   .clk            ( sys_clock                 ),
   .clkref         ( cpu_clock                 ),
   .init           ( !pll_locked               ),	
	
   // cpu interface
   .din            ( bus_din                 ),
   .addr           ( bus_addr                ),
   .we             ( bus_wr                  ),
   .oe         	 ( 1                       ),
   .dout           ( sdram_dout              )
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @CASSETTE **************************************/
/******************************************************************************************/
/******************************************************************************************/

/*
wire [7:0] aci_dout;
wire CASOUT;
ACI ACI(
  .clk(sys_clock),
  .cpu_clken(cpu_clken),
  .addr(bus_addr[15:0]),
  .dout(aci_dout),
  .tape_in(CASIN),
  .tape_out(CASOUT),
);

// latches cassette audio input
reg CASIN;
always @(posedge sys_clock) begin
	CASIN <= ~UART_RX;    // on the Mistica UART_RX is the audio input
end

wire audio_tape_monitor = st_audio_mon_tape_in ? CASIN : CASOUT ;

wire audio_tape_monitor_1bit;
dac #(.C_bits(16)) dac_tape_monitor
(
	.clk_i(sys_clock),
   .res_n_i(pll_locked),	
	.dac_i({ audio_tape_monitor, 15'b0000000 }),
	.dac_o(audio_tape_monitor_1bit)
);
*/


/******************************************************************************************/
/******************************************************************************************/
/***************************************** @led *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire dummy = is_downloading && download_wr;
assign LED = ~dummy;

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @clock *****************************************/
/******************************************************************************************/
/******************************************************************************************/

// VDP clock enable is vdp_clock divided by two, use a simple bit flip
/*
reg vdp_div;

always @(posedge vdp_clock or posedge reset) begin
	if(reset) vdp_div <= 0;
	else vdp_div <= ~vdp_div;
end

wire vpd_clken = vdp_div;  
*/

// CPU clock enable is sys_clock divided by 8

reg [2:0] cpu_div;

always @(posedge sys_clock or posedge reset) begin
	if(reset) cpu_div <= 0;
	else cpu_div <= cpu_div + 1;		
end

wire cpu_clken = cpu_div == 0;   // clock enable for the 6502 CPU 
wire cpu_clock = cpu_div  < 4;   // cpu clock for sdram and downloader

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @bus *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire [24:0] bus_addr;
wire  [7:0] bus_din;
wire        bus_wr;

always @(posedge sys_clock) begin
	if(is_downloading && download_wr) begin
		bus_addr   <= download_addr;
		bus_din    <= download_data;
		bus_wr     <= download_wr;
	end
	else begin
		bus_addr   <= { 9'b0, cpu_addr[15:0] };
		bus_din    <= cpu_dout;		
		bus_wr     <= cpu_wr;
	end	
end

wire ram_cs   = bus_addr <  'h4000;                         // 0x0000 -> 0x3FFF
wire sdram_cs = bus_addr >= 'h4000 && bus_addr <= 'hBFFF;   // 0x4000 -> 0xBFFF
//wire aci_cs   = bus_addr >= 'hC000 && bus_addr <= 'hC1FF; // 0xC000 -> 0xC1FF
wire rom_cs   = bus_addr >= 'hFF00;                         // 0xFF00 -> 0xFFFF
wire sid_cs   = bus_addr >= 'hC800 && bus_addr <= 'hC8FF;   // 0xC800 -> 0xC8FF 
wire tms_sel  = ~VDPR | ~VDPW;  

wire [7:0] cpu_din =  rom_cs   ? rom_dout   :                      
						    tms_sel  ? vdp_dout   :
 						  //sid_cs   ? sid_dout   :
						  //aci_cs   ? aci_dout   :
                      sdram_cs ? sdram_dout :
					       ram_cs   ? ram_dout   :
					       8'b0;							 

wire ROM0_n;
wire ROM1_n;
wire ROM2_n;
wire e_u15b;
							 
ic74ls139 u15a
(
	.e(0),
	.a1(cpu_addr[15]),
	.a0(cpu_addr[14]),
	
	.o3(ROM0_n),
	.o2(ROM1_n),
	.o1(ROM2_n),
	.o0(e_u15b)
);							 

wire VDPW;
wire VDPR;
wire PIA;
wire RAM_n;

ic74ls139 u15b
(
	.e(e_u15b),
	.a1(cpu_addr[13]),
	.a0(cpu_addr[12]),
		
	.o3(VDPW),
	.o2(VDPR),
	.o1(PIA),
	.o0(RAM_n)
);							 
							 
/******************************************************************************************/
/******************************************************************************************/
/***************************************** @cpu *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire [15:0] cpu_addr;
wire [7:0]  cpu_dout;
wire        cpu_wr;

arlet_6502 arlet_6502(
	.clk    (sys_clock),
	.enable (cpu_clken & ~is_downloading),
	.rst    (reset),
	.ab     (cpu_addr),
	.dbi    (cpu_din),
	.dbo    (cpu_dout),
	.we     (cpu_wr),
	.irq_n  (VDP_INT_n),
	.nmi_n  (1'b1),
	.ready  (cpu_clken)
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @pia *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire [7:0] pia_dout;

wire [7:0] PA_out;
wire [7:0] PA_in;
wire       PA_oe;

wire [7:0] PB_out;
wire [7:0] PB_in;
wire       PB_oe;

wire pia_cs = PIA | ~cpu_clken;
wire pia_cb2;

pia6821 pia6821 (	
	.clk      (sys_clock),
	.rst      (reset),
	.cs       (pia_cs),         // 1 = active 
	.rw       (cpu_wr),         // R/W
	.addr     (cpu_addr[1:0]),  // RS0, RS1
	.data_in  (cpu_dout),
	.data_out (pia_dout),
	.irqa     (1),
	.irqb     (1),
	.pa_i     (PA_in),
	.pa_o     (PA_out),
	.pa_oe    (PA_oe),
	.ca1      (1),
	.ca2_i    (1),
	.ca2_o    (1),
	.ca2_oe   (1),
	.pb_i      (PB_in),
	.pb_o      (PB_out),
	.pb_oe     (PB_oe),
	.cb1       (sn76489_ready),	
	.cb2_i     (1),             // CB2 feeds SN76489 CE       
	.cb2_o     (pia_cb2),       //
	.cb2_oe    (1),             //
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @vdp *******************************************/
/******************************************************************************************/
/******************************************************************************************/

wire        vram_we;
wire [0:13] vram_a;        
wire [0:7]  vram_din;      
wire [0:7]  vram_dout;

vram vram
(
  .clock  ( vdp_clock  ),  
  .address( vram_a     ),  
  .data   ( vram_din   ),                       
  .wren   ( vram_we    ),                       
  .q      ( vram_dout  )
);

wire [7:0] vdp_dout;
wire VDP_INT_n;         

// divide by two the vdp_clock (which is doubled for the scandoubler)
reg vdp_ena;
always @(posedge vdp_clock) begin
	vdp_ena <= ~vdp_ena;
end

wire csr   = tms_sel;
wire csw   = tms_sel & bus_wr;
wire mode  = bus_addr[0];

wire         tms_HS;
wire         tms_VS;
wire   [5:0] tms_R;
wire   [5:0] tms_G;
wire   [5:0] tms_B;

tms9918_async 
#(
	.HORIZONTAL_SHIFT(-36)    // -36 good empiric value to center the image on the screen
) 
tms9918
(
	// clock
	.RESET(reset),
	
	.clk(vdp_clock),
	.ena(vdp_ena),
		
	// control signals
   .csr_n  ( VDPR          ),
   .csw_n  ( VDPW          ),
	.mode   ( mode          ),	    
   .int_n  ( VDP_INT_n     ),

	// cpu I/O 	
   .cd_i          ( bus_din     ),    // TODO should this be cpu_dout ???
   .cd_o          ( vdp_dout    ),
		
	//	vram	
   .vram_we       ( vram_we     ),
   .vram_a        ( vram_a      ),
   .vram_d_o      ( vram_din    ),
   .vram_d_i      ( vram_dout   ),		
		
	// video 
	.HS(tms_HS),
	.VS(tms_VS),
	.R (tms_R),
	.G (tms_G),
	.B (tms_B)
);

/******************************************************************************************/
/******************************************************************************************/
/***************************************** @sn76489 ***************************************/
/******************************************************************************************/
/******************************************************************************************/

wire sn76489_ready;

sn76489_top sn76489_top(
    .clock_i    (sys_clock),
    .clock_en_i (cpu_clken),
    .res_n_i    (~reset),
    .ce_n_i     (pia_cb2),
    .we_n_i     (sn76489_ready),    // WE directly from READY
    .ready_o    (sn76489_ready),    // READY goees into WE
    .d_i        (PB_dout),
    .aout_o     (sid_dout),         // 8 bit audio
);

// TODO check generic g_filter_div
wire [7:0] sid_dout;

endmodule 


