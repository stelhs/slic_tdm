
/*
 * dtmf table for convert DTMF - ASCII
 */
char dtmf_codes[] = {"D1234567890*#ABC"};


/*
 * si3226x DTMF tone registers configure per digit
 */
struct si3226x_dtmf_digit slic_dtmf_table[] =
{
	
    {  // D
        .osc1amp = 0x10b0000,
        .osc2amp = 0xb2c000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x5e9c000,
    },
	
    {  // 1
        .osc1amp = 0xed2000,
        .osc2amp = 0x818000,
        .osc1freq = 0x4a82000,
        .osc2freq = 0x6d4c000,
    },
	
    {  // 2
        .osc1amp = 0x10b0000,
        .osc2amp = 0x818000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x6d4c000,
    },
	
    {  // 3
        .osc1amp = 0x12e4000,
        .osc2amp = 0x818000,
        .osc1freq = 0x331e000,
        .osc2freq = 0x6d4c000,
    },
	
    {  // 4
        .osc1amp = 0xed2000,
        .osc2amp = 0x900000,
        .osc1freq = 0x4a82000,
        .osc2freq = 0x694e000,
    },
	
    {  // 5
        .osc1amp = 0x10b0000,
        .osc2amp = 0x900000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x694e000,
    },
	
    {  // 6
        .osc1amp = 0x12e4000,
        .osc2amp = 0x900000,
        .osc1freq = 0x331e000,
        .osc2freq = 0x694e000,
    },
	
    {  // 7
        .osc1amp = 0xed2000,
        .osc2amp = 0xa06000,
        .osc1freq = 0x4a82000,
        .osc2freq = 0x6466000,
    },
	
    {  // 8
        .osc1amp = 0x10b0000,
        .osc2amp = 0xa06000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x6466000,
    },
	
    {  // 9
        .osc1amp = 0x12e4000,
        .osc2amp = 0xa06000,
        .osc1freq = 0x331e000,
        .osc2freq = 0x6466000,
    },
	
    {  // 0
        .osc1amp = 0x10b0000,
        .osc2amp = 0xb2c000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x5e9c000,
    },
	
    {  // *
        .osc1amp = 0xed2000,
        .osc2amp = 0xb2c000,
        .osc1freq = 0x4a82000,
        .osc2freq = 0x5e9c000,
    },
	
    {  // #
        .osc1amp = 0x10b0000,
        .osc2amp = 0xb2c000,
        .osc1freq = 0x3fc6000,
        .osc2freq = 0x5e9c000,
    },
	
    {  // A
        .osc1amp = 0x1586000,
        .osc2amp = 0x818000,
        .osc1freq = 0x2464000,
        .osc2freq = 0x6d4c000,
    },
	
    {  // B
        .osc1amp = 0x1586000,
        .osc2amp = 0x900000,
        .osc1freq = 0x2464000,
        .osc2freq = 0x694e000,
    },
	
    {  // C
        .osc1amp = 0x1586000,
        .osc2amp = 0xa06000,
        .osc1freq = 0x2464000,
        .osc2freq = 0x6466000,
    },
	
};

