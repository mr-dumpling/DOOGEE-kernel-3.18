/*
 * No Warranty
 * parthibx24 <e.inxpired@gmail.com>
 */

#include "lcm_drv.h"

/* Local Constants */
#define LCM_NAME "nt35521_boe_hd720_5p0_xingliangda_t591_dg"
#define LCM_ID (0x5521)
#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY (0xFE)
#define REGFLAG_END_OF_TABLE (0xFF) /* END OF REGISTERS MARKER */

/* Local Variables */
#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_DBG_TAG "[LCM][T591]"
#ifndef BUILD_LK
#define LCM_LOGD(str, args...) pr_info(LCM_DBG_TAG "[%s][%s] " str, LCM_NAME, __func__, ##args)
#endif

/* Local Functions */
#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define read_reg_v2(cmd, buffer, buffer_size)	            lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define write_regs(addr, pdata, byte_nums)	                lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)   lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define wrtie_cmd(cmd)	lcm_util.dsi_write_cmd(cmd)

/* LCM Driver Implementations */

static LCM_UTIL_FUNCS lcm_util = { 0 };
int ata_checking = 0;

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

    {0xff, 4, {0x55,0x25,0x01,0x00}},
    {0xfc, 1, {0x08}},
    {0xfc, 1, {0x00}},
    {0x6f, 1, {0x21}},
    {0xf7, 1, {0x01}},
    {0x6f, 1, {0x21}},
    {0xf7, 1, {0x00}},
    {0x6f, 1, {0x1a}},
    {0xf7, 1, {0x05}},
    {0x6f, 1, {0x16}},
    {0xf7, 1, {0x10}},
    // 
    {0xff, 4, {0xaa,0x55,0x25,0x00}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x00}},
    {0xb1, 2, {0x68,0x27}},
    {0xb8, 4, {0x01,0x02,0x0c,0x02}},
    {0xbb, 2, {0x11,0x11}},
    {0xbc, 2, {0x00,0x00}},
    {0xb6, 1, {0x04}},
    {0xc8, 1, {0x80}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x01}},
    {0xb0, 2, {0x09,0x09}},
    {0xb1, 2, {0x09,0x09}},
    {0xbc, 2, {0x7b,0x00}},
    {0xbd, 2, {0x7b,0x00}},
    {0xca, 1, {0x00}},
    {0xc0, 1, {0x0c}},
    {0xb5, 2, {0x03,0x03}},
    {0xbe, 1, {0x38}},
    {0xb3, 2, {0x19,0x19}},
    {0xb4, 2, {0x19,0x19}},
    {0xb9, 2, {0x26,0x26}},
    {0xba, 2, {0x36,0x36}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x02}},
    {0xee, 1, {0x01}},
    {0xb0, 16, {0x00,0x00,0x00,0x0a,0x00,0x1d,0x00,0x2f,0x00,0x3e,0x00,0x5c,0x00,0x76,0x00,0xa2}},
    {0xb1, 16, {0x00,0xc6,0x01,0x04,0x01,0x36,0x01,0x85,0x01,0xc7,0x01,0xc8,0x02,0x03,0x02,0x46}},
    {0xb2, 16, {0x02,0x71,0x02,0xae,0x02,0xd8,0x03,0x11,0x03,0x37,0x03,0x68,0x03,0x89,0x03,0xb0}},
    {0xb3, 4, {0x03,0xdd,0x03,0xff}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x06}},
    {0xb0, 2, {0x10,0x12}},
    {0xb1, 2, {0x14,0x16}},
    {0xb2, 2, {0x00,0x02}},
    {0xb3, 2, {0x31,0x31}},
    {0xb4, 2, {0x31,0x34}},
    {0xb5, 2, {0x34,0x34}},
    {0xb6, 2, {0x34,0x31}},
    {0xb7, 2, {0x31,0x31}},
    {0xb8, 2, {0x31,0x31}},
    {0xb9, 2, {0x2d,0x2e}},
    {0xba, 2, {0x2e,0x2d}},
    {0xbb, 2, {0x31,0x31}},
    {0xbc, 2, {0x31,0x31}},
    {0xbd, 2, {0x31,0x34}},
    {0xbe, 2, {0x34,0x34}},
    {0xbf, 2, {0x34,0x31}},
    {0xc0, 2, {0x31,0x31}},
    {0xc1, 2, {0x03,0x01}},
    {0xc2, 2, {0x17,0x15}},
    {0xc3, 2, {0x13,0x11}},
    {0xe5, 2, {0x31,0x31}},
    {0xc4, 2, {0x17,0x15}},
    {0xc5, 2, {0x13,0x11}},
    {0xc6, 2, {0x03,0x01}},
    {0xc7, 2, {0x31,0x31}},
    {0xc8, 2, {0x31,0x34}},
    {0xc9, 2, {0x34,0x34}},
    {0xca, 2, {0x34,0x31}},
    {0xcb, 2, {0x31,0x31}},
    {0xcc, 2, {0x31,0x31}},
    {0xcd, 2, {0x2e,0x2d}},
    {0xce, 2, {0x2d,0x2e}},
    {0xcf, 2, {0x31,0x31}},
    {0xd0, 2, {0x31,0x31}},
    {0xd1, 2, {0x31,0x34}},
    {0xd2, 2, {0x34,0x34}},
    {0xd3, 2, {0x34,0x31}},
    {0xd4, 2, {0x31,0x31}},
    {0xd5, 2, {0x00,0x02}},
    {0xd6, 2, {0x10,0x12}},
    {0xd7, 2, {0x14,0x16}},
    {0xe6, 2, {0x32,0x32}},
    {0xd8, 5, {0x00,0x00,0x00,0x00,0x00}},
    {0xd9, 5, {0x00,0x00,0x00,0x00,0x00}},
    {0xe7, 1, {0x00}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x05}},
    {0xed, 1, {0x30}},
    {0xb0, 2, {0x17,0x06}},
    {0xb8, 1, {0x00}},
    {0xc0, 1, {0x0d}},
    {0xc1, 1, {0x0b}},
    {0xc2, 1, {0x23}},
    {0xc3, 1, {0x40}},
    {0xc4, 1, {0x84}},
    {0xc5, 1, {0x82}},
    {0xc6, 1, {0x82}},
    {0xc7, 1, {0x80}},
    {0xc8, 2, {0x0b,0x30}},
    {0xc9, 2, {0x05,0x10}},
    {0xca, 2, {0x01,0x10}},
    {0xcb, 2, {0x01,0x10}},
    {0xd1, 5, {0x03,0x05,0x05,0x07,0x00}},
    {0xd2, 5, {0x03,0x05,0x09,0x03,0x00}},
    {0xd3, 5, {0x00,0x00,0x6a,0x07,0x10}},
    {0xd4, 5, {0x30,0x00,0x6a,0x07,0x10}},
    {0xf0, 5, {0x55,0xaa,0x52,0x08,0x03}},
    {0xb0, 2, {0x00,0x00}},
    {0xb1, 2, {0x00,0x00}},
    {0xb2, 5, {0x05,0x00,0x0a,0x00,0x00}},
    {0xb3, 5, {0x05,0x00,0x0a,0x00,0x00}},
    {0xb4, 5, {0x05,0x00,0x0a,0x00,0x00}},
    {0xb5, 5, {0x05,0x00,0x0a,0x00,0x00}},
    {0xb6, 5, {0x02,0x00,0x0a,0x00,0x00}},
    {0xb7, 5, {0x02,0x00,0x0a,0x00,0x00}},
    {0xb8, 5, {0x02,0x00,0x0a,0x00,0x00}},
    {0xb9, 5, {0x02,0x00,0x0a,0x00,0x00}},
    {0xba, 5, {0x53,0x00,0x0a,0x00,0x00}},
    {0xbb, 5, {0x53,0x00,0x0a,0x00,0x00}},
    {0xbc, 5, {0x53,0x00,0x0a,0x00,0x00}},
    {0xbd, 5, {0x53,0x00,0x0a,0x00,0x00}},
    {0xc4, 1, {0x60}},
    {0xc5, 1, {0x40}},
    {0xc6, 1, {0x64}},
    {0xc7, 1, {0x44}},
    {0x11, 0, {0x00,0x00}},
    {REGFLAG_DELAY, 120, {}},
    {0x29, 0, {0x00,0x00}},
    {REGFLAG_DELAY, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}

};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    { 0x28, 1, { 0x00 } },
    { REGFLAG_DELAY, 20, { } },

    // Sleep Mode On
    { 0x10, 1, { 0x00 } },
    { REGFLAG_DELAY, 120, {} },

    { REGFLAG_END_OF_TABLE, 0x00, {} }
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;
        
        switch (cmd) {
        case REGFLAG_DELAY :
            MDELAY(table[i].count);
            break; 
        case REGFLAG_END_OF_TABLE :
            break;
        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
    
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params) {
	memset(params, 0, sizeof(LCM_PARAMS));
    
    params->type = 2;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dsi.mode = 1;
    params->dsi.LANE_NUM = 3;
    params->dsi.PS = 2;
    params->dsi.packet_size = 512;
    params->dsi.word_count = FRAME_WIDTH * 3; // 2160
    params->dsi.PLL_CLOCK = 250;
    params->dsi.cont_clock = 1;
    params->dsi.ssc_disable = 1;
    params->dsi.clk_lp_per_line_enable = 1;

    params->dsi.data_format.format = 2;
    params->dsi.data_format.color_order = 0;
    params->dsi.data_format.trans_seq = 0;
    params->dsi.data_format.padding = 0;

    params->dsi.esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 10;
    params->dsi.lcm_esd_check_table[0].count = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9Cu; // -100 // ?0xFF9C
    params->dsi.customization_esd_check_enable = 1;

    params->dsi.horizontal_sync_active = 4;
    params->dsi.horizontal_backporch = 40;
    params->dsi.horizontal_frontporch = 40;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 16;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.intermediat_buffer_num = 0;

    params->dbi.te_mode = 1;
    params->dbi.te_edge_polarity = 0;

}

static void lcm_init(void)
{
    LCM_LOGD("!");

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting,
     sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void) {

    unsigned int data_array[16];
    unsigned int id = 0;
    unsigned char buffer[3];
    unsigned int tmp = 0x02;

    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(20);

    data_array[0] = 0x63902;
    data_array[1] = 0x52AA55F0;
    data_array[2] = 0x108;
    dsi_set_cmdq(data_array, 3, 1);
    
    if(ata_checking) tmp = 0x03; // [lcm_ata_check]

    data_array[0] = (tmp << 16) | (0x37 << 8) | 0x00; // 0x23700, 0x33700
    dsi_set_cmdq(data_array, 1, 1);

    // vC11A8CD0 = 1; [lcm_ata_check]

    read_reg_v2(0xC5, buffer, 3);
    
    id = (buffer[1] << 8) | buffer[2];
    LCM_LOGD("Synced id is 0x%2x", id);
    LCM_LOGD("buffer[3] = { %x, %x, %x }", buffer[0], buffer[1], buffer[2]);

    // vC11A8CD0 = 0; [lcm_ata_check]
    ata_checking = 0;

    return (LCM_ID == id) ? 1 : 0;
}


static unsigned int rgk_lcm_compare_id()
{

    int data[4] = {0,0,0,0};
    int res = 0;
    int lcm_vol = 0;
    int rawdata = 0;

    res = IMM_GetOneChannelValue(12, data, &rawdata);

    lcm_vol = data[0] * 1000 + data[1] * 10;
    LCM_LOGD("lcm_vol = %d", lcm_vol);

    if (res < 0 || lcm_vol > 100)
        return 0; // error if here

    return lcm_compare_id();
}

static void lcm_suspend(void) 
{
    unsigned int data_array[16];

    LCM_LOGD("Using dsi_set_cmdq v1!");

    // Display Off
    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);

    MDELAY(20);

    // Sleep In
    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);

    MDELAY(120);
    SET_RESET_PIN(0);
    MDELAY(50);

    LCM_LOGD("Using dsi_set_cmdq v2/p!");
    push_table(lcm_deep_sleep_mode_in_setting,
     sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

    if(rgk_lcm_compare_id())
        LCM_LOGD("yay! lcm id is correct.");
}

static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_ata_check(unsigned char *buffer)
{
    ata_checking = 1;
    return lcm_compare_id();
}

/* Get LCM Driver Hooks */
LCM_DRIVER nt35521_boe_hd720_5p0_xingliangda_t591_dg_lcm_drv = 
{
	.name		= LCM_NAME,
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
    	.suspend        = lcm_suspend,
    	.resume         = lcm_resume,
	.compare_id     = rgk_lcm_compare_id,
    	.ata_check      = lcm_ata_check
};
