
/*
 * Note.
 * The below compile option shall be used when compile option CONFIG_ENABLE_ITO_MP_TEST or
	CONFIG_UPDATE_FIRMWARE_BY_SW_ID is enabled.
 * One or more than one the below compile option can be enabled based on the touch ic
	that customer project are used.
 * By default, the below compile option are all disabled.
 */
/*#define CONFIG_ENABLE_CHIP_TYPE_MSG21XXA*/
/*#define CONFIG_ENABLE_CHIP_TYPE_MSG22XX*/
/*#define CONFIG_ENABLE_CHIP_TYPE_MSG26XXM*/
#define CONFIG_ENABLE_CHIP_TYPE_MSG28XX	/*This compile can be used for MSG28XX/MSG58XX/MSG58XXA */

/*
 * Note.
 * If this compile option is not defined, the SW ID mechanism for updating firmware will be disabled.
 * By default, this compile option is disabled.
 */
#define CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*------------------- #ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID ------------------- //*/
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*
 * Note.
 * If this compile option is defined, the update firmware bin file shall be stored in a two dimensional array format.
 * Else, the update firmware bin file shall be stored in an one dimensional array format.
 * Be careful, MSG22XX only support storing update firmware bin file in an one dimensional array format,
	it does not support two dimensional array format.
 * By default, this compile option is enabled.
 */
#define CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY

#endif /*CONFIG_UPDATE_FIRMWARE_BY_SW_ID */
/*------------------- #endif CONFIG_UPDATE_FIRMWARE_BY_SW_ID ------------------- //*/
/*
 * Note.
 * Please change the below touch screen resolution according to the touch panel that you are using
 */
#define TOUCH_SCREEN_X_MAX   (720)	/*LCD_WIDTH */
#define TOUCH_SCREEN_Y_MAX   (1280)	/*LCD_HEIGHT */

typedef struct {
	u16 nSwId;

#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY
	u8 (*pUpdateBin)[1024];
#else /* ONE DIMENSIONAL ARRAY */
	u8 *pUpdateBin;
#endif /* CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY */

} SwIdData_t;

/*
 * Note.
 * The following is sw id enum definition for MSG21XXA.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor that you are using.
 */
typedef enum {
	MSG21XXA_SW_ID_XXXX = 0,
	MSG21XXA_SW_ID_YYYY,
	MSG21XXA_SW_ID_UNDEFINED
} Msg21xxaSwId_e;

/*
 * Note.
 * The following is sw id enum definition for MSG22XX.
 * 0x0000 and 0xFFFF are not allowed to be defined as SW ID.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor that you are using.
 */
typedef enum {
	MSG22XX_SW_ID_XXXX = 0x0001,
	MSG22XX_SW_ID_YYYY = 0x0002,
	MSG22XX_SW_ID_UNDEFINED = 0xFFFF
} Msg22xxSwId_e;

/*
 * Note.
 * The following is sw id enum definition for MSG26XXM.
 * 0x0000 and 0xFFFF are not allowed to be defined as SW ID.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor that you are using.
 */
typedef enum {
	MSG26XXM_SW_ID_XXXX = 0x0001,
	MSG26XXM_SW_ID_YYYY = 0x0002,
	MSG26XXM_SW_ID_UNDEFINED = 0xFFFF
} Msg26xxmSwId_e;

/*
 * Note.
 * The following is sw id enum definition for MSG28XX.
 * 0x0000 and 0xFFFF are not allowed to be defined as SW ID.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor that you are using.
 */
typedef enum {
	MSG28XX_SW_ID_XXXX = 0x0307,
	MSG28XX_SW_ID_YYYY = 0x0002,
	MSG28XX_SW_ID_UNDEFINED = 0xFFFF
} Msg28xxSwId_e;
