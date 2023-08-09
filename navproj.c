/*
 *
 *
 *  PROOF OF CONCEPT NAV PROGRAM - Written and Developed by Chris Nielsen - Intern Test Engineer
 *
 *
 *
 */

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

//kaitlin
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>


//ser
/* nailib include files */
#include "nai_libs/nailib/include/naitypes.h"
#include "nai_libs/nailib/include/nailib.h"
#include "nai_libs/nailib/include/nailib_utils.h"
/* naibrd include files */
#include "nai_libs/naibrd/include/naibrd.h"
#include "nai_libs/naibrd/include/functions/naibrd_ser.h"
/* naiif include files */
#include "nai_libs/naiif/include/naiif_stdio.h"
/* Common Sample Program include files */
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_menu.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_query.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_access.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_display.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_utils.h"


//DA
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
#include <pthread.h>
#endif
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
#include <taskLib.h>
#endif
/* nailib include files */
#include "nai_libs/nailib/include/naitypes.h"
#include "nai_libs/nailib/include/nailib.h"
#include "nai_libs/nailib/include/nailib_utils.h"
/* naibrd include files */
#include "nai_libs/naibrd/include/naibrd.h"
#include "nai_libs/naibrd/include/functions/naibrd_da.h"
/* naiif include files */
#include "nai_libs/naiif/include/naiif_stdio.h"
/* Common Sample Program include files */
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_menu.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_query.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_access.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_display.h"
#include "nai_sample_apps/naiapp_common/include/naiapp_boardaccess_utils.h"


#define DEF_VOLTAGE           2.0
#define DEF_RANGE             10.0
#define DEF_OUTPUT_TRIGGER    0
#define DEF_SIGNAL_MODE       1
#define DEF_STATUS            0
#define DEF_WRAP_VOLTAGE      0.0
#define DEF_CURRENT           0.0

#define BACK_CHAR 'B'

static const int8_t *SAMPLE_PGM_NAME = (const int8_t*)"DA Basic Operations";
static const int8_t *DEF_CONFIG_FILE = (const int8_t*)"default_DA_BasicOps.txt";

/* Function prototypes */
static bool_t Run_DA_BasicOps(int32_t cardIndex, int32_t module, uint32_t modId);
static bool_t Run_CF1DA_BasicOps(int32_t cardIndex, int32_t module, uint32_t modId);

static nai_status_t Handle_DA_Data(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_Mode(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_ClearStatus(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_ClearStatusAllChannels(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_Range(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_Polarity(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_PowerSupply(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_EnableOutput(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_UpdateRate(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_TestEnable(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_CheckPowerOnBIT(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_FloatingPointMode(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_FloatingPointOffset(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_FloatingPointScaleFactor(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_BITThresholds(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_ChannelStatusEnable(int32_t paramCount, int32_t* params);

static nai_status_t Handle_DA_WriteThru(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_Strobe(int32_t paramCount, int32_t* params);

#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
static const int8_t *SAMPLE_WD_PGM_NAME = (const int8_t*)"DA Watchdog Operations";

static nai_status_t Handle_DA_WatchDogQuietTime(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_WatchDogWindowTime(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_DisplayWatchdog(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_StrobeWatchdog(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_kill_WDStrobe_Thread(int32_t paramCount, int32_t* params);
static nai_status_t Handle_DA_WatchdogShowMenu(int32_t paramCount, int32_t* params);

static void naiapp_kill_WDStrobe_Thread();

static bool_t terminateThread;
#endif

static nai_status_t Handle_DA_ModulePowerResetMenu(int32_t paramCount, int32_t* p_params);
static nai_status_t Handle_DA_ClearModulePowerResetStatus(int32_t paramCount, int32_t* p_params);
static nai_status_t Handle_DA_SetModulePowerReset(int32_t paramCount, int32_t* p_params);

void DA_DisplayData(int32_t cardIndex, int32_t module, int32_t maxchan, uint32_t modId);

static const int32_t DEF_DA_CHANNEL = 1;

#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
DWORD WINAPI WD_Strobe_ThreadEntryPoint(LPVOID param);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
void* WD_Strobe_ThreadEntryPoint(void* arg);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
static int WD_Strobe_ThreadEntryPoint(int32_t nParam);
#endif

/********************/
/* Global Variables */
/********************/

/* TX Thread */
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
static HANDLE thread = NULL;
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
static pthread_t thread;
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
static int thread;
#endif

/****** Command Tables *******/
enum da_gen5_da_common_commands
{
   DA_COMMON_CMD_DATA,
   DA_COMMON_CMD_RANGE,
   DA_COMMON_CMD_POLARITY,
   DA_COMMON_CMD_UPDATE_RATE,
   DA_COMMON_CMD_TEST_ENABLE,
   DA_COMMON_CMD_PBIT,
   DA_COMMON_CMD_CLEAR_STATUS,
   DA_COMMON_CMD_CLEAR_STATUS_ALL_CHANS,
   DA_COMMON_CMD_FLOATING_POINT_MODE,
   DA_COMMON_CMD_FLOATING_POINT_OFFSET,
   DA_COMMON_CMD_FLOATING_POINT_SCALE_FACTOR,
   DA_COMMON_CMD_BIT_THRESHOLD,
   DA_COMMON_CMD_CHANNEL_STATUS_ENABLE,
#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
   DA_COMMON_CMD_WD_MENU,
#endif
   DA_COMMON_CMD_MODULE_POWER_RESET_MENU,
   DA_COMMON_CMD_COUNT
};

#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
enum da_gen5_da_watchdog_commands
{
   DA_COMMON_CMD_WD_QUIETTIME,
   DA_COMMON_CMD_WD_WINDOWTIME,
   DA_COMMON_CMD_WD_DISPLAY,
   DA_COMMON_CMD_WD_PET,
   DA_COMMON_CMD_WD_KILL,
   DA_COMMON_CMD_WD_BACK,
   DA_COMMON_CMD_WD_COUNT
};
#endif

enum da_module_power_reset_commands
{
   DA_MODULE_POWER_RESET_CMD_BACK,
   DA_MODULE_POWER_RESET_CMD_CLEAR_MODULE_POWER_RESET_STATUS,
   DA_MODULE_POWER_RESET_CMD_SET_MODULE_POWER_RESET,
   DA_MODULE_POWER_RESET_CMD_COUNT
};
/* NOTE: Leave room at the end of this structure so the DA_SPECIFIC commands can be added later on. The max number of DA_SPECIFIC commands is
         defined by DA_ADDITIONAL_MAX_CMD_COUNT. */
#define DA_ADDITIONAL_MAX_CMD_COUNT 20
naiapp_cmdtbl_params_t DA_StandardOpMenuCmds[DA_COMMON_CMD_COUNT + DA_ADDITIONAL_MAX_CMD_COUNT] =
{
   {"DATA",       "Set Data",                                                 DA_COMMON_CMD_DATA,                        Handle_DA_Data},
   {"RANGE",      "Set Voltage Range",                                        DA_COMMON_CMD_RANGE,                       Handle_DA_Range},
   {"POLARITY",   "Set Polarity",                                             DA_COMMON_CMD_POLARITY,                    Handle_DA_Polarity},
   {"UPDATE_RATE","Set the update rate",                                      DA_COMMON_CMD_UPDATE_RATE,                 Handle_DA_UpdateRate},
   {"TEST_ENABLE","Enable/Disable internal test",                             DA_COMMON_CMD_TEST_ENABLE,                 Handle_DA_TestEnable},
   {"PBIT",       "Check Power-On BIT",                                       DA_COMMON_CMD_PBIT,                        Handle_DA_CheckPowerOnBIT},
   {"CLEAR",      "Clear DA Statuses",                                        DA_COMMON_CMD_CLEAR_STATUS,                Handle_DA_ClearStatus},
   {"ALL CLEAR",  "Clear DA Statuses All Channels",                           DA_COMMON_CMD_CLEAR_STATUS_ALL_CHANS,      Handle_DA_ClearStatusAllChannels},
   {"FP_MODE",    "Enable/Disable Hardware Floating-Point Conversion Mode",   DA_COMMON_CMD_FLOATING_POINT_MODE,         Handle_DA_FloatingPointMode},
   {"OFFSET",     "Set Hardware Floating-Point Conversion Mode Offset",       DA_COMMON_CMD_FLOATING_POINT_OFFSET,       Handle_DA_FloatingPointOffset},
   {"SCALE",      "Set Hardware Floating-Point Conversion Mode Scale Factor", DA_COMMON_CMD_FLOATING_POINT_SCALE_FACTOR, Handle_DA_FloatingPointScaleFactor},
   {"BIT_THRESH",  "Set BIT Error Threshold",                                  DA_COMMON_CMD_BIT_THRESHOLD,              Handle_DA_BITThresholds},
   {"CHANSTAT",   "Channel Status Enable/Disable",                            DA_COMMON_CMD_CHANNEL_STATUS_ENABLE,       Handle_DA_ChannelStatusEnable},
#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
   {"WATCHDOG",   "Show Watchdog Menu Options",                               DA_COMMON_CMD_WD_MENU,                     Handle_DA_WatchdogShowMenu},
#endif
   {"PWRRESET",  "Show Module Power Reset Menu Options", DA_COMMON_CMD_MODULE_POWER_RESET_MENU, Handle_DA_ModulePowerResetMenu}
   /* NOTE: There is room here for DA_ADDITIONAL_MAX_CMD_COUNT commands. They will be copied in later. */
};

#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
naiapp_cmdtbl_params_t DA_WatchdogOpMenuCmds[DA_COMMON_CMD_WD_COUNT] =
{
   {"BACK",           "Back to Main Menu",                             0,                                         NULL},
   {"DISPLAY",        "Display Watchdog Settings",                     DA_COMMON_CMD_WD_DISPLAY,                  Handle_DA_DisplayWatchdog},
   {"TIME QUIET",     "Set Watchdog Quiet Time",                       DA_COMMON_CMD_WD_QUIETTIME,                Handle_DA_WatchDogQuietTime},
   {"WINDOW",         "Set Watchdog Window Time",                      DA_COMMON_CMD_WD_WINDOWTIME,               Handle_DA_WatchDogWindowTime},
   {"STROBE",         "Start thread to continuously strobe watchdog",  DA_COMMON_CMD_WD_PET,                      Handle_DA_StrobeWatchdog},
   {"KILL",           "Kill Watchdog strobing thread",                 DA_COMMON_CMD_WD_KILL,                     Handle_DA_kill_WDStrobe_Thread}
};
#endif

naiapp_cmdtbl_params_t DA_ModulePowerResetMenuCmds[DA_MODULE_POWER_RESET_CMD_COUNT] =
{
   {"BACK",  "Back to Main Menu",               DA_MODULE_POWER_RESET_CMD_BACK,                            NULL},
   {"CLEAR", "Clear Module Power Reset Status", DA_MODULE_POWER_RESET_CMD_CLEAR_MODULE_POWER_RESET_STATUS, Handle_DA_ClearModulePowerResetStatus},
   {"SET",   "Set Module Power Reset Request",  DA_MODULE_POWER_RESET_CMD_SET_MODULE_POWER_RESET,          Handle_DA_SetModulePowerReset}
};

/* DA additional commands. Different DA modules may use any combination of the below commands.*/
#define DA_ADDITIONAL_CMD_ITEM_CH_ENABLE(DA_CMD_NUM)      \
   {"CH_ENABLE",  "Enable/Disable Channel Output",                            DA_CMD_NUM,                     Handle_DA_EnableOutput}
#define DA_ADDITIONAL_CMD_ITEM_MODE(DA_CMD_NUM)           \
   {"MODE",       "Set Voltage/Current Mode",                                 DA_CMD_NUM,                     Handle_DA_Mode}
#define DA_ADDITIONAL_CMD_ITEM_POWER(DA_CMD_NUM)          \
   {"POWER",      "Set Power Supply ON/OFF",                                  DA_CMD_NUM,                     Handle_DA_PowerSupply}

/* NOTE: This structure is meant to be appended to DA_StandardOpMenuCmds, so the first enum MUST be set to DA_COMMON_CMD_COUNT and the number
         of specific commands cannot exceeded DA_ADDITIONAL_MAX_CMD_COUNT. */
enum da_gen5_da2_additional_commands
{
   DA2_ADDITIONAL_CMD_CHANNEL_ENABLE = DA_COMMON_CMD_COUNT,
   DA2_ADDITIONAL_CMD_COUNT
};

naiapp_cmdtbl_params_t DA2_AdditionalMenuCommands[] =
{
   DA_ADDITIONAL_CMD_ITEM_CH_ENABLE(DA2_ADDITIONAL_CMD_CHANNEL_ENABLE),
};

/* NOTE: This structure is meant to be appended to DA_StandardOpMenuCmds, so the first enum MUST be set to DA_COMMON_CMD_COUNT and the number
         of specific commands cannot exceeded DA_ADDITIONAL_MAX_CMD_COUNT. */
enum da_gen5_da3_additional_commands
{
   DA3_ADDITIONAL_CMD_MODE = DA_COMMON_CMD_COUNT,
   DA3_ADDITIONAL_CMD_POWER,
   DA3_ADDITIONAL_CMD_COUNT
};

naiapp_cmdtbl_params_t DA3_AdditionalMenuCommands[] =
{
   DA_ADDITIONAL_CMD_ITEM_MODE(DA3_ADDITIONAL_CMD_MODE),
   DA_ADDITIONAL_CMD_ITEM_POWER(DA3_ADDITIONAL_CMD_POWER)
};

enum da_gen5_cf1_additional_commands
{
   DA_CF1_CMD_WRITETHRU = DA_COMMON_CMD_COUNT,
   DA_CF1_CMD_STROBE,
   DA_CF1_ADDITIONAL_CMD_COUNT
};

#define CF1DA_COMMON_CMD_COUNT 5
naiapp_cmdtbl_params_t DA_CF1OpMenuCmds[CF1DA_COMMON_CMD_COUNT] =
{
   {"DATA",       "Set Data",                                                 DA_COMMON_CMD_DATA,                        Handle_DA_Data},
   {"RANGE",      "Set Voltage Range",                                        DA_COMMON_CMD_RANGE,                       Handle_DA_Range},
   {"POLARITY",   "Set Voltage Range",                                        DA_COMMON_CMD_POLARITY,                    Handle_DA_Polarity},
   {"WriteThru",  "Set Write-Through Mode",                                   DA_CF1_CMD_WRITETHRU,                      Handle_DA_WriteThru},
   {"Strobe",     "Strobe Output",                                            DA_CF1_CMD_STROBE,                         Handle_DA_Strobe},
};

static int32_t g_numBasicMenuCmds = 0;


//needed ser
#define MAX_DATA_RX  1000000              /* Maximum number words to receive in one fifo read */
static const int8_t *CONFIG_FILE = (const int8_t *)"default_SerAsync_Rx.txt";
uint32_t RecvDataBuffer1[MAX_DATA_RX]; // forward ch1
uint32_t RecvDataBuffer2[MAX_DATA_RX]; // left ch5
uint32_t RecvDataBuffer3[MAX_DATA_RX]; // right ch6
uint32_t data1[MAX_DATA_RX]; // front lidar ch1
uint32_t data2[MAX_DATA_RX]; // left lidar ch5
uint32_t data3[MAX_DATA_RX]; // right lidar ch6


			int32_t channel;
		   bool_t stop = NAI_FALSE;
		   int32_t NAI_CARD_INDEX;
		   int32_t moduleCnt;
		   int32_t module;
		   uint32_t NAI_MOD_NUM = 0;
		   int8_t inputBuffer[80];
		   int32_t inputResponseCnt;
		   int32_t numWordsToReadL1;
           int32_t numWordsToReadL2;
           int32_t numWordsToReadL3;
            int JUMPTOBACK;
//needed da
#define CF1DA_COMMON_CMD_COUNT 5
		   int32_t cardIndex = -1;
		   int32_t module = 0;
		   int32_t moduleCount = 0;

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the contents of the data register of the channel specified by the user to the value
 * specified by the user.  For Gen5 modules, the voltage or current will be set based on the current value in
 * the op mode register.  Voltage will be set if the op mode is set to voltage mode, whereas current will be
 * set if the op mode is set to current mode.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_Data(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   naibrd_da_mode_t mode = NAIBRD_DA_MODE_VOLTAGE;
   float64_t data = 0.0;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
      if (!bQuit)
      {
         check_status(naibrd_DA_GetOpMode(da_params->cardIndex, da_params->module, da_params->channel, &mode));
         naiif_printf("Type data value to set in V/mA (depending on the mode): ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if (!bQuit)
         {
            if (inputResponseCnt > 0)
            {
               sscanf((const char*)inputBuffer, "%lf", &data);
               status = check_status(naibrd_DA_SetData(da_params->cardIndex, da_params->module, da_params->channel, mode,
                  data));
            }
         }
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the channels operation mode: Voltage or Current.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_Mode(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   bool_t done = NAI_FALSE;
   char c = '\0';
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      if (da_params->modId == NAIBRD_MODULE_ID_DA3)
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         while (!bQuit && !done)
         {
            naiif_printf("Type 'C' for Current Mode, or 'V' for Voltage mode: ");
            bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
            if (!bQuit)
            {
               if (inputResponseCnt > 0)
               {
                  sscanf((const char*)inputBuffer, "%c", &c);

                  if (c == 'C' || c == 'c')
                  {
                     status = check_status(naibrd_DA_SetOpMode(da_params->cardIndex, da_params->module, da_params->channel,
                        NAIBRD_DA_MODE_CURRENT));
                     done = NAI_TRUE;
                  }
                  else if (c == 'V' || c == 'v')
                  {
                     status = check_status(naibrd_DA_SetOpMode(da_params->cardIndex, da_params->module, da_params->channel,
                        NAIBRD_DA_MODE_VOLTAGE));
                     done = NAI_TRUE;
                  }
                  else
                  {
                     naiif_printf("Invalid entry...\r\n");
                     done = NAI_FALSE;
                  }
               }
            }
         }
      }
      else
      {
         naiif_printf("This feature is not supported by this module. Press Enter\r\n");
         sscanf((const char*)inputBuffer, "%c", &c);
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function clears the status(es) of the channel specified by the user.  Clearing the status of a channel
 * sets the latched status bit of the given status corresponding to the given channel to 0.  Statuses that can
 * be cleared are BIT and Overcurrent statuses.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_ClearStatus(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   uint32_t statnum = 0u;
   uint32_t mask = 0x1u;
   uint32_t shift = 0;
   naibrd_da_chan_mapped_status_type_t type = NAIBRD_DA_STATUS_BIT_LATCHED;
   bool_t bQuit = NAI_FALSE;
   bool_t errFlag = NAI_FALSE;
   bool_t clearAllFlag = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("Type Status type to be cleared ('0' = BIT, '1' = Overcurrent, '2' = All Statuses): ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      sscanf((const char*)inputBuffer, "%d", &statnum);

      switch (statnum)
      {
         case 0:
            type = NAIBRD_DA_STATUS_BIT_LATCHED;
            break;
         case 1:
            type = NAIBRD_DA_STATUS_OVERCURRENT_LATCHED;
            check_status(naibrd_DA_ResetOverload(da_params->cardIndex, da_params->module));
            break;
         case 2:
            clearAllFlag = NAI_TRUE;
            check_status(naibrd_DA_ResetOverload(da_params->cardIndex, da_params->module));
            break;
         default:
            errFlag = NAI_TRUE;
            if (!bQuit)
            {
               naiif_printf("\r\nError! Invalid Status Type\r\n");
            }
            break;
      }

      if ((!bQuit) && (inputResponseCnt > 0) && (errFlag == NAI_FALSE) && (clearAllFlag == NAI_FALSE))
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         shift = da_params->channel - 1;
         if (!bQuit)
         {
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, type, (mask << shift)));
            naiif_printf("\r\nStatus Cleared!\r\n");
         }
      }
      else if ((!bQuit) && (inputResponseCnt > 0) && (errFlag == NAI_FALSE) && (clearAllFlag == NAI_TRUE))
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         shift = da_params->channel - 1;
         if (!bQuit)
         {
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, NAIBRD_DA_STATUS_BIT_LATCHED, (mask << shift)));
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, NAIBRD_DA_STATUS_OVERCURRENT_LATCHED,
               (mask << shift)));
            naiif_printf("\r\nStatuses Cleared!\r\n");
         }
      }

      status = NAI_SUCCESS;
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function clears the status(es) of all of the channels on the module.  Clearing the status of a channel
 * sets the latched status bit of the given status corresponding to the given channel to 0.  Statuses that can
 * be cleared are BIT and Overcurrent statuses.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_ClearStatusAllChannels(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   uint32_t statnum = 0;
   uint32_t mask = 0x1u;
   uint32_t shift = 0;
   naibrd_da_chan_mapped_status_type_t type = NAIBRD_DA_STATUS_BIT_LATCHED;
   bool_t bQuit = NAI_FALSE;
   bool_t errFlag = NAI_FALSE;
   bool_t clearAllFlag = NAI_FALSE;
   int32_t chan = 0;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("Type Status type to be cleared ('0' = BIT, '1' = Overcurrent, '2' = All Statuses): ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      sscanf((const char*)inputBuffer, "%d", &statnum);

      switch (statnum)
      {
         case 0:
            type = NAIBRD_DA_STATUS_BIT_LATCHED;
            break;
         case 1:
            type = NAIBRD_DA_STATUS_OVERCURRENT_LATCHED;
            check_status(naibrd_DA_ResetOverload(da_params->cardIndex, da_params->module));
            break;
         case 2:
            clearAllFlag = NAI_TRUE;
            check_status(naibrd_DA_ResetOverload(da_params->cardIndex, da_params->module));
            break;
         default:
            errFlag = NAI_TRUE;
            if (!bQuit)
            {
               naiif_printf("\r\nError! Invalid Status Type\r\n");
            }
            break;
      }

      if ((!bQuit) && (inputResponseCnt > 0) && (errFlag == NAI_FALSE) && (clearAllFlag == NAI_FALSE))
      {
         for (chan = 1; chan <= da_params->maxChannels; chan++)
         {
            shift = chan - 1;
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, type, (mask << shift)));
         }
         naiif_printf("\r\nStatuses Cleared!\r\n");
      }
      else if ((!bQuit) && (inputResponseCnt > 0) && (errFlag == NAI_FALSE) && (clearAllFlag == NAI_TRUE))
      {
         for (chan = 1; chan <= da_params->maxChannels; chan++)
         {
            shift = chan - 1;
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, NAIBRD_DA_STATUS_BIT_LATCHED, (mask << shift)));
            check_status(naibrd_DA_ClearChanMappedStatusRaw(da_params->cardIndex, da_params->module, NAIBRD_DA_STATUS_OVERCURRENT_LATCHED,
               (mask << shift)));
         }
         naiif_printf("\r\nStatuses Cleared!\r\n");
      }
      status = NAI_SUCCESS;
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the voltage/current range of the channel specified by the user to the range specified by
 * the user.  For Gen5 modules, the voltage or current range will be set based on the current value in the op mode
 * register.  Voltage range will be set if the op mode is set to voltage mode, whereas current range will be set
 * if the op mode is set to current mode.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_Range(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   naibrd_da_mode_t mode = NAIBRD_DA_MODE_VOLTAGE;
   float64_t range = 0.0;
   float64_t tempRange = 0.0;
   float64_t defualtRange = 0.0;
   naibrd_da_polarity_t polarity = NAIBRD_DA_POLARITY_UNIPOLAR;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
      if (!bQuit)
      {
         check_status(naibrd_DA_GetOpMode(da_params->cardIndex, da_params->module, da_params->channel, &mode));
         if (da_params->modId == NAIBRD_MODULE_ID_DA3)
         {
            if (mode == NAIBRD_DA_MODE_VOLTAGE)
            {
               defualtRange = 10.0;
               naiif_printf("Type Voltage Range to set ('10' for 10V, '20' for 20V, '40' for 40V)");
            }
            else if (mode == NAIBRD_DA_MODE_CURRENT)
            {
               defualtRange = 25.0;
               naiif_printf("Type Current Range to set ('25' for 25mA, '50' for 50mA, '100' for 100ma)");
            }
            else
            {
            }
         }
         else
         {
            defualtRange = 5.0;
            naiif_printf("Type Voltage Range to set ('2.5' for 2.5V, '5' for 5V, '10' for 10V)");
         }

         naiif_printf("(default:%lf): ", defualtRange);

         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if (!bQuit)
         {
            if (inputResponseCnt > 0)
            {
               sscanf((const char*)inputBuffer, "%lf", &range);

               check_status(naibrd_DA_GetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel, mode, &polarity,
                  &tempRange));
               check_status(naibrd_DA_SetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel, mode, polarity,
                  range));
            }
            else if (inputResponseCnt == 0)
            {
               check_status(naibrd_DA_GetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel, mode, &polarity,
                  &tempRange));
               check_status(naibrd_DA_SetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel, mode, polarity,
                  defualtRange));
            }
         }
      }

      status = NAI_SUCCESS;
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the range polarity mode of the channel specified by the user to the mode specified by
 * the user.  Note that this function does not modify the range or the op mode of any of the channels; it only
 * sets the range mode to unipolar or bipolar based on user input.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_Polarity(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   bool_t errFlag = NAI_FALSE;
   naibrd_da_mode_t mode = NAIBRD_DA_MODE_VOLTAGE;
   naibrd_da_polarity_t polarity = NAIBRD_DA_POLARITY_UNIPOLAR;
   float64_t rangeRead = 0.0;
   naibrd_da_polarity_t tempPolarity = NAIBRD_DA_POLARITY_UNIPOLAR;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
      if (!bQuit)
      {
         check_status(naibrd_DA_GetOpMode(da_params->cardIndex, da_params->module, da_params->channel, &mode));
         naiif_printf("Type polarity mode to set ('U' for Unipolar, 'B' for Bipolar)(default:B): ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if (!bQuit)
         {
            if (inputResponseCnt > 0)
            {
               if (inputBuffer[0] == 'u' || inputBuffer[0] == 'U')
               {
                  polarity = NAIBRD_DA_POLARITY_UNIPOLAR;
               }
               else if (inputBuffer[0] == 'b' || inputBuffer[0] == 'B')
               {
                  polarity = NAIBRD_DA_POLARITY_BIPOLAR;
               }
               else
               {
                  errFlag = NAI_TRUE;
                  naiif_printf("\r\nError! Invalid polarity mode!\r\n");
               }

               if (errFlag == NAI_FALSE)
               {
                  check_status(naibrd_DA_GetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel,
                     mode, &tempPolarity, &rangeRead));
                  check_status(naibrd_DA_SetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel,
                     mode, polarity, rangeRead));
               }
            }
            else if (inputResponseCnt == 0)
            {
               check_status(naibrd_DA_GetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel,
                  mode, &tempPolarity, &rangeRead));
               check_status(naibrd_DA_SetRangePolarity(da_params->cardIndex, da_params->module, da_params->channel,
                  mode, NAIBRD_DA_POLARITY_BIPOLAR, rangeRead));
            }
         }
      }
      status = NAI_SUCCESS;
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function enables/disables the module/channel power supply based on user input.  Applies to Gen5 modules
 * only.  DA1 has one power supply for the entire module, whereas DA3 has a power supply for each channel on the
 * module.  The user can enable or disable the module power supply of a DA1 module or enable or disable any of the
 * power supplies of the channels of a DA3 module.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_PowerSupply(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   bool_t errFlag = NAI_FALSE;
   bool_t enableVal;
   char c = '\0';
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      if (da_params->modId == NAIBRD_MODULE_ID_DA3)
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         if (!bQuit)
         {
            naiif_printf("Type '0' to turn off/disable channel power supply or '1' to turn on/enable channel power supply. (default:1): ");
            bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
            if (!bQuit)
            {
               if (inputResponseCnt > 0)
               {
                  sscanf((const char*)inputBuffer, "%c", &c);
                  enableVal = (c == '0' ? NAI_FALSE : NAI_TRUE);

                  if (errFlag == NAI_FALSE)
                  {
                     check_status(naibrd_DA_SetEnablePowerSupply(da_params->cardIndex, da_params->module, da_params->channel, enableVal));
                     if (enableVal == NAI_FALSE)
                     {
                        naiif_printf("\r\nChannel %d Power Supply Disabled\r\n", da_params->channel);
                     }
                     else
                     {
                        naiif_printf("\r\nChannel %d Power Supply Enabled\r\n", da_params->channel);
                     }
                  }
               }
               else if (inputResponseCnt == 0)
               {
                  check_status(naibrd_DA_SetEnablePowerSupply(da_params->cardIndex, da_params->module, da_params->channel, NAI_TRUE));
                  naiif_printf("\r\nChannel %d Power Supply Enabled\r\n", da_params->channel);
               }
            }
         }
         status = NAI_SUCCESS;
      }
      else
      {
         naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function enables/disables the module/channel's output based on user input.  Applies to Gen5 modules
 * only.  DA1 has one power supply for the entire module, whereas DA3 has a power supply for each channel on the
 * module.  The user can enable or disable the module power supply of a DA1 module or enable or disable any of the
 * power supplies of the channels of a DA3 module.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_EnableOutput(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   bool_t errFlag = NAI_FALSE;
   bool_t enableVal;
   char c = '\0';
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      if (da_params->modId == NAIBRD_MODULE_ID_DA2)
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         if (!bQuit)
         {
            naiif_printf("Type '0' to turn off/disable channel's output or '1' to turn on/enable channel's output. (default:1): ");
            bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
            if (!bQuit)
            {
               if (inputResponseCnt > 0)
               {
                  sscanf((const char*)inputBuffer, "%c", &c);
                  enableVal = (c == '0' ? NAI_FALSE : NAI_TRUE);

                  if (errFlag == NAI_FALSE)
                  {
                     check_status(naibrd_DA_SetOutputEnable(da_params->cardIndex, da_params->module, da_params->channel, enableVal));
                     if (enableVal == NAI_FALSE)
                     {
                        naiif_printf("\r\nChannel %d Output Disabled\r\n", da_params->channel);
                     }
                     else
                     {
                        naiif_printf("\r\nChannel %d Output Enabled\r\n", da_params->channel);
                     }
                  }
               }
               else if (inputResponseCnt == 0)
               {
                  check_status(naibrd_DA_SetOutputEnable(da_params->cardIndex, da_params->module, da_params->channel, NAI_TRUE));
                  naiif_printf("\r\nChannel %d Output Enabled\r\n", da_params->channel);
               }
            }
         }
         status = NAI_SUCCESS;
      }
      else
      {
         naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the update rate for the module. The user is prompted for the value in Hz. If the value is
 * within range for the specific D/A used, then the range will be set, otherwise the original setting will not
 * be affected.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_UpdateRate(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   uint32_t updateRateHz = 0;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("Enter new update rate in Hz: ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            sscanf((const char*)inputBuffer, "%u", &updateRateHz);
            status = check_status(naibrd_DA_SetUpdateRate(da_params->cardIndex, da_params->module, updateRateHz));
         }
      }
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_TestEnable(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   naibrd_da_test_type_t testType = NAIBRD_DA_D3_TEST;
   bool_t testEnable = NAI_FALSE;
   char c = 0;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      while (!bQuit && (c != '2') && (c != '3'))
      {
         naiif_printf("Which test would you like to access? D3 (enter '3'): ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);

         if (!bQuit)
         {
            if (inputResponseCnt > 0)
            {
               sscanf((const char*)inputBuffer, "%c", &c);
               if (c == '3')
               {
                  testType = NAIBRD_DA_D3_TEST;
               }
            }
         }
      }

      if (!bQuit)
      {
         naiif_printf("Enable (enter 'e') or any other value to disable: ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);

         if (inputResponseCnt > 0)
         {
            sscanf((const char*)inputBuffer, "%c", &c);
            if (c == 'e' || c == 'E')
            {
               testEnable = NAI_TRUE;
            }
            else
            {
               testEnable = NAI_FALSE;
            }

            status = check_status(naibrd_DA_SetModuleBITEnable(da_params->cardIndex, da_params->module, testType, testEnable));
         }
      }
   }

   return status;
}

/*****************************************************************************/
/**
 * <summary>
 * Handle_DA_CheckPowerOnBIT() Checks to see if the power-on BIT test
 * has been run on the module. If the PBIT test has run, it checks the result
 * of the test and reports it back.
 * </summary>
 */
/*****************************************************************************/
static nai_status_t Handle_DA_CheckPowerOnBIT(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int32_t channelCount = 0, channel = 0;
   bool_t pbitComplete;
   nai_status_bit_t bitFailed;

   if (APP_PARAM_COUNT == paramCount)
   {
      switch (da_params->modId)
      {
         case NAIBRD_MODULE_ID_DA2:
         case NAIBRD_MODULE_ID_DA3:
         case NAIBRD_MODULE_ID_DA4:
         {
            channelCount = naibrd_DA_GetChannelCount(da_params->modId);

            /* Check to see if PBIT ran for the module. */
            naiif_printf("Checking if the Power-On BIT test has run...\r\n");
            status = naibrd_DA_CheckPowerOnBITComplete(da_params->cardIndex, da_params->module, &pbitComplete);
            naiif_printf("PBIT Complete: %s", (pbitComplete) ? "COMPLETED\r\n" : "NOT COMPLETED\r\n");

            if (pbitComplete)
            {
               /* Read the BIT status */
               naiif_printf("Checking the result of the Power-on BIT test...\r\n");
               for (channel = 1; channel <= channelCount; channel++)
               {
                  status = naibrd_DA_GetChanMappedStatus(da_params->cardIndex, da_params->module, channel, NAIBRD_DA_STATUS_BIT_LATCHED,
                     &bitFailed);
                  naiif_printf("Ch. %d: %s", channel, bitFailed ? "BIT FAILED\r\n" : "BIT Passed\r\n");
               }
            }
         }
         break;
         case NAIBRD_MODULE_ID_DA5:
         case NAIBRD_MODULE_ID_DA1:
         default:
            naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
            status = NAI_ERROR_NOT_SUPPORTED;
            break;
      }
   }

   return status;
}


/**************************************************************************************************************/
/**
 * <summary>
 * This function enables/disables the hardware floating-point conversion mode of the DA module, as specified
 * by the user.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_FloatingPointMode(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   bool_t fpCapable = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naibrd_GetFloatingPointModeCapability(da_params->cardIndex, da_params->module, &fpCapable);
      if (fpCapable == NAI_TRUE)
      {
         naiif_printf("Select Floating-Point Mode to set ('0' for DISABLED, '1' for ENABLED)(default:DISABLED): ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if ((!bQuit) && (inputResponseCnt > 0))
         {
            if (inputBuffer[0] == '0')
            {
               status = naibrd_SetFloatingPointModeEnable(da_params->cardIndex, da_params->module, 0x0);
            }
            else if (inputBuffer[0] == '1')
            {
               status = naibrd_SetFloatingPointModeEnable(da_params->cardIndex, da_params->module, 0x1);
            }
         }
      }
      else
      {
         naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }
   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the hardware floating-point conversion mode offset for the DA channel specified by the
 * user. This function is only applicable when the hardware floating-point conversion mode for the module is
 * enabled.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_FloatingPointOffset(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   float64_t offset = 0.0;
   bool_t fpCapable = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naibrd_GetFloatingPointModeCapability(da_params->cardIndex, da_params->module, &fpCapable);
      if (fpCapable == NAI_TRUE)
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         if (!bQuit)
         {
            naiif_printf("Type Hardware Floating-Point Conversion Mode Offset setting to set: ");
            bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
            if ((!bQuit) && (inputResponseCnt > 0))
            {
               sscanf((const char*)inputBuffer, "%lf", &offset);
               status = check_status(naibrd_DA_SetFloatingPointAttribute(da_params->cardIndex, da_params->module, da_params->channel,
                  NAIBRD_DA_ATTRIBUTE_OFFSET, offset));
            }
         }
      }
      else
      {
         naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }
   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the hardware floating-point conversion mode scale factor for the DA channel specified by
 * the user. This function is only applicable when the hardware floating-point conversion mode for the module
 * is enabled.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_FloatingPointScaleFactor(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   float64_t scale = 0.0;
   bool_t fpCapable = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naibrd_GetFloatingPointModeCapability(da_params->cardIndex, da_params->module, &fpCapable);
      if (fpCapable == NAI_TRUE)
      {
         bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
         if (!bQuit)
         {
            naiif_printf("Type Hardware Floating-Point Conversion Mode Scale Factor setting to set: ");
            bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
            if ((!bQuit) && (inputResponseCnt > 0))
            {
               sscanf((const char*)inputBuffer, "%lf", &scale);
               status = check_status(naibrd_DA_SetFloatingPointAttribute(da_params->cardIndex, da_params->module, da_params->channel,
                  NAIBRD_DA_ATTRIBUTE_SCALE_FACTOR, scale));
            }
         }
      }
      else
      {
         naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }
   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * Handle_DA_BITThresholds() allows the user to set and get the BIT error thresholds.
 * This is an advanced feature.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_BITThresholds(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   uint32_t bitThreshold = 0u;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      switch (da_params->modId)
      {
         case NAIBRD_MODULE_ID_DA2:
         case NAIBRD_MODULE_ID_DA3:
         case NAIBRD_MODULE_ID_DA4:
         {
            naiif_printf("Set or Get BIT Error Threshold? ('S' = Set, 'G' = Get, 'C' = Clear BIT Counter): ");
            naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);

            if (inputBuffer[0] == 'S')
            {
               naiif_printf("\r\nType the desired BIT Error Threshold (Default = 5): ");
               naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
               naiif_printf("\r\n");
               bitThreshold = atoi((const char*)inputBuffer);
               status = naibrd_DA_SetModuleBITErrorThreshold(da_params->cardIndex, da_params->module, bitThreshold);
            }
            else if (inputBuffer[0] == 'G')
            {
               status = naibrd_DA_GetModuleBITErrorThreshold(da_params->cardIndex, da_params->module, &bitThreshold);
               naiif_printf("\r\nBIT Error threshold: %d", bitThreshold);
            }
            else if (inputBuffer[0] == 'C')
            {
               int32_t ch;
               int32_t channels = naibrd_DA_GetChannelCount(da_params->modId);

               naiif_printf("\r\nClearing BIT counters on all channels.\r\n");
               for (ch = 1; ch <= channels; ch++)
                  status = naibrd_DA_ClearModuleBITLogic(da_params->cardIndex, da_params->module, ch);
            }
            else
            {
               naiif_printf("\r\nSelection not recognized.\r\n");
            }
         }
         break;
         case NAIBRD_MODULE_ID_DA1:
         case NAIBRD_MODULE_ID_DA5:
         default:
            naiif_printf("\r\n\r\n*** This feature is not supported by this module.***\r\n");
            status = NAI_ERROR_NOT_SUPPORTED;
            break;
      }
   }
   else
   {
      status = NAI_ERROR_INVALID_VALUE;
   }
   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function Enables\Disables the reporting of the Channel Status. When enabled, the user will get status
 * updates. When disabled, the statuses will not report and status-based interrupts will not assert.
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_ChannelStatusEnable(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      bQuit = naiapp_query_ChannelNumber(da_params->maxChannels, da_params->channel, &(da_params->channel));
      if (!bQuit)
      {
         naiif_printf("Enable Channel Status (Y = YES, [any other key] = NO: ");
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if (!bQuit)
         {
            status = check_status(naibrd_DA_SetChanStatusEnable(da_params->cardIndex, da_params->module, da_params->channel,
               ((inputResponseCnt > 0) && (inputBuffer[0] == 'Y' || inputBuffer[0] == 'y')) ? NAI_TRUE : NAI_FALSE));
         }
      }
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function displays the DA Standard Operations data.
 * </summary>
 */
/**************************************************************************************************************/
void DA_DisplayData(int32_t cardIndex, int32_t module, int32_t maxchan, uint32_t modId)
{
   int32_t chan = 0;
   float64_t data = 0.0;
   naibrd_da_mode_t mode = NAIBRD_DA_MODE_VOLTAGE;
   char* sMode = NULL;
   float64_t range = 0.0;
   naibrd_da_polarity_t polarity = NAIBRD_DA_POLARITY_UNIPOLAR;
   nai_status_bit_t bitStatLatched = 0;
   nai_status_bit_t overCurrentStatLatched = 0;
   nai_status_bit_t bitStatRealtime = 0;
   nai_status_bit_t overCurrentStatRealtime = 0;
   float64_t wrapVolt = 0.0;
   float64_t wrapCurrent = 0.0;
   bool_t floatMode = NAI_FALSE;
   float64_t offset = 0.0;
   float64_t scale = 0.0;
   bool_t enable = NAI_FALSE;
   char* sVoltPolarity = NULL;
   uint32_t updateRateHz = 0u;
   bool_t d3enabled = NAI_FALSE;
   bool_t chanStatusEnabled = NAI_TRUE;
   const char* chanStatus = NULL;

   naiif_printf("\r\n\r\nDA Standard Operations Data:\r\n\r\n");
   if (modId == NAIBRD_MODULE_ID_DA3)
   {
      naiif_printf("    Power   Voltage           Data              Wrap     Wrap    Floating-   Floating-   Channel    BIT   Overcurrent\r\n");
      naiif_printf("    Supply  Current   Data    Range  Voltage   Voltage  Current    Point    Point Scale   Staus    Status   Status   \r\n");
      naiif_printf("Ch  Enable    Mode   (V/mA)    (V)   Polarity    (V)     (mA)     Offset      Factor     Enabled   (R/L)    (R/L)    \r\n");
      naiif_printf("---------------------------------------------------------------------------------------------------------------------\r\n");
   }
   else
   {
      naiif_printf("                    Voltage            Wrap     Wrap    Floating-   Floating-   Channel    BIT   Overcurrent\r\n");
      naiif_printf("    Output   Data    Range  Voltage   Voltage  Current    Point    Point Scale   Staus    Status   Status   \r\n");
      naiif_printf("Ch  Enable   (V)      (V)   Polarity    (V)     (mA)     Offset      Factor     Enabled   (R/L)    (R/L)    \r\n");
      naiif_printf("------------------------------------------------------------------------------------------------------------\r\n");
   }

   for (chan = 1; chan <= maxchan; chan++)
   {
      check_status(naibrd_DA_GetOpMode(cardIndex, module, chan, &mode));
      check_status(naibrd_DA_GetData(cardIndex, module, chan, mode, &data));
      check_status(naibrd_DA_GetRangePolarity(cardIndex, module, chan, mode, &polarity, &range));

      switch (mode)
      {
         case NAIBRD_DA_MODE_VOLTAGE:
            sMode = "Voltage";
            break;
         case NAIBRD_DA_MODE_CURRENT:
            sMode = "Current";
            break;
         default:
            sMode = " ERROR ";
            break;
      }

      switch (polarity)
      {
         case NAIBRD_DA_POLARITY_UNIPOLAR:
            sVoltPolarity = "UNIPOLAR";
            break;
         case NAIBRD_DA_POLARITY_BIPOLAR:
            sVoltPolarity = "BIPOLAR ";
            break;
         default:
            sVoltPolarity = " ERROR  ";
            break;
      }

      check_status(naibrd_DA_GetWrapVoltage(cardIndex, module, chan, &wrapVolt));
      check_status(naibrd_DA_GetWrapCurrent(cardIndex, module, chan, &wrapCurrent));
      check_status(naibrd_DA_GetFloatingPointAttribute(cardIndex, module, chan, NAIBRD_DA_ATTRIBUTE_OFFSET, &offset));
      check_status(naibrd_DA_GetFloatingPointAttribute(cardIndex, module, chan, NAIBRD_DA_ATTRIBUTE_SCALE_FACTOR, &scale));
      check_status(naibrd_DA_GetChanMappedStatus(cardIndex, module, chan, NAIBRD_DA_STATUS_BIT_LATCHED, &bitStatLatched));
      check_status(naibrd_DA_GetChanMappedStatus(cardIndex, module, chan, NAIBRD_DA_STATUS_BIT_REALTIME, &bitStatRealtime));
      check_status(naibrd_DA_GetChanMappedStatus(cardIndex, module, chan, NAIBRD_DA_STATUS_OVERCURRENT_LATCHED, &overCurrentStatLatched));
      check_status(naibrd_DA_GetChanMappedStatus(cardIndex, module, chan, NAIBRD_DA_STATUS_OVERCURRENT_REALTIME, &overCurrentStatRealtime));
      check_status(naibrd_DA_GetChanStatusEnable(cardIndex, module, chan, &chanStatusEnabled));
      chanStatus = (chanStatusEnabled == NAI_FALSE) ? "NO " : "YES";

      if (modId == NAIBRD_MODULE_ID_DA3)
      {
         check_status(naibrd_DA_GetEnablePowerSupply(cardIndex, module, chan, &enable));
         naiif_printf("%2d    %d     %s %7.3f  %7.3f %s  %7.3f  %7.3f  %7.3f      %7.3f      %s     (%1d/%1d)    (%1d/%1d)\r\n", chan, enable, sMode,
            data, range, sVoltPolarity, wrapVolt, wrapCurrent, offset, scale, chanStatus, bitStatRealtime, bitStatLatched,
            overCurrentStatRealtime, overCurrentStatLatched);
      }
      else
      {
         check_status(naibrd_DA_GetOutputEnable(cardIndex, module, chan, &enable));
         naiif_printf("%2d    %d    %7.3f  %7.3f %s  %7.3f  %7.3f  %7.3f      %7.3f      %s     (%1d/%1d)    (%1d/%1d)\r\n", chan, enable,
            data, range, sVoltPolarity, wrapVolt, wrapCurrent, offset, scale, chanStatus, bitStatRealtime, bitStatLatched,
            overCurrentStatLatched, overCurrentStatRealtime);
      }

   }

   check_status(naibrd_DA_GetUpdateRate(cardIndex, module, &updateRateHz));
   naiif_printf("\r\n\r\nUpdate Rate: %uHz\r\n", updateRateHz);

   check_status(naibrd_DA_GetModuleBITEnable(cardIndex, module, NAIBRD_DA_D3_TEST, &d3enabled));
   naiif_printf("Internal tests enabled: ");
   if (d3enabled == NAI_FALSE)
   {
      naiif_printf("None");
   }
   else
   {
      if (d3enabled == NAI_TRUE)
      {
         naiif_printf("D3");
      }
   }
   naiif_printf("\r\n");

   check_status(naibrd_GetRunningInFloatingPointMode(cardIndex, module, &floatMode));
   if (floatMode == NAI_TRUE)
   {
      naiif_printf("Floating-Point Mode: ENABLED\r\n");
   }
   else
   {
      naiif_printf("Floating-Point Mode: DISABLED\r\n");
   }
}

#if !defined (NAIBSP_CONFIG_SOFTWARE_OS_DEOS)
/**************************************************************************************************************/
/**
 * <summary>
 * This function displays the menu for watchdog commands
 * </summary>
 */
/**************************************************************************************************************/
static nai_status_t Handle_DA_WatchdogShowMenu(int32_t paramCount, int32_t* params)
{
   bool_t bQuit = NAI_FALSE;
   bool_t bContinue = NAI_TRUE;
   bool_t bCmdFound = NAI_FALSE;
   int32_t cmd = 0;
   int32_t numMenuCmds = 0;
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   numMenuCmds = DA_COMMON_CMD_WD_COUNT;

   naiapp_utils_LoadParamMenuCommands(numMenuCmds, DA_WatchdogOpMenuCmds);
   while (bContinue)
   {
      Handle_DA_DisplayWatchdog(paramCount,params);
      naiapp_display_ParamMenuCommands((int8_t*)SAMPLE_WD_PGM_NAME);
      naiif_printf("\r\nType DA Watchdog command or %c to quit : main > watchdog >", NAI_QUIT_CHAR);
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            if(inputBuffer[0] == 'B' || inputBuffer[0] == 'b' )
            {
               bContinue = NAI_FALSE;
               numMenuCmds = DA_COMMON_CMD_COUNT;
               naiapp_utils_LoadParamMenuCommands(numMenuCmds, DA_StandardOpMenuCmds);
            }
            else
            {
               bCmdFound = naiapp_utils_GetParamMenuCmdNum(inputResponseCnt, inputBuffer, &cmd);
               if (bCmdFound)
               {
                  DA_WatchdogOpMenuCmds[cmd].func(APP_PARAM_COUNT, (int32_t*)da_params);
               }
               else
               {
                  naiif_printf("Invalid command entered\r\n");
               }
            }
         }
         else
            naiif_printf("Invalid command entered\r\n");
      }
      else
         bContinue = NAI_FALSE;
   }
   return NAI_SUCCESS;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the watchdog quiet time for the module. The user is prompted for the value in ms.
 * </summary>
 */
 /**************************************************************************************************************/
static nai_status_t Handle_DA_WatchDogQuietTime(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   uint32_t quietTime = 0u;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("\r\n*** To use this sample strobe it is recommended to set a quiet time > 500 ms **");
      naiif_printf("\r\nEnter the desired Watchdog Quiet Time (ms): ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            quietTime = (uint32_t)atoi((const char *)inputBuffer);
            status = check_status(naibrd_DA_SetWatchdogQuietTime(da_params->cardIndex, da_params->module, quietTime*1000));
         }
      }
   }

   return status;
}
/**************************************************************************************************************/
/**
 * <summary>
 * This function sets the watchdog window time for the module. The user is prompted for the value in ms.
 * </summary>
 */
 /**************************************************************************************************************/
static nai_status_t Handle_DA_WatchDogWindowTime(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   uint32_t windowTime = 0u;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("\r\n*** To use this sample strobe it is recommended to set a window time > 500 ms **");
      naiif_printf("\r\nEnter the desired Watchdog Quiet Time (ms): ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            windowTime = (uint32_t)atoi((const char *)inputBuffer);
            status = check_status(naibrd_DA_SetWatchdogWindow(da_params->cardIndex, da_params->module, windowTime * 1000));
         }
      }
   }

   return status;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function displays the DA Watchdog Operations data.
 * </summary>
 */
 /**************************************************************************************************************/
static nai_status_t Handle_DA_DisplayWatchdog(int32_t paramCount, int32_t* params)
{
   nai_status_bit_t wdStatLatched = NAI_STATUS_BIT_LO;
   nai_status_bit_t wdStatRT = NAI_STATUS_BIT_LO;
   uint32_t windowTime = 0u;
   uint32_t quietTime = 0u;
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;

   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("\r\n\r\nDA Watchdog Data:\r\n");
      check_status(naibrd_DA_GetWatchdogQuietTime(da_params->cardIndex, da_params->module, &quietTime));
      quietTime = quietTime/1000;
      naiif_printf("Quiet Time: %dmS\r\n", quietTime);
      check_status(naibrd_DA_GetWatchdogWindow(da_params->cardIndex, da_params->module, &windowTime));
      windowTime = windowTime/1000;
      naiif_printf("Window Time: %dmS\r\n", windowTime);

      check_status(naibrd_DA_GetChanMappedStatus(da_params->cardIndex, da_params->module, 1, NAIBRD_DA_STATUS_WATCHDOG_TIMER_FAULT_LATCHED, &wdStatLatched));
      check_status(naibrd_DA_GetChanMappedStatus(da_params->cardIndex, da_params->module, 1, NAIBRD_DA_STATUS_WATCHDOG_TIMER_FAULT_REALTIME, &wdStatRT));
      naiif_printf("WatchDog Status (R/L):  (%1d/%1d)\r\n", wdStatRT, wdStatLatched);

      naiif_printf("\n");
   }
   return NAI_SUCCESS;
}

/**************************************************************************************************************/
/**
 * <summary>
 * This function will start a thread to continuously strobe the watchdog. The user is prompted for the value in ms.
 * NOTE: When this thread/application exits the module will shut off all outputs and will need to be power cycled
 *       in order to be operational.
 * </summary>
 */
 /**************************************************************************************************************/
static nai_status_t Handle_DA_StrobeWatchdog(int32_t paramCount, int32_t* params)
{
   p_naiapp_AppParameters_t da_params = (p_naiapp_AppParameters_t)params;
   bool_t bQuit = NAI_FALSE;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   int32_t* arg = (int32_t*)malloc(sizeof(int32_t) * 3);
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("\r\n**NOTE: When this thread/application exits the module will shut off all outputs and will need to be power cycled in order to be operational **");
      naiif_printf("\r\nEnter Y if you want to continue and N to go back: ");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            if (inputBuffer[0] == 'Y' || inputBuffer[0] == 'y')
            {
               naiif_printf("\r\nStrobing Watchdog every (QuietTime) + (Window)/2...");
               naiif_printf("\r\nStarting thread...");
               /* Spawn thread here */

               arg[0] = da_params->cardIndex;
               arg[1] = da_params->module;

#if defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
               if (thread == 0)
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
               if (thread != ((int32_t*)NULL))
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
               if (thread != ((pthread_t)NULL))
#endif
               {
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
                  LPDWORD threadID = 0;
                  thread = CreateThread(NULL, 0, WD_Strobe_ThreadEntryPoint, arg, 0, threadID);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
                  pthread_create(&thread, NULL, WD_Strobe_ThreadEntryPoint, arg);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
                  thread = taskSpawn("WD_Strobe_Thread", 100, 0, 10000, (FUNCPTR)WD_Strobe_ThreadEntryPoint, (int32_t)arg, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#else
#error Unsupported OS
#endif
                  if (thread != 0) {}
                  else
                  {
                     free(arg);
                     naiif_printf("\r\nFailed to Create Thread");
                  }
               }
               else
               {
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
                  LPDWORD threadID = 0;
#endif
                  /* kill previous thread and create new one. Report this to them. */

                  naiapp_kill_WDStrobe_Thread();

#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
                  thread = CreateThread(NULL, 0, WD_Strobe_ThreadEntryPoint, arg, 0, threadID);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
                  pthread_create(&thread, NULL, WD_Strobe_ThreadEntryPoint, arg);
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
                  thread = taskSpawn("WD_Strobe_Thread", 100, 0, 10000, (FUNCPTR)WD_Strobe_ThreadEntryPoint, (int32_t)arg, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#else
#error Unsupported OS
#endif

#if defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
                  if (thread != 0) {}
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
               if (thread != ((int32_t*)NULL)){}
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
               if (thread != ((pthread_t)NULL)){}
#endif
                  else
                  {

                     free(arg);
                     naiif_printf("\r\nFailed to Create Thread");
                  }

               }

            }
            else
            {
               naiif_printf("\r\nReturning to Menu...");
            }

         }
      }
   }

   return status;
}
/**************************************************************************************************************/
/**
 * <summary>
 * This function will terminate the WD strobing thread. Module will shut off outputs at this state and
 * will need to be power cycled to be operational.
 * </summary>
 */
 /**************************************************************************************************************/
static void naiapp_kill_WDStrobe_Thread()
{
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
   if (thread != 0)
   {
      terminateThread = NAI_TRUE;
      thread = 0;
   }
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
   if (thread != ((int32_t*)NULL))
   {
      terminateThread = NAI_TRUE;
      thread = ((int32_t*)NULL);
   }
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
   if (thread != ((pthread_t)NULL))
   {
      terminateThread = NAI_TRUE;
      thread = ((pthread_t)NULL);
   }
#endif
}
/**************************************************************************************************************/
/**
 * <summary>
 * This function will continuously loop, strobing the watchdog every QuietTime + Window/2.
 * </summary>
 */
 /**************************************************************************************************************/
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_WINDOWS)
DWORD WINAPI WD_Strobe_ThreadEntryPoint(LPVOID param)
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_LINUX)
void* WD_Strobe_ThreadEntryPoint(void* param)
#elif defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
static int WD_Strobe_ThreadEntryPoint(int32_t param)
#else
#error Unsupported OS
#endif
{
   uint32_t windowTime = 0u;
   uint32_t quietTime = 0u;
   int32_t delayTime = 0;
   int32_t* modInfo = (int32_t*)param;
   int32_t cardIndex = modInfo[0];
   int32_t module = modInfo[1];
   terminateThread = NAI_FALSE;
   free(modInfo);


   check_status(naibrd_DA_GetWatchdogQuietTime(cardIndex, module,&quietTime));
   check_status(naibrd_DA_GetWatchdogWindow(cardIndex, module, &windowTime));
   quietTime = quietTime/1000;
   windowTime = windowTime/1000;
   delayTime = quietTime + (windowTime / 2);
   naibrd_DA_WatchdogStrobe(cardIndex, module);
   do
   {
      naiif_msDelay(delayTime);
      check_status(naibrd_DA_WatchdogStrobe(cardIndex, module));
   } while (!terminateThread);
   return NAI_SUCCESS;
}
/**************************************************************************************************************/
/**
 * <summary>
 * This function will terminate the WD strobing thread. Module will shut off outputs at this state and
 * will need to be power cycled to be operational.
 * </summary>
 */
 /**************************************************************************************************************/
static nai_status_t Handle_DA_kill_WDStrobe_Thread(int32_t paramCount, int32_t* params)
{
   NAIBSP_UNREFERENCED_PARAMETER(paramCount);
   NAIBSP_UNREFERENCED_PARAMETER(params);

   naiapp_kill_WDStrobe_Thread();
   return NAI_SUCCESS;
}
#endif

/**************************************************************************************************************/
/**
<summary>
Configure_DA_ModulePowerResetMenu displays the menu for module power reset commands.
</summary>
*/
/**************************************************************************************************************/
static nai_status_t Handle_DA_ModulePowerResetMenu(int32_t paramCount, int32_t* p_params)
{
   bool_t bQuit = NAI_FALSE;
   bool_t bContinue = NAI_TRUE;
   bool_t bCmdFound = NAI_FALSE;
   int32_t cmd = 0;
   int32_t numMenuCmds = 0;
   bool_t poweredDownStatus = NAI_FALSE;
   bool_t notDetectedStatus = NAI_FALSE;
   bool_t notLinkInitStatus = NAI_FALSE;
   bool_t notFWNotStatus = NAI_FALSE;
   bool_t commErrorStatus = NAI_FALSE;
   bool_t resetRequest = NAI_FALSE;
   bool_t powerDownRequest = NAI_FALSE;
   bool_t powerUpRequest = NAI_FALSE;
   p_naiapp_AppParameters_t p_da_params = (p_naiapp_AppParameters_t)p_params;
   int32_t cardIndex = p_da_params->cardIndex;
   int32_t module = p_da_params->module;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   numMenuCmds = DA_MODULE_POWER_RESET_CMD_COUNT;
   naiapp_utils_LoadParamMenuCommands(numMenuCmds, DA_ModulePowerResetMenuCmds);
   while (bContinue)
   {
      naiif_printf("\r\n\r\n\r\n");
      naiif_printf(" -----------------------------Status------------------------------    ----------Request----------\r\n");
      naiif_printf(" Powered Down  Not Detected  Not Link Init  Not FW Not  Comm Error    Reset  Power Down  Power Up\r\n");
      naiif_printf(" ------------  ------------  -------------  ----------  ----------    -----  ----------  --------\r\n");
      check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_STATUS_POWERED_DOWN, &poweredDownStatus));
      check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_STATUS_NOT_DETECTED, &notDetectedStatus));
      check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_STATUS_NOT_LINK_INIT, &notLinkInitStatus));
      check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_STATUS_FW_NOT_READY, &notFWNotStatus));
      check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_STATUS_COMM_ERROR, &commErrorStatus));
      check_status(naibrd_DA_GetModulePowerReset(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_REQUEST_RESET, &resetRequest));
      check_status(naibrd_DA_GetModulePowerReset(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_REQUEST_POWER_DOWN, &powerDownRequest));
      check_status(naibrd_DA_GetModulePowerReset(cardIndex, module, NAIBRD_DA_MODULE_POWER_RESET_REQUEST_POWER_UP, &powerUpRequest));
      naiif_printf("      %1d             %1d              %1d            %1d           %1d           %1d        %1d          %1d\r\n",
         poweredDownStatus, notDetectedStatus, notLinkInitStatus, notFWNotStatus, commErrorStatus, resetRequest, powerDownRequest, powerUpRequest);
      naiapp_display_ParamMenuCommands((int8_t*)"DT Power Reset Operation Menu");
      naiif_printf("\r\nType DA Module Power Reset command or %c to quit : main > module power reset >", NAI_QUIT_CHAR);
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            if ((inputBuffer[0] == 'B') || (inputBuffer[0] == 'b'))
            {
               bContinue = NAI_FALSE;
            }
            else
            {
               bCmdFound = naiapp_utils_GetParamMenuCmdNum(inputResponseCnt, inputBuffer, &cmd);
               if (bCmdFound)
               {
                  DA_ModulePowerResetMenuCmds[cmd].func(paramCount, p_params);
               }
               else
               {
                  naiif_printf("\r\nInvalid command entered\r\n");
               }
            }
         }
      }
      else
         bContinue = NAI_FALSE;
   }
   naiapp_utils_LoadParamMenuCommands(g_numBasicMenuCmds, DA_StandardOpMenuCmds);
   return (bQuit) ? NAI_ERROR_UNKNOWN : NAI_SUCCESS;
}

/**************************************************************************************************************/
/**
<summary>
Configure_DA_ClearModulePowerResetStatus handles the user request to clear the module power reset status
and calls the method in the naibrd library to clear the module power reset status. The user is
prompted for the module power reset status type to clear.
</summary>
*/
/**************************************************************************************************************/
static nai_status_t Handle_DA_ClearModulePowerResetStatus(int32_t paramCount, int32_t* p_params)
{
   bool_t bQuit = NAI_FALSE;
   naibrd_da_module_power_reset_status_type_t statusTypeToClear = (naibrd_da_module_power_reset_status_type_t)0u;
   bool_t modulePowerResetStatusRead = 0u;
   char statusTypeStr[14] = "";
   p_naiapp_AppParameters_t p_da_params = (p_naiapp_AppParameters_t)p_params;
   int32_t cardIndex = p_da_params->cardIndex;
   int32_t module = p_da_params->module;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   NAIBSP_UNREFERENCED_PARAMETER(paramCount);

   naiif_printf("\r\nSelect Module Power Reset Status type to clear: (0 for Powered Down, 1 for Not Detected, 2 for Not Link Init, 3 for Not FW Not, 4 for Comm Error, q for quit): ");
   bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
   if ((!bQuit) && (inputResponseCnt > 0))
   {
      switch (inputBuffer[0])
      {
      case '0':
         statusTypeToClear = NAIBRD_DA_MODULE_POWER_RESET_STATUS_POWERED_DOWN;
         sprintf(statusTypeStr, "Powered Down");
         break;
      case '1':
         statusTypeToClear = NAIBRD_DA_MODULE_POWER_RESET_STATUS_NOT_DETECTED;
         sprintf(statusTypeStr, "Not Detected");
         break;
      case '2':
         statusTypeToClear = NAIBRD_DA_MODULE_POWER_RESET_STATUS_NOT_LINK_INIT;
         sprintf(statusTypeStr, "Not Link Init");
         break;
      case '3':
         statusTypeToClear = NAIBRD_DA_MODULE_POWER_RESET_STATUS_FW_NOT_READY;
         sprintf(statusTypeStr, "Not FW Not");
         break;
      case '4':
         statusTypeToClear = NAIBRD_DA_MODULE_POWER_RESET_STATUS_COMM_ERROR;
         sprintf(statusTypeStr, "Comm Error");
         break;
      case 'q':
      case 'Q':
         bQuit = NAI_TRUE;
         break;
      default:
         bQuit = NAI_TRUE;
         naiif_printf("\r\nInvalid module power reset status type entered\r\n");
         break;
      }

      if (!bQuit)
      {
         naiif_printf("\r\nAre you sure you want to clear the %s status? (Y for Yes or N for No): ", statusTypeStr);
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if ((!bQuit) && (inputResponseCnt > 0) && (toupper(inputBuffer[0]) == 'Y'))
         {
            check_status(naibrd_DA_GetModulePowerResetStatus(cardIndex, module, statusTypeToClear, &modulePowerResetStatusRead));
            if (modulePowerResetStatusRead == 1u)
            {
               check_status(naibrd_DA_ClearModulePowerResetStatus(cardIndex, module, statusTypeToClear));
            }
         }
      }
   }

   return (bQuit) ? NAI_ERROR_UNKNOWN : NAI_SUCCESS;
}

/**************************************************************************************************************/
/**
<summary>
Configure_DA_SetModulePowerReset handles the user request to set the module power reset request
and calls the method in the naibrd library to set the module power reset request. The user is
prompted for the module power reset request type to set, and then the user is prompted to set
or reset the request bit.
</summary>
*/
/**************************************************************************************************************/
static nai_status_t Handle_DA_SetModulePowerReset(int32_t paramCount, int32_t* p_params)
{
   bool_t bQuit = NAI_FALSE;
   naibrd_da_module_power_reset_type_t resetTypeToSet = (naibrd_da_module_power_reset_type_t)0u;
   char resetTypeStr[11] = "";
   p_naiapp_AppParameters_t p_da_params = (p_naiapp_AppParameters_t)p_params;
   int32_t cardIndex = p_da_params->cardIndex;
   int32_t module = p_da_params->module;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;


   NAIBSP_UNREFERENCED_PARAMETER(paramCount);

   naiif_printf("\r\nSelect Module Power Reset Request type to set: (0 for Reset, 1 for Power Down, 2 for Power Up, q for quit): ");
   bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
   if ((!bQuit) && (inputResponseCnt > 0))
   {
      switch (inputBuffer[0])
      {
      case '0':
         resetTypeToSet = NAIBRD_DA_MODULE_POWER_RESET_REQUEST_RESET;
         sprintf(resetTypeStr, "Reset");
         break;
      case '1':
         resetTypeToSet = NAIBRD_DA_MODULE_POWER_RESET_REQUEST_POWER_DOWN;
         sprintf(resetTypeStr, "Power Down");
         break;
      case '2':
         resetTypeToSet = NAIBRD_DA_MODULE_POWER_RESET_REQUEST_POWER_UP;
         sprintf(resetTypeStr, "Power Up");
         break;
      case 'q':
      case 'Q':
         bQuit = NAI_TRUE;
         break;
      default:
         bQuit = NAI_TRUE;
         naiif_printf("\r\nInvalid module power reset request type entered\r\n");
         break;
      }

      if (!bQuit)
      {
         naiif_printf("\r\nDo you want to set or reset the %s request? (1 to set, 0 to reset): ", resetTypeStr);
         bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
         if ((!bQuit) && (inputResponseCnt > 0))
         {
            if (inputBuffer[0] == '0')
            {
               check_status(naibrd_DA_SetModulePowerReset(cardIndex, module, resetTypeToSet, (bool_t)NAI_FALSE));
            }
            else if (inputBuffer[0] == '1')
            {
               check_status(naibrd_DA_SetModulePowerReset(cardIndex, module, resetTypeToSet, (bool_t)NAI_TRUE));
            }
            else
            {
               naiif_printf("\r\nInvalid selection entered\r\n");
            }
         }
      }
   }

   return (bQuit) ? NAI_ERROR_UNKNOWN : NAI_SUCCESS;
}

static bool_t Run_CF1DA_BasicOps(int32_t cardIndex, int32_t module, uint32_t modId)
{
   bool_t bQuit = NAI_FALSE;
   bool_t bCmdFound = NAI_FALSE;
   naiapp_AppParameters_t da_params;
   int32_t channel = 0;
   float64_t data = 0.0;
   float64_t range = 0.0;
   naibrd_da_polarity_t polarity;
   bool_t writeThru = NAI_FALSE;
   int32_t cmd = 0;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;

   da_params.cardIndex = cardIndex;
   da_params.module = module;
   da_params.channel = 1;
   da_params.displayHex = NAI_FALSE;
   da_params.modId = modId;
   da_params.maxChannels = naibrd_DA_GetChannelCount(modId);

   naiapp_utils_LoadParamMenuCommands(g_numBasicMenuCmds, DA_CF1OpMenuCmds);

   do
   {
      naiif_printf("\r\n\r\n");
      naiif_printf("              Data         Range\r\n");
      naiif_printf("Channel      (Volts)       (Volts)    Polarity \r\n");
      naiif_printf("-------------------------------------------------\r\n");

      for (channel = 1; channel <= da_params.maxChannels; channel++)
      {
         check_status(naibrd_DA_GetData(cardIndex, module, channel, NAIBRD_DA_MODE_VOLTAGE,&data));
         check_status(naibrd_DA_GetRangePolarity(cardIndex, module, channel, NAIBRD_DA_MODE_VOLTAGE, &polarity, &range));

         naiif_printf("%2d         %7.3f       %7.3f        %s\r\n", channel, data, range,
            NAIBRD_DA_POLARITY_UNIPOLAR == polarity ? "UNIPOLAR" : "BIPOLAR");
      }

      check_status(naibrd_DA_GetWriteThroughMode(cardIndex, module, &writeThru));
      naiif_printf("\r\nWrite-Through Mode: %s\r\n", NAI_FALSE == writeThru ? "OFF" : "ON" );

      naiapp_display_ParamMenuCommands((int8_t*)SAMPLE_PGM_NAME);
      naiif_printf("\r\nType DA command or %c to quit : main >", NAI_QUIT_CHAR);
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            bCmdFound = naiapp_utils_GetParamMenuCmdNum(inputResponseCnt, inputBuffer, &cmd);
            if (bCmdFound)
            {
               DA_CF1OpMenuCmds[cmd].func(APP_PARAM_COUNT, (int32_t*)&da_params);
            }
            else
            {
               naiif_printf("Invalid command entered\r\n");
            }
         }
         else
            naiif_printf("Invalid command entered\r\n");
      }
   } while (!bQuit);

   return bQuit;
}

static nai_status_t Handle_DA_WriteThru(int32_t paramCount, int32_t* p_params)
{
   p_naiapp_AppParameters_t p_da_params = (p_naiapp_AppParameters_t)p_params;
   int8_t inputBuffer[5];
   int32_t inputResponseCnt;
   nai_status_t status = NAI_ERROR_UNKNOWN;
   bool_t bQuit = NAI_TRUE;
   bool_t writeThruMode = NAI_FALSE;

   if (APP_PARAM_COUNT == paramCount)
   {
      naiif_printf("\r\n\r\nEnter '1' to set write-through mode on, any other key to turn off.\r\n");
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            if ( '1' == inputBuffer[0] )
            {
               writeThruMode = NAI_TRUE;
            }
         }

         status = check_status(naibrd_DA_SetWriteThroughMode(p_da_params->cardIndex, p_da_params->module, writeThruMode));
      }
   }

   return status;
}

static nai_status_t Handle_DA_Strobe(int32_t paramCount, int32_t* p_params)
{
   p_naiapp_AppParameters_t p_da_params = (p_naiapp_AppParameters_t)p_params;
   nai_status_t status = NAI_ERROR_UNKNOWN;

   if (APP_PARAM_COUNT == paramCount)
   {
      status = check_status(naibrd_DA_UpdateStrobe(p_da_params->cardIndex, p_da_params->module));
   }

   return status;
}


/**************************************************************************************************************/
/**
 * <summary>
 * This function runs the basic operations DA program.  It controls the top level menu of the DA_BasicOps
 * program and calls Handle_DA_Configuration or Handle_DA_StandardOps, depending on what the user specifies.
 * </summary>
 */
/**************************************************************************************************************/
static bool_t Run_DA_BasicOps(int32_t cardIndex, int32_t module, uint32_t modId)
{
   bool_t bQuit = NAI_FALSE;
   bool_t bContinue = NAI_TRUE;
   bool_t bCmdFound = NAI_FALSE;
   int32_t cmd = 0;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;
   naiapp_AppParameters_t da_params;
   da_params.cardIndex = cardIndex;
   da_params.module = module;
   da_params.channel = DEF_DA_CHANNEL;
   da_params.modId = modId;


   /* Add the module specific menu items to the menu. */
   switch (da_params.modId)
   {
      case NAIBRD_MODULE_ID_DA2:
         memcpy(&DA_StandardOpMenuCmds[DA_COMMON_CMD_COUNT], DA2_AdditionalMenuCommands, sizeof(DA2_AdditionalMenuCommands));
         g_numBasicMenuCmds = DA2_ADDITIONAL_CMD_COUNT;
         break;
      case NAIBRD_MODULE_ID_DA3:
         memcpy(&DA_StandardOpMenuCmds[DA_COMMON_CMD_COUNT], DA3_AdditionalMenuCommands, sizeof(DA3_AdditionalMenuCommands));
         g_numBasicMenuCmds = DA3_ADDITIONAL_CMD_COUNT;
         break;
      default:
         naiif_printf("WARNING- No specific menu commands for module id: 0x%X8. Only the standard menu will be displayed.\r\n", da_params.modId);
         g_numBasicMenuCmds = DA_COMMON_CMD_COUNT;
         break;
   }

   /* Get the Maximum DA Channels */
   da_params.maxChannels = naibrd_DA_GetChannelCount(da_params.modId);

   naiapp_utils_LoadParamMenuCommands(g_numBasicMenuCmds, DA_StandardOpMenuCmds);

   while (bContinue)
   {
      DA_DisplayData(cardIndex, module, da_params.maxChannels, modId);
      naiapp_display_ParamMenuCommands((int8_t*)SAMPLE_PGM_NAME);
      naiif_printf("\r\nType DA command or %c to quit : main >", NAI_QUIT_CHAR);
      bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      if (!bQuit)
      {
         if (inputResponseCnt > 0)
         {
            bCmdFound = naiapp_utils_GetParamMenuCmdNum(inputResponseCnt, inputBuffer, &cmd);
            if (bCmdFound)
            {
               DA_StandardOpMenuCmds[cmd].func(APP_PARAM_COUNT, (int32_t*)&da_params);
            }
            else
            {
               naiif_printf("Invalid command entered\r\n");
            }
         }
         else
            naiif_printf("Invalid command entered\r\n");
      }
      else
         bContinue = NAI_FALSE;
   }

   return bQuit;
}





/**************************************************************************************************************/
/**
 * <summary>
 * The purpose of the DA_BasicOps is to illustrate the methods to call in the naibrd library to perform basic
 * operations with the DA modules for configuration setup, controlling the drive outputs, and reading
 * the channels.
 *
 * The following system configuration routines from the nai_sys_cfg.c file are called to assist with the configuration
 * setup for this program prior to calling the naibrd DA routines.
 * - ConfigDevice
 * - DisplayDeviceCfg
 * - GetBoardSNModCfg
 * - CheckModule
 * </summary>
 */
/**************************************************************************************************************/
#if defined (NAIBSP_CONFIG_SOFTWARE_OS_VXWORKS)
int32_t DA_BasicOps(void)
#else
int32_t configda(void)
#endif
{
   bool_t bQuit = NAI_FALSE;
   uint32_t modId = 0u;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;

   if (naiapp_RunBoardMenu(DEF_CONFIG_FILE) == NAI_TRUE)
   {
      while (!bQuit)
      {
         naiapp_query_CardIndex(naiapp_GetBoardCnt(), 0, &cardIndex);
         naibrd_GetModuleCount(cardIndex, &moduleCount);
         naiapp_query_ModuleNumber(moduleCount, 1, &module);
         naibrd_GetModuleName(cardIndex, module, &modId);
         if (NAIBRD_MODULE_ID_CF1 == modId)
         {
          //  g_numBasicMenuCmds = CF1DA_COMMON_CMD_COUNT;
          //  bQuit = Run_CF1DA_BasicOps(cardIndex, module, modId);
         }
         else
         {
            bQuit = Run_DA_BasicOps(cardIndex, module, modId);
         }
      }

      naiif_printf("Type the Enter key to exit the program: ");
      naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
   }

   naiapp_access_CloseAllOpenCards();

   return 0;
}





/**************************************************************************************************************/
/** \ingroup SERAsyncRx
Configures a serial module for asynchronous receiving. The receiving channel is hard-coded to channel #2.
\param cardIndex (Input) Logical Card Index assigned to connection with the NAI_BOARD (0 - NAI_MAX_CARDS-1).
\param module    (Input) Module Number of the module to access (1 - [max modules for board]).
*/
/**************************************************************************************************************/
void Run_SER_ASync_Rx(int32_t cardIndex, int32_t module)
{
   int32_t i;
   int32_t chanNum = 0;
    int run_once = 1;
   uint32_t RecvDataBuffer[MAX_DATA_RX];
   bool_t bQuit = NAI_FALSE;
   int8_t inputBuffer[80];
   int32_t inputResponseCnt;

   /* channelCount = naibrd_SER_GetChannelCount(moduleID); */
   /* naiapp_query_ChannelNumber(channelCount, 1, &chanNum); */
   /* Hard Code channel to ch 2 */
   chanNum = 1; // config 1

   naibrd_SER_ChannelReset(cardIndex, module, chanNum);
   naibrd_SER_ClearRxFifo(cardIndex, module, chanNum);
   naibrd_SER_ClearTxFifo(cardIndex, module, chanNum);

   //naiif_printf ("\r\nSerial Channel # %d\r\n", chanNum);

   /* Configure for ASync on the channel selected. */
   check_status(naibrd_SER_SetCommProtocol(cardIndex, module, chanNum, NAIBRD_SER_PROTOCOL_ASYNC));   /* Async mode  */
   check_status(naibrd_SER_SetInterface(cardIndex, module, chanNum, NAIBRD_SER_INTF_RS232));          /* RS422       */
   check_status(naibrd_SER_SetParityType(cardIndex, module, chanNum, NAIBRD_SER_PARITY_NONE));        /* No Parity   */
   check_status(naibrd_SER_SetNumDataBits(cardIndex, module, chanNum, NAIBRD_SER_DATA_BITS_8));       /* 8 Data Bits */
   check_status(naibrd_SER_SetNumStopBits(cardIndex, module, chanNum, NAIBRD_SER_STOP_BITS_1));       /* 1 Stop Bits */
   check_status(naibrd_SER_SetBaudrate(cardIndex, module, chanNum, 115200));                            /* 9600 baud   */
   check_status(naibrd_SER_SetReceiverEnable(cardIndex, module, chanNum, 1));                         /* Enable the Receiver */

   /* Wait for configuration to be ready */
   naibrd_msDelay(100);

chanNum = 5; // config 2

   naibrd_SER_ChannelReset(cardIndex, module, chanNum);
   naibrd_SER_ClearRxFifo(cardIndex, module, chanNum);
   naibrd_SER_ClearTxFifo(cardIndex, module, chanNum);

  // naiif_printf ("\r\nSerial Channel # %d\r\n", chanNum);

   /* Configure for ASync on the channel selected. */
   check_status(naibrd_SER_SetCommProtocol(cardIndex, module, chanNum, NAIBRD_SER_PROTOCOL_ASYNC));   /* Async mode  */
   check_status(naibrd_SER_SetInterface(cardIndex, module, chanNum, NAIBRD_SER_INTF_RS232));          /* RS422       */
   check_status(naibrd_SER_SetParityType(cardIndex, module, chanNum, NAIBRD_SER_PARITY_NONE));        /* No Parity   */
   check_status(naibrd_SER_SetNumDataBits(cardIndex, module, chanNum, NAIBRD_SER_DATA_BITS_8));       /* 8 Data Bits */
   check_status(naibrd_SER_SetNumStopBits(cardIndex, module, chanNum, NAIBRD_SER_STOP_BITS_1));       /* 1 Stop Bits */
   check_status(naibrd_SER_SetBaudrate(cardIndex, module, chanNum, 115200));                            /* 9600 baud   */
   check_status(naibrd_SER_SetReceiverEnable(cardIndex, module, chanNum, 1));                         /* Enable the Receiver */

   /* Wait for configuration to be ready */
   naibrd_msDelay(100);

chanNum = 6;

   naibrd_SER_ChannelReset(cardIndex, module, chanNum);
   naibrd_SER_ClearRxFifo(cardIndex, module, chanNum);
   naibrd_SER_ClearTxFifo(cardIndex, module, chanNum);

   ///naiif_printf ("\r\nSerial Channel # %d\r\n", chanNum);

   /* Configure for ASync on the channel selected. */
   check_status(naibrd_SER_SetCommProtocol(cardIndex, module, chanNum, NAIBRD_SER_PROTOCOL_ASYNC));   /* Async mode  */
   check_status(naibrd_SER_SetInterface(cardIndex, module, chanNum, NAIBRD_SER_INTF_RS232));          /* RS422       */
   check_status(naibrd_SER_SetParityType(cardIndex, module, chanNum, NAIBRD_SER_PARITY_NONE));        /* No Parity   */
   check_status(naibrd_SER_SetNumDataBits(cardIndex, module, chanNum, NAIBRD_SER_DATA_BITS_8));       /* 8 Data Bits */
   check_status(naibrd_SER_SetNumStopBits(cardIndex, module, chanNum, NAIBRD_SER_STOP_BITS_1));       /* 1 Stop Bits */
   check_status(naibrd_SER_SetBaudrate(cardIndex, module, chanNum, 115200));                            /* 9600 baud   */
   check_status(naibrd_SER_SetReceiverEnable(cardIndex, module, chanNum, 1));                         /* Enable the Receiver */

   /* Wait for configuration to be ready */
   naibrd_msDelay(100);


  // naiif_printf ("Please press Enter to read back data...");
   //while (naiapp_query_ForQuitResponse(sizeof(inputBuffer), 0x0A, inputBuffer, &inputResponseCnt));

   /* Receive data */
   do
   {

chanNum = 1; // config 1
    // LIDAR ONE READ ( FRONT ) //
      /* Check how many words we have waiting in the RX buffer */
      naibrd_SER_GetRxBufferCnt(cardIndex, module, chanNum, (uint32_t*)&numWordsToReadL1);
   //   naiif_printf ("\r\nReading back %d words ...", numWordsToReadL1);

      /* Read data stored in RX buffer */
      check_status(naibrd_SER_ReceiveBufferWithTimeOut32(cardIndex, module, chanNum, RecvDataBuffer1, MAX_DATA_RX, numWordsToReadL1, NAIBRD_FIFO_TIMEOUT_NONE,
         &numWordsToReadL1));
     /// naiif_printf (" %d words read\r\n", numWordsToReadL1);

      /* Print buffer contents - First 8 bits are for data, last 8 bits are status */
      for (i = 0; i < numWordsToReadL1; i++)
      {
        // naiif_printf("Recd 0x%02X, Status= 0x%02X\r\n", (RecvDataBuffer[i] & 0x00FF), (RecvDataBuffer[i] >> 8) & 0x00FF);
         data1[i] = (RecvDataBuffer1[i] & 0x00FF);
      }


chanNum = 5; // config 1
  // LIDAR TWO READ ( LEFT ) //
      /* Check how many words we have waiting in the RX buffer */
      naibrd_SER_GetRxBufferCnt(cardIndex, module, chanNum, (uint32_t*)&numWordsToReadL2);
     /// naiif_printf ("\r\nReading back %d words ...", numWordsToReadL2);

      /* Read data stored in RX buffer */
      check_status(naibrd_SER_ReceiveBufferWithTimeOut32(cardIndex, module, chanNum, RecvDataBuffer2, MAX_DATA_RX, numWordsToReadL2, NAIBRD_FIFO_TIMEOUT_NONE,
         &numWordsToReadL2));
    ///  naiif_printf (" %d words read\r\n", numWordsToReadL2);

      /* Print buffer contents - First 8 bits are for data, last 8 bits are status */
      for (i = 0; i < numWordsToReadL2; i++)
      {
        // naiif_printf("Recd 0x%02X, Status= 0x%02X\r\n", (RecvDataBuffer[i] & 0x00FF), (RecvDataBuffer[i] >> 8) & 0x00FF);
         data2[i] = (RecvDataBuffer2[i] & 0x00FF);
      }

chanNum = 6; // config 1

// LIDAR THREE READ ( RIGHT ) //
      /* Check how many words we have waiting in the RX buffer */
      naibrd_SER_GetRxBufferCnt(cardIndex, module, chanNum, (uint32_t*)&numWordsToReadL3);
    ///  naiif_printf ("\r\nReading back %d words ...", numWordsToReadL3);

      /* Read data stored in RX buffer */
      check_status(naibrd_SER_ReceiveBufferWithTimeOut32(cardIndex, module, chanNum, RecvDataBuffer3, MAX_DATA_RX, numWordsToReadL3, NAIBRD_FIFO_TIMEOUT_NONE,
         &numWordsToReadL3));
     /// naiif_printf (" %d words read\r\n", numWordsToReadL1);

      /* Print buffer contents - First 8 bits are for data, last 8 bits are status */
      for (i = 0; i < numWordsToReadL3; i++)
      {
        // naiif_printf("Recd 0x%02X, Status= 0x%02X\r\n", (RecvDataBuffer[i] & 0x00FF), (RecvDataBuffer[i] >> 8) & 0x00FF);
         data3[i] = (RecvDataBuffer3[i] & 0x00FF);
      }



      //naiif_printf("Press ENTER to receive again, or '%c' to exit program : ", NAI_QUIT_CHAR);
      //bQuit = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
      run_once = run_once - 1;
   } while ( run_once );

   return;
}





	/**************************************************************************************************************/
		/** \ingroup Samples
		\defgroup SERAsyncRx Serial Asynchronous Receive
		The purpose of the Serial Asynchronous Receive sample application is to illustrate the methods to call in the
		naibrd library to configure a given serial channel for receiving RS422. The \ref SERAsyncTx application can be run
		in unison to this application to transmit data.
		\note A physical connection between channel 1 and channel 2 is required.
		*/
		/**************************************************************************************************************/
#ifdef __VXWORKS__
int32_t SER_ASync_Rx_Sample(void)
#else
int32_t configser(void)
#endif
{


		   if (naiapp_RunBoardMenu(CONFIG_FILE) == NAI_TRUE)
		   {
		      while (stop != NAI_TRUE)
		      {
		         /* Query user for the card index */
		         stop = naiapp_query_CardIndex(naiapp_GetBoardCnt(), 0, &NAI_CARD_INDEX);
		         if (stop != NAI_TRUE)
		         {
		            check_status(naibrd_GetModuleCount(NAI_CARD_INDEX, &moduleCnt));

		            /* Query user for the module number */
		            stop = naiapp_query_ModuleNumber(moduleCnt, 1, &module);
		            if (stop != NAI_TRUE)
		            {
		               check_status(naibrd_GetModuleName(NAI_CARD_INDEX, module, &NAI_MOD_NUM));
		               if ((NAI_MOD_NUM != 0))
		               {
		                  return 0;
		               }
		            }
		         }
		     	nai_status_t status;
		     	nai_status_bit_t chan_config_status;
		     	status = naibrd_SER_SetChannelEnable(NAI_CARD_INDEX, NAI_MOD_NUM, 1, NAI_FALSE);
		     	status = naibrd_SER_GetEventMappedStatus(NAI_CARD_INDEX, NAI_MOD_NUM, channel, NAIBRD_SER_EVENT_MAP_STATUS_COMM_CHANNEL_CONFIGURED_REALTIME, &chan_config_status);

		     while (chan_config_status == NAI_STATUS_BIT_HI)
		     {
		         status = naibrd_SER_GetEventMappedStatus(NAI_CARD_INDEX, NAI_MOD_NUM, channel,
		                 NAIBRD_SER_EVENT_MAP_STATUS_COMM_CHANNEL_CONFIGURED_REALTIME, &chan_config_status);
		         printf("config 1");
		     }
		     /* interface level */
		     status = naibrd_SER_SetInterface(NAI_CARD_INDEX, NAI_MOD_NUM, channel, NAIBRD_SER_INTF_RS232);
		     status = naibrd_SER_SetChannelEnable(NAI_CARD_INDEX, NAI_MOD_NUM, channel, NAI_TRUE);
		     status = naibrd_SER_GetEventMappedStatus(NAI_CARD_INDEX, NAI_MOD_NUM, channel,
		              NAIBRD_SER_EVENT_MAP_STATUS_COMM_CHANNEL_CONFIGURED_REALTIME, &chan_config_status);
		     while (chan_config_status == NAI_STATUS_BIT_LO)
		     {
		         status = naibrd_SER_GetEventMappedStatus(NAI_CARD_INDEX, NAI_MOD_NUM, channel,
		                 NAIBRD_SER_EVENT_MAP_STATUS_COMM_CHANNEL_CONFIGURED_REALTIME, &chan_config_status);
		         printf("config 2");
		     }

		         naiif_printf("\r\nType Q to quit or Enter key to restart application:\r\n");
		         stop = naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
		      }
		   }

		   naiif_printf("\r\nType the Enter key to exit the program: ");
		   naiapp_query_ForQuitResponse(sizeof(inputBuffer), NAI_QUIT_CHAR, inputBuffer, &inputResponseCnt);
		   naiapp_access_CloseAllOpenCards();

		   return 0;
		}


                     // all three buffer datas for front left and right lidars , then the returned lidar distances for 3 directions.
bool extract(uint32_t *arrayL1, uint32_t *arrayL2 , uint32_t *arrayL3, int size1, int size2, int size3, int* distL1, int* distL2, int* distL3) {
    int i;
    uint32_t intValue1 ;
    uint32_t intValue2 ;




    for (i = 0; i < size1 - 2; i++) {
    	 if (arrayL1[i] == 0x59 && arrayL1[i+1] == 0x59) {
    	      intValue1  = arrayL1[i+2];
    	      intValue2  = arrayL1[i+3];

    // Convert hex values to integers      // EXTRACT LIDAR 1 DATA
    // Print the integers
    // printf("Integer values: %d %d\n", intValue1, intValue2);
    // convert due to TFLUNAS nature
    *distL1 = intValue1 + (256 * intValue2);
        }
    }


    for (i = 0; i < size2 - 2; i++) {
    	 if (arrayL2[i] == 0x59 && arrayL2[i+1] == 0x59) {
    	          intValue1  = arrayL2[i+2];
    	         intValue2  = arrayL2[i+3];
                                                      // EXTRACT LIDAR 2 DATA
    // Convert hex values to integers
    // Print the integers
    // printf("Integer values: %d %d\n", intValue1, intValue2);
    // convert due to TFLUNAS nature
    *distL2 = intValue1 + (256 * intValue2);
        }
    }


    for (i = 0; i < size3 - 2; i++) {
    	 if (arrayL3[i] == 0x59 && arrayL3[i+1] == 0x59) {
    	           intValue1  = arrayL3[i+2];
    	         intValue2  = arrayL3[i+3];
                                                      // EXTRACT LIDAR 3 DATA
    // Convert hex values to integers
    // Print the integers
    // printf("Integer values: %d %d\n", intValue1, intValue2);
    // convert due to TFLUNAS nature
    *distL3 = intValue1 + (256 * intValue2);
        }

        return true;
    }






    return false;
}



void move_forward() {

printf("DA TURN ON\n");


//  card index, module number, channel , disable/enable
naibrd_DA_SetOutputEnable(0, 2, 1, 1);
naibrd_DA_SetOutputEnable(0, 2, 2, 1);
// both motors drive

    //command for enabling the channel1.

};

void dont_move() {

	naibrd_DA_SetOutputEnable(0, 2, 1, 0);
    naibrd_DA_SetOutputEnable(0, 2, 2, 0);
	printf("DA TURN OFF\n");
    //command for disabling the channel1.

}

void turnright() {

//turn only left motor

	naibrd_DA_SetOutputEnable(0, 2, 1, 1); // ch1 left motor on
    naibrd_DA_SetOutputEnable(0, 2, 2, 0); // ch1 right motor off
    //turn for a short time the disable both motors again.
    sleep(2); // format is sleep(x); where x is # of seconds.
    naibrd_DA_SetOutputEnable(0, 2, 1, 0); // ch1 left motor off
    naibrd_DA_SetOutputEnable(0, 2, 2, 0); // ch1 right motor off
	printf("TURN RIGHT\n");

}


void turnleft() {

//turn only left motor

	naibrd_DA_SetOutputEnable(0, 2, 1, 0); // ch1 left motor off
    naibrd_DA_SetOutputEnable(0, 2, 2, 1); // ch1 right motor on
    //turn for a short time the disable both motors again.
    sleep(2); // format is sleep(x); where x is # of seconds.
    naibrd_DA_SetOutputEnable(0, 2, 1, 0); // ch1 left motor off
    naibrd_DA_SetOutputEnable(0, 2, 2, 0); // ch1 right motor off
	printf("TURN LEFT\n");

}



// MAIN FUNCTION
int main() {
	const int threshold_distance = 10; // Replace with your desired threshold value in CM
    // Threshold difference to determine if there is more space on the right or left
const int turn_threshold = 10; // Replace with your desired turn threshold value in CM
int dist_actualL1, dist_actualL2, dist_actualL3;
int b;
int choice;
    int continueNavigation = 1;
int autoloop;

	// config process from sample apps.
		printf("serial/n");
		configser();
		printf("now da/n");
        //configda();
      //  naibrd_DA_SetPowerSupplyEnable();
      //  naibrd_DA_SetData();
      //  naibrd_DA_SetOutputEnable();


	                              //   card index , module index, unknown , this bit is enable
		naibrd_DA_SetEnablePowerSupply(0, 2, 2, 1);//ch1 Left Wheel

        naibrd_DA_SetEnablePowerSupply(0, 2, 2, 1);//ch2 Right Wheel

                                                                        // for loop step up to spin up slolwy, and a delay?
		naibrd_DA_SetData(0, 2, 1, NAIBRD_DA_MODE_VOLTAGE, 6.0); //ch1 Left Wheel
        naibrd_DA_SetData(0, 2, 2, NAIBRD_DA_MODE_VOLTAGE, 6.0); //ch2 Right Wheel

		naibrd_DA_SetOpMode(0, 2, 1, NAIBRD_DA_MODE_VOLTAGE); //ch1 Right Wheel
        naibrd_DA_SetOpMode(0, 2, 2, NAIBRD_DA_MODE_VOLTAGE); //ch2 Right Wheel
		                                                                              // off or on
		naibrd_DA_SetOutputEnable(0, 2, 1, 0);
        naibrd_DA_SetOutputEnable(0, 2, 2, 0); // turn both off




        while (continueNavigation) {
    {
																						// Part 1: Taking in distance data
    	printf("Select an option:\n");
    	        printf("1. Auto navigate\n");
    	        printf("2. Move forward\n");
    	        printf("3. Turn right\n");
    	        printf("4. Turn left\n");
    	        printf("5. Report distances\n");
    	        printf("6. Stop the program\n");

    	        printf("Enter your choice (1-6): ");
    	        scanf("%d", &choice);



    	        switch (choice) {
    	                  case 1:
    	                      // Add code for auto navigate


    	                	  autoloop=150;
    	        for ( autoloop=1; autoloop <= 150; autoloop++){

			// Call the function to retrieve rx_buffer data for everything.
			Run_SER_ASync_Rx(NAI_CARD_INDEX, module);

        //debug to read data we grabbed:
		//	for (b = 0; b < numWordsToRead; b++)
		//	      {
			       //  printf("Data: 0x%02X\n", data[b]);
		//	      }

	//uint32_t result1, result2;

			int size1 = sizeof(data1) / sizeof(data1[0]);

            int size2 = sizeof(data2) / sizeof(data2[0]);

            int size3 = sizeof(data3) / sizeof(data3[0]);



			bool headerFound = extract(data1, data2, data3 , size1, size2, size3, &dist_actualL1, &dist_actualL2, &dist_actualL3);
			if (headerFound) {
             //printf("Header found\n");
            // printf("The Hex values were: 0x%02X 0x%02X\n", result1, result2);
           printf("the forward distance is: %d cm\n", dist_actualL1);
           printf("the left distance is: %d cm\n", dist_actualL2);
           printf("the right distance is: %d cm\n", dist_actualL3);
			}
			else {
				printf("Header not found\n");
			}

        // Part 2: Processing the data to determine movement
        if (dist_actualL1 > threshold_distance)
        {
            // The robot can continue moving forward in a straight line
            move_forward(); // Call the function to move
            printf("not near an obstacle. continuing\n");

        }
        else if (dist_actualL2 > (dist_actualL3 + turn_threshold )){
        // The robot should turn left, where there is more room.
            turnleft();
            printf("Turning Left\n");
        }

        else if (dist_actualL3 > dist_actualL2 + turn_threshold){
        // The robot should turn left, where there is more room.
            turnright();
            printf("turning right\n");
        }

        else
        {
            dont_move(); // Call the function to not move
            printf("near an obstacle, and there is no better option either left or right within the threshold. stopping\n");

        }
    	        }
    	        naibrd_DA_SetOutputEnable(0, 2, 1, 0);
    	            naibrd_DA_SetOutputEnable(0, 2, 2, 0);
        //printf("press a key to continue: ");
       // getchar();
        break;
         case 2:
         move_forward();
        break;

        case 3:
            turnright();
         break;
        case 4:
                    turnleft();
                 break;
        case 5:
        printf("There is: %d cm between the front of the vehicle and an object.\n", dist_actualL1);
      printf("There is: %d cm between the left side of the vehicle and an object.\n", dist_actualL2);
        printf("There is: %d cm between the right side of the vehicle and an object.\n", dist_actualL3);

                        break;
        case 6:
         continueNavigation = 0;
           break;
				default:
					printf("Invalid choice. Please try again.\n");
			}

			printf("\n");
		}

		printf("Program stopped.\n");

		return 0;
	}
}
