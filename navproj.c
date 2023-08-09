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
