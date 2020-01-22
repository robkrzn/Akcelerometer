#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_lpsci_dma.h"
#include "fsl_i2c.h"
#include "math.h"

#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "clock_config.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_LPSCI UART0
#define DEMO_LPSCI_CLK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)
#define LPSCI_TX_DMA_CHANNEL 0U
#define LPSCI_RX_DMA_CHANNEL 1U
#define LPSCI_TX_DMA_REQUEST kDmaRequestMux0LPSCI0Tx
#define LPSCI_RX_DMA_REQUEST kDmaRequestMux0LPSCI0Rx
#define EXAMPLE_LPSCI_DMAMUX_BASEADDR DMAMUX0
#define EXAMPLE_LPSCI_DMA_BASEADDR DMA0
#define ECHO_BUFFER_LENGTH 8

#define I2C_RELEASE_SDA_PORT PORTE
#define I2C_RELEASE_SCL_PORT PORTE
#define I2C_RELEASE_SDA_GPIO GPIOE
#define I2C_RELEASE_SDA_PIN 25U
#define I2C_RELEASE_SCL_GPIO GPIOE
#define I2C_RELEASE_SCL_PIN 24U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U
#define ACCEL_STATUS 0x00
#define ACCEL_XYZ_DATA 0x01
#define ACCEL_XYZ_DATA_CFG 0x0E
#define ACCEL_CTRL_REG1 0x2A
#define ACCEL_CTRL_REG4 0x2D
#define ACCEL_CTRL_REG5 0x2E
#define ACCEL_PULSE_CFG 0x21
#define ACCEL_PULSE_THSY 0x24
#define ACCEL_PULSE_TMLT 0x26
#define ACCEL_PULSE_LTCY 0x27


#define ALFA 0.0591
#define PI 3.1415926535
/*******************************************************************************
 * Variables
 ******************************************************************************/
i2c_master_handle_t g_m_handle;
uint8_t g_accel_addr_found = 0x1D;
int16_t x, y, z;
float x1,y1,z1;

volatile bool completionFlag = false;
volatile bool nakFlag = false;
lpsci_dma_handle_t g_lpsciDmaHandle;
dma_handle_t g_lpsciTxDmaHandle;
dma_handle_t g_lpsciRxDmaHandle;
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;
volatile int8_t pripravene_data = 0;	//informacia o pripravenosti dat
int16_t filterX,filterY,filterZ=0;
int16_t naklonX,naklonY=0;

/****************************************************************
 * F-U-N-K-C-I-O-N-A-L-I-T-A-
 ****************************************************************/
const int8_t funkcionalita=2;			//0-ciste data zo senzora, 1-vyfiltrovane data zo sentoza, ostatne cisla - filter a uhol naklonu

/*******************************************************************************
 * Code
 ******************************************************************************/
void PORTA_IRQHandler(){		//prerusenie na obsluhu čitania dát akcelerometra
	if(GPIO_GetPinsInterruptFlags(GPIOA) & (1<<15)){	//pin na prerusenie po precitani dat
		GPIO_ClearPinsInterruptFlags(GPIOA, (1 << 15));
		pripravene_data = 1;
	}
	if(GPIO_GetPinsInterruptFlags(GPIOA) & (1<<14)){	//pin na prerusenie po detekcii freefall
		GPIO_ClearPinsInterruptFlags(GPIOA, (1 << 14));
		GPIO_TogglePinsOutput(BOARD_INITPINS_LED_RED_GPIO,1U << BOARD_INITPINS_LED_RED_PIN);
	}
}
static void i2c_release_bus_delay(void)
{
	uint32_t i = 0;
	for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
	{
		__NOP();
	}
}
void BOARD_I2C_ReleaseBus(void){
	uint8_t i = 0;
	gpio_pin_config_t pin_config;
	port_pin_config_t i2c_pin_config = {0};

	/* Config pin mux as gpio */
	i2c_pin_config.pullSelect = kPORT_PullUp;
	i2c_pin_config.mux = kPORT_MuxAsGpio;

	pin_config.pinDirection = kGPIO_DigitalOutput;
	pin_config.outputLogic = 1U;
	CLOCK_EnableClock(kCLOCK_PortE);
	PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
	PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

	GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
	GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

	/* Drive SDA low first to simulate a start */
	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	i2c_release_bus_delay();

	/* Send 9 pulses on SCL and keep SDA low */
	for (i = 0; i < 9; i++){
		GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
		i2c_release_bus_delay();

		GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
		i2c_release_bus_delay();

		GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
		i2c_release_bus_delay();
		i2c_release_bus_delay();
	}

	/* Send stop */
	GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
	i2c_release_bus_delay();

	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
	i2c_release_bus_delay();

	GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
	i2c_release_bus_delay();

	GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
	i2c_release_bus_delay();
}
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
	/* Signal transfer success when received success status. */
	if (status == kStatus_Success)
	{
		completionFlag = true;
	}
	/* Signal transfer success when received success status. */
	if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
	{
		nakFlag = true;
	}
}
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));

	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Write;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = &value;
	masterXfer.dataSize = 1;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*  direction=write : start+device_write;cmdbuff;xBuff; */
	/*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
	i2c_master_transfer_t masterXfer;
	memset(&masterXfer, 0, sizeof(masterXfer));
	masterXfer.slaveAddress = device_addr;
	masterXfer.direction = kI2C_Read;
	masterXfer.subaddress = reg_addr;
	masterXfer.subaddressSize = 1;
	masterXfer.data = rxBuff;
	masterXfer.dataSize = rxSize;
	masterXfer.flags = kI2C_TransferDefaultFlag;

	/*  direction=write : start+device_write;cmdbuff;xBuff; */
	/*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

	I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

	/*  wait for transfer completed. */
	while ((!nakFlag) && (!completionFlag))
	{
	}

	nakFlag = false;

	if (completionFlag == true)
	{
		completionFlag = false;
		return true;
	}
	else
	{
		return false;
	}
}
void LPSCI_UserCallback(UART0_Type *base, lpsci_dma_handle_t *handle, status_t status, void *userData)
{
	userData = userData;

	if (kStatus_LPSCI_TxIdle == status)
	{
		txBufferFull = false;
		txOnGoing = false;
	}

	if (kStatus_LPSCI_RxIdle == status)
	{
		rxBufferEmpty = false;
		rxOnGoing = false;
	}
}
static void LPSCIinit(){
	///////////////////////////////////////////////////////////////////////////////////
	//LPSCI
	lpsci_config_t config;

	BOARD_InitPins();
	BOARD_BootClockRUN();
	CLOCK_SetLpsci0Clock(0x1U);

	LPSCI_GetDefaultConfig(&config);
	config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
	config.enableTx = true;
	config.enableRx = true;

	LPSCI_Init(DEMO_LPSCI, &config, DEMO_LPSCI_CLK_FREQ);

	/* Init DMAMUX */
	DMAMUX_Init(EXAMPLE_LPSCI_DMAMUX_BASEADDR);

	/* Set channel for LPSCI  */
	DMAMUX_SetSource(EXAMPLE_LPSCI_DMAMUX_BASEADDR, LPSCI_TX_DMA_CHANNEL, LPSCI_TX_DMA_REQUEST);
	DMAMUX_EnableChannel(EXAMPLE_LPSCI_DMAMUX_BASEADDR, LPSCI_TX_DMA_CHANNEL);
	DMAMUX_SetSource(EXAMPLE_LPSCI_DMAMUX_BASEADDR, LPSCI_RX_DMA_CHANNEL, LPSCI_RX_DMA_REQUEST);
	DMAMUX_EnableChannel(EXAMPLE_LPSCI_DMAMUX_BASEADDR, LPSCI_RX_DMA_CHANNEL);

	/* Init the DMA module */
	DMA_Init(EXAMPLE_LPSCI_DMA_BASEADDR);
	DMA_CreateHandle(&g_lpsciTxDmaHandle, EXAMPLE_LPSCI_DMA_BASEADDR, LPSCI_TX_DMA_CHANNEL);
	DMA_CreateHandle(&g_lpsciRxDmaHandle, EXAMPLE_LPSCI_DMA_BASEADDR, LPSCI_RX_DMA_CHANNEL);

	/* Create LPSCI DMA handle. */
	LPSCI_TransferCreateHandleDMA(DEMO_LPSCI, &g_lpsciDmaHandle, LPSCI_UserCallback, NULL, &g_lpsciTxDmaHandle,
			&g_lpsciRxDmaHandle);

}
static bool I2Cinit(){
	BOARD_I2C_ReleaseBus();
	BOARD_I2C_ConfigurePins();
	BOARD_InitDebugConsole();
	I2C_MasterTransferCreateHandle(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
	i2c_master_config_t masterConfig;
	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.baudRate_Bps = I2C_BAUDRATE;
	uint32_t sourceClock = CLOCK_GetFreq(I2C0_CLK_SRC);;
	I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);
	return true;
}
static void MMAwrite(uint8_t write_reg, uint8_t databyte){
	I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, write_reg, databyte);
}

static void MMA8451init(){
	MMAwrite(ACCEL_CTRL_REG1, 0x00);
	MMAwrite(ACCEL_XYZ_DATA_CFG, 0x01);
	MMAwrite(ACCEL_CTRL_REG4, 0x09);
	MMAwrite(ACCEL_CTRL_REG5, 0x08);

	MMAwrite(ACCEL_PULSE_THSY, 15);
	MMAwrite(ACCEL_PULSE_TMLT, 5);
	MMAwrite(ACCEL_PULSE_CFG, 0x04);
	MMAwrite(ACCEL_PULSE_LTCY, 20);

	MMAwrite(ACCEL_CTRL_REG1, 0x1D);
}

static void dolnopriepustnyFilter(){
	filterX = ((1-ALFA) * filterX) + (ALFA * x);
	filterY = ((1-ALFA) * filterY) + (ALFA * y);
	filterZ = ((1-ALFA) * filterZ) + (ALFA * z);
}
static void prepocetNaJednoG(){
	x1=((float)filterX/2050);
	y1=((float)filterY/2050);
	z1=((float)filterZ/2050);
}
static void vypocetNaklonu(){
	naklonX=atan(filterX/(sqrt((filterY*filterY)+(filterZ*filterZ)))) *(180.0/PI);
	naklonY=atan(filterY/(sqrt((filterX*filterX)+(filterZ*filterZ)))) *(180.0/PI);
}
static void odosliData(char buff[10]){
	lpsci_transfer_t xfer;
	xfer.data = buff;
	xfer.dataSize = 10;
	txOnGoing = true;
	LPSCI_TransferSendDMA(DEMO_LPSCI, &g_lpsciDmaHandle, &xfer);
}
int main(void)
{
	EnableIRQ(PORTA_IRQn);	//povolenie prerušenia
	LPSCIinit();
	BOARD_InitPins();
	BOARD_BootClockRUN();
	I2Cinit();
	uint8_t readBuff[7];
	MMA8451init();		//inicializacia akcelerometra
	char buff[10];		//buffer na odosielanie dat
	buff[0] = 0xAB;		//hlavicla pre odoslanie suborov do programu SIM PLOT
	buff[1] = 0xCD;		//http://web.archive.org/web/20140125023847/http://www.negtronics.com/simplot
	while(1)
	{
		while(pripravene_data==0);		//caka na vyvolanie prerusenia
		pripravene_data = 0;
		I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_STATUS, readBuff, 7);
		x = ((int16_t)(((readBuff[1] * 256U) | readBuff[2]))) / 4U;
		y = ((int16_t)(((readBuff[3] * 256U) | readBuff[4]))) / 4U;
		z = ((int16_t)(((readBuff[5] * 256U) | readBuff[6]))) / 4U;

		if(funkcionalita==0){//vypis surovych dat
			buff[2] = 6;		//pocet bitov ktore idu za tym 4- 2 hondnoty, 6-3 hodnoty
			buff[3] = 0;		//https://github.com/infomaniac50/projectsimplot/releases
			buff[4] = x & 0xFF;
			buff[5] = x >> 8;
			buff[6] = y & 0xFF;
			buff[7] = y >> 8;
			buff[8] = z & 0xFF;
			buff[9] = z >> 8;
		}else if(funkcionalita==1){
			dolnopriepustnyFilter();
			buff[2] = 6;		//pocet bitov ktore idu za tym 4- 2 hondnoty, 6-3 hodnoty
			buff[3] = 0;		//https://github.com/infomaniac50/projectsimplot/releases
			buff[4] = filterX & 0xFF;
			buff[5] = filterX >> 8;
			buff[6] = filterY & 0xFF;
			buff[7] = filterY >> 8;
			buff[8] = filterZ & 0xFF;
			buff[9] = filterZ >> 8;
		}else{
			dolnopriepustnyFilter();
			prepocetNaJednoG();
			vypocetNaklonu();
			buff[2] = 4;		//pocet bitov ktore idu za tym 4- 2 hondnoty, 6-3 hodnoty
			buff[3] = 0;		//https://github.com/infomaniac50/projectsimplot/releases
			buff[4] = naklonX & 0xFF;
			buff[5] = naklonX >> 8;
			buff[6] = naklonY & 0xFF;
			buff[7] = naklonY >> 8;
		}
		odosliData(buff);		//odosle vypis na COM
	}

}
