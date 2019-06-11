#define LED_PIO_IDB	   ID_PIOC
#define LED_PIOB        PIOC
#define LED_PINB		   13
#define LED_PIN_MASKB   (1<<LED_PINB)

#define LED_PIO_IDY	   ID_PIOA
#define LED_PIOY        PIOA
#define LED_PINY		   4
#define LED_PIN_MASKY   (1<<LED_PINY)

#define LED_PIO_IDR	   ID_PIOA
#define LED_PIOR        PIOA
#define LED_PINR		   24
#define LED_PIN_MASKR   (1<<LED_PINR)

#define LED_PIO_IDG	   ID_PIOD
#define LED_PIOG        PIOD
#define LED_PING		   26
#define LED_PIN_MASKG   (1<<LED_PING)

#define LED_PIO_IDW	   ID_PIOD
#define LED_PIOW        PIOD
#define LED_PINW		   11
#define LED_PIN_MASKW   (1<<LED_PINW)

#define LED_PIO_IDLA	   ID_PIOD
#define LED_PIOLA        PIOD
#define LED_PINLA		   28
#define LED_PIN_MASKLA   (1<<LED_PINLA)

#define LED_PIO_IDSI	   ID_PIOD
#define LED_PIOSI        PIOD
#define LED_PINSI		   27
#define LED_PIN_MASKSI   (1<<LED_PINSI)

#define LED_PIO_IDDO2	   ID_PIOD
#define LED_PIODO2        PIOD
#define LED_PINDO2		   18
#define LED_PIN_MASKDO2   (1<<LED_PINDO2)

typedef struct nota t_nota;
typedef struct notadastruct stru_nota;
struct nota{
	int note;
	int tempo;
};

struct notadastruct{
	int n;
	int led;
	int mask;
};

//MUSICA TESTE

stru_nota task_DO = {.n = 1,
	.led = LED_PIOB,
.mask = LED_PIN_MASKB};
stru_nota task_RE = {.n = 2,
	.led = LED_PIOY,
.mask = LED_PIN_MASKY};
stru_nota task_MI = {.n = 3,
	.led = LED_PIOR,
.mask = LED_PIN_MASKR};
stru_nota task_FA = {.n = 4,
	.led = LED_PIOG,
.mask = LED_PIN_MASKG};
stru_nota task_SOL = {.n = 5,
	.led = LED_PIOW,
.mask = LED_PIN_MASKW};
stru_nota task_LA = {.n = 6,
	.led = LED_PIOLA,
.mask = LED_PIN_MASKLA};
stru_nota task_SI = {.n = 7,
	.led = LED_PIOSI,
.mask = LED_PIN_MASKSI};
stru_nota task_DO2 = {.n = 8,
	.led = LED_PIODO2,
.mask = LED_PIN_MASKDO2};
//do maior
const t_nota DO1 = {.note = 1,
.tempo = 1,};
const t_nota RE1 = {.note = 2,
.tempo = 1,};
const t_nota MI1 = {.note = 3,
.tempo = 1,};
const t_nota FA1 = {.note = 4,
.tempo = 1,};
const t_nota SOL1 = {.note = 5,
.tempo = 1,};
const t_nota LA1 = {.note = 6,
.tempo = 1,};
const t_nota SI1 = {.note = 7,
.tempo = 1,};
const t_nota DO2_1 = {.note = 8,
.tempo = 1,};

t_nota *DOREMIFA[] = {&DO1,&RE1, &MI1,&FA1,&SOL1,&LA1,&SI1,&DO2_1,&DO2_1,&SI1,&LA1,&SOL1,&FA1,&MI1,&RE1,&DO1,&DO1,&RE1, &MI1,&FA1,&SOL1,&LA1,&SI1,&DO2_1,&DO2_1,&SI1,&LA1,&SOL1,&FA1,&MI1,&RE1,&DO1};
static int beat = 1000;
static int teste[][24] = {{1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};