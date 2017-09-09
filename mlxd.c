#include <fcntl.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <getopt.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <bcm2835.h>

#define VERSION "0.1.0"
#define EXIT_FAILURE 1
#define DEBUG 0
#define DEBUG_TO 0
#define MLX90620 0

#define MLX_ADD (MLX90620 ? 0x90 : 0x40)

char *xmalloc ();
char *xrealloc ();
char *xstrdup ();

inline int signConv16 (int in) {
   return (in >= 32768 ? in - 65536 : in);
}

inline int signConv8 (char in) {
   return (in >= 128 ? in - 256 : in);
}


float temperatures[64];
unsigned short temperaturesInt[64];
static int usage (int status);

/* The name the program was run with, stripped of any leading path. */
char *program_name;

/* getopt_long return codes */
enum {DUMMY_CODE=129
};

/* Option flags and variables */


static struct option const long_options[] =
{
  {"help", no_argument, 0, 'h'},
  {"version", no_argument, 0, 'V'},
  {NULL, 0, NULL, 0}
};

static int decode_switches (int argc, char **argv);
int mlx9062x_init ();
int mlx9062x_read_eeprom ();
int mlx9062x_write_config (unsigned char *lsb, unsigned char *msb);
int mlx9062x_read_config (unsigned char *lsb, unsigned char *msb);
int mlx9062x_write_trim (char t);
char mlx9062x_read_trim ();
int mlx9062x_por ();
int mlx9062x_set_refresh_hz (int hz);
int mlx9062x_ptat ();
int mlx9062x_cp ();
float mlx9062x_ta ();
int mlx9062x_ir_read ();


char EEPROM[256];
signed char ir_pixels[128];

char mlxFifo[] = "/var/run/mlx9062x.sock" ;


void got_sigint(int sig) {
    unlink(mlxFifo); 
    bcm2835_i2c_end();
    exit(0);
   
}

void waitFor(unsigned int secs){
   	unsigned int retTime = time(0) + 1;
	while(time(0) < retTime);
}

main (int argc, char **argv)
{
    signal(SIGINT, got_sigint);
    int fd;

    mkfifo(mlxFifo, 0666);

    int x;
    int i, j;

    float to;
    float ta;
    int vir;
    int vcp;
    double alpha, alpha_comp;
    float vir_compensated;
    float vcp_off_comp, vir_off_comp, vir_tgc_comp;
    float ksta, ks4, ta4;

    /* IR pixel individual offset coefficient */
    int ai;
    /* Individual Ta dependence (slope) of IR pixels offset */
    int bi;
    /* Individual sensitivity coefficient */
    int delta_alpha;
    /* Compensation pixel individual offset coefficients */
    int acp;
    /* Individual Ta dependence (slope) of the compensation pixel offset */
    int bcp;
    /* Sensitivity coefficient of the compensation pixel */
    int alphacp;
    /* Thermal Gradient Coefficient */
    float tgc;
    /* Scaling coefficient for slope of IR pixels offset */
    int bi_scale;
    /* Common sensitivity coefficient of IR pixels */
    int alpha0;
    /* Scaling coefficient for common sensitivity */
    unsigned char alpha0_scale;
    /* Scaling coefficient for individual sensitivity */
    int delta_alpha_scale;
    /* Emissivity */
    float epsilon;


    program_name = argv[0];

    i = decode_switches (argc, argv);


    printf("\n");

    if ( mlx9062x_init() ) {
        printf("OK, MLX9062x init\n");
    } else {
        printf("MLX9062x init failed!\n");
        exit(1);
    }
    usleep(10000);

    ta = mlx9062x_ta();
    //int ir = mlx9062x_ir_read();
    //printf("IR=%4.8f\n",ir);
    // If calibration fails then TA will be WAY too high. check and reinitialize if that happens
    while (ta > 350) 
    {
    	printf("Ta out of bounds! Max is 350, reading: %4.8f C\n", ta);
    	//out of bounds, reset and check again
    	mlx9062x_init();
    	ta = mlx9062x_ta();
    	usleep(10000);
    }

    printf("Ta = %4.8f C %4.8f F\n\n", ta, ta * (9.0/5.0) + 32.0);

    /* To calc parameters */

    vcp = mlx9062x_cp();
    int acommon  = signConv16 ((EEPROM[0xD1] << 8) | EEPROM[0xD0]);
    acp = signConv16 ((EEPROM[0xD4] << 8) | EEPROM[0xD3]);
    bcp = signConv8 (EEPROM[0xD5]);
    alphacp = ( EEPROM[0xD7] << 8 ) | EEPROM[0xD6];
    int tgc_int  = signConv8 (EEPROM[0xD8]);
    tgc = (float)tgc_int/32;
    int ai_scale = (EEPROM[0xD9] & 0xF0) >> 4;
    bi_scale = (EEPROM[0xD9] & 0x0F);
    alpha0 = ( EEPROM[0xE1] << 8 ) | EEPROM[0xE0];
    alpha0_scale = EEPROM[0xE2];
    delta_alpha_scale = EEPROM[0xE3];
    int epsilon_i = ( EEPROM[0xE5] << 8 ) | EEPROM[0xE4];
    epsilon = (float)epsilon_i/ 32768.0;

    if (MLX90620) {
       ksta  = 0;
       ks4   = 1;
    } else {
       ksta  = signConv16 ((EEPROM[0xE7] << 8) | EEPROM[0xE6]);
       ks4   = signConv8 (EEPROM[0xC4]);
       if (DEBUG) printf("KSTA: %f KS4:%f\n", ksta, ks4); 
       ksta /= 1 << 20;
       ks4  /= 1 << ((EEPROM[0xC0] & 0xf)+8);
       if (DEBUG) printf("KSTA: %.2f KS4:%.2f\n", ksta, ks4); 
    
    } 

    ta4   = pow((ta + 273.15), 4);

    vcp_off_comp = (float)vcp - (acp + (bcp * (ta - 25.0)/ (1 << bi_scale)));

    if (DEBUG) {
      printf ("Vcp: %d \n", vcp );
      printf ("Acp: %d \n", acp );
      printf ("Bcp: %d \n", bcp );
      printf ("E: %d \n", epsilon_i );
      printf ("E_s: %f \n", epsilon );
      printf ("Alphacp: %d \n", alphacp );
      printf ("TGC_int: %d \n", tgc_int );
      printf ("TGC: %0.2f \n", tgc );
      printf ("BiScale: %d \n", bi_scale );
      printf ("Alpha_Scale: %d \n", EEPROM[0xE3] );
      printf ("D_Alpha_Scale: %d \n", EEPROM[0xE2] );
    }
    /* do the work */
    do {
        /* POR/Brown Out flag */
        while (!mlx9062x_por()) {
            sleep(1);
            mlx9062x_init();
	    if (DEBUG) printf("Waiting to Init\n");
        }

        if ( !mlx9062x_ir_read() ) { printf ("Not able to read\n\n"); exit(0); }
        for ( i = 0; i < 4; i++ ) {
	  	printf("\n");
		//waitFor(2);
	   	  
		printf("\n");
	    
            for ( j = 0; j < 16; j++ ) {
                x = ((j * 4) + i); /* index */
                vir = signConv16 (( ir_pixels[x*2+1] << 8 ) | ir_pixels[x*2]);
                ai = acommon + (EEPROM[x]*ai_scale);
                bi = signConv8 (EEPROM[0x40 + x]);
                float bi_s = (float)bi/(1 << bi_scale);
                delta_alpha = EEPROM[0x80 + x];

    		/* Calculate To */
        	vir_off_comp = (float)vir - (ai  + (bi_s  * (ta - 25.0)));
        	vir_tgc_comp = vir_off_comp - tgc * vcp_off_comp;
        	vir_compensated = vir_tgc_comp / epsilon;

		if (DEBUG) printf("Vir: %d, Ai: %d, Bi_ee:%d, Bi: %.6f, Ta: %.2f, TGC:%.2f, VIRcp:%.2f\n", vir, ai, bi, bi_s, ta, vir_tgc_comp, vir_compensated); 

                if (DEBUG) 
                  printf ("Alpha0:%d, TGC:%0.2f, Alphacp:%d, BiScale:%d, DAlpha:%d, DAlpha_scale:%d, Alpha_scale:%d\n", alpha0, tgc, alphacp, bi_scale, delta_alpha, delta_alpha_scale, alpha0_scale );

		alpha = (double)delta_alpha/pow(2, delta_alpha_scale) + ((double)alpha0 - tgc*alphacp)/pow(2,alpha0_scale);
		alpha_comp = (1 + ksta*(ta - 25))*alpha;

                if (DEBUG) printf ("Vcp:%.2f, Alpha:%le, Alphacmp=%le, Ta4:%.2f\n", vir_compensated, alpha, alpha_comp, ta4); 
	        double calc  = pow((vir_compensated/alpha_comp + ta4), 1/4.0);
		//printf("calc = %4.8f",calc);
                if (MLX90620) {
	           to = calc - 273.15;
                } else {
		   double sx            = alpha_comp * ks4 * calc;
		   float alpha_comp_upd = alpha_comp*(1 - ks4*273.15) + sx;
	                 to             = pow((vir_compensated/alpha_comp_upd + ta4), 1/4.0) - 273.15;
		   if (DEBUG) printf("Calc:%le, AlphaComp:%le, KS4:%le, Sx:%le, AlphaCompSx:%le\n", calc, alpha_comp, ks4, sx, alpha_comp_upd);
                   
		}

	        temperaturesInt[x] = (unsigned short)((to + 273.15) * 100.0) ; 
		//temperaturesInt[x] = mlx9062x_ta();

		//printf("Ta = %4.8f",to);
	        temperatures[x] = to;
		fd = open(mlxFifo, O_WRONLY);
	   	printf("Ta = %4.8f\n",temperaturesInt[x]);
           	write(fd, temperaturesInt, sizeof(temperaturesInt));
           	close(fd);
		if (DEBUG_TO) printf("%.2f ", to); 
            }
	    if (DEBUG_TO) printf("\n"); 
        }
        if (DEBUG_TO == 0) {
           //fd = open(mlxFifo, O_WRONLY);
	   //printf("Ta = %4.8f\n",temperaturesInt);
           //write(fd, temperaturesInt, sizeof(temperaturesInt));
           //close(fd);
           usleep(100000);
        } else {
           printf("Updated Temperatures!\n");
           usleep(1000000);
	}
    } while (1);

    unlink(mlxFifo);

    exit (0);
}


/* Init */

int
mlx9062x_init()
{
    if (!bcm2835_init()) return 0;
    bcm2835_i2c_begin();
    bcm2835_i2c_set_baudrate(25000);
    
    //sleep 5ms per datasheet
    usleep(5000);
    if ( !mlx9062x_read_eeprom() ) return 0;
    if ( !mlx9062x_write_trim( EEPROM[0xF7] ) ) return 0;
    // Forcing scaling/precision to 3
    if ( !mlx9062x_write_config( &EEPROM[0xF5], &EEPROM[0xF6] ) ) return 0;
    
    mlx9062x_set_refresh_hz( 16 );

    unsigned char lsb, msb;
    mlx9062x_read_config( &lsb, &msb );

    return 1;
}

/* Read the whole EEPROM */

int
mlx9062x_read_eeprom()
{
    const unsigned char read_eeprom[] = {
        0x00 // command
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x50);
    if (
        bcm2835_i2c_write_read_rs((char *)&read_eeprom, 1, EEPROM, 256)
        == BCM2835_I2C_REASON_OK
        ) return 1;

    return 0;
}

/* Write device configuration value */

int
mlx9062x_write_config(unsigned char *lsb, unsigned char *msb)
{
    lsb[0] |= 0x18; 
    unsigned char lsb_check = lsb[0] - 0x55;
    unsigned char msb_check = msb[0] - 0x55;

    unsigned char write_config[] = {
        0x03, // command
        lsb_check,
        lsb[0],
        msb_check,
        msb[0]
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        bcm2835_i2c_write((const char *)&write_config, 5)
        == BCM2835_I2C_REASON_OK
        ) return 1;

    return 0;
}

/* Reading configuration */

int
mlx9062x_read_config(unsigned char *lsb, unsigned char *msb)
{
    unsigned char config[2];

    const unsigned char read_config[] = {
        0x02, // command
        0x92, // start address
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        !bcm2835_i2c_write_read_rs((char *)&read_config, 4, config, 2)
        == BCM2835_I2C_REASON_OK
        ) return 0;

    *lsb = config[0];
    *msb = config[1];
    return 1;
}

/* Write the oscillator trimming value */

int
mlx9062x_write_trim(char t)
{
    unsigned char trim[] = {
        0x00, // MSB
        t     // LSB
    };
    unsigned char trim_check_lsb = trim[1] - 0xAA;
    unsigned char trim_check_msb = trim[0] - 0xAA;
    
    unsigned char write_trim[] = {
        0x04, // command
        trim_check_lsb,
        trim[1],
        trim_check_msb,
        trim[0]
    };
    
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        bcm2835_i2c_write((char *)&write_trim, 5)
        == BCM2835_I2C_REASON_OK
        ) return 1;
    
    return 0;
}

/* Read oscillator trimming register */

char
mlx9062x_read_trim()
{
    unsigned char trim_bytes[2];

    const unsigned char read_trim[] = {
        0x02, // command
        0x93, // start address
        0x00, // address step
        0x01  // number of reads
    };
    
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        bcm2835_i2c_write_read_rs((char *)&read_trim, 4, trim_bytes, 2)
        == BCM2835_I2C_REASON_OK
        ) return 1;
    
    return trim_bytes[0];
}

/* Return POR/Brown-out flag */

int
mlx9062x_por()
{
    unsigned char config_lsb, config_msb;

    mlx9062x_read_config( &config_lsb, &config_msb );
    return ((config_msb & 0x04) == 0x04);
}

/* Set IR Refresh rate */

int
mlx9062x_set_refresh_hz(int hz)
{
    char rate_bits;
    
    switch (hz) {
        case 512:
            rate_bits = 0b0000;
            break;
        case 256:
            rate_bits = 0b0110;
            break;
        case 128:
            rate_bits = 0b0111;
            break;
        case 64:
            rate_bits = 0b1000;
            break;
        case 32:
            rate_bits = 0b1001;
            break;
        case 16:
            rate_bits = 0b1010;
            break;
        case 8:
            rate_bits = 0b1011;
            break;
        case 4:
            rate_bits = 0b1100;
            break;
        case 2:
            rate_bits = 0b1101;
            break;
        case 1:
            rate_bits = 0b1110; // default
            break;
        case 0:
            rate_bits = 0b1111; // 0.5 Hz
            break;
        default:
            rate_bits = 0b1110;
    }

    unsigned char config_lsb, config_msb;
    if ( !mlx9062x_read_config( &config_lsb, &config_msb ) ) return 0;
    if (DEBUG) printf ("Old config: %x %x\n", config_msb, config_lsb);

    config_lsb = (config_lsb & 0xf0) | rate_bits;
    if (DEBUG) printf ("New config: %x %x\n", config_msb, config_lsb);

    if ( !mlx9062x_write_config( &config_lsb, &config_msb ) ) return 0;

    return 1;
}

/* Return PTAT (Proportional To Absolute Temperature) */

int
mlx9062x_ptat()
{
    int ptat;
    unsigned char ptat_bytes[2];

    const unsigned char read_ptat[] = {
        0x02, // command
        MLX_ADD, // start address
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        !bcm2835_i2c_write_read_rs((char *)&read_ptat, 4, (char *)&ptat_bytes, 2)
        == BCM2835_I2C_REASON_OK
        ) return 0;

    ptat = ( ptat_bytes[1] << 8 ) | ptat_bytes[0];
    if (DEBUG) printf ("PTAT: %d\n", ptat);
    return ptat;
}

/* Compensation pixel read */

int
mlx9062x_cp()
{
    int cp;
    signed char VCP_BYTES[2];

    const unsigned char compensation_pixel_read[] = {
        0x02, // command
        (MLX_ADD+1), // start address
        0x00, // address step
        0x01  // number of reads
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        !bcm2835_i2c_write_read_rs((char *)&compensation_pixel_read, 4, (char *)&VCP_BYTES, 2)
        == BCM2835_I2C_REASON_OK
        ) return 0;

    cp = signConv16 (( VCP_BYTES[1] << 8 ) | VCP_BYTES[0]);
    return cp;
}

/* calculation of absolute chip temperature */

float
mlx9062x_ta()
{
    int ptat  = mlx9062x_ptat();
    int vth   =  signConv16 (( EEPROM[0xDB] << 8 ) | EEPROM[0xDA]);
    unsigned int   kt1_scale = EEPROM[0xD2];
	           kt1_scale = (kt1_scale >> 4) & 0xf;
    unsigned int   kt2_scale = EEPROM[0xD2];
                   kt2_scale &= 0xf;

    float kt1 = signConv16 (( EEPROM[0xDD] << 8 ) | EEPROM[0xDC]) ;
    float kt2 = signConv16 (( EEPROM[0xDF] << 8 ) | EEPROM[0xDE]) ;

    if (DEBUG) printf ("KT1=%f, KT2=%f, KT_SCALE=%d %d\n", kt1, kt2, kt1_scale, kt2_scale);
    kt1 /= (1 << kt1_scale); 
    kt2 /= (1 << (kt2_scale+10)); 

    if (DEBUG) printf ("VTH=%d, KT1=%f, KT2=%f, KT_SCALE=%x %d %d\n", vth, kt1, kt2, EEPROM[0xD2], kt1_scale, kt2_scale);

    return (float) ((-kt1 + sqrt( kt1*kt1 - (4 * kt2) * (vth - ptat) )) / (2 * kt2) ) + 25.0;
}

/* IR data read */

int
mlx9062x_ir_read()
{
    const unsigned char ir_whole_frame_read[] = {
        0x02, // command
        0x00, // start address
        0x01, // address step
        0x40  // number of reads
    };

    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(0x60);
    if (
        bcm2835_i2c_write_read_rs((char *)&ir_whole_frame_read, 4, ir_pixels, 128)
        == BCM2835_I2C_REASON_OK
        ) return 1;

    return 0;
}


/* Set all the option flags according to the switches specified.
   Return the index of the first non-option argument.  */

static int
decode_switches (int argc, char **argv)
{
  int c;


  while ((c = getopt_long (argc, argv, 
			   "h"	/* help */
			   "V",	/* version */
			   long_options, (int *) 0)) != EOF)
    {
      switch (c)
	{
	case 'V':
	  printf ("mlx %s\n", VERSION);
      exit (0);

	case 'h':
	  usage (0);

	default:
	  usage (EXIT_FAILURE);
	}
    }

  return optind;
}


static int
usage (int status)
{
  printf ("%s - \
\n", program_name);
  printf ("Usage: %s [OPTION]... [FILE]...\n", program_name);
  printf ("\
Options:\n\
  -h, --help                 display this help and exit\n\
  -V, --version              output version information and exit\n\
");
  exit (status);
}

