/*
 * UART extended driver(Modbus driver)
 * 
 * Copyright 2020 Dejan Milojica <dejan.milojica@student.etf.unibl.org>
 * 
 * 			This program is free software; you can redistribute it and/or modify
 * 			it under the terms of the GNU General Public License as published by
 *  		the Free Software Foundation; either version 2 of the License, or
 *  		(at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 */


#include <linux/module.h>   			/* Neophodno za sve module.  									*/
#include <linux/kernel.h>   			/* Neophodno za KERN_INFO.   									*/
#include <linux/init.h>     			/* Neophodno za makroe.  										*/
#include <linux/fs.h>  					/* Neophodno za filp_open/close, register_chardev_region        */
#include <linux/device.h>
#include <asm/processor.h>
#include <linux/cdev.h>					/* Neophodno za cdev strukturu.      	    				    */
#include <linux/tty.h>	
#include <linux/gpio.h>					/* Neophodno za gpio kontrolu.		    						*/
#include <linux/gpio/driver.h>			/* Neophodno za gpio driver.		    						*/
#include <linux/uaccess.h>              /* Neophodno za copy_to/from_user().    						*/
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/errno.h>				/* Neophodno za Kod greske!										*/
#include <linux/delay.h>				/* Neophodno za usleep_range.	     							*/



/*************** Driver Functions **********************/
int module_initialization(void);
void module_exit_clean(void);
static int device_open(struct inode *, struct file *);
static int device_release(struct inode *, struct file *);
static ssize_t device_read(struct file *, char *, size_t, loff_t *);
static ssize_t device_write(struct file *, const char *, size_t, loff_t *);
static long virtual_tty_ioctl(struct file *, unsigned int, unsigned long);

/*****************  Methodes *******************/
static void file_close(struct file* filp);
static struct file* file_open(const char* path, int flags, int rights);
static void change_UART_configuration(const struct termios);
static void gpio_set_configure(int cmd, int value);
static int default_UART_configuration(void);
static unsigned long time_of_data_transfer(void);

#define DEVICE_NAME "prosireni_uart" /* Karakterski uredjaj koji predstavlja prosirenje UART-a! */
#define UART_NAME  "/dev/ttyAMA0"     /* Originalni UART!  */

#define GPIO_NAME "/dev/gpio_driver" /* Karakterski uredjaj koji sluzi kao prosirenje GPIO drivera! */

#define CHANGE_VALUE_GPIO _IOW('a','a',int32_t*)	 /* Promjena vrijednosti pina unutar GPIO drivera! */
#define CHANGE_GPIO_PIN   _IOW('a','b',int32_t*)	 /* Promjena pina kao BCM_PIN_DE-a! unutar GPIO drivera*/	

#define WR_BCM_PIN_DE _IOW('a','a',int32_t*)		 /* Promjena vrijednosti BCM_PIN_DE! */
#define RD_BCM_PIN_DE _IOR('a','b',int32_t*)		 /* Citanje vrijednosti BCM_PIN_DE! */
#define WR_SERIAL_PARAM _IOW('a','C',int32_t*)		 /* Prenos parametara serijskog prenosa! */

#define SUCCESS 0		/* Povratna vrijednost u slucaju korektnog rezultata izvrsavanja!*/

enum GPIO_KONFIGURACIJA {PROMJENA_PINA, PROMJENA_VRIJEDNOSTI};
enum PIN_VRIJEDNOSTI {LOW, HIGH};  /* Vrijednosti upisane u odgovarajuci pin! */

/*
* Globalne varijable su deklarisane kao 'static', tako da su globalne sa fajlom!
*/
static struct file *uart_filp = NULL;  /* Omogucava otvaranje UART datoteke! */
static struct file *gpio_filp = NULL;  /* Omogucava otvaranje GPIO datoteke! */

static dev_t virt_tty_device;		   /* 32-bitni podataka koji cuva MAJOR(12bita) i MINOR(20bita) brojeva uredjaja! */
static struct cdev c_dev;			   /* Da bi interno predstavilo karakterske uredjaje, linux jezgro koristi cdev str*/
static struct class *cls;			   /* Klasa uredjaja! */

static struct termios setting; 		   /* TERMIOS struktura! */

/* Struktura koja reprezentuje prosireni UART driver(Konfiguracija serijskog prenosa)! */
typedef struct struktura_prosireni_uart{
    /* Bauds: 9600, 19200, 57600, 115200, etc */
    int baud;
    /* Data bit */
    uint8_t data_bit;
    /* Stop bit */
    uint8_t stop_bit;
    /* Parity: 'N', 'O', 'E' */
    char parity;
}PARAMETRI_UART_INTERFEJSA;

/* Pin za kontrolu RS-485 transivera! */
static int BCM_PIN_DE = 22;
static PARAMETRI_UART_INTERFEJSA uart_konf;

/*
 * Struktura sadrzi pokazivace na funkcije definisane za dati drajver,
 * koje izvrsavaju razlicite operacije nad uredjajem!
*/
static struct file_operations fops =
{
	.owner 			=   THIS_MODULE,
    .read       	=   device_read,
    .write      	=   device_write,
    .open       	=   device_open,
	.unlocked_ioctl =   virtual_tty_ioctl,
    .release        =   device_release
};

/* Deklaracija init i exit funkcija. */
module_init(module_initialization);
module_exit(module_exit_clean);

/*
* Funkcija za inicijalizaciju modula, poziva se pri njegovom ucitavanju!
*/
int __init module_initialization(void)
{
	int ret;
	struct device *dev_ret;

    pr_info("======================\n");
    pr_info(" MODBUS KERNEL MODUL!\n");
    pr_info("======================\n");
	pr_info("Device name %s!\n",DEVICE_NAME);
    pr_info("Inserting device module\n");

	/* Otvaranje UART device file-a. */
	uart_filp =  file_open(UART_NAME, O_RDWR|O_NOCTTY|O_SYNC|O_EXCL|O_NONBLOCK|O_LARGEFILE|O_CLOEXEC, 0777);
	if(uart_filp == NULL){
		return -EINVAL;	
	}

	/* Otvaranje GPIO device file-a. */
	gpio_filp =  file_open(GPIO_NAME, O_RDWR|O_NOCTTY|O_SYNC|O_EXCL|O_NONBLOCK|O_LARGEFILE|O_CLOEXEC, 0777); 
	if(gpio_filp == NULL){
		file_close(uart_filp);
		return -EINVAL;	
	}

	// Postavljanje parametara UART interfejsa na DEFAULT!
	ret = default_UART_configuration();
	if(ret < 0){
		file_close(uart_filp);
		file_close(gpio_filp);
		return -EINVAL;	
	}
	//change_UART_configuration(setting);

    /* Dinamicko alociranje MAJOR i MINOR brojeva uredjaja koji se smjestaju u element strukture "dev_t"! */
	/* Brojeve mozemo saznati citanjem /proc/devices ili koriscenjem makroa MAJOR i MINOR! */
	if((ret = alloc_chrdev_region(&virt_tty_device, 0, 1, DEVICE_NAME)) < 0){
		pr_alert("Registring char device failed with %d\n", ret);
		file_close(uart_filp);
		file_close(gpio_filp);
		return ret;
	}

	pr_info("Assigned major number %d\n", MAJOR(virt_tty_device));
	
	/* Kreiranje klase uredjaja unutar /sys/class. U slucaju greske, vrsimo dealociranje zauzetih resursa! */
	/* Karakterski uredjaj! */
	if(IS_ERR(cls = class_create(THIS_MODULE, "chardev"))){
		pr_alert("Creating class char failed with %p\n", cls);
    	unregister_chrdev_region(virt_tty_device, 1);
		file_close(uart_filp);
		file_close(gpio_filp);
		return PTR_ERR(cls);
	}

	/* Kreiranje datoteke uredjaja tokom inicijalizacije, na osnovu dinamicki dodjeljenih MAJOR i MINOR brojeva! */
	/* Na ovaj nacin se postize da ne moramo rucno traziti brojeve driver-a, pa naknadno kreirati odgovarajuci fajl! */
	if(IS_ERR(dev_ret = device_create(cls, NULL, MKDEV(MAJOR(virt_tty_device), MINOR(virt_tty_device)), NULL, DEVICE_NAME))){
		pr_alert("Creating device %s failed with %p\n, ",DEVICE_NAME, dev_ret);
		class_destroy(cls);
    	unregister_chrdev_region(virt_tty_device, 1);
		file_close(uart_filp);
		file_close(gpio_filp);
		return PTR_ERR(dev_ret);
	}
	pr_info("Device created on /dev/%s\n",DEVICE_NAME);

	/* Automatsko povezivanje strukture cdev(element c_dev, koji se popunjava), sa operacijama koje uredjaj pruza! */
	cdev_init(&c_dev, &fops);
	c_dev.owner = THIS_MODULE;

	/* cdev_add, vrsi dodavanje uredjaja u sistem, te po njenom izvrsavanju, uredjaj je aktivan, i njegove operacije 	mogu da budu koriscene!  */
	/* U slucaju greske, uredjaj nije dodan u sistem, te je neophodno izvrsiti dealokaciju zauzetih resursa! */
	if((ret  = cdev_add(&c_dev, virt_tty_device, 1)) < 0 ){
		device_destroy(cls, MKDEV(MAJOR(virt_tty_device),MINOR(virt_tty_device)));
		class_destroy(cls);
    	unregister_chrdev_region(virt_tty_device, 1);
		file_close(uart_filp);
		file_close(gpio_filp);
		return ret;
	}

    return SUCCESS;
}

/*
 * File open funkcija.
 *  Parametri:
 *   path   - Putanja do fajla za otvaranje;
 *   flags  - Flegovi korisceni pri otvaranju;
 *   rights	- Prava koriscena prilikom otvaranja;
 *  Operacija:
 *   file_open funkcija, koristi se za dobijanje file deskriptora zeljene datoteke,
 *   odnosno njeno otvaranje iz kernel prostora.
 */
static struct file* file_open(const char* path, int flags, int rights){
	struct file* filp = NULL;
	int err = 0;
	
	filp = filp_open(path,flags,rights);

	if(IS_ERR(filp)){
		pr_info("file: %s could not be opened!\n",path);
		err = PTR_ERR(filp);
		return NULL;
	}else{
		pr_info("file: %s, with %d opened!\n", path, IS_ERR(filp));
	}

	return filp;
}

/*
 * File close funkcija.
 *  Parametri:
 *   filp   - File desktiptor fajla za zatvaranje;
 *  Operacija:
 *   file_close funkcija, koristi se za zatvaranje vec otvorenog fajla.
 */
static void file_close(struct file* filp){
	int ret;
	ret = filp_close(filp, NULL);
	if(ret){
		pr_info("Error code %d on closing!\n",ret);
	}
}

/*
* Funkcija koja se poziva prilikom uklanjanja modula!
*/
void __exit module_exit_clean(void)
{
	/* Zatvaranje otvorenih fajlova! */
	file_close(uart_filp);	
	file_close(gpio_filp);
	cdev_del(&c_dev); /* Uklanjanje uredjaja iz sistema! Uredjaj nije vise aktivan za koriscenje! */

    pr_info("Removing device module...\n");
	/* Uklanjanje odgovarajuce datoteke uredjaja! */
	device_destroy(cls, MKDEV(MAJOR(virt_tty_device),MINOR(virt_tty_device)));	
	/* Uklanjanje odgovarajuce klase uredjaja u /sys/class!*/
	class_destroy(cls);
    unregister_chrdev_region(virt_tty_device, 1); /* Dealokacija brojeva uredjaja! */
	pr_info("Driver unregistered!\n");
}


/*
* device_open, funkcija koja se poziva prilikom otvaranja datoteke datog modula.
*/
static int device_open(struct inode *inode, struct file *file)
{

	/* Po otvaranju datoteke datog modula, dolazi do inkrementovanje brojaca modula! */
	/* Na taj nacin se ne dozvoljava njegovo uklanjanje, ukoliko se koristi od strane nekog programa, ili drugog modula*/
	try_module_get(THIS_MODULE);	

	return SUCCESS;
}

/*
* device_release, poziva se prilikom zatvaranja datoteke datog modula.
*/
static int device_release(struct inode *inode, struct file *file)
{
	/* Po zatvaranju datoteke datog modula, dolazi do dekrementovanje brojaca modula! */
	module_put(THIS_MODULE);

	return 0;
}


/*
 * File read funkcija.
 *  Parametri:
 *   filp  - file deskriptor;
 *   buf   - buffer, iz kojeg ce user space funkcija (fread) citati;
 *   len   - broj bajtova za prenos;
 *   f_pos - pozicija sa koje se zapocinje citanje;
 *  Operacija:
 *   device_read funkcija vrsi prenos podataka iz driver buffera (device_buffer)
 *   u user space, koristeci funkciju copy_to_user.
 */
static ssize_t device_read(struct file *filp, char *buf, size_t len, loff_t *f_pos)
{
	int bytes_read = 0;
	char poruka[len];

	pr_alert("Reading using %s.\n", filp->f_path.dentry->d_name.name);
	pr_info("Postavljanje GPIO pina %d, u stanje PRIHVATANJA podataka!\n", BCM_PIN_DE);

	// Upis logicke NULE!
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, LOW);
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, LOW);

	change_UART_configuration(setting); 	
	
	bytes_read =  kernel_read(uart_filp, poruka, len, &uart_filp->f_pos);
	if(copy_to_user((char __user *)buf, poruka, sizeof(poruka))){
		pr_alert("Greska prilikom upisa u USER space-a! \n");
		return -EINVAL;		
	}
	//bytes_read =  kernel_read(uart_filp, buf, len, &uart_filp->f_pos);

	// Upis logicke JEDINICE!
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, HIGH);
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, HIGH);
	return bytes_read;
}



/*
 * File write funkcija
 *  Parametri:
 *   filp   - file structure;
 *   buf    - buffer u koji user space funkcija (fwrite) upisuje sadrzaj;
 *   len    - broj bajtova za prenos;
 *   f_pos  - pozicija na kojoj se zapocinje upis podataka;
 *
 *  Operacija:
 *   funkcija copy_from_user vrsi prenos podataka iz user space u kernel space.
 */
static ssize_t device_write(struct file *filp, const char *buf, size_t len, loff_t *f_pos)
{
    int bytes_write = 0;
	char poruka[len];
	unsigned long sleep;	 //Izrazeno vrijeme spavanja u us!

	// Upis logicke JEDINICE!
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, HIGH);
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, HIGH);

	pr_alert("Writing using %s.\n", filp->f_path.dentry->d_name.name);
	pr_info("Postavljanje GPIO pina %d, u stanje SLANJA podataka!\n",BCM_PIN_DE);

	// Promjena parametara UART interfejsa!
	change_UART_configuration(setting); 

	// Upis u UART FILE!
	if(copy_from_user(poruka, (char __user *)buf, sizeof(poruka))){
		pr_alert("Greska prilikom citanja iz USER space-a! \n");
		return -EINVAL;
	}
	bytes_write =  kernel_write(uart_filp, poruka, len, &uart_filp->f_pos);

	// Upis logicke NULE!
	sleep = time_of_data_transfer();
	pr_info("Vrijeme transfera: %lu\n", len*sleep);
	usleep_range(len*sleep, len*sleep);
	//udelay(len*sleep);
	//ndelay((len*sleep)*1000-10);
	//usleep_range(8325,8329);

	gpio_set_configure(PROMJENA_VRIJEDNOSTI, LOW);	
	gpio_set_configure(PROMJENA_VRIJEDNOSTI, LOW);
	return bytes_write;
}


/*
 * IOCTL funkcija
 *  Parametri:
 *   filp   - file structure;
 *   cmd	- komanda kojom definisemo operaciju(SET/GET) serijskog prenosa, ili podesavanje odgovarajuceg pina.
 *   arg	- struktura koja definise parametre prenosa, ili redni broj pina za kontrolu RS-485 transivera.
 *
 *  Operacija:
 *   podesavanje parametara serijskog prenosa, kao i podesavanje rednog broja pina za kontrolu RS-485 transivera.
 */
static long virtual_tty_ioctl(struct file *file, unsigned int cmd, unsigned long arg){

	switch(cmd){
		case TCGETS:			/* Citanje parametara serijskog prenosa! */
			if((struct termios *)arg == NULL){
				pr_info("Argument is NULL! Termios expected! \n");
				return -EINVAL;
			}
			if(copy_to_user((struct termios __user *)arg, &setting, sizeof(struct termios)))
				return -EINVAL;
			return SUCCESS;

		case TCSETS:			/* Postavljanje parametara serijskog prenosa! */
			if((struct termios *)arg == NULL){
				pr_info("Argument is NULL! Termios expected! \n");
				return -EINVAL;
			}
			if(copy_from_user(&setting, (struct termios __user *)arg, sizeof(struct termios)))
				return -EINVAL;
			return SUCCESS;

		case WR_BCM_PIN_DE:		/* Promjena pina za koriscenje! */
			if((int32_t*)arg == NULL){
				pr_info("Argument is NULL! int32_t expected! \n");
				return -EINVAL;
			}
			if(copy_from_user(&BCM_PIN_DE, (int32_t __user *)arg, sizeof(int32_t)))
				return -EINVAL;

			pr_info("Postavljanje pina %d, kao BCM_PIN_DE!\n",BCM_PIN_DE);
			gpio_set_configure(PROMJENA_PINA, BCM_PIN_DE);
			gpio_set_configure(PROMJENA_PINA, BCM_PIN_DE);
			return SUCCESS;

		case RD_BCM_PIN_DE:		/* Citanje rednog broja trenutno koriscenog pina! */
            if(copy_to_user((int32_t*) arg, &BCM_PIN_DE, sizeof(BCM_PIN_DE)))
				return -EINVAL;
			return SUCCESS;

		case WR_SERIAL_PARAM: /* Prenos parametara serijskog prenosa, iz user space-a! */
			if((int32_t*)arg == NULL){
				pr_info("Argument is NULL! int32_t expected! \n");
				return -EINVAL;
			}
			if(copy_from_user(&uart_konf, (int32_t __user *)arg, sizeof(uart_konf)))
				return -EINVAL;
			return SUCCESS;

		default:
			pr_info("DEFAULT! \n");
			return -ENOTTY;
	}	
}


/*
 * Promjena parametara serijskog prenosa funkcija,
 *  Parametri:
 *   settings - parametri serijskog prenosa.
 *
 *  Operacija:
 *   funkcija change_UART_configuration vrsi promjenu parametara serijskog prenosa.
 */
static void change_UART_configuration(const struct termios settings){
	mm_segment_t oldfs;
	int ret;

	if(uart_filp && uart_filp->f_op && uart_filp->f_op->unlocked_ioctl){
		oldfs = get_fs();
		set_fs(get_ds());
		ret = uart_filp->f_op->unlocked_ioctl(uart_filp, TCSETS, (unsigned long)&settings);
		set_fs(oldfs);
		if(ret){
			pr_info("TCSETS returned error code %d\n", ret);
			return;
		}	
	}else{
		pr_info("UART configuration cannot be changed!!!\n");	
	}
}
/*
* Postavljanje parametara serijskog prenosa na unaprijed definisane vrijednosti(Defaultne vrijednosti).
*
*/
static int default_UART_configuration(){
	struct termios term;

	term.c_cflag = B9600 | CLOCAL | CREAD;

	term.c_cflag &= ~PARENB; 	/* NO parity! */
	term.c_cflag &= ~CSTOPB;	/* 1 stop bit! */
	term.c_cflag &= ~CSIZE;
	term.c_cflag |= CS8;		/* 8 bits! */

	term.c_lflag &= ~(ICANON | ECHOE | ISIG);		/* Canonical mode! */
	term.c_lflag |= ECHO;
	term.c_oflag &= ~OPOST;		/* RAW output*/

	term.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);

	term.c_cc[VTIME] = 5;
	term.c_cc[VMIN]  = 0;

	change_UART_configuration(term);

	return SUCCESS;
}

/*
* Upravljanje i kontrola odgovarajucim BCM pinom.
*
*/
static void gpio_set_configure(int cmd, int value){
	mm_segment_t oldfs;
	int ret = 0;

	if(gpio_filp && gpio_filp->f_op && gpio_filp->f_op->unlocked_ioctl){
		oldfs = get_fs();
		set_fs(get_ds());

		if(cmd == PROMJENA_PINA)
			ret = gpio_filp->f_op->unlocked_ioctl(gpio_filp, CHANGE_GPIO_PIN, (unsigned long)&value);
		else if(cmd == PROMJENA_VRIJEDNOSTI)
			ret = gpio_filp->f_op->unlocked_ioctl(gpio_filp, CHANGE_VALUE_GPIO, (unsigned long)&value);

		set_fs(oldfs);
		if(ret){
			pr_info("GPIO returned error code %d\n", ret);
			return;
		}	
	}else{
		pr_info("GPIO set/configuration cannot be executed!!!\n");	
	}
}

/* Definisemo vrijeme neophodno za prenos podataka do relejnih izlaza, preko RS-485! */
static unsigned long time_of_data_transfer(){  //Izrazeno vrijeme u us!
	return (unsigned long)((uart_konf.data_bit + uart_konf.stop_bit + (uart_konf.parity == 'N' ? 0 : 1) +1)*1000000/uart_konf.baud);
}


/* Informacije o modulu: */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dejan Milojica <dejan.milojica@student.etf.unibl.org>");
MODULE_DESCRIPTION("Prosireni/Nadogradjeni UART driver, sa mogucnoscu kontrole GPIO pina koji upravlja radom RS-485 transivera!");
MODULE_VERSION("3.1");


