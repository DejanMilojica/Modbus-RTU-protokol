/*
 * GPIO extended driver(Modbus driver)
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

#include <linux/module.h>   			/* Needed by all modules.  			*/
#include <linux/kernel.h>   			/* Needed for KERN_INFO.   			*/
#include <linux/init.h>     			/* Needed for the macros.  			*/
#include <linux/moduleparam.h>			/* Needed for driver param. 		*/
#include <linux/fs.h>  					/* Needed for filp_open/close!      */
#include <linux/device.h>
#include <asm/processor.h>
#include <linux/cdev.h>
#include <linux/tty.h>
#include <linux/gpio.h>					/* Needed for gpio controlling!		*/
#include <linux/gpio/driver.h>
#include <linux/uaccess.h>              /* Needed for copy_to/from_user()!	*/
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/errno.h>


/*************** Driver Functions **********************/
void gpio_driver_exit(void);
int gpio_driver_init(void);
static int gpio_driver_open(struct inode *, struct file *);
static int gpio_driver_release(struct inode *, struct file *);
static long gpio_ioctl(struct file *, unsigned int, unsigned long);

#define DEVICE_NAME  "gpio_driver" 		

#define CHANGE_VALUE_GPIO _IOW('a','a',int32_t*)	 /* Promjena vrijednosti pina! */
#define CHANGE_GPIO_PIN   _IOW('a','b',int32_t*)	 /* Promjena pina kao BCM_PIN_DE-a! */	

#define SUCCESS 0		/* Uspjesno izvrseno! */
#define UN_SUCCESS -1	/* Neuspjesno izvrseno! */

enum pin_vrijednost {LOW, HIGH};  /* Vrijednosti upisane u odgovarajuci pin! */

/*
 * Struktura sadrzi pokazivace na funkcije definisane za dati drajver,
 * koje izvrsavaju razlicite operacije nad uredjajem!
*/
struct file_operations gpio_driver_fops =
{
	owner   		:   THIS_MODULE,
    open    		:   gpio_driver_open,
    release 		:   gpio_driver_release,
	unlocked_ioctl  :   gpio_ioctl
};

/* Deklaracija init i exit funkcija. */
module_init(gpio_driver_init);
module_exit(gpio_driver_exit);


/* Globalne varijable driver-a. */
static dev_t gpio_device;
static struct cdev c_dev;
static struct class *cls;

static int BCM_PIN_DE = 22;
static int gpio_value = 0;

/*
 * Inicijalizacija:
 */
int gpio_driver_init(void)
{
	int ret;
	struct device *dev_ret;

	pr_info("Device name %s!\n",DEVICE_NAME);
    pr_info("Inserting device module\n");

	gpio_request(BCM_PIN_DE, "BCM_PIN_DE");		// Zahtjevanje pristupa pinu!
	gpio_request(BCM_PIN_DE, "BCM_PIN_DE");
	
	gpio_export(BCM_PIN_DE, 1);					// Eksportovanje pina u /sys/class/gpio!
	gpio_export(BCM_PIN_DE, 1);

	gpio_direction_output(BCM_PIN_DE, 1);		// Postavljanje pina kao izlaznog, uz pocetnu vrijednost!
	gpio_direction_output(BCM_PIN_DE, 1);
	pr_info("File otvoren, gpio PIN: %d\n", BCM_PIN_DE);

    /* Alokacija glavnog i sporednog broja uredjaja. */
	if((ret = alloc_chrdev_region(&gpio_device, 0, 1, DEVICE_NAME)) < 0){
		pr_alert("Registring char device failed with %d\n", ret);
		return ret;
	}

	pr_info("Assigned major number %d\n", MAJOR(gpio_device));
	
	/* Kreiranje klase uredjaja. */
	if(IS_ERR(cls = class_create(THIS_MODULE, "chardev_gpio"))){
		pr_alert("Creating class char failed with %p\n", cls);
    	unregister_chrdev_region(gpio_device, 1);
		return PTR_ERR(cls);
	}

	/* Kreirenje datoteke uredjaja. */
	if(IS_ERR(dev_ret = device_create(cls, NULL, MKDEV(MAJOR(gpio_device), MINOR(gpio_device)), NULL, DEVICE_NAME))){
		pr_alert("Creating device %s failed with %p\n, ",DEVICE_NAME, dev_ret);
		class_destroy(cls);
    	unregister_chrdev_region(gpio_device, 1);
		return PTR_ERR(dev_ret);
	}
	pr_info("Device created on /dev/%s\n",DEVICE_NAME);

	/* Povezivanje uredjaja sa definisanim operacijama. */
	cdev_init(&c_dev, &gpio_driver_fops);
	c_dev.owner = THIS_MODULE;

	/* Aktiviranje uredjaja za funkcjonisanje! */
	if((ret  = cdev_add(&c_dev, gpio_device, 1)) < 0 ){
		device_destroy(cls, MKDEV(MAJOR(gpio_device),MINOR(gpio_device)));
		class_destroy(cls);
    	unregister_chrdev_region(gpio_device, 1);
		return ret;
	}

    return SUCCESS;

}

/*
 * Uklanjanje uredjaja:
 */
void gpio_driver_exit(void)
{
	gpio_direction_input(BCM_PIN_DE);
	gpio_unexport(BCM_PIN_DE);
	gpio_free(BCM_PIN_DE);
  	
	cdev_del(&c_dev);

    pr_info("Removing GPIO device module...\n");
	device_destroy(cls, MKDEV(MAJOR(gpio_device),MINOR(gpio_device)));	
	class_destroy(cls);
    unregister_chrdev_region(gpio_device, 1);
	pr_info("GPIO Driver unregistered!\n");
}


/* File open funkcija. */
static int gpio_driver_open(struct inode *inode, struct file *filp)
{
	try_module_get(THIS_MODULE);

    /* Success. */
    return SUCCESS;
}

/* File close funkcija. */
static int gpio_driver_release(struct inode *inode, struct file *filp)
{
	module_put(THIS_MODULE);

    /* Success. */
	return SUCCESS;
}


/*
 * gpio_ioctl funkcija
 *  Parametri:
 *   filp   - file structure;
 *   cmd	- komanda kojom definisemo operaciju.
 *   arg	- podatak koji sadrzi redni broj pina, ili vrijednost za upis u pin!
 *
 *  Operacija:
 *   podesavanje rednog broja pina za kontrolu RS-485 transivera / vrijednosti pina.
 */
static long gpio_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){

	switch(cmd){
		case CHANGE_VALUE_GPIO:		// Promjena vrijednosti pin-a!
			if((int32_t*)arg == NULL){
				pr_info("Argument is NULL! int32_t expected! \n");
				return -EINVAL;
			}
			if(copy_from_user(&gpio_value, (int32_t __user *)arg, sizeof(int32_t))){
				pr_alert("Greska prilikom citanja iz USER space-a! \n");
				return -EINVAL;
			}

			/*if(gpio_value == 0)
				gpio_direction_output(BCM_PIN_DE, 0);
			else
				gpio_direction_output(BCM_PIN_DE, 1);*/
			gpio_direction_output(BCM_PIN_DE, (gpio_value ? 1 : 0));

			pr_alert("Upisana vrijednost %d! \n", gpio_value);			
			return SUCCESS;

		case CHANGE_GPIO_PIN:		// Promjena rednog broja pin-a!
			if((int32_t*)arg == NULL){
				pr_info("Argument is NULL! int32_t expected! \n");
				return -EINVAL;
			}

			/* Neophodno je da oslobodimo prethodno zauzeti PIN, te zauzmemo novi, sa rednim brojem def. f-jom! */
			gpio_unexport(BCM_PIN_DE);
			gpio_free(BCM_PIN_DE);

			if(copy_from_user(&BCM_PIN_DE, (int32_t __user *)arg, sizeof(int32_t))){
				pr_alert("Greska prilikom citanja iz USER space-a! \n");
				return -EINVAL;
			}
			pr_info("PIN promjenjen, nova vrijednost: %d\n", BCM_PIN_DE);

			gpio_request(BCM_PIN_DE, "BCM_PIN_DE");
			gpio_request(BCM_PIN_DE, "BCM_PIN_DE");

			gpio_export(BCM_PIN_DE, 1);
			gpio_export(BCM_PIN_DE, 1);

			gpio_direction_output(BCM_PIN_DE, 1);
			gpio_direction_output(BCM_PIN_DE, 1);
			
			return SUCCESS;

		default:
			pr_info("DEFAULT! \n");
			return -ENOTTY;
	}
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dejan Milojica <dejan.milojica@student.etf.unibl.org>");
MODULE_DESCRIPTION("Prosireni/Nadogradjeni GPIO modul, za konfiguraciju GPIO pina koji upravlja radom RS-485 transivera!");
MODULE_VERSION("3.1");
