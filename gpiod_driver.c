#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/sysfs.h>
/* YOU WILL NEED OTHER HEADER FILES */

/* YOU WILL HAVE TO DECLARE SOME VARIABLES HERE */
unsigned int irq_number; // Variable to store the IRQ number
struct gpio_desc* proj3_encoder; // GPIO descriptor pointer for project 3 encoder
static ktime_t last_interrupt_time; // Stores the timestamp of the last interrupt
static int isFirstInterrupt = 1; // Flag to check the first interrupt
static int elapsed_ns = 0;
module_param(elapsed_ns, int, S_IRUGO);

/* ADD THE INTERRUPT SERVICE ROUTINE HERE */

static struct kobj_attribute last_interrupt_attr = {
    .attr = {
        .name = "last_interrupt_time",
        .mode = S_IRUGO, // Read-only attribute
    },
    .show = NULL, // Defined later for the read operation
};

// Sysfs entry
static struct attribute *attrs[] = {
    &last_interrupt_attr.attr,
    NULL,
};

static struct attribute_group attr_group = {
    .attrs = attrs,
};

static struct kobject *proj3_kobj; // Pointer to the kernel object

static ssize_t last_interrupt_time_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
    return sprintf(buf, "%lld\n", last_interrupt_time); // Displaying last_interrupt_time in nanoseconds
}

static void read_encoder(void)
{
    ktime_t current_time = ktime_get();
    uint64_t time_diff_ns = 0;

    if (!isFirstInterrupt) {
        time_diff_ns = ktime_to_ns(ktime_sub(current_time, last_interrupt_time));
        // Calculate the time difference between the last two interrupts in nanoseconds
        printk("Time between wheel rotations: %llu ns\n", time_diff_ns);
        elapsed_ns = time_diff_ns;
    } else {
        printk("First interrupt received.\n");
        isFirstInterrupt = 0;
    }

    last_interrupt_time = current_time; // Update last_interrupt_time with the current time
}

static irq_handler_t gpiod_irq_handler(unsigned int irq, void *dev_id) {
    printk("gpiod_irq: Wheel was turned, interrupt was triggered and ISR was called!\n");
    read_encoder(); // Invoking function to read encoder details
    return (irq_handler_t) IRQ_HANDLED; 
}

// probe function
static int encoder_probe(struct platform_device *pdev)
{
    
    proj3_encoder = devm_gpiod_get(&pdev->dev, "proj3encoder", GPIOD_IN);
    gpiod_set_debounce(proj3_encoder, 200000);
    irq_number = gpiod_to_irq(proj3_encoder);
    printk("IRQ number: %d\n", irq_number);
    if (IS_ERR(proj3_encoder)) {
        printk("Failed to get proj3encoder GPIO\n");
        return PTR_ERR(proj3_encoder);
    }

    irq_number = gpiod_to_irq(proj3_encoder);
    if (irq_number < 0) {
        printk("Failed to get IRQ for proj3encoder\n");
        return irq_number;
    }
    if(request_irq(irq_number, (irq_handler_t) gpiod_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0){
		printk("Error!\nCan not request interrupt nr.: %d\n", irq_number);
		return -1;
    }

    // Creating a sysfs entry
    proj3_kobj = kobject_create_and_add("proj3_sysfs", kernel_kobj);
    if (!proj3_kobj) {
        return -ENOMEM;
    }

    // Creating the sysfs attribute file for last_interrupt_time
    last_interrupt_attr.show = last_interrupt_time_show;
    if (sysfs_create_group(proj3_kobj, &attr_group) != 0) {
        kobject_put(proj3_kobj);
        return -ENOMEM;
    }

    printk("probed\n");
    return 0;
}

// remove function
static int encoder_remove(struct platform_device *pdev)
{
    kobject_put(proj3_kobj); // Release the sysfs entry
    free_irq(irq_number, NULL); // Free the IRQ
    printk("freed\n");
    return 0;
}

static struct of_device_id matchy_match[] = {
    {.compatible = "proj3"},
    {/* leave alone - keep this here (end node) */},
};

// MODULE_DEVICE_TABLE_OF(of, matchy_match);

// platform driver object
static struct platform_driver adam_driver = {
    .probe   = encoder_probe,
    .remove  = encoder_remove,
    .driver  = {
           .name  = "The Rock: this name doesn't even matter",
           .owner = THIS_MODULE,
           .of_match_table = matchy_match,
    },
};

module_platform_driver(adam_driver);

MODULE_DESCRIPTION("424\'s finest");
MODULE_AUTHOR("Daniel Li");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
