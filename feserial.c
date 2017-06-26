#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/serial_reg.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>

#define SERIAL_BUFSIZE 1024

struct feserial_dev
{
  struct miscdevice miscdev;
  void __iomem *regs;

  int irq;

  char serial_buf[SERIAL_BUFSIZE];
  int serial_buf_rd;
  int serial_buf_wr;
  wait_queue_head_t queue;
};

#ifdef CONFIG_OF
static const struct of_device_id fe_serial_of_match[] = {
  {.compatible = "free-electrons,serial",},
  {}
};

MODULE_DEVICE_TABLE (of, fe_serial_of_match);
#endif

static unsigned int
reg_read (struct feserial_dev *fedev, int offset)
{
  pr_info ("Trying to read at %p\n", (unsigned int *) fedev->regs + offset);
  return readw ((unsigned int *) fedev->regs + offset);
}

static void
reg_write (struct feserial_dev *fedev, int val, int offset)
{
  pr_info ("Trying to write at %p\n", (unsigned int *) fedev->regs + offset);
  writew (val, (unsigned int *) fedev->regs + offset);
}

static void
uart_line_init (struct platform_device *pdev, struct feserial_dev *fedev)
{
  unsigned int baud_divisor, uartclk;

  of_property_read_u32 (pdev->dev.of_node, "clock-frequency", &uartclk);

  baud_divisor = uartclk / 16 / 115200;

  reg_write (fedev, 0x07, UART_OMAP_MDR1);
  reg_write (fedev, 0x00, UART_LCR);
  reg_write (fedev, UART_LCR_DLAB, UART_LCR);
  reg_write (fedev, baud_divisor & 0xff, UART_DLL);
  reg_write (fedev, (baud_divisor >> 8) & 0xff, UART_DLM);
  reg_write (fedev, UART_LCR_WLEN8, UART_LCR);
  reg_write (fedev, UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT, UART_FCR);
  reg_write (fedev, 0x00, UART_OMAP_MDR1);
}

static void
send_byte (struct feserial_dev *dev, char ch[], size_t sz)
{
  int rval, i;

  while (((rval = reg_read (dev, UART_LSR)) & UART_LSR_THRE) == 0)
    cpu_relax ();

  for (i = 0; i < sz; ++i)
    reg_write (dev, ch[i], UART_TX);
}

ssize_t
feserial_read (struct file *f, char __user * buf, size_t sz, loff_t * off)
{
  struct feserial_dev *fedev;
  unsigned long result;

  fedev = container_of (f->private_data, struct feserial_dev, miscdev);

  wait_event_interruptible (fedev->queue,
			    fedev->serial_buf_rd != fedev->serial_buf_wr);

  result = copy_to_user (buf, &(fedev->serial_buf[fedev->serial_buf_rd]), 1);
  fedev->serial_buf_rd = (fedev->serial_buf_rd + 1) % SERIAL_BUFSIZE;

  *off += 1;

  return 1;
}

ssize_t
feserial_write (struct file * f, const char __user * buf,
		size_t sz, loff_t * off)
{
  char buffer[SERIAL_BUFSIZE];
  unsigned long result;

  struct feserial_dev *fedev;
  fedev = container_of (f->private_data, struct feserial_dev, miscdev);

  result = copy_from_user (buffer, buf, sz);

  if (result == 0)
    {
      send_byte (fedev, buffer, sz);

      *off += sz;
      return sz;
    }

  return -EINVAL;
}

static struct file_operations myfops = {
  .read = feserial_read,
  .write = feserial_write,
};

irqreturn_t
myint_handler (int irq, void *dev)
{
  char ch;
  struct feserial_dev *fedev;

  fedev = (struct feserial_dev *) dev;
  pr_info ("within handler.\n");
  ch = reg_read (fedev, UART_RX);
  pr_info ("Received %c\n", ch);

  fedev->serial_buf[fedev->serial_buf_wr] = ch;
  fedev->serial_buf_wr = (fedev->serial_buf_wr + 1) % SERIAL_BUFSIZE;

  wake_up (&fedev->queue);

  return IRQ_HANDLED;

}

static int
feserial_probe (struct platform_device *pdev)
{
  int retval = 0;
  struct resource *res;
  struct feserial_dev *fedev;

  pr_info ("Called feserial_probe\n");

  res = platform_get_resource (pdev, IORESOURCE_MEM, 0);
  if (res == NULL)
    {
      pr_info ("NULL resource\n");
      retval = -1;
      goto jump;
    }

  pr_info ("Start address: %x\n", res->start);

  fedev = devm_kzalloc (&pdev->dev, sizeof (struct feserial_dev), GFP_KERNEL);
  if (fedev == NULL)
    {
      retval = -ENOMEM;
      goto jump;
    }

  fedev->regs = devm_ioremap_resource (&pdev->dev, res);
  if (!fedev->regs)
    {
      dev_err (&pdev->dev, "Cannot remap registers\n");
      devm_kfree (&pdev->dev, fedev);
      retval = -ENOMEM;
      goto jump;
    }

  pm_runtime_enable (&pdev->dev);
  pm_runtime_get_sync (&pdev->dev);

  uart_line_init (pdev, fedev);

  {
    platform_set_drvdata (pdev, fedev);

    fedev->miscdev.minor = MISC_DYNAMIC_MINOR;
    fedev->miscdev.fops = &myfops;

    fedev->miscdev.name =
      devm_kasprintf (&pdev->dev, GFP_KERNEL, "feserial-%x", res->start);
    misc_register (&(fedev->miscdev));
  }

  {
    fedev->irq = platform_get_irq (pdev, 0);
    devm_request_irq (&pdev->dev, fedev->irq, &myint_handler, 0, "feserial",
		      fedev);
    reg_write (fedev, UART_IER_RDI, UART_IER);
  }

  {
    fedev->serial_buf_rd = 0;
    fedev->serial_buf_wr = 0;

    init_waitqueue_head (&fedev->queue);
  }

jump:
  return retval;
}

static int
feserial_remove (struct platform_device *pdev)
{
  struct feserial_dev *mydev;
  struct device dd;

  pr_info ("Called feserial_remove\n");

  pm_runtime_disable (&pdev->dev);

  dd = pdev->dev;
  mydev = (struct feserial_dev *) (dd.driver_data);
  misc_deregister (&mydev->miscdev);

  return 0;
}

static struct platform_driver feserial_driver = {
  .driver = {
	     .name = "feserial",
	     .owner = THIS_MODULE,
	     .of_match_table = of_match_ptr (fe_serial_of_match),
	     },
  .probe = feserial_probe,
  .remove = feserial_remove,
};

#undef SERIAL_BUFSIZE

module_platform_driver (feserial_driver);

MODULE_LICENSE ("GPL");
MODULE_AUTHOR ("Jongmin Kim <jmkim@pukyong.ac.kr>");
MODULE_DESCRIPTION ("BeagleBone Black UART driver");
