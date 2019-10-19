#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>    //  인터럽트 해더 선언
#include <linux/irq.h>          //  인터럽트 헤더 선언
#include <linux/types.h>
#include <linux/time.h>

#define SONIC_MAJOR 268         //  소니 드라이버 메이저 번호 설정
#define SONIC_NAME "sonic_name" //  드라이버 이름
 
#define SONIC_TRIG IMX_GPIO_NR(2,0) //	External Connector GPIO2_0 핀 -> SONIC_TRIG 로 선언
#define SONIC_ECHO IMX_GPIO_NR(2,1) //	External Connector GPIO2_1 핀 -> SONIC_ECHO 로 선언

static int distance = 0;
static int sonic_port_usage = 0; 
struct timeval after, before;  // 타임 변수 계산할 변수 , after -> 에코핀에 인터럽트 걸릴때, before -> 트리거 신호 나갈때
u32 irq; //인터럽트 등록을 위한 변수

int register_itrp(void);
void gpio_init(void);
void output_sonicburst(void);

ssize_t sonic_read(struct file *inode, char *gdata, size_t length, loff_t *off_what);    
int sonic_open(struct inode *minode, struct file *mfile);
int sonic_release(struct inode *minode, struct file *mfile);
////////////////////////////////////////////////////////////////////////////////////
struct file_operations sonic_fops=
{
        .owner=          THIS_MODULE,
        .read=          sonic_read,
        .open=           sonic_open,
        .release=        sonic_release,
}; 
/////////////////////////////////////////////////////////////////////////
//에코핀에 신호가 들어와 인터럽트가 걸릴 때 실행되는 인터럽트 서비스 루틴, 인터럽트가 걸릴때 실행되는 함수들
static irqreturn_t irq_handler (int irq, void *dev_id)
{
       do_gettimeofday( &after); //인터럽트가 발생 시 그때의 시간을 저장하는 변수
        
        printk("%ldcm\n", (after.tv_usec - before.tv_usec) / 58); 
        
        distance = (after.tv_usec - before.tv_usec) / 58; //두 차를 58로 나눈 것이 cm

       return IRQ_HANDLED;
}
/*irqreturn_t int_interrupt(int irq, void *dev_id, struct pt_regs *regs)
{
	//int distance; //application 영역에서 사용할 거리 저장 변수
	
	do_gettimeofday( &after); //인터럽트가 발생 시 그때의 시간을 저장하는 변수
	
	printk("%ldcm\n", (after.tv_usec - before.tv_usec) / 58); 
	
	distance = (after.tv_usec - before.tv_usec) / 58; //두 차를 58로 나눈 것이 cm
	
	return IRQ_HANDLED;
}*/
/////////////////////////////////////////////////////////////////////////
//인터럽트를 등록하는 함수 사용할 ,인터럽트핀의 정보를 가져와서 인터럽트를 등록하는 함수 , 쓰기전 등록을 해주는것, 옵션 설정도 해주고, 설정 초기화 비슷
int register_itrp(void)
{
	irq = gpio_to_irq(SONIC_ECHO);
	
	irq_set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);    //인터럽트 발생 설정, falling edge 일때
	
	if(request_irq(irq, irq_handler, IRQF_DISABLED, NULL, NULL))
	{
		return -EINVAL;	
	}
 /*
	if(request_irq(gpio_to_irq(SONIC_ECHO), int_interrupt, IRQF_DISABLED, NULL, NULL))
	{
		return -EINVAL;	
	}
  */
	//gpio_init에서는 gpio_request(SONIC_ECHO)만 선언 gpio_to_irq 선언 하지말고 위처럼 바로 시도
	return 0;	

}


/////////////////////////////////////////////////////////////////////////
void gpio_init(void)
{
	gpio_request(SONIC_TRIG,NULL);
	gpio_direction_output(SONIC_TRIG, 0); 
	
	gpio_request(SONIC_ECHO,NULL); ///???
	//gpio_to_irq(SONIC_ECHO);  ///???
}
/////////////////////////////////////////////////////////////////////////
//SRF04 초음파 센서 트리거 핀에서 트리거 신호 발사 조건 : HIGH 10usec 딜레이
void output_sonicburst(void)
{
	gpio_set_value(SONIC_TRIG, 1);
	udelay(10);
	gpio_set_value(SONIC_TRIG, 0);
	
	do_gettimeofday( &before);     //트리거핀에서 신호 발사하고 바로 그 시점에서의 시간 체킹 before에 저장
}
/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
int sonic_open(struct inode *minode, struct file *mfile)
{
	if(sonic_port_usage != 0)
		return -EBUSY;
	
	sonic_port_usage = 1;
	
//	gpio_init();
  //      register_itrp();

     	//output_sonicburst();
     	//mdelay(100);
	//printk(KERN_WARNING "distance = %dcm\n", distance);

	return 0;
}
ssize_t sonic_read(struct file *inode, char *gdata, size_t length, loff_t *off_what) //retval = read(dev, *data, 4);
{
	//gpio_init();
	//register_itrp();
	
	output_sonicburst();
	mdelay(100);
//	printk(KERN_WARNING "distance = %dcm\n", distance);	

	if(copy_to_user(gdata, &distance, 1))
		return -EFAULT;

	//printk(KERN_WARNING "distance = %dcm\n", distance);
	//copy_to_user(gdata, &distance, 10);
	return length;	
}
int sonic_release(struct inode *minode, struct file *mfile)
{
	sonic_port_usage = 0;
	
	return 0;
}
/////////////////////////////////////////////////////////////////////////
int __init sonic_init(void)
{
	int result;
	result = register_chrdev(SONIC_MAJOR, SONIC_NAME, &sonic_fops);
	if(result<0)
	{
		printk(KERN_WARNING "can't get any major\n");
		return result;
	}
	
	gpio_init();
	register_itrp();
	
//	while( )
//	{	
//		output_sonicburst();
//		mdelay(100);	
//		printk(KERN_WARNING "distance = %dcm\n", distance);
//
//	}
	return 0;
}
/////////////////////////////////////////////////////////////////////////
void  __exit sonic_exit(void)
{
	unregister_chrdev(SONIC_MAJOR, SONIC_NAME);
	
	free_irq(irq, NULL);  //인터럽트 해제 함수
}

/////////////////////////////////////////////////////////////////////////

module_init(sonic_init);
module_exit(sonic_exit);

MODULE_LICENSE("GPL");


