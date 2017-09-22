#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include "synopGMAC_Host.h"
#include "synopGMAC_network_interface.h"

static int init_one_platform(struct platform_device *pdev)
{
	synopGMACPciNetworkAdapter *synopGMACadapter;
	u8 *synopGMACMappedAddr;
	u32 synopGMACMappedAddrSize;
	u32 irq;
	struct resource *res;


	irq = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	synopGMACMappedAddrSize=res->end-res->start+1;
	synopGMACMappedAddr=ioremap_nocache(res->start,synopGMACMappedAddrSize);
	printk("<0>init_one_platform %p\n",synopGMACMappedAddr);

	if((synopGMACadapter=synopGMAC_init_network_interface(&pdev->dev,synopGMACMappedAddr,synopGMACMappedAddrSize,irq)))
		platform_set_drvdata(pdev,synopGMACadapter);
	return 0;
}

static int remove_one_platform(struct platform_device *pdev)
{
	synopGMACPciNetworkAdapter *adapter=platform_get_drvdata(pdev);
	struct net_device *netdev=adapter->synopGMACnetdev;

	/* Do the reverse of what probe does */ 
	if (adapter->synopGMACMappedAddr)
	{
		TR0 ("Releaseing synopGMACMappedAddr 0x%p whose size is %d\n",  adapter->synopGMACMappedAddr, adapter->synopGMACMappedAddrSize);

		/*release the memory region which we locked using request_mem_region */ 
		release_mem_region ((resource_size_t) adapter->synopGMACMappedAddr, adapter->synopGMACMappedAddrSize);
	}
	TR0 ("Unmapping synopGMACMappedAddr =0x%p\n",  adapter->synopGMACMappedAddr);
	iounmap (adapter->synopGMACMappedAddr);
	if(netdev) {
		unregister_netdev(netdev);
		free_netdev(netdev);
		platform_set_drvdata(pdev, NULL);
	}

	return 0;
}

#ifdef CONFIG_PM

extern void synopGMAC_linux_powerdown_mac(synopGMACdevice *gmacdev);
extern void synopGMAC_linux_powerup_mac(synopGMACdevice *gmacdev);
extern s32 synopGMAC_linux_open(struct net_device *netdev);
extern s32 synopGMAC_linux_close(struct net_device *netdev);
extern void prom_printf(char *fmt, ...);
#define MYDBG prom_printf("%d\n", __LINE__);

static int loongson_gmac_suspend(struct platform_device *pdev, pm_message_t state)
{
	synopGMACPciNetworkAdapter *adapter=platform_get_drvdata(pdev);
	synopGMACdevice * gmacdev = adapter->synopGMACdev;
	struct net_device *ndev = adapter->synopGMACnetdev;

	if (ndev) {
		if (!netif_running(ndev))
			return 0;

		netif_device_detach(ndev);

		synopGMAC_linux_close(ndev);			
		//dev_close(ndev);			
		//synopGMAC_linux_powerdown_mac(gmacdev);			
	}

	//printk("%s",__FUNCTION__);
	return 0;
}
static int loongson_gmac_resume(struct platform_device *pdev)
{
	synopGMACPciNetworkAdapter *adapter=platform_get_drvdata(pdev);
	synopGMACdevice * gmacdev = adapter->synopGMACdev;
	struct net_device *ndev = adapter->synopGMACnetdev;


	if (ndev) {
		if (!netif_running(ndev)) 
			return 0;

		//synopGMAC_power_down_disable(gmacdev);
		//synopGMAC_linux_powerup_mac(gmacdev);
		netif_device_attach(ndev);
		synopGMAC_linux_open(ndev);			
		//dev_open(ndev);			

	}

	//printk("%s",__FUNCTION__);
	return 0;
}
#else
#define loongson_gmac_suspend	NULL
#define loongson_gmac_resume	NULL
#endif
static struct platform_driver loongson_gmac_driver = {
	.probe			= init_one_platform,
	.remove			= remove_one_platform,
	.suspend		= loongson_gmac_suspend,
	.resume			= loongson_gmac_resume,
	.driver		= {
		.name	= "loongson-gmac",
		.owner	= THIS_MODULE,
	},
};

static int __init loongson_gmac_init(void)
{
	platform_driver_register(&loongson_gmac_driver);
	return 0;
}


module_init(loongson_gmac_init);

static void __exit loongson_gmac_exit(void)
{
	platform_driver_unregister(&loongson_gmac_driver);
}

module_exit(loongson_gmac_exit);

MODULE_AUTHOR("China Loongson");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SYNOPSYS GMAC DRIVER Network INTERFACE");
